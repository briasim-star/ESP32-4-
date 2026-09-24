#include "waveform.h"
#include "pins.h"
#include <math.h>

// ----------------------------------------------------------------------
// NOTE ON ESP32 ARDUINO CORE VERSION:
// This file uses the "classic" LEDC API (ledcSetup / ledcAttachPin) and
// the classic hw_timer API (timerBegin(num,divider,countUp)). These exist
// in ESP32 Arduino core 2.0.x. If you install core 3.x, the LEDC and timer
// APIs changed shape (ledcAttach(pin,freq,res), timerBegin(freqHz) with no
// divider argument, etc). See the README for exactly which core version to
// install so this compiles without edits.
// ----------------------------------------------------------------------

static const int PWM_CHANNEL_MAG    = 0;  // LEDC channel for the PWM (magnitude) pin
static const int PWM_CHANNEL_SQUARE = 1;  // LEDC channel used for the DIR pin in square mode
static const int PWM_RES_BITS       = 8;  // 0-255 duty resolution - fine for the MAG channel's fixed 20kHz carrier
static const uint32_t SINE_CARRIER_HZ = 20000; // MD10C's rated max switching frequency
static const uint32_t LEDC_APB_CLK_HZ = 80000000UL;

// The SQUARE channel's frequency varies across our whole PEMF range
// (as low as 1Hz up to MAX_OUTPUT_FREQ_HZ = 20kHz), and frequency and
// duty resolution are interdependent on this hardware - a resolution
// that works at 20kHz fails at 1Hz, and vice versa (confirmed via a
// real hardware error: "ledc: requested frequency and duty resolution
// can not be achieved"). This computes the right resolution for
// whatever frequency is actually being requested, using ESP-IDF's own
// documented formula (bits = floor(log2(clock / freq))), capped at a
// safe, well-supported value.
static int calcLedcBits(float freqHz) {
  if (freqHz < 1) freqHz = 1;
  float bitsF = logf((float)LEDC_APB_CLK_HZ / freqHz) / logf(2.0f);
  int bits = (int)bitsF;
  if (bits > 14) bits = 14; // comfortably supported even at low frequencies; higher isn't needed here
  if (bits < 1) bits = 1;
  return bits;
}

static const int SIN_TABLE_MAX = 100;
static float sinTable[SIN_TABLE_MAX];
static volatile int sinTableLen = 0;
static volatile int sinIndex = 0;

static hw_timer_t* sineTimer = nullptr;
static volatile bool running = false;
static volatile float currentFreq = 0;
static volatile WaveShape currentShape = WAVE_SQUARE;
static volatile float intensityFrac = 1.0f; // 0.0 - 1.0, scales output magnitude

// ---- Sawtooth and Layered shapes -------------------------------------
// Driven from a small task (1 ms steps) that only calls ledcWrite - no
// interrupt-time LEDC calls. SAW: the magnitude duty ramps 0 -> set power
// over each period, then drops; DIR flips polarity every pulse. LAYERED:
// DIR runs as a hardware 500 Hz square (the fast "background" flips) and
// the magnitude is switched on for a burst at the start of each slow
// period, off for the rest. Power always scales with intensityFrac, so the
// normal power setting and the 40% ceiling apply unchanged.
static const float LAYER_CARRIER_HZ = 500.0f;
static const float SAW_MAX_HZ = 50.0f;      // above this the 1 ms steps get too coarse - square is used instead
static TaskHandle_t shapeTask = nullptr;
static volatile bool shapeActive = false;   // the task only drives the coil while this is true

static void shapeTaskFn(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  float phase = 0;
  bool polarity = false;
  for (;;) {
    vTaskDelayUntil(&lastWake, 1); // 1 ms
    if (!shapeActive || !running) { phase = 0; continue; }
    float f = currentFreq;
    if (f <= 0) continue;
    phase += f / 1000.0f;
    bool wrapped = false;
    if (phase >= 1.0f) { phase -= (float)(int)phase; wrapped = true; }
    uint8_t full = (uint8_t)(255.0f * intensityFrac);
    if (currentShape == WAVE_SAW) {
      if (wrapped) { polarity = !polarity; digitalWrite(PIN_MD10C_DIR, polarity ? HIGH : LOW); }
      ledcWrite(PWM_CHANNEL_MAG, (uint8_t)(full * phase));
    } else { // WAVE_LAYERED: burst = first 25% of each period, at most 60 ms
      float burstFrac = 0.060f * f;
      if (burstFrac > 0.25f) burstFrac = 0.25f;
      ledcWrite(PWM_CHANNEL_MAG, phase < burstFrac ? full : 0);
    }
  }
}

static void startShapeTask() {
  if (!shapeTask) xTaskCreatePinnedToCore(shapeTaskFn, "pulse", 2048, NULL, 2, &shapeTask, 1);
}

static void buildSinTable(int len) {
  if (len > SIN_TABLE_MAX) len = SIN_TABLE_MAX;
  sinTableLen = len;
  for (int i = 0; i < len; i++) {
    sinTable[i] = sinf(2.0f * (float)M_PI * (float)i / (float)len);
  }
}

void IRAM_ATTR onSineTick() {
  if (!running || sinTableLen == 0) return;
  float v = sinTable[sinIndex];
  sinIndex++;
  if (sinIndex >= sinTableLen) sinIndex = 0;
  digitalWrite(PIN_MD10C_DIR, v >= 0 ? HIGH : LOW);
  uint8_t duty = (uint8_t)(fabsf(v) * 255.0f * intensityFrac);
  ledcWrite(PWM_CHANNEL_MAG, duty);
}

static void stopSineTimer() {
  if (sineTimer) {
    timerAlarmDisable(sineTimer);
    timerDetachInterrupt(sineTimer);
    timerEnd(sineTimer);
    sineTimer = nullptr;
  }
}

void waveform_begin() {
  pinMode(PIN_MD10C_DIR, OUTPUT);
  digitalWrite(PIN_MD10C_DIR, LOW);

  // IO25 is also the chip's DAC1 pin. pinMode() (gpio_config) releases it
  // from the analog/RTC side in case audio setup ever touched it, so the
  // coil PWM is guaranteed to reach the pin.
  pinMode(PIN_MD10C_PWM, OUTPUT);
  digitalWrite(PIN_MD10C_PWM, LOW);
  ledcSetup(PWM_CHANNEL_MAG, SINE_CARRIER_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MD10C_PWM, PWM_CHANNEL_MAG);
  ledcWrite(PWM_CHANNEL_MAG, 0);

  running = false;
  currentFreq = 0;
}

void waveform_stop() {
  running = false;
  shapeActive = false;
  delay(2); // let the pulse task finish its current step before the pins are reset
  stopSineTimer();

  // Make sure DIR is released from LEDC (used in square mode) before we
  // drive it as a plain digital pin again.
  ledcDetachPin(PIN_MD10C_DIR);
  pinMode(PIN_MD10C_DIR, OUTPUT);
  digitalWrite(PIN_MD10C_DIR, LOW);

  ledcWrite(PWM_CHANNEL_MAG, 0);
  currentFreq = 0;
}

void waveform_start(float freqHz, WaveShape shape, uint8_t intensityPercent) {
  waveform_stop();
  if (freqHz <= 0 || freqHz > MAX_OUTPUT_FREQ_HZ) return;
  if (intensityPercent > 100) intensityPercent = 100;

  if (shape == WAVE_SAW && freqHz > SAW_MAX_HZ) shape = WAVE_SQUARE;
  currentFreq = freqHz;
  currentShape = shape;
  intensityFrac = intensityPercent / 100.0f;
  running = true;

  if (shape == WAVE_SAW || shape == WAVE_LAYERED) {
    startShapeTask();
    ledcWrite(PWM_CHANNEL_MAG, 0);
    if (shape == WAVE_LAYERED) { // fast polarity flips from the hardware; the task gates the power
      int bits = calcLedcBits(LAYER_CARRIER_HZ);
      ledcSetup(PWM_CHANNEL_SQUARE, (uint32_t)LAYER_CARRIER_HZ, bits);
      ledcAttachPin(PIN_MD10C_DIR, PWM_CHANNEL_SQUARE);
      ledcWrite(PWM_CHANNEL_SQUARE, 1 << (bits - 1));
    } else {
      pinMode(PIN_MD10C_DIR, OUTPUT);
    }
    shapeActive = true;
  } else if (shape == WAVE_SQUARE) {
    // PWM pin's duty sets the overall magnitude (scaled by intensity);
    // DIR is hardware-toggled at the target frequency with 50% duty,
    // giving a clean bipolar square wave whose amplitude is set by PWM.
    ledcWrite(PWM_CHANNEL_MAG, (uint8_t)(255.0f * intensityFrac));
    int sqBits = calcLedcBits(freqHz);
    ledcSetup(PWM_CHANNEL_SQUARE, (uint32_t)freqHz, sqBits);
    ledcAttachPin(PIN_MD10C_DIR, PWM_CHANNEL_SQUARE);
    ledcWrite(PWM_CHANNEL_SQUARE, 1 << (sqBits - 1)); // 50% duty at whatever resolution was actually configured
  } else {
    // Sign-magnitude sine: fewer samples/cycle at higher frequencies so the
    // ISR rate stays bounded (~8kHz ceiling on the sample clock).
    int samplesPerCycle = (int)(8000.0f / freqHz);
    if (samplesPerCycle < 8) samplesPerCycle = 8;
    if (samplesPerCycle > SIN_TABLE_MAX) samplesPerCycle = SIN_TABLE_MAX;
    buildSinTable(samplesPerCycle);
    sinIndex = 0;

    pinMode(PIN_MD10C_DIR, OUTPUT);
    sineTimer = timerBegin(0, 80, true); // 80MHz / 80 = 1MHz tick (1us)
    timerAttachInterrupt(sineTimer, &onSineTick, true);
    uint32_t tickUs = (uint32_t)(1000000.0f / (freqHz * samplesPerCycle));
    if (tickUs < 20) tickUs = 20; // practical ISR floor on ESP32
    timerAlarmWrite(sineTimer, tickUs, true);
    timerAlarmEnable(sineTimer);
  }
}

bool waveform_isRunning() { return running; }
float waveform_currentFreq() { return currentFreq; }
WaveShape waveform_currentShape() { return currentShape; }

void waveform_setIntensity(uint8_t intensityPercent) {
  if (intensityPercent > 100) intensityPercent = 100;
  intensityFrac = intensityPercent / 100.0f;
  // For square wave, magnitude is a static duty on PWM_CHANNEL_MAG - update
  // it live. For sine, the ISR reads intensityFrac every tick already.
  if (running && currentShape == WAVE_SQUARE) {
    ledcWrite(PWM_CHANNEL_MAG, (uint8_t)(255.0f * intensityFrac));
  }
}

// Live frequency update while running - added for Programs (harmonics
// cycling, ramps). Square wave: re-configuring an already-attached LEDC
// channel updates its frequency in place. Sine: recompute the lookup
// table size and timer tick rate, same math waveform_start() uses for
// its sine branch, then re-arm the existing timer without a full
// stop/restart (avoids tearing down and rebuilding the ISR object).
void waveform_setFrequency(float newFreqHz) {
  if (!running) return;
  if (newFreqHz <= 0 || newFreqHz > MAX_OUTPUT_FREQ_HZ) return;
  if (currentShape == WAVE_SAW && newFreqHz > SAW_MAX_HZ) newFreqHz = SAW_MAX_HZ;
  currentFreq = newFreqHz;

  if (currentShape == WAVE_SAW || currentShape == WAVE_LAYERED) {
    return; // the pulse task reads currentFreq every step
  } else if (currentShape == WAVE_SQUARE) {
    int sqBits = calcLedcBits(newFreqHz);
    ledcSetup(PWM_CHANNEL_SQUARE, (uint32_t)newFreqHz, sqBits);
    ledcWrite(PWM_CHANNEL_SQUARE, 1 << (sqBits - 1)); // keep the duty at 50% for whatever resolution this frequency now needs
  } else {
    int samplesPerCycle = (int)(8000.0f / newFreqHz);
    if (samplesPerCycle < 8) samplesPerCycle = 8;
    if (samplesPerCycle > SIN_TABLE_MAX) samplesPerCycle = SIN_TABLE_MAX;
    buildSinTable(samplesPerCycle);
    sinIndex = 0;
    uint32_t tickUs = (uint32_t)(1000000.0f / (newFreqHz * samplesPerCycle));
    if (tickUs < 20) tickUs = 20;
    if (sineTimer) {
      timerAlarmWrite(sineTimer, tickUs, true);
    }
  }
}

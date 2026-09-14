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
static const int PWM_RES_BITS       = 8;  // 0-255 duty resolution
static const uint32_t SINE_CARRIER_HZ = 20000; // MD10C's rated max switching frequency

static const int SIN_TABLE_MAX = 100;
static float sinTable[SIN_TABLE_MAX];
static volatile int sinTableLen = 0;
static volatile int sinIndex = 0;

static hw_timer_t* sineTimer = nullptr;
static volatile bool running = false;
static volatile float currentFreq = 0;
static volatile WaveShape currentShape = WAVE_SQUARE;
static volatile float intensityFrac = 1.0f; // 0.0 - 1.0, scales output magnitude

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

  ledcSetup(PWM_CHANNEL_MAG, SINE_CARRIER_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MD10C_PWM, PWM_CHANNEL_MAG);
  ledcWrite(PWM_CHANNEL_MAG, 0);

  running = false;
  currentFreq = 0;
}

void waveform_stop() {
  running = false;
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

  currentFreq = freqHz;
  currentShape = shape;
  intensityFrac = intensityPercent / 100.0f;
  running = true;

  if (shape == WAVE_SQUARE) {
    // PWM pin's duty sets the overall magnitude (scaled by intensity);
    // DIR is hardware-toggled at the target frequency with 50% duty,
    // giving a clean bipolar square wave whose amplitude is set by PWM.
    ledcWrite(PWM_CHANNEL_MAG, (uint8_t)(255.0f * intensityFrac));
    ledcSetup(PWM_CHANNEL_SQUARE, (uint32_t)freqHz, PWM_RES_BITS);
    ledcAttachPin(PIN_MD10C_DIR, PWM_CHANNEL_SQUARE);
    ledcWrite(PWM_CHANNEL_SQUARE, 128); // 50% duty
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

// ============================================================================
// Flux12_Lab - coil experiment program for the LCDWiki 4.0" ESP32-32E (E32R40T)
// by The MADD Resonist. Hobby magnet/coil experiments only.
// Same look and controls as the MADD PEMF app: aurora background, MADD cut-corner
// buttons, "- value +" steppers (no sliders), FreeSansBold fonts.
// ----------------------------------------------------------------------------
// Drives a Cytron MD30C / MD10C on the board's I2C header (IO25 = PWM, IO32 = DIR),
// reads up to two A1324 Hall sensors (2:1 dividers) and logs a CSV on USB serial (115200).
// Modes:  SQUARE = H-bridge square wave (DIR flips at the set frequency, PWM = power)
//         PULSE  = one-direction pulses at frequency / duty
//         STEADY = constant current (PWM = power)
//         SWEEP  = SQUARE stepping 1 -> 60 Hz over 30 s, Hall peak per step on serial
// Coils:  STAR  = 20 AWG 14+14 star coil (~3.24 ohm): power to 100 % (7.4 A at 24 V)
//         BIG16 = 16 AWG 12-layer star coil (~0.43 ohm): power capped at 35 % (~20 A avg)
// Safety: run cap 60 s then 60 s cooldown; STOP always works; driver off on boot/fault/watchdog.
// Not a medical device. No body claims.
// ============================================================================
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include "pins.h"
#include "lab_types.h"

const char* FIRMWARE_VERSION = "0.4.1";   // 0.4.1: MODE picker screen (all modes listed) + OFF button (deep sleep, tap to wake)   // 0.4.0: plain-words MAIN screen + SETTINGS screen (Brian 2026-10-11: "clean this up")   // 0.3.1: MADD hexagon logo badge + boot splash, live Hall trace (Brian's pick A, 2026-10-11)

#define PIN_DRV_PWM   PIN_MD10C_PWM   // 25
#define PIN_DRV_DIR   PIN_MD10C_DIR   // 32
#define PIN_HALL_A    34              // BAT connector + pin: the board already halves it onto IO34 (no resistors)
#define PIN_HALL_B    34              // only one spare input on this board: Hall B mirrors A until a second input is wired

static const uint32_t PWM_HZ = 20000; static const int PWM_BITS = 8, PWM_CH = 0;
static const uint32_t RUN_MAX_S = 60, COOLDOWN_S = 60;

struct Coil { const char* name; float ohms; int powerCap; };
static const Coil COILS[] = { {"STAR 3.2", 3.24f, 100}, {"BIG16 0.43", 0.43f, 35} };
static const char* MODE_NAMES[] = {"SQUARE", "PULSE", "STEADY", "SWEEP", "SPIN", "HOVER", "BOUNCE"};
// SPIN  = square wave + Hall pole counting (RPM).  HOVER = sphere hangs below the coil, Hall gap held by PID (hold the
// sphere where you want it, tap RUN: that Hall reading becomes the target).  BOUNCE = HOVER with the target moving.
// In HOVER/BOUNCE: POWER = base duty, DUTY = gain (1-99), FREQ/10 = bounce rate in Hz.

TFT_eSPI tft; Preferences prefs;
const GFXfont* FONT_SM = &FreeSansBold9pt7b; const GFXfont* FONT_LG = &FreeSansBold12pt7b;
uint16_t MADD_PANEL, MADD_EDGE, MADD_MAGENTA, MADD_CYAN, MADD_COIL2, MADD_TEXT, MADD_DIM, COLOR_GOOD, COLOR_DANGER, COLOR_WARN, COLOR_PANEL_LIT;
uint16_t MADD_SPECTRUM[6]; static const int AURORA_BANDS = 16; uint16_t AURORA[AURORA_BANDS];

// ---- state ----------------------------------------------------------------------
int coilIdx = 0; Mode mode = SQUARE;
int freqHz = 10, dutyPct = 50, powerPct = 20;
bool running = false; uint32_t runStartMs = 0, cooldownUntilMs = 0;
bool bipolar = true;                         // BI = H-bridge flips direction each half cycle; UNI = one direction, pulsed (Brian 2026-10-11)
int trX = 0; uint32_t lastTraceMs = 0; int trScale = 300;   // live Hall trace (sweep style, one column per tick)
int hallA_mV = 0, hallB_mV = 0, hallA_peak = 0, hallB_peak = 0, hallA_zero = 0, hallB_zero = 0;
hw_timer_t* swTimer = nullptr; volatile bool polarity = false; volatile uint32_t halfPeriodUs = 50000;
uint32_t lastLogMs = 0, lastUiMs = 0, sweepStepMs = 0; float sweepF = 1.0f; int sweepPeak = 0;
volatile int hoverTarget = 0, hoverDuty = 0; volatile float hoverI = 0; int lastHallSign = 0; uint32_t poleCount = 0, rpmWindowMs = 0; int rpm = 0;
TaskHandle_t ctlTask = nullptr;
int readMilliVoltsFast(int pin) { return analogReadMilliVolts(pin) * 2; }
void controlLoop(void*) {                      // ~1 kHz PID for HOVER / BOUNCE, one direction only
  float lastErr = 0; uint32_t t0 = millis();
  for (;;) {
    if (running && (mode == HOVER || mode == BOUNCE)) {
      float target = hoverTarget;
      if (mode == BOUNCE) target += (float)hoverTarget * 0.25f * sinf(2 * 3.14159f * (freqHz / 10.0f) * ((millis() - t0) / 1000.0f));
      float err = target - (readMilliVoltsFast(PIN_HALL_A) - hallA_zero);
      float kp = dutyPct / 100.0f, kd = kp * 8.0f;
      hoverI += err * 0.002f; hoverI = constrain(hoverI, -60.0f, 60.0f);
      int duty = powerToDuty(powerPct) + (int)(kp * err + kd * (err - lastErr) + hoverI); lastErr = err;
      duty = constrain(duty, 0, powerToDuty(COILS[coilIdx].powerCap)); hoverDuty = duty;
      digitalWrite(PIN_DRV_DIR, HIGH); ledcWrite(PWM_CH, duty);
    } else { hoverI = 0; lastErr = 0; t0 = millis(); }
    vTaskDelay(1);                             // 1 ms tick: NEVER busy-wait here (0.3.1 starved the loop task -> watchdog reboot loop)
  }
}

// ---- driver ----------------------------------------------------------------------
static inline int powerToDuty(int pct) { int cap = COILS[coilIdx].powerCap; if (pct > cap) pct = cap; return (255 * pct) / 100; }
void driverOff() { ledcWrite(PWM_CH, 0); digitalWrite(PIN_DRV_DIR, LOW); }
void IRAM_ATTR onSwitchTimer() { polarity = !polarity; digitalWrite(PIN_DRV_DIR, polarity ? HIGH : LOW); timerAlarmWrite(swTimer, halfPeriodUs, true); }
void IRAM_ATTR onPulseTimer() {
  static bool on = false; on = !on;
  uint32_t period = (uint32_t)(1000000.0f / freqHz); uint32_t onUs = period * dutyPct / 100; if (onUs < 50) onUs = 50;
  ledcWrite(PWM_CH, on ? powerToDuty(powerPct) : 0); timerAlarmWrite(swTimer, on ? onUs : (period - onUs), true);
}
void applyOutput() {
  if (!running) { driverOff(); return; }
  if (swTimer) { timerAlarmDisable(swTimer); timerDetachInterrupt(swTimer); timerEnd(swTimer); swTimer = nullptr; }
  float f = (mode == SWEEP) ? sweepF : (float)freqHz;
  if (mode == STEADY) { digitalWrite(PIN_DRV_DIR, HIGH); ledcWrite(PWM_CH, powerToDuty(powerPct)); return; }
  if (mode == HOVER || mode == BOUNCE) { return; }   // the control task drives the PWM
  swTimer = timerBegin(0, 80, true);
  if (mode == PULSE || !bipolar) { digitalWrite(PIN_DRV_DIR, HIGH); timerAttachInterrupt(swTimer, &onPulseTimer, true); timerAlarmWrite(swTimer, 1000, true); }   // UNI: one direction, on/off at freq x duty
  else { halfPeriodUs = (uint32_t)(500000.0f / f); ledcWrite(PWM_CH, powerToDuty(powerPct)); timerAttachInterrupt(swTimer, &onSwitchTimer, true); timerAlarmWrite(swTimer, halfPeriodUs, true); }
  timerAlarmEnable(swTimer);
}
void startRun() {
  if (millis() < cooldownUntilMs) return;
  running = true; runStartMs = millis(); hallA_peak = hallB_peak = 0; sweepF = 1.0f; sweepStepMs = millis(); sweepPeak = 0;
  readHall(); hoverTarget = hallA_mV; hoverI = 0; poleCount = 0; rpmWindowMs = millis(); rpm = 0;
  applyOutput(); Serial.println("# RUN start"); Serial.printf("# polarity %s\n", bipolar ? "BI" : "UNI"); Serial.println("ms,mode,coil,freq_hz,duty_pct,power_pct,hallA_mV,hallB_mV");
}
void stopRun(const char* why) {
  bool wasFull = running && (millis() - runStartMs) >= RUN_MAX_S * 1000UL;
  running = false; applyOutput(); driverOff(); hoverDuty = 0;
  if (wasFull) cooldownUntilMs = millis() + COOLDOWN_S * 1000UL;
  Serial.printf("# RUN stop (%s)\n", why);
}

// ---- Hall ----------------------------------------------------------------------
int readMilliVolts(int pin) { uint32_t s = 0; for (int i = 0; i < 16; i++) s += analogReadMilliVolts(pin); return (int)(s / 16) * 2; }
void readHall() {
  hallA_mV = readMilliVolts(PIN_HALL_A) - hallA_zero; hallB_mV = readMilliVolts(PIN_HALL_B) - hallB_zero;
  if (abs(hallA_mV) > abs(hallA_peak)) hallA_peak = hallA_mV;
  if (abs(hallB_mV) > abs(hallB_peak)) hallB_peak = hallB_mV;
  if (abs(hallA_mV) > abs(sweepPeak)) sweepPeak = hallA_mV;
  int sg = hallA_mV > 40 ? 1 : (hallA_mV < -40 ? -1 : 0);          // pole passes for SPIN rpm (2 per turn)
  if (sg && lastHallSign && sg != lastHallSign) poleCount++;
  if (sg) lastHallSign = sg;
  if (millis() - rpmWindowMs >= 1000) { rpm = poleCount * 30; poleCount = 0; rpmWindowMs = millis(); }
}
void zeroHall() { hallA_zero = hallB_zero = 0; readHall(); hallA_zero = hallA_mV; hallB_zero = hallB_mV; hallA_peak = hallB_peak = 0; }

// ---- MADD look (same as the PEMF app) -----------------------------------------------
void initPalette() {
  MADD_PANEL = tft.color565(18, 10, 36); MADD_EDGE = tft.color565(74, 53, 112); MADD_MAGENTA = tft.color565(232, 62, 156);
  MADD_CYAN = tft.color565(63, 216, 255); MADD_COIL2 = tft.color565(91, 124, 255); MADD_TEXT = tft.color565(244, 238, 255);
  MADD_DIM = tft.color565(183, 169, 214); COLOR_GOOD = tft.color565(40, 170, 120); COLOR_DANGER = tft.color565(210, 60, 60);
  COLOR_WARN = tft.color565(230, 175, 60); COLOR_PANEL_LIT = tft.color565(30, 70, 120);
  const uint8_t spec[6][3] = {{255,166,43},{255,107,69},{232,69,122},{181,60,201},{123,70,230},{47,139,255}};
  for (int i = 0; i < 6; i++) MADD_SPECTRUM[i] = tft.color565(spec[i][0], spec[i][1], spec[i][2]);
  for (int i = 0; i < AURORA_BANDS; i++) { float t = (float)i / (AURORA_BANDS - 1); AURORA[i] = tft.color565(13 + (int)(35 * t), 8 + (int)(3 * t), 38 + (int)(13 * t)); }
}
uint16_t auroraAt(int y) { int b = y * AURORA_BANDS / 320; if (b < 0) b = 0; if (b >= AURORA_BANDS) b = AURORA_BANDS - 1; return AURORA[b]; }
void fillAurora(int x, int y, int w, int h) { const int bandH = 320 / AURORA_BANDS; int yEnd = y + h; while (y < yEnd) { int next = ((y / bandH) + 1) * bandH; if (next > yEnd) next = yEnd; tft.fillRect(x, y, w, next - y, auroraAt(y)); y = next; } }
void drawChamfer(Rect r, uint16_t fill, uint16_t edge, int c = 10) {
  tft.fillRect(r.x + c, r.y, r.w - 2 * c, r.h, fill); tft.fillRect(r.x, r.y + c, c, r.h - 2 * c, fill); tft.fillRect(r.x + r.w - c, r.y + c, c, r.h - 2 * c, fill);
  tft.fillTriangle(r.x, r.y + c, r.x + c, r.y, r.x + c, r.y + c, fill); tft.fillTriangle(r.x + r.w - 1, r.y + c, r.x + r.w - 1 - c, r.y, r.x + r.w - 1 - c, r.y + c, fill);
  tft.fillTriangle(r.x, r.y + r.h - 1 - c, r.x + c, r.y + r.h - 1, r.x + c, r.y + r.h - 1 - c, fill); tft.fillTriangle(r.x + r.w - 1, r.y + r.h - 1 - c, r.x + r.w - 1 - c, r.y + r.h - 1, r.x + r.w - 1 - c, r.y + r.h - 1 - c, fill);
  int x0 = r.x, y0 = r.y, x1 = r.x + r.w - 1, y1 = r.y + r.h - 1;
  tft.drawLine(x0 + c, y0, x1 - c, y0, edge); tft.drawLine(x1 - c, y0, x1, y0 + c, edge); tft.drawLine(x1, y0 + c, x1, y1 - c, edge);
  tft.drawLine(x1, y1 - c, x1 - c, y1, edge); tft.drawLine(x1 - c, y1, x0 + c, y1, edge); tft.drawLine(x0 + c, y1, x0, y1 - c, edge);
  tft.drawLine(x0, y1 - c, x0, y0 + c, edge); tft.drawLine(x0, y0 + c, x0 + c, y0, edge);
}
void drawChamferButton(Rect r, const char* label, uint16_t fill, uint16_t edge, uint16_t textColor, const GFXfont* font) {
  drawChamfer(r, fill, edge, r.h >= 40 ? 10 : 7); tft.setFreeFont(font); tft.setTextColor(textColor, fill); tft.setTextDatum(MC_DATUM);
  char buf[40]; strncpy(buf, label, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  while (strlen(buf) > 1 && tft.textWidth(buf) > r.w - 14) buf[strlen(buf) - 1] = 0;
  tft.drawString(buf, r.x + r.w / 2, r.y + r.h / 2); tft.setTextDatum(TL_DATUM);
}
// kind: 0 plain, 1 lit (active), 2 good, 3 danger, 4 warn
void drawButton(Rect r, const char* label, int kind = 0, const GFXfont* font = nullptr) {
  uint16_t fill = MADD_PANEL, edge = MADD_EDGE;
  if (kind == 1) { fill = tft.color565(12, 58, 68); edge = MADD_CYAN; } else if (kind == 2) { fill = tft.color565(12, 58, 40); edge = COLOR_GOOD; }
  else if (kind == 3) { fill = tft.color565(58, 13, 26); edge = MADD_SPECTRUM[2]; } else if (kind == 4) { fill = tft.color565(90, 54, 6); edge = MADD_SPECTRUM[0]; }
  drawChamferButton(r, label, fill, edge, MADD_TEXT, font ? font : FONT_LG);
}
bool inRect(int px, int py, Rect r) { return px >= r.x && px < r.x + r.w && py >= r.y && py < r.y + r.h; }

// ---- steppers ("- value +", same geometry as the PEMF app) -----------------------------
Rect stepperMinusRect(const Stepper& s) { return {s.x, s.y, 46, 40}; }
Rect stepperValueRect(const Stepper& s) { return {s.x + 52, s.y, 100, 40}; }
Rect stepperPlusRect(const Stepper& s)  { return {s.x + 158, s.y, 46, 40}; }
Stepper stFreq  = {"FREQUENCY  Hz",      20, 144, 10, 1, 200, 1};     // main screen, left
Stepper stPower = {"POWER  % of 24 V",   250, 144, 20, 0, 100, 5};     // main screen, right
Stepper stDuty  = {"PULSE WIDTH %  (UNI / PULSE)  =  hover gain", 20, 184, 50, 1, 99, 5};   // settings screen
void drawStepper(const Stepper& s) {
  fillAurora(s.x, s.y - 24, 206, 22); tft.setFreeFont(FONT_SM); tft.setTextColor(MADD_DIM, auroraAt(s.y - 12)); tft.setTextDatum(TL_DATUM); tft.drawString(s.label, s.x, s.y - 22);
  drawButton(stepperMinusRect(s), "-"); drawButton(stepperPlusRect(s), "+");
  Rect vr = stepperValueRect(s); tft.fillRect(vr.x, vr.y, vr.w, vr.h, MADD_PANEL); tft.drawRect(vr.x, vr.y, vr.w, vr.h, MADD_EDGE);
  tft.setFreeFont(FONT_LG); tft.setTextColor(MADD_TEXT, MADD_PANEL); tft.setTextDatum(MC_DATUM);
  char buf[16]; snprintf(buf, sizeof(buf), "%d", s.value); tft.drawString(buf, vr.x + vr.w / 2, vr.y + vr.h / 2); tft.setTextDatum(TL_DATUM);
}
bool handleStepperTouch(Stepper& s, int x, int y) {
  if (inRect(x, y, stepperMinusRect(s))) { s.value -= s.step; } else if (inRect(x, y, stepperPlusRect(s))) { s.value += s.step; } else return false;
  if (s.value < s.minVal) s.value = s.minVal; if (s.value > s.maxVal) s.value = s.maxVal; return true;
}

// ---- screens: MAIN (what you need to run) + SETTINGS (coil, polarity, pulse width, sensor zero, touch) ------------
int uiScreen = 0;                            // 0 = main, 1 = settings, 2 = mode picker
Rect bMode = {20, 52, 250, 40}, bSettings = {290, 52, 170, 40};
Rect bRunStop = {20, 192, 440, 40};
Rect rHall = {20, 238, 440, 68}, rTrace = {21, 262, 438, 43};
Rect bOff = {236, 8, 54, 30}, bOffBig = {20, 276, 210, 32};
Rect bBack = {360, 6, 100, 32}, bCoil = {20, 56, 440, 40}, bPol = {20, 104, 440, 40}, bZero = {20, 232, 440, 36}, bCal = {20, 276, 210, 32};
static const char* MODE_HINTS[] = {"field flips + and - at the frequency", "one-way pulses, width = pulse width %", "steady field, no switching",
                                   "1 to 60 Hz over 30 s, logs the peak", "flips the field to spin the ball", "holds the ball below (needs the sensor)",
                                   "hover, with the ball moving up and down"};
// ---- MADD hexagon logo (drawn, no SD card): pointed-top hexagon, rainbow wave stripes, magenta edge --------------
void drawLogoHex(int cx, int cy, int r) {
  int half_w = (int)(r * 0.866f);
  for (int dx = -half_w; dx <= half_w; dx++) {
    int top = cy - r + (int)(abs(dx) * 0.577f), bot = cy + r - (int)(abs(dx) * 0.577f);
    int wave = (int)(0.22f * r * sinf(((float)dx / r) * 2.4f)); float band = 2.0f * r / 6.0f;
    for (int i = 0; i < 6; i++) {
      int y0 = (int)(cy - r + i * band + wave + 0.08f * band), y1 = (int)(cy - r + (i + 1) * band + wave - 0.08f * band);
      if (y0 < top) y0 = top; if (y1 > bot) y1 = bot; if (y1 > y0) tft.drawFastVLine(cx + dx, y0, y1 - y0, MADD_SPECTRUM[i]);
    }
  }
  int px[6], py[6];
  for (int i = 0; i < 6; i++) { float a = radians(60 * i - 90); px[i] = cx + (int)lroundf(r * cosf(a)); py[i] = cy + (int)lroundf(r * sinf(a)); }
  for (int i = 0; i < 6; i++) { int j = (i + 1) % 6; tft.drawLine(px[i], py[i], px[j], py[j], MADD_MAGENTA); tft.drawLine(px[i] + 1, py[i], px[j] + 1, py[j], MADD_MAGENTA); }
}
void drawSplash() {
  fillAurora(0, 0, 480, 320);
  for (int i = 0; i < 6; i++) for (int x = 0; x < 479; x += 2) {   // neon wave lines across the bottom
    int y0 = 250 + i * 9 + (int)(14 * sinf(x / 70.0f + i)), y1 = 250 + i * 9 + (int)(14 * sinf((x + 2) / 70.0f + i));
    tft.drawLine(x, y0, x + 2, y1, MADD_SPECTRUM[i]);
  }
  drawLogoHex(120, 150, 78);
  tft.setTextDatum(ML_DATUM); tft.setFreeFont(FONT_LG); tft.setTextColor(MADD_TEXT, auroraAt(125)); tft.drawString("MADD Flux 12", 228, 125);
  tft.setFreeFont(FONT_SM); tft.setTextColor(MADD_MAGENTA, auroraAt(170)); tft.drawString("by The MADD Resonist", 230, 170);
  tft.setTextColor(MADD_DIM, auroraAt(200)); tft.drawString("coil + magnet experiments", 230, 200); tft.setTextDatum(TL_DATUM);
}
void drawStatusPill() {                      // top-right: COIL OFF / RUNNING / COOLING, in words
  char l[24]; uint16_t col = MADD_DIM;
  if (running) { snprintf(l, sizeof(l), "RUNNING %lus", (unsigned long)((millis() - runStartMs) / 1000)); col = COLOR_GOOD; }
  else if (millis() < cooldownUntilMs) { snprintf(l, sizeof(l), "COOLING %lus", (unsigned long)((cooldownUntilMs - millis()) / 1000)); col = COLOR_WARN; }
  else snprintf(l, sizeof(l), "COIL OFF");
  Rect r = {296, 8, 164, 30}; drawChamfer(r, MADD_PANEL, col, 7); tft.setFreeFont(FONT_SM); tft.setTextColor(col, MADD_PANEL); tft.setTextDatum(MC_DATUM);
  tft.drawString(l, r.x + r.w / 2, r.y + r.h / 2); tft.setTextDatum(TL_DATUM);
}
void drawHeader(const char* title) {
  fillAurora(0, 0, 480, 44); drawLogoHex(36, 21, 17); tft.setFreeFont(FONT_LG); tft.setTextColor(MADD_TEXT, auroraAt(10)); tft.setTextDatum(TL_DATUM); tft.drawString(title, 62, 8);
  for (int i = 0; i < 6; i++) tft.fillRect(62 + i * 36, 36, 34, 3, MADD_SPECTRUM[i]);
}
void drawRunStop() {
  if (running) drawButton(bRunStop, "STOP", 3);
  else if (millis() < cooldownUntilMs) drawButton(bRunStop, "COOLING - wait", 4, FONT_SM);
  else drawButton(bRunStop, "RUN", 2);
}
void drawMain() {
  uiScreen = 0; fillAurora(0, 0, 480, 320); drawHeader("MADD Flux 12"); drawStatusPill();
  drawButton(bOff, "OFF", 3, FONT_SM);
  char l[40]; snprintf(l, sizeof(l), "MODE: %s  (tap to pick)", MODE_NAMES[mode]); drawButton(bMode, l, 1, FONT_SM); drawButton(bSettings, "SETTINGS", 0, FONT_SM);
  fillAurora(20, 96, 440, 20); tft.setFreeFont(FONT_SM); tft.setTextColor(MADD_DIM, auroraAt(100)); tft.setTextDatum(TL_DATUM); tft.drawString(MODE_HINTS[mode], 24, 98);
  stFreq.value = freqHz; stPower.value = powerPct; drawStepper(stFreq); drawStepper(stPower);
  drawRunStop();
  tft.fillRect(rHall.x, rHall.y, rHall.w, rHall.h, MADD_PANEL); tft.drawRect(rHall.x, rHall.y, rHall.w, rHall.h, MADD_EDGE);
  tft.drawFastHLine(rTrace.x, rTrace.y + rTrace.h / 2, rTrace.w, MADD_EDGE); trX = 0;
}
void drawSettings() {
  uiScreen = 1; fillAurora(0, 0, 480, 320); drawHeader("SETTINGS"); drawButton(bBack, "BACK", 1, FONT_SM);
  char l[64];
  snprintf(l, sizeof(l), "COIL: %s   (%s)", coilIdx ? "BIG 16 AWG" : "STAR 20 AWG", coilIdx ? "power capped at 35%" : "full power OK"); drawButton(bCoil, l, coilIdx ? 4 : 0, FONT_SM);
  snprintf(l, sizeof(l), "POLARITY: %s", bipolar ? "BI  (field flips + and -)" : "UNI  (one direction, pulses)"); drawButton(bPol, l, bipolar ? 1 : 0, FONT_SM);
  stDuty.value = dutyPct; drawStepper(stDuty);
  tft.setFreeFont(FONT_SM); tft.setTextColor(MADD_DIM, auroraAt(195)); tft.setTextDatum(TL_DATUM); tft.drawString("1-99.  Only used by UNI, PULSE, HOVER, BOUNCE.", 236, 196);
  drawButton(bZero, "ZERO THE FIELD SENSOR  (no magnet near it)", 0, FONT_SM);
  drawButton(bCal, "RECALIBRATE TOUCH", 0, FONT_SM); drawButton(bOffBig, "POWER OFF", 3, FONT_SM);
  snprintf(l, sizeof(l), "v%s  log on USB 115200", FIRMWARE_VERSION); tft.setTextColor(MADD_DIM, auroraAt(290)); tft.drawString(l, 244, 284);
}
void drawModePicker() {                       // all 7 modes on one screen, the current one lit
  uiScreen = 2; fillAurora(0, 0, 480, 320); drawHeader("PICK A MODE"); drawButton(bBack, "BACK", 1, FONT_SM);
  char l[80];
  for (int i = 0; i < 7; i++) {
    Rect r = {20, 50 + i * 38, 440, 34}; snprintf(l, sizeof(l), "%-7s %s", MODE_NAMES[i], MODE_HINTS[i]);
    drawButton(r, l, i == mode ? 1 : 0, FONT_SM);
  }
}
void powerOff() {                             // OFF button: coil off, screen dark, deep sleep; a tap on the screen wakes it (reboots)
  stopRun("power off"); driverOff(); Serial.println("# OFF (deep sleep, tap screen to wake)"); Serial.flush();
  fillAurora(0, 0, 480, 320); tft.setFreeFont(FONT_LG); tft.setTextColor(MADD_DIM, auroraAt(160)); tft.setTextDatum(MC_DATUM); tft.drawString("off - tap to wake", 240, 160); delay(900);
  digitalWrite(TFT_BL, LOW); tft.writecommand(0x10);                         // backlight off, panel sleep-in
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 0);                              // touch IRQ (IO36) goes LOW when pressed
  esp_deep_sleep_start();
}
void drawTrace() {                             // one new column per call: sweep-scope style, no block shifting
  if (uiScreen != 0) return;
  if (abs(hallA_peak) * 11 / 10 > trScale) trScale = abs(hallA_peak) * 11 / 10;
  int mid = rTrace.y + rTrace.h / 2, half = rTrace.h / 2 - 1;
  int y = mid - (int)((long)constrain(hallA_mV, -trScale, trScale) * half / trScale);
  int x = rTrace.x + trX;
  tft.drawFastVLine(x, rTrace.y, rTrace.h, MADD_PANEL); tft.drawPixel(x, mid, MADD_EDGE);
  tft.drawFastVLine(x, y - 1, 3, running ? MADD_CYAN : MADD_DIM);
  if (running) { int xc = rTrace.x + (trX + 1) % rTrace.w; tft.drawFastVLine(xc, rTrace.y, rTrace.h, MADD_COIL2); }   // sweep cursor only while the coil runs
  trX = (trX + 1) % rTrace.w;
}
void drawLive() {
  if (uiScreen != 0) return;
  char l[64]; tft.setFreeFont(FONT_SM); tft.setTextDatum(TL_DATUM);
  tft.fillRect(rHall.x + 1, rHall.y + 1, rHall.w - 2, 22, MADD_PANEL); tft.setTextColor(MADD_TEXT, MADD_PANEL);
  if (mode == SPIN) snprintf(l, sizeof(l), "FIELD %5d mV   peak %5d   %d rpm", hallA_mV, hallA_peak, rpm);
  else if (running && (mode == HOVER || mode == BOUNCE)) snprintf(l, sizeof(l), "FIELD %5d mV   target %d   coil %d%%", hallA_mV, hoverTarget, hoverDuty * 100 / 255);
  else if (running) snprintf(l, sizeof(l), "FIELD %5d mV   peak %5d   %.1f Hz  ~%.1f A", hallA_mV, hallA_peak, mode == SWEEP ? sweepF : (float)freqHz, 24.0f * powerToDuty(powerPct) / 255 / COILS[coilIdx].ohms);
  else snprintf(l, sizeof(l), "FIELD %5d mV   peak %5d", hallA_mV, hallA_peak);
  tft.drawString(l, rHall.x + 8, rHall.y + 5);
  static bool wasRunning = false; static bool wasCooling = false; bool cooling = !running && millis() < cooldownUntilMs;
  if (running != wasRunning || cooling != wasCooling) { drawRunStop(); wasRunning = running; wasCooling = cooling; }
  drawStatusPill();
}
void drawAll() { if (uiScreen == 1) drawSettings(); else if (uiScreen == 2) drawModePicker(); else drawMain(); }
void persist() { prefs.putInt("f", freqHz); prefs.putInt("d", dutyPct); prefs.putInt("p", powerPct); prefs.putInt("m", mode); prefs.putInt("c", coilIdx); prefs.putBool("bp", bipolar); }
void handleTouch(int x, int y) {
  bool changed = false;
  if (uiScreen == 0) {
    if (handleStepperTouch(stFreq, x, y)) { freqHz = stFreq.value; changed = true; if (running && mode != SWEEP) applyOutput(); drawStepper(stFreq); }
    else if (handleStepperTouch(stPower, x, y)) { powerPct = min(stPower.value, COILS[coilIdx].powerCap); stPower.value = powerPct; changed = true; if (running) applyOutput(); drawStepper(stPower); }
    else if (inRect(x, y, bMode)) { drawModePicker(); return; }
    else if (inRect(x, y, bOff)) { powerOff(); }
    else if (inRect(x, y, bSettings)) { drawSettings(); return; }
    else if (inRect(x, y, bRunStop)) { if (running) stopRun("button"); else startRun(); drawRunStop(); }
  } else if (uiScreen == 1) {
    if (inRect(x, y, bBack)) { drawMain(); drawLive(); return; }
    else if (inRect(x, y, bCoil)) { stopRun("coil change"); coilIdx = (coilIdx + 1) % 2; if (powerPct > COILS[coilIdx].powerCap) powerPct = COILS[coilIdx].powerCap; changed = true; drawSettings(); }
    else if (inRect(x, y, bPol)) { bipolar = !bipolar; changed = true; if (running) applyOutput(); drawSettings(); }
    else if (handleStepperTouch(stDuty, x, y)) { dutyPct = stDuty.value; changed = true; drawStepper(stDuty); }
    else if (inRect(x, y, bZero)) { zeroHall(); trScale = 300; drawButton(bZero, "SENSOR ZEROED", 2, FONT_SM); delay(400); drawButton(bZero, "ZERO THE FIELD SENSOR  (no magnet near it)", 0, FONT_SM); }
    else if (inRect(x, y, bCal)) { prefs.remove("cal"); ESP.restart(); }
    else if (inRect(x, y, bOffBig)) { powerOff(); }
  }
  if (uiScreen == 2) {
    if (inRect(x, y, bBack)) { drawMain(); drawLive(); return; }
    for (int i = 0; i < 7; i++) { Rect r = {20, 50 + i * 38, 440, 34}; if (inRect(x, y, r)) { if (i != mode) { stopRun("mode change"); mode = (Mode)i; persist(); } drawMain(); drawLive(); return; } }
    return;
  }
  if (changed) persist();
  drawLive();
}

void setup() {
  pinMode(PIN_DRV_DIR, OUTPUT); digitalWrite(PIN_DRV_DIR, LOW);
  ledcSetup(PWM_CH, PWM_HZ, PWM_BITS); ledcAttachPin(PIN_DRV_PWM, PWM_CH); ledcWrite(PWM_CH, 0);
  Serial.begin(115200); Serial.printf("# Flux12_Lab v%s\n", FIRMWARE_VERSION);
  analogReadResolution(12); analogSetPinAttenuation(PIN_HALL_A, ADC_11db); analogSetPinAttenuation(PIN_HALL_B, ADC_11db);
  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  tft.init(); tft.setRotation(1); initPalette();
  prefs.begin("f12lab", false);
  freqHz = prefs.getInt("f", 10); dutyPct = prefs.getInt("d", 50); powerPct = prefs.getInt("p", 20); mode = (Mode)prefs.getInt("m", 0); coilIdx = prefs.getInt("c", 0); bipolar = prefs.getBool("bp", true);
  uint16_t cal[5];
  if (prefs.getBytesLength("cal") == sizeof(cal)) { prefs.getBytes("cal", cal, sizeof(cal)); tft.setTouch(cal); }
  else { fillAurora(0, 0, 480, 320); tft.setFreeFont(FONT_LG); tft.setTextColor(MADD_TEXT, auroraAt(160)); tft.setTextDatum(MC_DATUM); tft.drawString("Touch the corners", 240, 160);
         tft.calibrateTouch(cal, MADD_CYAN, auroraAt(0), 15); prefs.putBytes("cal", cal, sizeof(cal)); tft.setTouch(cal); }
  zeroHall(); esp_task_wdt_init(5, true); esp_task_wdt_add(NULL);
  xTaskCreatePinnedToCore(controlLoop, "ctl", 4096, NULL, 2, &ctlTask, 0);   // core 0, away from loop()
  drawSplash(); delay(1800);
  drawMain(); drawLive();
}
void loop() {
  esp_task_wdt_reset();
  uint16_t x, y;
  if (tft.getTouch(&x, &y)) { handleTouch(x, y); delay(110); }   // ~9 steps/s when held
  readHall();
  if (running) {
    if (millis() - runStartMs >= RUN_MAX_S * 1000UL) { stopRun("time cap"); drawRunStop(); }
    if (mode == SWEEP && millis() - sweepStepMs >= 1000) {
      Serial.printf("# sweep %.1f Hz peakA %d mV\n", sweepF, sweepPeak); sweepPeak = 0; sweepStepMs = millis();
      sweepF += 59.0f / 30.0f; if (sweepF > 60.0f) { stopRun("sweep done"); drawRunStop(); } else applyOutput();
    }
    if (millis() - lastLogMs >= 100) { lastLogMs = millis(); Serial.printf("%lu,%s,%s,%.1f,%d,%d,%d,%d\n", (unsigned long)(millis() - runStartMs), MODE_NAMES[mode], COILS[coilIdx].name, mode == SWEEP ? sweepF : (float)freqHz, dutyPct, (mode == HOVER || mode == BOUNCE) ? hoverDuty * 100 / 255 : powerPct, hallA_mV, hallB_mV); }
  }
  if (millis() - lastTraceMs >= 40) { lastTraceMs = millis(); drawTrace(); }
  if (millis() - lastUiMs >= 250) { lastUiMs = millis(); drawLive(); }
  delay(5);
}

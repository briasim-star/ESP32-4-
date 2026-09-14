// ============================================================================
// PEMF_Controller.ino
// Touchscreen-driven PEMF signal generator for the LCDWiki 4.0" ESP32-32E
// display, driving a Cytron MD10C motor driver into a coil load, with
// optional Bluetooth audio monitoring.
//
// Libraries required (install via Arduino IDE Library Manager):
//   - TFT_eSPI (Bodmer)          -> display + touch driving
//   - ESP32-A2DP (pschatzmann)   -> optional Bluetooth audio streaming
// Board core required: "ESP32 by Espressif Systems" v2.0.x
//   (see README for why the version matters)
//
// See README.md for full wiring, library configuration, and setup steps.
// ============================================================================

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <string.h>
#include "pins.h"
#include "presets.h"
#include "waveform.h"
#include "bt_audio.h"

// Set this to false if you don't want to build/wire the Bluetooth audio
// feature at all (skips linking the A2DP library's Bluetooth stack).
#define ENABLE_BT_AUDIO true

TFT_eSPI tft = TFT_eSPI();

// ---------------------------------------------------------------------
// Theme: dark blue / calming, framed highlighted buttons
// ---------------------------------------------------------------------
uint16_t COLOR_BG, COLOR_PANEL, COLOR_PANEL_LIT, COLOR_ACCENT,
         COLOR_HIGHLIGHT, COLOR_GOOD, COLOR_DANGER, COLOR_MUTED, COLOR_TEXT_DIM;

void initTheme() {
  COLOR_BG        = tft.color565(8, 18, 40);    // deep navy
  COLOR_PANEL     = tft.color565(22, 42, 74);   // button fill, unselected
  COLOR_PANEL_LIT = tft.color565(34, 70, 116);  // button fill, active/on
  COLOR_ACCENT    = tft.color565(110, 180, 230);// frame / highlight border
  COLOR_HIGHLIGHT = tft.color565(70, 150, 210); // slider fill
  COLOR_GOOD      = tft.color565(45, 140, 105); // start button
  COLOR_DANGER    = tft.color565(180, 70, 70);  // stop / back button
  COLOR_MUTED     = tft.color565(60, 90, 120);  // inactive/back tone
  COLOR_TEXT_DIM  = tft.color565(150, 180, 210);
}

// ---------------------------------------------------------------------
// UI state
// ---------------------------------------------------------------------
enum Screen { SCR_CATEGORY, SCR_LIST, SCR_RUN, SCR_PIN };
Screen screen = SCR_CATEGORY;

// ---------------------------------------------------------------------
// Duty-cycle safety cap
// ---------------------------------------------------------------------
// Recommended/default intensity is capped at this value. Going above it
// requires the override code below, and any override automatically
// expires and resets back down to the cap after ELEVATED_TIMEOUT_MIN
// minutes - at that point the code must be re-entered to go back above
// the cap. Note the code is stored in plain text in the firmware - it's
// a safety interlock against casual/accidental overshoot, not a
// security mechanism against someone with access to the source or the
// flashed device.
static const int INTENSITY_CAP_DEFAULT = 10;
static const int INTENSITY_CAP_UNLOCKED = 100;
static const unsigned long ELEVATED_TIMEOUT_MIN = 30;
static const char* OVERRIDE_PASSWORD = "5882300";
bool intensityUnlocked = false;
unsigned long elevatedStartMillis = 0;

enum ListSource { LIST_BASE, LIST_HARMONIC };
ListSource listSource = LIST_BASE;

HarmonicEntry harmonics[MAX_HARMONIC_ENTRIES];
int numHarmonics = 0;

int listPage = 0;
const int ITEMS_PER_PAGE = 6;

int selectedIndex = -1;      // index into BASE_PRESETS or harmonics[]
const char* selName = "";
float selFreq = 0;
WaveShape selWave = WAVE_SQUARE;
const char* selClaim = "";

// Adjustable settings, controlled by sliders on the Run screen.
int intensityPercent = INTENSITY_CAP_DEFAULT; // starts at the safety cap
int timerMinutes = 0;        // 0 = continuous (no auto-stop), else 5-60

bool btAudioOn = false;

// Shown on every screen. This app uses wellness-style association tags
// (sleep, calm, energy, etc) the same way commercial PEMF gadgets do -
// they are NOT clinically tied to the specific frequency. Keep this
// banner visible; don't remove it if you customize the UI.
void drawDisclaimerBanner() {
  tft.fillRect(0, 0, 320, 14, COLOR_BG);
  tft.setTextColor(tft.color565(230, 200, 90), COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("For entertainment purposes only - not medical advice", 160, 2, 1);
}

// ---------------------------------------------------------------------
// Widgets: framed button + slider with -/+ buttons
// ---------------------------------------------------------------------
struct Rect { int x, y, w, h; };

bool touchInRect(int tx, int ty, Rect r) {
  return tx >= r.x && tx <= r.x + r.w && ty >= r.y && ty <= r.y + r.h;
}

void drawButton(Rect r, const char* label, uint16_t fillColor = 0xFFFF, bool active = false) {
  if (fillColor == 0xFFFF) fillColor = active ? COLOR_PANEL_LIT : COLOR_PANEL;
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, fillColor);
  // Double-drawn border reads as a deliberate frame rather than a thin outline
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, COLOR_ACCENT);
  tft.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 9, COLOR_ACCENT);
  tft.setTextColor(TFT_WHITE, fillColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2, 2);
}

// Cheaper to draw: flat rect + single-pixel border + smaller font, no
// rounded-corner math or double border pass. Use this anywhere several
// buttons get redrawn together (lists, keypads, slider +/-) so a screen
// refresh doesn't feel sluggish over SPI. Still clearly a "framed" button,
// just without the extra draw calls.
void drawButtonFast(Rect r, const char* label, uint16_t fillColor = 0xFFFF, bool active = false) {
  if (fillColor == 0xFFFF) fillColor = active ? COLOR_PANEL_LIT : COLOR_PANEL;
  tft.fillRect(r.x, r.y, r.w, r.h, fillColor);
  tft.drawRect(r.x, r.y, r.w, r.h, COLOR_ACCENT);
  tft.setTextColor(TFT_WHITE, fillColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2, 1);
}

struct Slider {
  const char* label;
  int x, y, w;          // bar geometry; buttons sit either side of it
  int value, minVal, maxVal, step;
  const char* (*formatFn)(int); // optional custom value formatter, may be null
};

Rect sliderMinusRect(const Slider& s) { return {s.x - 44, s.y, 38, 34}; }
Rect sliderPlusRect(const Slider& s)  { return {s.x + s.w + 6, s.y, 38, 34}; }

void drawSlider(const Slider& s) {
  tft.setTextColor(COLOR_ACCENT, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(s.label, s.x - 44, s.y - 18, 2);

  // Track
  tft.fillRoundRect(s.x, s.y, s.w, 34, 8, COLOR_PANEL);
  tft.drawRoundRect(s.x, s.y, s.w, 34, 8, COLOR_ACCENT);

  // Fill proportional to value
  float frac = (float)(s.value - s.minVal) / (float)(s.maxVal - s.minVal);
  int fillW = (int)(frac * (s.w - 4));
  if (fillW > 0) tft.fillRoundRect(s.x + 2, s.y + 2, fillW, 30, 6, COLOR_HIGHLIGHT);

  // Value label, right-aligned inside the track
  char buf[16];
  if (s.formatFn) {
    tft.drawString(s.formatFn(s.value), s.x + s.w - 8, s.y + 8, 2);
  } else {
    snprintf(buf, sizeof(buf), "%d", s.value);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, s.x + s.w - 8, s.y + 8, 2);
    tft.setTextDatum(TL_DATUM);
  }

  drawButtonFast(sliderMinusRect(s), "-");
  drawButtonFast(sliderPlusRect(s), "+");
}

// Returns true (and mutates s.value) if the touch hit one of this slider's buttons.
bool handleSliderTouch(Slider& s, int x, int y) {
  if (touchInRect(x, y, sliderMinusRect(s))) {
    s.value -= s.step;
    if (s.value < s.minVal) s.value = s.minVal;
    return true;
  }
  if (touchInRect(x, y, sliderPlusRect(s))) {
    s.value += s.step;
    if (s.value > s.maxVal) s.value = s.maxVal;
    return true;
  }
  return false;
}

const char* formatTimerValue(int v) {
  static char buf[16];
  if (v == 0) { snprintf(buf, sizeof(buf), "Off"); }
  else { snprintf(buf, sizeof(buf), "%d min", v); }
  return buf;
}

const char* formatPercentValue(int v) {
  static char buf[16];
  snprintf(buf, sizeof(buf), "%d%%", v);
  return buf;
}

// ---------------------------------------------------------------------
// Screen: numeric PIN entry (used to override the intensity safety cap)
// ---------------------------------------------------------------------
char pinBuf[10] = "";
int pinLen = 0;
bool pinWrongFlash = false;

Rect pinKeyRects[12];
Rect pinCancelBtn = {90, 400, 140, 45};
const char* PIN_KEY_LABELS[12] = {"1","2","3","4","5","6","7","8","9","<","0","OK"};

void drawPinScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Enter override code", 160, 30, 2);
  tft.drawString("to raise intensity above 10%", 160, 48, 1);

  char mask[11] = "";
  for (int i = 0; i < pinLen; i++) mask[i] = '*';
  mask[pinLen] = 0;
  tft.setTextColor(pinWrongFlash ? COLOR_DANGER : TFT_WHITE, COLOR_BG);
  tft.drawString(pinLen > 0 ? mask : "-", 160, 80, 4);
  pinWrongFlash = false;

  int gx = 30, gy = 110, cellW = 80, cellH = 55, gap = 10;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 3; c++) {
      int idx = r * 3 + c;
      Rect rct = {gx + c * (cellW + gap), gy + r * (cellH + gap), cellW, cellH};
      pinKeyRects[idx] = rct;
      drawButtonFast(rct, PIN_KEY_LABELS[idx]);
    }
  }
  drawButton(pinCancelBtn, "Cancel", COLOR_MUTED);
}

void handlePinTouch(int x, int y) {
  for (int i = 0; i < 12; i++) {
    if (touchInRect(x, y, pinKeyRects[i])) {
      if (i == 9) { // backspace "<"
        if (pinLen > 0) pinBuf[--pinLen] = 0;
      } else if (i == 11) { // "OK"
        if (strcmp(pinBuf, OVERRIDE_PASSWORD) == 0) {
          intensityUnlocked = true;
          elevatedStartMillis = millis();
          pinLen = 0; pinBuf[0] = 0;
          screen = SCR_RUN;
          return;
        } else {
          pinLen = 0; pinBuf[0] = 0;
          pinWrongFlash = true;
        }
      } else if (pinLen < 9) {
        pinBuf[pinLen++] = PIN_KEY_LABELS[i][0];
        pinBuf[pinLen] = 0;
      }
      screen = SCR_PIN; // stay, force redraw
      return;
    }
  }
  if (touchInRect(x, y, pinCancelBtn)) {
    pinLen = 0; pinBuf[0] = 0;
    screen = SCR_RUN;
  }
}

// Forces intensity back down to the safety cap and re-locks it.
void resetIntensityToCap() {
  intensityUnlocked = false;
  if (intensityPercent > INTENSITY_CAP_DEFAULT) {
    intensityPercent = INTENSITY_CAP_DEFAULT;
    if (waveform_isRunning()) waveform_setIntensity((uint8_t)intensityPercent);
  }
}

// ---------------------------------------------------------------------
// Screen: category picker
// ---------------------------------------------------------------------
Rect btnPresets  = {30, 150, 260, 60};
Rect btnHarmonic = {30, 230, 260, 60};

void drawCategoryScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PEMF Controller", 160, 65, 4);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Reference frequencies only -", 20, 100, 1);
  tft.drawString("not a verified medical protocol.", 20, 113, 1);
  drawButton(btnPresets, "Base Presets");
  drawButton(btnHarmonic, "Harmonics");
  drawDisclaimerBanner();
}

void handleCategoryTouch(int x, int y) {
  if (touchInRect(x, y, btnPresets)) {
    listSource = LIST_BASE;
    listPage = 0;
    screen = SCR_LIST;
  } else if (touchInRect(x, y, btnHarmonic)) {
    listSource = LIST_HARMONIC;
    listPage = 0;
    screen = SCR_LIST;
  }
}

// ---------------------------------------------------------------------
// Screen: preset list (paged)
// ---------------------------------------------------------------------
Rect itemRects[ITEMS_PER_PAGE];
Rect btnPrevPage = {20, 420, 90, 45};
Rect btnNextPage = {210, 420, 90, 45};
Rect btnBackFromList = {120, 420, 80, 45};

int listCount() {
  return (listSource == LIST_BASE) ? NUM_BASE_PRESETS : numHarmonics;
}

void getListLabel(int idx, char* buf, size_t bufLen) {
  if (listSource == LIST_BASE) {
    snprintf(buf, bufLen, "%s (%.2fHz) - %s", BASE_PRESETS[idx].name,
             BASE_PRESETS[idx].freqHz, BASE_PRESETS[idx].claim);
  } else {
    snprintf(buf, bufLen, "%s x%d (%.1fHz) - %s", harmonics[idx].baseName,
             harmonics[idx].order, harmonics[idx].freqHz, harmonics[idx].claim);
  }
}

void drawListScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(listSource == LIST_BASE ? "Base Presets" : "Harmonics", 20, 18, 4);

  int total = listCount();
  int start = listPage * ITEMS_PER_PAGE;
  char buf[64];
  for (int i = 0; i < ITEMS_PER_PAGE; i++) {
    int idx = start + i;
    Rect r = {20, 60 + i * 55, 280, 45};
    itemRects[i] = r;
    if (idx < total) {
      getListLabel(idx, buf, sizeof(buf));
      drawButtonFast(r, buf);
    }
  }
  drawButton(btnPrevPage, "< Prev");
  drawButton(btnNextPage, "Next >");
  drawButton(btnBackFromList, "Back", COLOR_MUTED);
  drawDisclaimerBanner();
}

void openRunScreenForIndex(int idx) {
  selectedIndex = idx;
  if (listSource == LIST_BASE) {
    selName = BASE_PRESETS[idx].name;
    selFreq = BASE_PRESETS[idx].freqHz;
    selWave = BASE_PRESETS[idx].wave;
    selClaim = BASE_PRESETS[idx].claim;
  } else {
    selName = harmonics[idx].baseName;
    selFreq = harmonics[idx].freqHz;
    selWave = harmonics[idx].wave;
    selClaim = harmonics[idx].claim;
  }
  screen = SCR_RUN;
}

void handleListTouch(int x, int y) {
  int total = listCount();
  int start = listPage * ITEMS_PER_PAGE;
  for (int i = 0; i < ITEMS_PER_PAGE; i++) {
    int idx = start + i;
    if (idx < total && touchInRect(x, y, itemRects[i])) {
      openRunScreenForIndex(idx);
      return;
    }
  }
  if (touchInRect(x, y, btnPrevPage)) {
    if (listPage > 0) listPage--;
    screen = SCR_LIST; // force redraw
  } else if (touchInRect(x, y, btnNextPage)) {
    if ((listPage + 1) * ITEMS_PER_PAGE < total) listPage++;
    screen = SCR_LIST;
  } else if (touchInRect(x, y, btnBackFromList)) {
    screen = SCR_CATEGORY;
  }
}

// ---------------------------------------------------------------------
// Screen: running / detail view
// ---------------------------------------------------------------------
Rect btnStartStop   = {80, 150, 240, 55};
Rect btnBtAudio     = {80, 345, 240, 45};
Rect btnBackFromRun = {80, 395, 240, 40};

Slider intensitySlider = {"Intensity", 90, 245, 150, 100, 0, 100, 5, formatPercentValue};
Slider timerSlider     = {"Session timer", 90, 300, 150, 0, 0, 60, 5, formatTimerValue};

// Session auto-stop bookkeeping
unsigned long sessionStartMillis = 0;
bool sessionTimerArmed = false;

void drawRunScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(selName, 20, 20, 4);

  char buf[48];
  snprintf(buf, sizeof(buf), "%.2f Hz", selFreq);
  tft.drawString(buf, 20, 60, 4);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString(selWave == WAVE_SQUARE ? "Waveform: Square" : "Waveform: Sine", 20, 98, 2);

  tft.setTextColor(tft.color565(220, 170, 90), COLOR_BG);
  snprintf(buf, sizeof(buf), "Tag: %s (entertainment only)", selClaim);
  tft.drawString(buf, 20, 122, 2);

  if (intensityUnlocked) {
    unsigned long remainMin = ELEVATED_TIMEOUT_MIN -
      ((millis() - elevatedStartMillis) / 60000UL);
    char statusBuf[48];
    snprintf(statusBuf, sizeof(statusBuf), "Unlocked - resets to 10%% in %lu min", remainMin);
    tft.setTextColor(tft.color565(220, 170, 90), COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(statusBuf, 20, 140, 1);
  } else {
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Capped at 10% - code required to raise", 20, 140, 1);
  }

  bool running = waveform_isRunning();
  drawButton(btnStartStop, running ? "STOP" : "START", running ? COLOR_DANGER : COLOR_GOOD);

  intensitySlider.value = intensityPercent;
  intensitySlider.maxVal = intensityUnlocked ? INTENSITY_CAP_UNLOCKED : INTENSITY_CAP_DEFAULT;
  timerSlider.value = timerMinutes;
  drawSlider(intensitySlider);
  drawSlider(timerSlider);

  drawButton(btnBtAudio, btAudioOn ? "BT Audio: ON" : "BT Audio: OFF", 0xFFFF, btAudioOn);
  drawButton(btnBackFromRun, "Stop & Back", COLOR_MUTED);
  drawDisclaimerBanner();
}

void handleRunTouch(int x, int y) {
  if (touchInRect(x, y, sliderPlusRect(intensitySlider)) &&
      !intensityUnlocked && intensityPercent >= INTENSITY_CAP_DEFAULT) {
    screen = SCR_PIN;
    return;
  }
  if (handleSliderTouch(intensitySlider, x, y)) {
    intensityPercent = intensitySlider.value;
    waveform_setIntensity((uint8_t)intensityPercent);
    screen = SCR_RUN;
    return;
  }
  if (handleSliderTouch(timerSlider, x, y)) {
    timerMinutes = timerSlider.value;
    if (waveform_isRunning()) {
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    screen = SCR_RUN;
    return;
  }
  if (touchInRect(x, y, btnStartStop)) {
    if (waveform_isRunning()) {
      waveform_stop();
      sessionTimerArmed = false;
    } else {
      waveform_start(selFreq, selWave, (uint8_t)intensityPercent);
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    screen = SCR_RUN;
  } else if (touchInRect(x, y, btnBtAudio)) {
#if ENABLE_BT_AUDIO
    btAudioOn = !btAudioOn;
    btaudio_setTargetFrequency(selFreq);
    btaudio_setEnabled(btAudioOn);
#endif
    screen = SCR_RUN;
  } else if (touchInRect(x, y, btnBackFromRun)) {
    waveform_stop();
    sessionTimerArmed = false;
#if ENABLE_BT_AUDIO
    btAudioOn = false;
    btaudio_setEnabled(false);
#endif
    screen = SCR_LIST;
  }
}

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------
Screen lastDrawnScreen = SCR_CATEGORY;
int lastDrawnPage = -1;

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // backlight on

  tft.init();
  tft.setRotation(0); // portrait; change to 2 if your image is upside down
  initTheme();

  // ---- Touch calibration (one-time, then reused every boot) ----
  // A resistive touch panel's raw readings don't map directly to screen
  // pixels - this runs an interactive "tap the corners" calibration the
  // very first time, then saves the result to flash (NVS) so you never
  // have to do it again. To force recalibration later (e.g. if the touch
  // feels off), hold the BOOT button while powering on... actually simpler:
  // just erase flash once via Arduino IDE's "Tools > Erase All Flash
  // Before Sketch Upload" the next time you re-upload.
  uint16_t calData[5];
  Preferences prefs;
  prefs.begin("tftcal", false);
  if (prefs.isKey("calData")) {
    prefs.getBytes("calData", calData, sizeof(calData));
    tft.setTouch(calData);
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Touch each corner as it appears", 160, 220, 2);
    tft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 15);
    prefs.putBytes("calData", calData, sizeof(calData));
  }
  prefs.end();

  waveform_begin();

#if ENABLE_BT_AUDIO
  btaudio_begin("PEMF-Headset");
#endif

  numHarmonics = buildHarmonicList(harmonics, MAX_HARMONIC_ENTRIES);

  drawCategoryScreen();
}

void loop() {
  uint16_t tx, ty;
  bool touched = tft.getTouch(&tx, &ty);

  // Auto-stop when the session timer elapses
  if (sessionTimerArmed && waveform_isRunning()) {
    unsigned long elapsedMin = (millis() - sessionStartMillis) / 60000UL;
    if ((int)elapsedMin >= timerMinutes) {
      waveform_stop();
      sessionTimerArmed = false;
      if (screen == SCR_RUN) drawRunScreen();
    }
  }

  // Auto-reset the intensity override after ELEVATED_TIMEOUT_MIN minutes
  if (intensityUnlocked) {
    unsigned long elapsedMin = (millis() - elevatedStartMillis) / 60000UL;
    if (elapsedMin >= ELEVATED_TIMEOUT_MIN) {
      resetIntensityToCap();
      if (screen == SCR_RUN) drawRunScreen();
    }
  }

  // Redraw when the screen or page changes
  if (screen != lastDrawnScreen || (screen == SCR_LIST && listPage != lastDrawnPage)) {
    switch (screen) {
      case SCR_CATEGORY: drawCategoryScreen(); break;
      case SCR_LIST:     drawListScreen(); break;
      case SCR_RUN:      drawRunScreen(); break;
      case SCR_PIN:      drawPinScreen(); break;
    }
    lastDrawnScreen = screen;
    lastDrawnPage = listPage;
  }

  static bool wasTouched = false;
  if (touched && !wasTouched) {
    switch (screen) {
      case SCR_CATEGORY: handleCategoryTouch(tx, ty); break;
      case SCR_LIST:     handleListTouch(tx, ty); break;
      case SCR_RUN:      handleRunTouch(tx, ty); break;
      case SCR_PIN:      handlePinTouch(tx, ty); break;
    }
    // Force a redraw on the next loop pass since state may have changed
    // even when the Screen enum value itself didn't (e.g. Start/Stop,
    // slider taps, PIN digit taps).
    if (screen == SCR_RUN) drawRunScreen();
    if (screen == SCR_PIN) drawPinScreen();
  }
  wasTouched = touched;

  delay(20); // simple debounce; touch is polled, not interrupt-driven
}

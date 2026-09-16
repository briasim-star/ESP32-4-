// ============================================================================
// PEMF_Controller.ino  -  "MADD PEMF"
// Touchscreen-driven PEMF signal generator for the LCDWiki 4.0" ESP32-32E
// display, driving a Cytron MD10C/MD30C motor driver into a coil load, with
// optional Bluetooth audio monitoring (synced modulated tone).
//
// Libraries required (install via Arduino IDE Library Manager):
//   - TFT_eSPI (Bodmer)          -> display + touch driving
//   - ESP32-A2DP (pschatzmann)   -> optional Bluetooth audio streaming
// Board core required: "ESP32 by Espressif Systems" v2.0.x
//   (see README for why the version matters)
//
// HARDWARE TIER: this codebase is shared across both hardware tiers (entry
// unit on MD10C/19V/10A, and the Pro unit on MD30C/24V/15A). The only
// difference is this display label - the drive logic and safety math are
// identical, since both motor drivers take the same PWM+DIR control signals.
// Change HW_TIER_NAME below to match whichever board you're building.
//
// See README.md for full wiring, library configuration, and setup steps.
// ============================================================================

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
// Bold GFXFF fonts (FreeSansBold9pt7b/12pt7b/18pt7b) are already made
// available globally by TFT_eSPI.h itself when LOAD_GFXFF is set in
// TFT_eSPI_User_Setup.h (it auto-includes every GFXFF font internally) -
// including them again here causes "redefinition" errors, so don't.
#include <qrcode.h>
#include "pins.h"
#include "presets.h"
#include "waveform.h"
#include "bt_audio.h"
#include "ui_types.h"

// Set this to false if you don't want to build/wire the Bluetooth audio
// feature at all (skips linking the A2DP library's Bluetooth stack).
#define ENABLE_BT_AUDIO true

// Change this one line per hardware tier when building for the other unit.
static const char* HW_TIER_NAME = "MADD PEMF - Entry (MD10C)";
// static const char* HW_TIER_NAME = "MADD PEMF - Pro (MD30C)";

static const char* FIRMWARE_VERSION = "1.0.0";
static const char* UPDATE_URL = "https://briasim-star.github.io/ESP32-4-/install.html";

TFT_eSPI tft = TFT_eSPI();

// ---------------------------------------------------------------------
// Fonts: bold throughout for visibility. Classic GFX bitmap fonts
// (compiled into flash, no SPIFFS/font-file upload needed).
// ---------------------------------------------------------------------
const GFXfont* FONT_SM = &FreeSansBold9pt7b;   // body text, fast/list buttons
const GFXfont* FONT_LG = &FreeSansBold12pt7b;  // primary buttons
const GFXfont* FONT_XL = &FreeSansBold18pt7b;  // screen titles, headline values

// ---------------------------------------------------------------------
// Theme: dark, high-contrast, single accent color per meaning -
// blue = primary action, amber = warning/caution only, red = stop/destructive only.
// ---------------------------------------------------------------------
uint16_t COLOR_BG, COLOR_PANEL, COLOR_PANEL_LIT, COLOR_ACCENT,
         COLOR_GOOD, COLOR_DANGER, COLOR_WARN, COLOR_MUTED, COLOR_TEXT_DIM;

void initTheme() {
  COLOR_BG        = tft.color565(6, 8, 14);     // near-black
  COLOR_PANEL     = tft.color565(20, 24, 34);   // button fill, unselected
  COLOR_PANEL_LIT = tft.color565(30, 70, 120);  // button fill, active/on
  COLOR_ACCENT    = tft.color565(70, 160, 255); // primary action blue - used ONLY for primary actions
  COLOR_GOOD      = tft.color565(40, 170, 120); // start button
  COLOR_DANGER    = tft.color565(210, 60, 60);  // stop / destructive - used ONLY for destructive actions
  COLOR_WARN      = tft.color565(230, 175, 60); // warning/caution - used ONLY for warnings
  COLOR_MUTED     = tft.color565(50, 56, 68);   // inactive/back tone
  COLOR_TEXT_DIM  = tft.color565(160, 170, 185);
}

// ---------------------------------------------------------------------
// UI state
// ---------------------------------------------------------------------
// Screen and PinPurpose enums themselves are declared in ui_types.h - see
// the comment there for why. Just the variables live here.
Screen screen = SCR_WELCOME;
PinPurpose pinPurpose = PIN_DEV_MODE;

// ---------------------------------------------------------------------
// Power safety ceiling
// ---------------------------------------------------------------------
// The Power control always shows 1-100% to the user. Underneath, actual
// coil output is rescaled onto 0-45% (a hard ceiling) UNLESS Developer
// Mode has been unlocked from Settings, in which case the displayed
// number maps 1:1 to actual output. Developer Mode auto-relocks after
// DEV_MODE_TIMEOUT_MIN minutes, and always relocks on power-off since
// it is never written to flash - only kept in RAM.
static const float LOCKED_CEILING_FRAC = 0.45f;
static const unsigned long DEV_MODE_TIMEOUT_MIN = 60;
static const char* DEV_MODE_PASSWORD = "5882300";
bool devModeUnlocked = false;
unsigned long devModeStartMillis = 0;

Category currentCategory = CAT_BONE_JOINT;
int catIndices[NUM_BASE_PRESETS]; // indices into BASE_PRESETS for the current category
int catCount = 0;

int listPage = 0;
const int ITEMS_PER_PAGE = 6;

int selectedIndex = -1;      // index into BASE_PRESETS
const char* selName = "";
float selFreq = 0;
WaveShape selWave = WAVE_SQUARE;
const char* selCategoryName = "";

// Adjustable settings, controlled by steppers on the Run screen.
int powerDisplay = 10;       // what the user sees, 1-100 - NOT the actual output
int timerMinutes = 0;        // 0 = continuous (no auto-stop), else 5-60
int volumePercent = 60;      // BT audio volume, independent of coil output

bool btAudioOn = false;

// Returns the real coil intensity percent, applying the safety rescale
// unless Developer Mode is currently unlocked.
uint8_t actualIntensityPercent() {
  if (devModeUnlocked) return (uint8_t)powerDisplay;
  return (uint8_t)round(powerDisplay * LOCKED_CEILING_FRAC);
}

// ---------------------------------------------------------------------
// Favorites - persisted across power cycles via NVS (Preferences)
// ---------------------------------------------------------------------
bool favoriteBits[NUM_BASE_PRESETS];

void loadFavorites() {
  Preferences p;
  p.begin("favs", true);
  for (int i = 0; i < NUM_BASE_PRESETS; i++) {
    char key[8];
    snprintf(key, sizeof(key), "f%d", i);
    favoriteBits[i] = p.getBool(key, false);
  }
  p.end();
}

void setFavorite(int idx, bool val) {
  favoriteBits[idx] = val;
  Preferences p;
  p.begin("favs", false);
  char key[8];
  snprintf(key, sizeof(key), "f%d", idx);
  p.putBool(key, val);
  p.end();
}

// ---------------------------------------------------------------------
// Session log - rolling last 5 sessions, persisted via NVS. Deliberately
// small (5 entries) to keep flash writes and RAM use light.
// ---------------------------------------------------------------------
static const int LOG_SIZE = 5;
struct LogEntry {
  char name[24];
  float freqHz;
  int durationMin;
};
LogEntry sessionLog[LOG_SIZE];
int logCount = 0; // how many of the 5 slots are actually filled

void loadSessionLog() {
  Preferences p;
  p.begin("log", true);
  logCount = p.getInt("count", 0);
  if (logCount > LOG_SIZE) logCount = LOG_SIZE;
  for (int i = 0; i < logCount; i++) {
    char nameKey[8], freqKey[8], durKey[8];
    snprintf(nameKey, sizeof(nameKey), "n%d", i);
    snprintf(freqKey, sizeof(freqKey), "q%d", i);
    snprintf(durKey, sizeof(durKey), "d%d", i);
    String n = p.getString(nameKey, "");
    strncpy(sessionLog[i].name, n.c_str(), sizeof(sessionLog[i].name) - 1);
    sessionLog[i].name[sizeof(sessionLog[i].name) - 1] = 0;
    sessionLog[i].freqHz = p.getFloat(freqKey, 0);
    sessionLog[i].durationMin = p.getInt(durKey, 0);
  }
  p.end();
}

void saveSessionLog() {
  Preferences p;
  p.begin("log", false);
  p.putInt("count", logCount);
  for (int i = 0; i < logCount; i++) {
    char nameKey[8], freqKey[8], durKey[8];
    snprintf(nameKey, sizeof(nameKey), "n%d", i);
    snprintf(freqKey, sizeof(freqKey), "q%d", i);
    snprintf(durKey, sizeof(durKey), "d%d", i);
    p.putString(nameKey, sessionLog[i].name);
    p.putFloat(freqKey, sessionLog[i].freqHz);
    p.putInt(durKey, sessionLog[i].durationMin);
  }
  p.end();
}

// Pushes a new entry to the front, dropping the oldest once full.
void addLogEntry(const char* name, float freqHz, int durationMin) {
  if (durationMin < 1) return; // skip near-instant taps, not real sessions
  int n = (logCount < LOG_SIZE) ? logCount + 1 : LOG_SIZE;
  for (int i = n - 1; i > 0; i--) sessionLog[i] = sessionLog[i - 1];
  strncpy(sessionLog[0].name, name, sizeof(sessionLog[0].name) - 1);
  sessionLog[0].name[sizeof(sessionLog[0].name) - 1] = 0;
  sessionLog[0].freqHz = freqHz;
  sessionLog[0].durationMin = durationMin;
  logCount = n;
  saveSessionLog();
}

// ---------------------------------------------------------------------
// First-boot setup: use type (Home / Wellness Center) + owner name.
// Runs once, ever, right after the wellness acknowledgment. Wellness
// Center mode unlocks a couple of extra settings a home user doesn't need
// (a per-client safety re-check before each session, a room label).
// ---------------------------------------------------------------------
bool setupDone = false;
bool isWellnessCenter = false;
char ownerName[24] = "";
char roomLabel[24] = "";
bool requireLoginPassword = false;
char loginPassword[10] = "";
char btDeviceName[32] = ""; // last-selected BT speaker/headphone, empty until scanned

void loadSetupInfo() {
  Preferences p;
  p.begin("setup", true);
  setupDone = p.getBool("done", false);
  isWellnessCenter = p.getBool("wellness", false);
  requireLoginPassword = p.getBool("reqLogin", false);
  String n = p.getString("name", "");
  strncpy(ownerName, n.c_str(), sizeof(ownerName) - 1);
  ownerName[sizeof(ownerName) - 1] = 0;
  String r = p.getString("room", "");
  strncpy(roomLabel, r.c_str(), sizeof(roomLabel) - 1);
  roomLabel[sizeof(roomLabel) - 1] = 0;
  String lp = p.getString("loginPw", "");
  strncpy(loginPassword, lp.c_str(), sizeof(loginPassword) - 1);
  loginPassword[sizeof(loginPassword) - 1] = 0;
  String bt = p.getString("btDevice", "");
  strncpy(btDeviceName, bt.c_str(), sizeof(btDeviceName) - 1);
  btDeviceName[sizeof(btDeviceName) - 1] = 0;
  p.end();
}

void saveSetupInfo() {
  Preferences p;
  p.begin("setup", false);
  p.putBool("done", setupDone);
  p.putBool("wellness", isWellnessCenter);
  p.putBool("reqLogin", requireLoginPassword);
  p.putString("name", ownerName);
  p.putString("room", roomLabel);
  p.putString("loginPw", loginPassword);
  p.putString("btDevice", btDeviceName);
  p.end();
}

// ---------------------------------------------------------------------
// Widgets: framed button + stepper (no sliders anywhere)
// ---------------------------------------------------------------------
bool touchInRect(int tx, int ty, Rect r) {
  return tx >= r.x && tx <= r.x + r.w && ty >= r.y && ty <= r.y + r.h;
}

// Draws label text centered in a button, shrinking or truncating with ".."
// as needed so it never overflows past the button's edges.
void drawFittedLabel(Rect r, const char* label, const GFXfont* font, uint16_t fillColor) {
  tft.setFreeFont(font);
  tft.setTextColor(TFT_WHITE, fillColor);
  tft.setTextDatum(MC_DATUM);
  int maxW = r.w - 12; // horizontal padding so text clears the border

  if (tft.textWidth(label) <= maxW) {
    tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
    return;
  }

  char buf[64];
  strncpy(buf, label, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  int len = strlen(buf);
  while (len > 1) {
    buf[len - 1] = 0;
    char tryBuf[64];
    snprintf(tryBuf, sizeof(tryBuf), "%s..", buf);
    if (tft.textWidth(tryBuf) <= maxW) {
      tft.drawString(tryBuf, r.x + r.w / 2, r.y + r.h / 2);
      return;
    }
    len--;
  }
  tft.drawString(buf, r.x + r.w / 2, r.y + r.h / 2); // last-resort fallback
}

void drawButton(Rect r, const char* label, uint16_t fillColor = 0xFFFF, bool active = false) {
  if (fillColor == 0xFFFF) fillColor = active ? COLOR_PANEL_LIT : COLOR_PANEL;
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, fillColor);
  // Double-drawn border reads as a deliberate frame rather than a thin outline
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, COLOR_ACCENT);
  tft.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 9, COLOR_ACCENT);
  drawFittedLabel(r, label, FONT_LG, fillColor);
}

// Cheaper to draw: flat rect + single-pixel border, no rounded-corner math
// or double border pass. Use this anywhere several buttons get redrawn
// together (lists, keypads, category grid) so a screen refresh doesn't
// feel sluggish over SPI. Still clearly a "framed" button.
void drawButtonFast(Rect r, const char* label, uint16_t fillColor = 0xFFFF, bool active = false) {
  if (fillColor == 0xFFFF) fillColor = active ? COLOR_PANEL_LIT : COLOR_PANEL;
  tft.fillRect(r.x, r.y, r.w, r.h, fillColor);
  tft.drawRect(r.x, r.y, r.w, r.h, COLOR_ACCENT);
  drawFittedLabel(r, label, FONT_SM, fillColor);
}

Rect btnHome = {380, 8, 80, 32};

// Small, consistently-placed Home button, top-right corner of every
// non-home screen. Always returns to the Category screen.
// Truncates left-aligned text with ".." if it would exceed maxW - the
// same idea as drawFittedLabel, but for plain (non-button) text where
// there's no button background/fill to match.
void drawFittedText(int x, int y, int maxW, const char* text, const GFXfont* font, uint16_t color, uint16_t bg) {
  tft.setFreeFont(font);
  tft.setTextColor(color, bg);
  tft.setTextDatum(TL_DATUM);
  if (tft.textWidth(text) <= maxW) {
    tft.drawString(text, x, y);
    return;
  }
  char buf[40];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  int len = strlen(buf);
  while (len > 1) {
    buf[len - 1] = 0;
    char tryBuf[40];
    snprintf(tryBuf, sizeof(tryBuf), "%s..", buf);
    if (tft.textWidth(tryBuf) <= maxW) {
      tft.drawString(tryBuf, x, y);
      return;
    }
    len--;
  }
  tft.drawString(buf, x, y);
}
void drawHomeButton() {
  drawButtonFast(btnHome, "Home", COLOR_MUTED);
}
bool handleHomeTouch(int x, int y) {
  if (touchInRect(x, y, btnHome)) {
    screen = SCR_CATEGORY;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------
// Stepper: "- value +" control, no track/bar.
// ---------------------------------------------------------------------
Rect stepperMinusRect(const Stepper& s) { return {s.x, s.y, 46, 40}; }
Rect stepperValueRect(const Stepper& s) { return {s.x + 52, s.y, 100, 40}; }
Rect stepperPlusRect(const Stepper& s)  { return {s.x + 158, s.y, 46, 40}; }

void drawStepper(const Stepper& s) {
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(s.label, s.x, s.y - 22);

  drawButton(stepperMinusRect(s), "-");
  drawButton(stepperPlusRect(s), "+");

  Rect vr = stepperValueRect(s);
  tft.fillRect(vr.x, vr.y, vr.w, vr.h, COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(MC_DATUM);

  char buf[16];
  const char* text;
  if (s.formatFn) {
    text = s.formatFn(s.value);
  } else {
    snprintf(buf, sizeof(buf), "%d", s.value);
    text = buf;
  }
  tft.drawString(text, vr.x + vr.w / 2, vr.y + vr.h / 2);
}

// Returns true (and mutates s.value) if the touch hit one of this stepper's buttons.
bool handleStepperTouch(Stepper& s, int x, int y) {
  if (touchInRect(x, y, stepperMinusRect(s))) {
    s.value -= s.step;
    if (s.value < s.minVal) s.value = s.minVal;
    return true;
  }
  if (touchInRect(x, y, stepperPlusRect(s))) {
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
// Screen: welcome / wellness acknowledgment (mandatory, shown on every boot)
// ---------------------------------------------------------------------
bool wellnessAck = false;
Rect checkboxRect = {24, 180, 26, 26};
Rect btnContinue  = {24, 226, 200, 46};

void drawWelcomeScreen() {
  tft.fillScreen(COLOR_BG);

  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("MADD PEMF", 20, 6);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_WARN, COLOR_BG);
  tft.drawString("Wellness device - not a medical device", 20, 30);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  const char* lines[] = {
    "Not intended to diagnose, treat, cure,",
    "or prevent any disease.",
    "",
    "Do NOT use if you have a pacemaker,",
    "insulin pump, or other implanted",
    "electronic device, or if pregnant."
  };
  int y = 54;
  for (int i = 0; i < 6; i++) {
    tft.drawString(lines[i], 20, y);
    y += 19;
  }

  // Checkbox
  tft.drawRect(checkboxRect.x, checkboxRect.y, checkboxRect.w, checkboxRect.h, COLOR_ACCENT);
  if (wellnessAck) {
    tft.fillRect(checkboxRect.x + 4, checkboxRect.y + 4, checkboxRect.w - 8, checkboxRect.h - 8, COLOR_ACCENT);
  }
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("I understand and agree", checkboxRect.x + checkboxRect.w + 10, checkboxRect.y + 4);

  drawButton(btnContinue, "Continue", wellnessAck ? COLOR_GOOD : COLOR_MUTED, wellnessAck);
}

void handleWelcomeTouch(int x, int y) {
  if (touchInRect(x, y, checkboxRect)) {
    wellnessAck = !wellnessAck;
    screen = SCR_WELCOME; // redraw
    return;
  }
  if (touchInRect(x, y, btnContinue) && wellnessAck) {
    if (!setupDone) {
      screen = SCR_SETUP_MODE;
    } else if (requireLoginPassword) {
      startPinEntry(PIN_CHECK_LOGIN);
    } else {
      screen = SCR_CATEGORY;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: first-boot setup - use type
// ---------------------------------------------------------------------
Rect btnHomeUse        = {20, 100, 220, 100};
Rect btnWellnessCenter = {260, 100, 200, 100};

void drawSetupModeScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Quick Setup", 20, 12);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("How will this device be used?", 20, 44);
  tft.drawString("(This only shows once.)", 20, 64);

  drawButton(btnHomeUse, "Home Use");
  drawButton(btnWellnessCenter, "Wellness Center");
}

void handleSetupModeTouch(int x, int y) {
  if (touchInRect(x, y, btnHomeUse)) {
    isWellnessCenter = false;
    startTextEntry(ownerName, "What's your name?", SCR_CATEGORY, true);
  } else if (touchInRect(x, y, btnWellnessCenter)) {
    isWellnessCenter = true;
    startTextEntry(ownerName, "What's your name?", SCR_CATEGORY, true);
  }
}

// ---------------------------------------------------------------------
// Screen: setup - require a login password? (Wellness Center only)
// ---------------------------------------------------------------------
Rect btnLoginYes = {20, 100, 220, 100};
Rect btnLoginNo  = {260, 100, 200, 100};

void drawSetupLoginChoiceScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("One More Thing", 20, 12);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Require a staff PIN each time this", 20, 44);
  tft.drawString("device is turned on?", 20, 64);

  drawButton(btnLoginYes, "Yes, require a PIN");
  drawButton(btnLoginNo, "No, skip login");
}

void handleSetupLoginChoiceTouch(int x, int y) {
  if (touchInRect(x, y, btnLoginYes)) {
    requireLoginPassword = true;
    startPinEntry(PIN_SET_LOGIN);
  } else if (touchInRect(x, y, btnLoginNo)) {
    requireLoginPassword = false;
    setupDone = true;
    saveSetupInfo();
    screen = SCR_CATEGORY;
  }
}

// ---------------------------------------------------------------------
// Screen: generic text entry with an on-screen keyboard. Reused for the
// setup-time name entry and, for Wellness Center units, the room label
// in Settings.
// ---------------------------------------------------------------------
char* textEntryTarget = nullptr;
size_t textEntryTargetSize = 0;
char textEntryPrompt[32] = "";
Screen textEntryReturnScreen = SCR_CATEGORY;
bool textEntryFinishesSetup = false;
char textEntryBuf[24] = "";
int textEntryLen = 0;

void startTextEntry(char* target, const char* prompt, Screen returnTo, bool finishesSetup) {
  textEntryTarget = target;
  textEntryTargetSize = 24;
  strncpy(textEntryPrompt, prompt, sizeof(textEntryPrompt) - 1);
  textEntryPrompt[sizeof(textEntryPrompt) - 1] = 0;
  textEntryReturnScreen = returnTo;
  textEntryFinishesSetup = finishesSetup;
  strncpy(textEntryBuf, target, sizeof(textEntryBuf) - 1);
  textEntryBuf[sizeof(textEntryBuf) - 1] = 0;
  textEntryLen = strlen(textEntryBuf);
  screen = SCR_TEXT_ENTRY;
}

// Keyboard layout: three letter rows, then a bottom row with Space,
// Backspace, and OK. Single-case (easier on a small touchscreen); the
// display capitalizes the first letter for a nicer look regardless of
// how it was typed.
const char* KB_ROW1 = "QWERTYUIOP";
const char* KB_ROW2 = "ASDFGHJKL";
const char* KB_ROW3 = "ZXCVBNM";
const char* KB_ROW4 = "0123456789";
Rect kbKeyRects[40];
int kbKeyCount = 0;
char kbKeyChars[40];
Rect btnKbSpace, btnKbBackspace, btnKbOk;

void layoutKeyboard() {
  kbKeyCount = 0;
  int y = 62, rowH = 32, gap = 3;

  const char* rows[4] = { KB_ROW1, KB_ROW2, KB_ROW3, KB_ROW4 };
  for (int r = 0; r < 4; r++) {
    int n = strlen(rows[r]);
    int w = (460 - (n - 1) * gap) / n;
    for (int i = 0; i < n; i++) {
      kbKeyRects[kbKeyCount] = {10 + i * (w + gap), y, w, rowH};
      kbKeyChars[kbKeyCount] = rows[r][i];
      kbKeyCount++;
    }
    y += rowH + gap;
  }

  y += 4;
  btnKbSpace     = {10, y, 260, 38};
  btnKbBackspace = {276, y, 90, 38};
  btnKbOk        = {372, y, 98, 38};
}

// Renders the buffer with the first letter capitalized, rest lowercase,
// purely for a nicer display - the stored/typed value is unaffected.
void formatDisplayName(const char* raw, char* out, size_t outSize) {
  size_t len = strlen(raw);
  if (len == 0) { out[0] = 0; return; }
  size_t n = (len < outSize - 1) ? len : outSize - 1;
  out[0] = toupper(raw[0]);
  for (size_t i = 1; i < n; i++) out[i] = tolower(raw[i]);
  out[n] = 0;
}

void drawTextEntryScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(textEntryPrompt, 20, 4);

  char shown[24];
  formatDisplayName(textEntryBuf, shown, sizeof(shown));
  tft.fillRect(20, 26, 440, 30, COLOR_PANEL);
  tft.drawRect(20, 26, 440, 30, COLOR_ACCENT);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_PANEL);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(textEntryLen > 0 ? shown : "-", 30, 41);

  layoutKeyboard();
  for (int i = 0; i < kbKeyCount; i++) {
    char label[2] = { kbKeyChars[i], 0 };
    drawButtonFast(kbKeyRects[i], label);
  }
  drawButtonFast(btnKbSpace, "Space", COLOR_MUTED);
  drawButtonFast(btnKbBackspace, "<-", COLOR_MUTED);
  drawButton(btnKbOk, "OK", COLOR_GOOD);
}

void handleTextEntryTouch(int x, int y) {
  layoutKeyboard(); // rects are cheap to recompute; keeps this self-contained
  for (int i = 0; i < kbKeyCount; i++) {
    if (touchInRect(x, y, kbKeyRects[i]) && textEntryLen < 22) {
      textEntryBuf[textEntryLen++] = kbKeyChars[i];
      textEntryBuf[textEntryLen] = 0;
      screen = SCR_TEXT_ENTRY;
      return;
    }
  }
  if (touchInRect(x, y, btnKbSpace) && textEntryLen < 22) {
    textEntryBuf[textEntryLen++] = ' ';
    textEntryBuf[textEntryLen] = 0;
    screen = SCR_TEXT_ENTRY;
    return;
  }
  if (touchInRect(x, y, btnKbBackspace)) {
    if (textEntryLen > 0) textEntryBuf[--textEntryLen] = 0;
    screen = SCR_TEXT_ENTRY;
    return;
  }
  if (touchInRect(x, y, btnKbOk)) {
    if (textEntryTarget) {
      strncpy(textEntryTarget, textEntryBuf, textEntryTargetSize - 1);
      textEntryTarget[textEntryTargetSize - 1] = 0;
    }
    if (textEntryFinishesSetup && isWellnessCenter) {
      // Don't finish setup yet - ask about the login PIN first.
      saveSetupInfo();
      screen = SCR_SETUP_LOGIN_CHOICE;
      return;
    }
    if (textEntryFinishesSetup) setupDone = true;
    saveSetupInfo();
    screen = textEntryReturnScreen;
  }
}

// ---------------------------------------------------------------------
// Screen: per-client safety re-check (Wellness Center mode only) - shown
// each time a new preset is opened, since the person in front of the
// device changes far more often than the device reboots.
// ---------------------------------------------------------------------
bool clientConfirmChecked = false;
Rect clientCheckboxRect = {24, 190, 30, 30};
Rect btnClientContinue = {60, 260, 200, 48};

void drawClientConfirmScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Client Safety Check", 20, 24);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_WARN, COLOR_BG);
  const char* lines[] = {
    "Confirm this client has no pacemaker,",
    "insulin pump, or other implanted",
    "device, and is not pregnant."
  };
  int y = 70;
  for (int i = 0; i < 3; i++) { tft.drawString(lines[i], 20, y); y += 22; }

  tft.drawRect(clientCheckboxRect.x, clientCheckboxRect.y, clientCheckboxRect.w, clientCheckboxRect.h, COLOR_ACCENT);
  if (clientConfirmChecked) {
    tft.fillRect(clientCheckboxRect.x + 5, clientCheckboxRect.y + 5,
                 clientCheckboxRect.w - 10, clientCheckboxRect.h - 10, COLOR_ACCENT);
  }
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Confirmed", clientCheckboxRect.x + clientCheckboxRect.w + 12, clientCheckboxRect.y + 6);

  drawButton(btnClientContinue, "Continue", clientConfirmChecked ? COLOR_GOOD : COLOR_MUTED, clientConfirmChecked);
}

void handleClientConfirmTouch(int x, int y) {
  if (touchInRect(x, y, clientCheckboxRect)) {
    clientConfirmChecked = !clientConfirmChecked;
    screen = SCR_CLIENT_CONFIRM;
    return;
  }
  if (touchInRect(x, y, btnClientContinue) && clientConfirmChecked) {
    clientConfirmChecked = false; // reset for the next client
    screen = SCR_RUN;
  }
}

// ---------------------------------------------------------------------
// Screen: numeric PIN entry (used only for Developer Mode unlock, via Settings)
// ---------------------------------------------------------------------
char pinBuf[10] = "";
int pinLen = 0;
bool pinWrongFlash = false;

// The PIN screen is reused for three purposes: unlocking Developer Mode,
// setting the Wellness Center login password during setup, and checking
// that login password on every subsequent power-on.
// PinPurpose and pinPurpose are declared up near the Screen enum, for the
// same forward-declaration reason described there.

Rect pinKeyRects[12];
Rect pinCancelBtn = {20, 200, 180, 46};
const char* PIN_KEY_LABELS[12] = {"1","2","3","4","5","6","7","8","9","<","0","OK"};

void startPinEntry(PinPurpose purpose) {
  pinPurpose = purpose;
  pinLen = 0; pinBuf[0] = 0;
  screen = SCR_PIN;
}

// Left column: title/subtitle/PIN display + Cancel. Right column: the
// 12-key keypad. Split this way since landscape doesn't have the vertical
// room for the keypad to sit below a lot of text the way portrait did.
void drawPinScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  const char* title = (pinPurpose == PIN_DEV_MODE) ? "Developer Mode PIN" :
                      (pinPurpose == PIN_SET_LOGIN) ? "Set Login Password" :
                                                       "Enter Login Password";
  tft.drawString(title, 20, 16);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  const char* subtitle = (pinPurpose == PIN_DEV_MODE) ? "Required to exceed the 45% ceiling" :
                         (pinPurpose == PIN_SET_LOGIN) ? "Choose a PIN staff will use to log in" :
                                                          "Staff PIN required to use this device";
  tft.drawString(subtitle, 20, 44);

  char mask[11] = "";
  for (int i = 0; i < pinLen; i++) mask[i] = '*';
  mask[pinLen] = 0;
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(pinWrongFlash ? COLOR_DANGER : TFT_WHITE, COLOR_BG);
  tft.drawString(pinLen > 0 ? mask : "-", 20, 78);
  pinWrongFlash = false;

  int gx = 220, gy = 20, cellW = 70, cellH = 44, gap = 8;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 3; c++) {
      int idx = r * 3 + c;
      Rect rct = {gx + c * (cellW + gap), gy + r * (cellH + gap), cellW, cellH};
      pinKeyRects[idx] = rct;
      drawButtonFast(rct, PIN_KEY_LABELS[idx]);
    }
  }
  if (pinPurpose != PIN_CHECK_LOGIN) {
    drawButton(pinCancelBtn, "Cancel", COLOR_MUTED);
  }
}

void handlePinTouch(int x, int y) {
  for (int i = 0; i < 12; i++) {
    if (touchInRect(x, y, pinKeyRects[i])) {
      if (i == 9) { // backspace "<"
        if (pinLen > 0) pinBuf[--pinLen] = 0;
      } else if (i == 11) { // "OK"
        if (pinPurpose == PIN_DEV_MODE) {
          if (strcmp(pinBuf, DEV_MODE_PASSWORD) == 0) {
            devModeUnlocked = true;
            devModeStartMillis = millis();
            pinLen = 0; pinBuf[0] = 0;
            if (waveform_isRunning()) waveform_setIntensity(actualIntensityPercent());
            screen = SCR_SETTINGS;
            return;
          } else {
            pinLen = 0; pinBuf[0] = 0;
            pinWrongFlash = true;
          }
        } else if (pinPurpose == PIN_SET_LOGIN) {
          // No "wrong" case here - whatever they entered becomes the password.
          strncpy(loginPassword, pinBuf, sizeof(loginPassword) - 1);
          loginPassword[sizeof(loginPassword) - 1] = 0;
          pinLen = 0; pinBuf[0] = 0;
          setupDone = true;
          saveSetupInfo();
          screen = SCR_CATEGORY;
          return;
        } else { // PIN_CHECK_LOGIN
          if (strcmp(pinBuf, loginPassword) == 0) {
            pinLen = 0; pinBuf[0] = 0;
            screen = SCR_CATEGORY;
            return;
          } else {
            pinLen = 0; pinBuf[0] = 0;
            pinWrongFlash = true;
          }
        }
      } else if (pinLen < 9) {
        pinBuf[pinLen++] = PIN_KEY_LABELS[i][0];
        pinBuf[pinLen] = 0;
      }
      screen = SCR_PIN; // stay, force redraw
      return;
    }
  }
  if (pinPurpose != PIN_CHECK_LOGIN && touchInRect(x, y, pinCancelBtn)) {
    pinLen = 0; pinBuf[0] = 0;
    screen = SCR_SETTINGS;
  }
}

// Relocks Developer Mode and rescales any running output back under the ceiling.
void relockDevMode() {
  devModeUnlocked = false;
  if (waveform_isRunning()) waveform_setIntensity(actualIntensityPercent());
}

// ---------------------------------------------------------------------
// Screen: Settings
// ---------------------------------------------------------------------
Rect btnDevMode = {20, 64, 220, 48};
Rect btnViewLog = {260, 64, 200, 48};
Rect btnCheckUpdates = {20, 122, 220, 48};
Rect btnBtDevice = {260, 122, 200, 48};
Rect btnRoomLabel = {20, 180, 220, 48};
Rect btnFactoryReset = {260, 180, 200, 48};

bool factoryResetArmed = false;
unsigned long factoryResetArmedAt = 0;
static const unsigned long FACTORY_RESET_ARM_WINDOW_MS = 5000;

void performFactoryReset() {
  Preferences p;
  p.begin("favs", false); p.clear(); p.end();
  p.begin("log", false);  p.clear(); p.end();
  p.begin("setup", false); p.clear(); p.end();
  // "tftcal" (touch calibration) deliberately NOT cleared - that's a
  // hardware characteristic of this physical screen, not a user setting.
  ESP.restart();
}

void drawSettingsScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Settings", 20, 8);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  char tierBuf[48];
  if (strlen(ownerName) > 0) {
    char shown[24];
    formatDisplayName(ownerName, shown, sizeof(shown));
    snprintf(tierBuf, sizeof(tierBuf), "%s - %s", HW_TIER_NAME, shown);
  } else {
    snprintf(tierBuf, sizeof(tierBuf), "%s", HW_TIER_NAME);
  }
  tft.drawString(tierBuf, 20, 34);

  drawHomeButton();

  if (devModeUnlocked) {
    drawButton(btnDevMode, "Dev Mode: ON", COLOR_GOOD, true);
  } else {
    drawButton(btnDevMode, "Unlock Dev Mode");
  }

  drawButton(btnViewLog, "Session Log", COLOR_MUTED);
  drawButton(btnCheckUpdates, "Check for Updates", COLOR_MUTED);

  char btBtnLabel[48];
  if (strlen(btDeviceName) > 0) {
    snprintf(btBtnLabel, sizeof(btBtnLabel), "BT: %s", btDeviceName);
  } else {
    snprintf(btBtnLabel, sizeof(btBtnLabel), "Set Up Bluetooth");
  }
  drawButton(btnBtDevice, btBtnLabel, COLOR_MUTED);

  if (isWellnessCenter) {
    char roomBtnLabel[40];
    if (strlen(roomLabel) > 0) {
      char shownRoom[24];
      formatDisplayName(roomLabel, shownRoom, sizeof(shownRoom));
      snprintf(roomBtnLabel, sizeof(roomBtnLabel), "Room: %s", shownRoom);
    } else {
      snprintf(roomBtnLabel, sizeof(roomBtnLabel), "Set Room Label");
    }
    drawButton(btnRoomLabel, roomBtnLabel, COLOR_MUTED);
  }

  // Auto-disarm the confirmation if the window has elapsed
  if (factoryResetArmed && millis() - factoryResetArmedAt > FACTORY_RESET_ARM_WINDOW_MS) {
    factoryResetArmed = false;
  }
  drawButton(btnFactoryReset, factoryResetArmed ? "Tap again to confirm" : "Factory Reset",
             factoryResetArmed ? COLOR_DANGER : COLOR_MUTED, factoryResetArmed);

  if (devModeUnlocked) {
    unsigned long remainMin = DEV_MODE_TIMEOUT_MIN -
      ((millis() - devModeStartMillis) / 60000UL);
    char buf[48];
    snprintf(buf, sizeof(buf), "Dev Mode auto-relocks in %lu min", remainMin);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(buf, 20, 240);
  }
}

void handleSettingsTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (touchInRect(x, y, btnDevMode)) {
    if (devModeUnlocked) {
      relockDevMode();
      screen = SCR_SETTINGS;
    } else {
      startPinEntry(PIN_DEV_MODE);
    }
    return;
  }
  if (touchInRect(x, y, btnViewLog)) {
    screen = SCR_LOG;
    return;
  }
  if (touchInRect(x, y, btnCheckUpdates)) {
    screen = SCR_UPDATE;
    return;
  }
  if (touchInRect(x, y, btnBtDevice)) {
    screen = SCR_BT_SCAN;
    return;
  }
  if (isWellnessCenter && touchInRect(x, y, btnRoomLabel)) {
    startTextEntry(roomLabel, "Room label", SCR_SETTINGS, false);
    return;
  }
  if (touchInRect(x, y, btnFactoryReset)) {
    if (factoryResetArmed) {
      performFactoryReset();
    } else {
      factoryResetArmed = true;
      factoryResetArmedAt = millis();
      screen = SCR_SETTINGS;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: session log (last 5 sessions)
// ---------------------------------------------------------------------
void drawLogScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Session Log", 20, 6);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  if (logCount == 0) {
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("No sessions recorded yet.", 20, 50);
    return;
  }

  // 2-column grid (up to 5 entries) instead of a single tall stack.
  for (int i = 0; i < logCount; i++) {
    int col = i % 2, row = i / 2;
    int x = 20 + col * 240, y = 40 + row * 90;
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString(sessionLog[i].name, x, y);
    char buf[48];
    snprintf(buf, sizeof(buf), "%.0f Hz - %d min", sessionLog[i].freqHz, sessionLog[i].durationMin);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString(buf, x, y + 22);
  }
}

void handleLogTouch(int x, int y) {
  handleHomeTouch(x, y);
}

// ---------------------------------------------------------------------
// Screen: Bluetooth device scan/pick
// ---------------------------------------------------------------------
Rect btnScanToggle = {20, 36, 440, 42};
Rect btScanResultRects[6];

void drawBtScanScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Bluetooth Device", 20, 6);
  drawHomeButton();

  bool scanning = btaudio_isScanning();
  drawButton(btnScanToggle, scanning ? "Scanning... (tap to stop)" : "Scan for Devices",
             scanning ? COLOR_WARN : COLOR_GOOD, scanning);

  int count = btaudio_scanResultCount();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  if (count == 0) {
    tft.drawString(scanning ? "Looking for nearby devices..." : "No devices found yet - tap Scan.", 20, 92);
  }

  // 2-column x 3-row grid, same pattern as the other list screens.
  int shown = count < 6 ? count : 6;
  int colW = 220, rowH = 42, gapX = 20, gapY = 8;
  for (int i = 0; i < 6; i++) {
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 88 + row * (rowH + gapY), colW, rowH};
    btScanResultRects[i] = r;
    if (i < shown) {
      drawButtonFast(r, btaudio_scanResultName(i));
    }
  }
  if (count > 6) {
    tft.drawString("More devices found - power off nearby", 20, 240);
    tft.drawString("ones you don't want to narrow it down.", 20, 260);
  }
}

void handleBtScanTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    if (btaudio_isScanning()) btaudio_stopScan();
    return;
  }
  if (touchInRect(x, y, btnScanToggle)) {
    if (btaudio_isScanning()) {
      btaudio_stopScan();
    } else {
      btaudio_startScan();
    }
    screen = SCR_BT_SCAN;
    return;
  }
  int count = btaudio_scanResultCount();
  int shown = count < 6 ? count : 6;
  for (int i = 0; i < shown; i++) {
    if (touchInRect(x, y, btScanResultRects[i])) {
      btaudio_stopScan();
      btaudio_connectToScanResult(i);
      strncpy(btDeviceName, btaudio_scanResultName(i), sizeof(btDeviceName) - 1);
      btDeviceName[sizeof(btDeviceName) - 1] = 0;
      saveSetupInfo();
      screen = SCR_SETTINGS;
      return;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: custom frequency entry - lets the user dial in any frequency
// directly, rather than only picking from the curated preset lists.
// Range capped 1-1000Hz (covers essentially everything in PEMF
// literature); the firmware's existing 20kHz hard ceiling still applies
// as an absolute backstop underneath this.
// ---------------------------------------------------------------------
char customFreqBuf[5] = "";
int customFreqLen = 0;
WaveShape customFreqWave = WAVE_SQUARE;
static const int CUSTOM_FREQ_MAX = 1000;

Rect cfKeyRects[12];
const char* CF_KEY_LABELS[12] = {"1","2","3","4","5","6","7","8","9","<","0","Start"};
Rect btnCfSquare = {20, 100, 85, 44};
Rect btnCfSine   = {115, 100, 85, 44};

void drawCustomFreqScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Custom Frequency", 20, 6);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  char rangeBuf[32];
  snprintf(rangeBuf, sizeof(rangeBuf), "Enter 1-%d Hz", CUSTOM_FREQ_MAX);
  tft.drawString(rangeBuf, 20, 34);

  char shown[16];
  snprintf(shown, sizeof(shown), "%s Hz", customFreqLen > 0 ? customFreqBuf : "-");
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.drawString(shown, 20, 58);

  drawButton(btnCfSquare, "Square", 0xFFFF, customFreqWave == WAVE_SQUARE);
  drawButton(btnCfSine, "Sine", 0xFFFF, customFreqWave == WAVE_SINE);

  int gx = 220, gy = 44, cellW = 70, cellH = 44, gap = 8; // gy=44 clears the Home button above (ends y=40)
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 3; c++) {
      int idx = r * 3 + c;
      Rect rct = {gx + c * (cellW + gap), gy + r * (cellH + gap), cellW, cellH};
      cfKeyRects[idx] = rct;
      if (idx == 11) {
        drawButton(rct, "Start", customFreqLen > 0 ? COLOR_GOOD : COLOR_MUTED, customFreqLen > 0);
      } else {
        drawButtonFast(rct, CF_KEY_LABELS[idx]);
      }
    }
  }
}

void handleCustomFreqTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    customFreqLen = 0; customFreqBuf[0] = 0;
    return;
  }
  if (touchInRect(x, y, btnCfSquare)) { customFreqWave = WAVE_SQUARE; screen = SCR_CUSTOM_FREQ; return; }
  if (touchInRect(x, y, btnCfSine))   { customFreqWave = WAVE_SINE;   screen = SCR_CUSTOM_FREQ; return; }

  for (int i = 0; i < 12; i++) {
    if (touchInRect(x, y, cfKeyRects[i])) {
      if (i == 9) { // backspace
        if (customFreqLen > 0) customFreqBuf[--customFreqLen] = 0;
      } else if (i == 11) { // Start
        if (customFreqLen == 0) { screen = SCR_CUSTOM_FREQ; return; }
        int val = atoi(customFreqBuf);
        if (val < 1) val = 1;
        if (val > CUSTOM_FREQ_MAX) val = CUSTOM_FREQ_MAX;
        selectedIndex = -1; // not a preset - favoriting doesn't apply
        selName = "Custom Frequency";
        selFreq = (float)val;
        selWave = customFreqWave;
        selCategoryName = "Custom";
        customFreqLen = 0; customFreqBuf[0] = 0;
        screen = isWellnessCenter ? SCR_CLIENT_CONFIRM : SCR_RUN;
        return;
      } else if (customFreqLen < 4) {
        customFreqBuf[customFreqLen++] = CF_KEY_LABELS[i][0];
        customFreqBuf[customFreqLen] = 0;
      }
      screen = SCR_CUSTOM_FREQ;
      return;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: Check for Updates - shows a QR code + the install page URL.
// This device has no browser/WiFi of its own; the person scans this with
// their phone (or types the URL) to reach the install page, where they
// can also grab a previous firmware version if a new one causes problems.
//
// Uses the ESP32 core's own built-in esp_qrcode_* API (no external
// library needed). Its display callback takes only the QR handle - no
// user-data pointer - so drawing parameters are plain constants inside
// the callback rather than passed through a context struct.
// ---------------------------------------------------------------------
// QR block sits on the right side of the screen now that there's width to
// spare; text/instructions live in the left column.
void drawUpdateQrCode(esp_qrcode_handle_t qrcode) {
  int scale = 4;
  int size = esp_qrcode_get_size(qrcode);
  int qrPixels = size * scale;
  int qx = 300;
  int qy = 20;

  // White quiet-zone margin around the code - required for reliable scanning.
  tft.fillRect(qx - 10, qy - 10, qrPixels + 20, qrPixels + 20, TFT_WHITE);
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      if (esp_qrcode_get_module(qrcode, x, y)) {
        tft.fillRect(qx + x * scale, qy + y * scale, scale, scale, TFT_BLACK);
      }
    }
  }
}

void drawUpdateScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Check for Updates", 20, 8);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  char verBuf[32];
  snprintf(verBuf, sizeof(verBuf), "Current version: %s", FIRMWARE_VERSION);
  tft.drawString(verBuf, 20, 40);
  tft.drawString("Scan with your phone to check", 20, 66);
  tft.drawString("for updates, or install an", 20, 86);
  tft.drawString("older version:", 20, 106);

  esp_qrcode_config_t qrCfg = ESP_QRCODE_CONFIG_DEFAULT();
  qrCfg.display_func = drawUpdateQrCode;
  qrCfg.max_qrcode_version = 5;
  qrCfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;
  esp_qrcode_generate(&qrCfg, UPDATE_URL);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  // Manually wrapped - the URL is too long for one line at this font size.
  tft.drawString("briasim-star.github.io/", 20, 140);
  tft.drawString("ESP32-4-/install.html", 20, 162);
}

void handleUpdateTouch(int x, int y) {
  handleHomeTouch(x, y);
}

// ---------------------------------------------------------------------
// Screen: category picker
// ---------------------------------------------------------------------
Rect catButtonRects[CATEGORY_COUNT];
Rect btnFavorites = {20, 40, 220, 48};
Rect btnSettings = {260, 40, 200, 48};
Rect btnCustomFreq = {20, 274, 440, 40}; // full-width row below the 6-category grid
bool viewingFavorites = false;

void buildCategoryIndex(Category cat) {
  catCount = 0;
  for (int i = 0; i < NUM_BASE_PRESETS; i++) {
    if (BASE_PRESETS[i].category == cat) catIndices[catCount++] = i;
  }
}

void buildFavoritesIndex() {
  catCount = 0;
  for (int i = 0; i < NUM_BASE_PRESETS; i++) {
    if (favoriteBits[i]) catIndices[catCount++] = i;
  }
}

void drawCategoryScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("MADD PEMF", 20, 8);

  drawButtonFast(btnFavorites, "* Favorites", COLOR_MUTED);
  drawButton(btnSettings, "Settings", COLOR_MUTED);

  int colW = 220, rowH = 48, gapX = 20, gapY = 10;
  for (int i = 0; i < CATEGORY_COUNT; i++) {
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 100 + row * (rowH + gapY), colW, rowH};
    catButtonRects[i] = r;
    drawButtonFast(r, CATEGORY_NAMES[i]);
  }

  drawButton(btnCustomFreq, "Custom Frequency", COLOR_MUTED);
}

void handleCategoryTouch(int x, int y) {
  if (touchInRect(x, y, btnFavorites)) {
    viewingFavorites = true;
    buildFavoritesIndex();
    listPage = 0;
    screen = SCR_LIST;
    return;
  }
  if (touchInRect(x, y, btnSettings)) {
    screen = SCR_SETTINGS;
    return;
  }
  if (touchInRect(x, y, btnCustomFreq)) {
    screen = SCR_CUSTOM_FREQ;
    return;
  }
  for (int i = 0; i < CATEGORY_COUNT; i++) {
    if (touchInRect(x, y, catButtonRects[i])) {
      viewingFavorites = false;
      currentCategory = (Category)i;
      buildCategoryIndex(currentCategory);
      listPage = 0;
      screen = SCR_LIST;
      return;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: preset list (paged)
// ---------------------------------------------------------------------
Rect itemRects[ITEMS_PER_PAGE];
Rect btnPrevPage = {20, 216, 140, 46};
Rect btnBackFromList = {180, 216, 120, 46};
Rect btnNextPage = {320, 216, 140, 46};

int listCount() {
  return catCount;
}

void getListLabel(int posInCategory, char* buf, size_t bufLen) {
  int realIdx = catIndices[posInCategory];
  snprintf(buf, bufLen, "%s (%.0f Hz)", BASE_PRESETS[realIdx].name,
           BASE_PRESETS[realIdx].freqHz);
}

// 2-column x 3-row grid, same pattern as the Category screen.
void drawListScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(viewingFavorites ? "Favorites" : CATEGORY_NAMES[currentCategory], 20, 8);
  drawHomeButton();

  int total = listCount();
  if (total == 0 && viewingFavorites) {
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("No favorites yet - tap the star on", 20, 48);
    tft.drawString("a preset's Run screen to add one.", 20, 70);
  }
  int start = listPage * ITEMS_PER_PAGE;
  char buf[64];
  int colW = 220, rowH = 44, gapX = 20, gapY = 8;
  for (int i = 0; i < ITEMS_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 48 + row * (rowH + gapY), colW, rowH};
    itemRects[i] = r;
    if (idx < total) {
      getListLabel(idx, buf, sizeof(buf));
      drawButtonFast(r, buf);
    }
  }
  drawButton(btnPrevPage, "< Prev");
  drawButton(btnBackFromList, "Back", COLOR_MUTED);
  drawButton(btnNextPage, "Next >");
}

void openRunScreenForIndex(int posInCategory) {
  int realIdx = catIndices[posInCategory];
  selectedIndex = realIdx;
  selName = BASE_PRESETS[realIdx].name;
  selFreq = BASE_PRESETS[realIdx].freqHz;
  selWave = BASE_PRESETS[realIdx].wave;
  selCategoryName = CATEGORY_NAMES[BASE_PRESETS[realIdx].category];
  screen = isWellnessCenter ? SCR_CLIENT_CONFIRM : SCR_RUN;
}

void handleListTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
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
Rect btnStartStop   = {20, 100, 220, 50};
Rect btnFavToggle   = {150, 36, 80, 28};
Rect btnBtAudio     = {20, 158, 220, 40};
Rect btnBackFromRun = {20, 206, 220, 40};

Stepper powerStepper  = {"Power",  260, 40, 10, 1, 100, 1, formatPercentValue};
Stepper timerStepper  = {"Session timer", 260, 130, 0, 0, 60, 5, formatTimerValue};
Stepper volumeStepper = {"BT Volume", 260, 220, 60, 0, 100, 5, formatPercentValue};

// Session auto-stop bookkeeping
unsigned long sessionStartMillis = 0;
bool sessionTimerArmed = false;

// Two-column landscape layout: left column is info + Start/Stop + BT
// toggle + Back, right column is the three steppers.
void drawRunScreen() {
  tft.fillScreen(COLOR_BG);
  drawHomeButton();

  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  drawFittedText(20, 8, 210, selName, FONT_LG, TFT_WHITE, COLOR_BG);

  char buf[48];
  snprintf(buf, sizeof(buf), "%.0f Hz", selFreq);
  tft.drawString(buf, 20, 40);

  bool isFav = (selectedIndex >= 0) && favoriteBits[selectedIndex];
  drawButtonFast(btnFavToggle, isFav ? "* ON" : "* Fav", isFav ? COLOR_WARN : COLOR_MUTED);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  snprintf(buf, sizeof(buf), "%s - %s",
           selWave == WAVE_SQUARE ? "Square" : "Sine", selCategoryName);
  tft.drawString(buf, 20, 70);

  bool running = waveform_isRunning();
  drawButton(btnStartStop, running ? "STOP" : "START", running ? COLOR_DANGER : COLOR_GOOD);

  drawButton(btnBtAudio, btAudioOn ? "BT Audio: ON" : "BT Audio: OFF", 0xFFFF, btAudioOn);
  drawButton(btnBackFromRun, "Stop & Back", COLOR_MUTED);

  powerStepper.value = powerDisplay;
  timerStepper.value = timerMinutes;
  volumeStepper.value = volumePercent;
  drawStepper(powerStepper);
  drawStepper(timerStepper);
  drawStepper(volumeStepper);
}

void handleRunTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    if (waveform_isRunning()) {
      int mins = (int)((millis() - sessionStartMillis) / 60000UL);
      addLogEntry(selName, selFreq, mins);
    }
    waveform_stop();
    sessionTimerArmed = false;
#if ENABLE_BT_AUDIO
    btAudioOn = false;
    btaudio_setEnabled(false);
#endif
    return;
  }
  if (touchInRect(x, y, btnFavToggle) && selectedIndex >= 0) {
    setFavorite(selectedIndex, !favoriteBits[selectedIndex]);
    screen = SCR_RUN;
    return;
  }
  if (handleStepperTouch(powerStepper, x, y)) {
    powerDisplay = powerStepper.value;
    waveform_setIntensity(actualIntensityPercent());
    screen = SCR_RUN;
    return;
  }
  if (handleStepperTouch(timerStepper, x, y)) {
    timerMinutes = timerStepper.value;
    if (waveform_isRunning()) {
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    screen = SCR_RUN;
    return;
  }
  if (handleStepperTouch(volumeStepper, x, y)) {
    volumePercent = volumeStepper.value;
#if ENABLE_BT_AUDIO
    btaudio_setVolume((uint8_t)volumePercent);
#endif
    screen = SCR_RUN;
    return;
  }
  if (touchInRect(x, y, btnStartStop)) {
    if (waveform_isRunning()) {
      int mins = (int)((millis() - sessionStartMillis) / 60000UL);
      addLogEntry(selName, selFreq, mins);
      waveform_stop();
      sessionTimerArmed = false;
    } else {
      waveform_start(selFreq, selWave, actualIntensityPercent());
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    screen = SCR_RUN;
  } else if (touchInRect(x, y, btnBtAudio)) {
#if ENABLE_BT_AUDIO
    btAudioOn = !btAudioOn;
    btaudio_setTargetFrequency(selFreq);
    btaudio_setVolume((uint8_t)volumePercent);
    btaudio_setEnabled(btAudioOn);
#endif
    screen = SCR_RUN;
  } else if (touchInRect(x, y, btnBackFromRun)) {
    if (waveform_isRunning()) {
      int mins = (int)((millis() - sessionStartMillis) / 60000UL);
      addLogEntry(selName, selFreq, mins);
    }
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
Screen lastDrawnScreen = SCR_WELCOME;
int lastDrawnPage = -1;

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // backlight on

  tft.init();
  tft.setRotation(1); // landscape, 480x320. If USB ends up on the left
                       // instead of the right, change this to 3.
  initTheme();

  // ---- Touch calibration (one-time, then reused every boot) ----
  // A resistive touch panel's raw readings don't map directly to screen
  // pixels - this runs an interactive "tap the corners" calibration the
  // very first time, then saves the result to flash (NVS) so you never
  // have to do it again. To force recalibration later (e.g. if the touch
  // feels off), erase flash once via Arduino IDE's "Tools > Erase All
  // Flash Before Sketch Upload" the next time you re-upload.
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
  loadFavorites();
  loadSessionLog();
  loadSetupInfo();
  btaudio_setSavedDeviceName(btDeviceName);

#if ENABLE_BT_AUDIO
  btaudio_begin("PEMF-Headset");
  btaudio_setVolume((uint8_t)volumePercent);
#endif

  drawWelcomeScreen();
}

void loop() {
  uint16_t tx, ty;
  bool touched = tft.getTouch(&tx, &ty);

  // Auto-stop when the session timer elapses
  if (sessionTimerArmed && waveform_isRunning()) {
    unsigned long elapsedMin = (millis() - sessionStartMillis) / 60000UL;
    if ((int)elapsedMin >= timerMinutes) {
      addLogEntry(selName, selFreq, (int)elapsedMin);
      waveform_stop();
      sessionTimerArmed = false;
      if (screen == SCR_RUN) drawRunScreen();
    }
  }

  // Auto-relock Developer Mode after its timeout
  if (devModeUnlocked) {
    unsigned long elapsedMin = (millis() - devModeStartMillis) / 60000UL;
    if (elapsedMin >= DEV_MODE_TIMEOUT_MIN) {
      relockDevMode();
      if (screen == SCR_RUN || screen == SCR_SETTINGS) {
        if (screen == SCR_RUN) drawRunScreen();
        else drawSettingsScreen();
      }
    }
  }

  // Redraw when the screen or page changes
  if (screen != lastDrawnScreen || (screen == SCR_LIST && listPage != lastDrawnPage)) {
    switch (screen) {
      case SCR_WELCOME:            drawWelcomeScreen(); break;
      case SCR_SETUP_MODE:         drawSetupModeScreen(); break;
      case SCR_SETUP_LOGIN_CHOICE: drawSetupLoginChoiceScreen(); break;
      case SCR_TEXT_ENTRY:         drawTextEntryScreen(); break;
      case SCR_CATEGORY:           drawCategoryScreen(); break;
      case SCR_LIST:               drawListScreen(); break;
      case SCR_RUN:                drawRunScreen(); break;
      case SCR_SETTINGS:           drawSettingsScreen(); break;
      case SCR_PIN:                drawPinScreen(); break;
      case SCR_LOG:                drawLogScreen(); break;
      case SCR_UPDATE:             drawUpdateScreen(); break;
      case SCR_CLIENT_CONFIRM:     drawClientConfirmScreen(); break;
      case SCR_BT_SCAN:            drawBtScanScreen(); break;
      case SCR_CUSTOM_FREQ:        drawCustomFreqScreen(); break;
    }
    lastDrawnScreen = screen;
    lastDrawnPage = listPage;
  }

  static bool wasTouched = false;
  if (touched && !wasTouched) {
    switch (screen) {
      case SCR_WELCOME:            handleWelcomeTouch(tx, ty); break;
      case SCR_SETUP_MODE:         handleSetupModeTouch(tx, ty); break;
      case SCR_SETUP_LOGIN_CHOICE: handleSetupLoginChoiceTouch(tx, ty); break;
      case SCR_TEXT_ENTRY:         handleTextEntryTouch(tx, ty); break;
      case SCR_CATEGORY:           handleCategoryTouch(tx, ty); break;
      case SCR_LIST:               handleListTouch(tx, ty); break;
      case SCR_RUN:                handleRunTouch(tx, ty); break;
      case SCR_SETTINGS:           handleSettingsTouch(tx, ty); break;
      case SCR_PIN:                handlePinTouch(tx, ty); break;
      case SCR_LOG:                handleLogTouch(tx, ty); break;
      case SCR_UPDATE:             handleUpdateTouch(tx, ty); break;
      case SCR_CLIENT_CONFIRM:     handleClientConfirmTouch(tx, ty); break;
      case SCR_BT_SCAN:            handleBtScanTouch(tx, ty); break;
      case SCR_CUSTOM_FREQ:        handleCustomFreqTouch(tx, ty); break;
    }
    // Force a redraw on the next loop pass since state may have changed
    // even when the Screen enum value itself didn't (e.g. Start/Stop,
    // stepper taps, PIN digit taps, checkbox toggle, keyboard taps).
    if (screen == SCR_WELCOME)        drawWelcomeScreen();
    if (screen == SCR_TEXT_ENTRY)     drawTextEntryScreen();
    if (screen == SCR_RUN)            drawRunScreen();
    if (screen == SCR_SETTINGS)       drawSettingsScreen();
    if (screen == SCR_PIN)            drawPinScreen();
    if (screen == SCR_CLIENT_CONFIRM) drawClientConfirmScreen();
    if (screen == SCR_BT_SCAN)        drawBtScanScreen();
    if (screen == SCR_CUSTOM_FREQ)    drawCustomFreqScreen();
  }
  wasTouched = touched;

  // Live-refresh the BT scan screen while a scan is running, so newly
  // discovered devices appear on screen without needing a touch.
  static unsigned long lastBtScanRedraw = 0;
  if (screen == SCR_BT_SCAN && btaudio_isScanning() && millis() - lastBtScanRedraw > 700) {
    drawBtScanScreen();
    lastBtScanRedraw = millis();
  }

  delay(20); // simple debounce; touch is polled, not interrupt-driven
}

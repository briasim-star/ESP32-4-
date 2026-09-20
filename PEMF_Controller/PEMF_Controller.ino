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
#include "wifi_time.h"
#include "sd_media.h"
#include "ota_update.h"
#include "subscription_check.h"
#include "local_audio.h"
#include <esp_task_wdt.h>
#include "ui_types.h"

// Set this to false if you don't want to build/wire the Bluetooth audio
// feature at all (skips linking the A2DP library's Bluetooth stack).
#define ENABLE_BT_AUDIO true

// Change this one line per hardware tier when building for the other unit.
static const char* HW_TIER_NAME = "MADD PEMF - Entry (MD10C)";
// static const char* HW_TIER_NAME = "MADD PEMF - Pro (MD30C)";

const char* FIRMWARE_VERSION = "1.0.0"; // not static - ota_update.cpp reads this via extern
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
int timerMinutes = 30;       // default 30 min auto-stop; 0 = continuous (Off), max 60
int volumePercent = 60;      // BT audio volume, independent of coil output

bool btAudioOn = false;

// Program-related mode flags - declared early (not down with the rest of
// the Program/Category code) since they're referenced by functions that
// appear earlier in the file than where that code naturally sits, and
// Arduino only auto-hoists function prototypes, not global variables.
int pendingSequenceIndex = -1; // one-shot: next Start on the Run screen runs SEQUENCES[this], -1 = none pending
// Tracks which screen led into the Run screen, so its Back button
// returns to the right place instead of always going to the preset
// list - confirmed real bug: entering via Harmonics/Custom Frequency
// and hitting Back incorrectly landed on whatever category was last
// browsed normally, since it always hardcoded SCR_LIST.
Screen runScreenOrigin = SCR_LIST;

// ---------------------------------------------------------------------
// People (Home mode only) - up to 4 named profiles, each with their own
// favorites. Stays logged in as whoever's active (persists across
// reboots) until manually switched via Settings. Wellness Center keeps
// its existing single-login model and never uses this.
// ---------------------------------------------------------------------
static const int MAX_PEOPLE = 4;
char peopleNames[MAX_PEOPLE][24];
int activePersonIndex = -1; // -1 = no one logged in - shared favorites fallback

int peopleCount() {
  int n = 0;
  for (int i = 0; i < MAX_PEOPLE; i++) if (strlen(peopleNames[i]) > 0) n++;
  return n;
}

// Returns the favorites NVS namespace for whoever's currently active, or
// the original shared "favs" namespace if no one's logged in - keeps
// behavior unchanged for anyone who never uses People at all.
const char* favsNamespaceForActivePerson() {
  static char buf[10];
  if (activePersonIndex < 0) return "favs";
  snprintf(buf, sizeof(buf), "favs_%d", activePersonIndex);
  return buf;
}

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
  p.begin(favsNamespaceForActivePerson(), true);
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
  p.begin(favsNamespaceForActivePerson(), false);
  char key[8];
  snprintf(key, sizeof(key), "f%d", idx);
  p.putBool(key, val);
  p.end();
}

void loadPeople() {
  Preferences p;
  p.begin("people", true);
  for (int i = 0; i < MAX_PEOPLE; i++) {
    char key[4];
    snprintf(key, sizeof(key), "n%d", i);
    String n = p.getString(key, "");
    strncpy(peopleNames[i], n.c_str(), sizeof(peopleNames[i]) - 1);
    peopleNames[i][sizeof(peopleNames[i]) - 1] = 0;
  }
  activePersonIndex = p.getInt("active", -1);
  if (activePersonIndex >= MAX_PEOPLE ||
      (activePersonIndex >= 0 && strlen(peopleNames[activePersonIndex]) == 0)) {
    activePersonIndex = -1; // guard against a stale/invalid saved index
  }
  p.end();
}

void savePeopleNames() {
  Preferences p;
  p.begin("people", false);
  for (int i = 0; i < MAX_PEOPLE; i++) {
    char key[4];
    snprintf(key, sizeof(key), "n%d", i);
    p.putString(key, peopleNames[i]);
  }
  p.end();
}

void saveActivePerson() {
  Preferences p;
  p.begin("people", false);
  p.putInt("active", activePersonIndex);
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
  time_t timestamp; // 0 if WiFi/NTP was never set up when this entry was logged
};
LogEntry sessionLog[LOG_SIZE];
int logCount = 0; // how many of the 5 slots are actually filled
unsigned long lifetimeSessionCount = 0; // every session ever completed, not just the last 5
unsigned long lifetimeMinutes = 0;      // total minutes of therapy ever delivered

void loadSessionLog() {
  Preferences p;
  p.begin("log", true);
  logCount = p.getInt("count", 0);
  if (logCount > LOG_SIZE) logCount = LOG_SIZE;
  for (int i = 0; i < logCount; i++) {
    char nameKey[8], freqKey[8], durKey[8], tsKey[8];
    snprintf(nameKey, sizeof(nameKey), "n%d", i);
    snprintf(freqKey, sizeof(freqKey), "q%d", i);
    snprintf(durKey, sizeof(durKey), "d%d", i);
    snprintf(tsKey, sizeof(tsKey), "t%d", i);
    String n = p.getString(nameKey, "");
    strncpy(sessionLog[i].name, n.c_str(), sizeof(sessionLog[i].name) - 1);
    sessionLog[i].name[sizeof(sessionLog[i].name) - 1] = 0;
    sessionLog[i].freqHz = p.getFloat(freqKey, 0);
    sessionLog[i].durationMin = p.getInt(durKey, 0);
    sessionLog[i].timestamp = (time_t)p.getULong64(tsKey, 0);
  }
  lifetimeSessionCount = p.getULong("lifeCount", 0);
  lifetimeMinutes = p.getULong("lifeMin", 0);
  p.end();
}

void saveSessionLog() {
  Preferences p;
  p.begin("log", false);
  p.putInt("count", logCount);
  for (int i = 0; i < logCount; i++) {
    char nameKey[8], freqKey[8], durKey[8], tsKey[8];
    snprintf(nameKey, sizeof(nameKey), "n%d", i);
    snprintf(freqKey, sizeof(freqKey), "q%d", i);
    snprintf(durKey, sizeof(durKey), "d%d", i);
    snprintf(tsKey, sizeof(tsKey), "t%d", i);
    p.putString(nameKey, sessionLog[i].name);
    p.putFloat(freqKey, sessionLog[i].freqHz);
    p.putInt(durKey, sessionLog[i].durationMin);
    p.putULong64(tsKey, (uint64_t)sessionLog[i].timestamp);
  }
  p.putULong("lifeCount", lifetimeSessionCount);
  p.putULong("lifeMin", lifetimeMinutes);
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
  sessionLog[0].timestamp = wifitime_now(); // 0 if WiFi/NTP was never set up
  logCount = n;
  lifetimeSessionCount++;
  lifetimeMinutes += durationMin;
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
bool btHeadphonesMode = false; // user-set: is that paired device headphones (true binaural beats work) or a speaker (mono-equivalent instead)?

// ---------------------------------------------------------------------
// Audio output auto-selection - this is the ONLY place that decides
// Bluetooth vs. the local wired speaker (local_audio.h), based on
// whether a Bluetooth device is currently configured. Every other piece
// of code calls these wrapper functions instead of btaudio_*/
// localaudio_* directly, so the two paths can never disagree about
// which one is actually "live" at any given moment.
// ---------------------------------------------------------------------
bool audioUsingBluetooth() { return strlen(btDeviceName) > 0; }

void audio_setEnabled(bool on) {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) { btaudio_setEnabled(on); return; }
#endif
  localaudio_setEnabled(on);
}

void audio_setTargetFrequency(float hz) {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) { btaudio_setTargetFrequency(hz); return; }
#endif
  localaudio_setTargetFrequency(hz);
}

void audio_setVolume(uint8_t percent) {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) { btaudio_setVolume(percent); return; }
#endif
  localaudio_setVolume(percent);
}

bool audio_isEnabled() {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) return btaudio_isEnabled();
#endif
  return localaudio_isEnabled();
}

bool audio_startSoundscape(const char* path) {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) return btaudio_startSoundscape(path);
#endif
  return localaudio_startSoundscape(path);
}

void audio_stopSoundscape() {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) { btaudio_stopSoundscape(); return; }
#endif
  localaudio_stopSoundscape();
}

bool audio_isSoundscapePlaying() {
#if ENABLE_BT_AUDIO
  if (audioUsingBluetooth()) return btaudio_isSoundscapePlaying();
#endif
  return localaudio_isSoundscapePlaying();
}


void loadSetupInfo() {
  Preferences p;
  p.begin("setup", true);
  setupDone = p.getBool("done", false);
  isWellnessCenter = p.getBool("wellness", false);
  requireLoginPassword = p.getBool("reqLogin", false);
  String n = p.getString("name", "");
  strncpy(ownerName, n.c_str(), sizeof(ownerName) - 1);
  ownerName[sizeof(ownerName) - 1] = 0;
  String lp = p.getString("loginPw", "");
  strncpy(loginPassword, lp.c_str(), sizeof(loginPassword) - 1);
  loginPassword[sizeof(loginPassword) - 1] = 0;
  String bt = p.getString("btDevice", "");
  strncpy(btDeviceName, bt.c_str(), sizeof(btDeviceName) - 1);
  btDeviceName[sizeof(btDeviceName) - 1] = 0;
  btHeadphonesMode = p.getBool("btHeadphones", false);
  p.end();

  // Room label lives in its own namespace, separate from "setup" - it's a
  // property of this physical unit's placement (matters for multi-unit
  // identification once WiFi/OTA exists), not a resettable user setting,
  // so it deliberately survives Factory Reset the same way touch
  // calibration ("tftcal") already does.
  Preferences rp;
  rp.begin("roomcfg", true);
  String r = rp.getString("room", "");
  strncpy(roomLabel, r.c_str(), sizeof(roomLabel) - 1);
  roomLabel[sizeof(roomLabel) - 1] = 0;
  rp.end();
}

void saveSetupInfo() {
  Preferences p;
  p.begin("setup", false);
  p.putBool("done", setupDone);
  p.putBool("wellness", isWellnessCenter);
  p.putBool("reqLogin", requireLoginPassword);
  p.putString("name", ownerName);
  p.putString("loginPw", loginPassword);
  p.putString("btDevice", btDeviceName);
  p.putBool("btHeadphones", btHeadphonesMode);
  p.end();

  Preferences rp;
  rp.begin("roomcfg", false);
  rp.putString("room", roomLabel);
  rp.end();
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
  int radius = min(r.w, r.h) / 2; // pill-shaped - fully rounded ends, not just softened corners
  tft.fillRoundRect(r.x, r.y, r.w, r.h, radius, fillColor);
  // Double-drawn border reads as a deliberate frame rather than a thin outline
  tft.drawRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_ACCENT);
  tft.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, radius - 1, COLOR_ACCENT);
  drawFittedLabel(r, label, FONT_LG, fillColor);
}

// Same pill shape as drawButton() for visual consistency across the
// whole UI (grids, lists, keypads used to look like sharp-cornered
// squares next to the rounder buttons elsewhere - now unified).
void drawButtonFast(Rect r, const char* label, uint16_t fillColor = 0xFFFF, bool active = false) {
  if (fillColor == 0xFFFF) fillColor = active ? COLOR_PANEL_LIT : COLOR_PANEL;
  int radius = min(r.w, r.h) / 2;
  tft.fillRoundRect(r.x, r.y, r.w, r.h, radius, fillColor);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_ACCENT);
  drawFittedLabel(r, label, FONT_SM, fillColor);
}

Rect btnHome = {365, 4, 105, 40};

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
Rect checkboxRect = {20, 172, 400, 36}; // widened to cover the whole row (box + label text), not just the small square - a much more forgiving tap target
Rect btnContinue  = {24, 226, 200, 46};

void drawWelcomeScreen() {
  // Splash image as background where we have one, falling back to the
  // plain solid color if there's no SD card. A solid dark panel sits
  // behind the disclaimer text specifically (not just the title) -
  // this screen carries real safety/medical disclaimer text, so
  // guaranteed readability matters more here than anywhere else in the
  // app, more than just picking a background area that "looks" like it
  // has enough contrast.
  if (!sdmedia_showSplash()) {
    tft.fillScreen(COLOR_BG);
  }
  tft.fillRect(0, 0, 480, 178, COLOR_BG);

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

  // Checkbox - the visual square stays small and fixed; checkboxRect
  // itself is intentionally wider (see its declaration) to cover the
  // whole row as the actual tap target, so only the square is drawn at
  // its own fixed size here rather than stretching to match.
  const int checkSquareSize = 36;
  tft.drawRect(checkboxRect.x, checkboxRect.y, checkSquareSize, checkSquareSize, COLOR_ACCENT);
  if (wellnessAck) {
    tft.fillRect(checkboxRect.x + 4, checkboxRect.y + 4, checkSquareSize - 8, checkSquareSize - 8, COLOR_ACCENT);
  }
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("I understand and agree", checkboxRect.x + checkSquareSize + 10, checkboxRect.y + 4);

  drawButton(btnContinue, "Continue", wellnessAck ? COLOR_GOOD : COLOR_MUTED, wellnessAck);

  // Small diagnostic lines, always shown, no navigation needed - so
  // this can be read directly off the screen instead of needing a
  // computer and Serial Monitor to check the same information. Solid
  // backing behind them for the same readability reason as the
  // disclaimer text above - plain text with no button panel of its own.
  tft.fillRect(0, 276, 480, 44, COLOR_BG);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  char sdDiag[48];
  if (!sdmedia_isAvailable()) {
    tft.drawString("SD card: not detected", 20, 282);
  } else if (!sdmedia_lastScanDirOpened()) {
    tft.drawString("SD card: OK, but /sounds folder not found", 20, 282);
  } else {
    snprintf(sdDiag, sizeof(sdDiag), "SD/sounds: %d entries, %d files, %d matched .wav",
             sdmedia_lastScanTotalEntries(), sdmedia_lastScanFileEntries(), sdmedia_soundscapeCount());
    tft.drawString(sdDiag, 20, 282);
  }

  // Same idea for the local speaker's I2S driver - confirm on-screen
  // whether it actually installed, rather than continuing to assume it
  // did without ever having verified it on real hardware.
  // Same idea for the local speaker's I2S driver - confirm on-screen
  // whether it actually installed, and whether writes to it are
  // actually succeeding, rather than continuing to assume it works
  // without ever having verified it on real hardware. Counts are
  // cumulative for the whole session, so testing audio then coming back
  // to this screen shows what actually happened during that test.
  if (!localaudio_didInstallSucceed()) {
    tft.drawString("Local speaker: I2S driver FAILED to install", 20, 302);
  } else {
    char audioDiag[56];
    snprintf(audioDiag, sizeof(audioDiag), "Local speaker: I2S OK, written %lu, failed %lu",
             localaudio_totalSamplesWritten(), localaudio_totalWriteFailures());
    tft.drawString(audioDiag, 20, 302);
  }
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
    } else if (!isWellnessCenter && peopleCount() > 0 && activePersonIndex < 0) {
      screen = SCR_PERSON_PICKER;
    } else {
      screen = SCR_CATEGORY;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: person picker (Home mode only) - shown after Welcome when
// people have been set up and no one's currently logged in. Once someone
// picks their name, they stay logged in (persisted across reboots) until
// they manually switch via Settings.
// ---------------------------------------------------------------------
Rect personPickerRects[MAX_PEOPLE];

void drawPersonPickerScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Who's using this?", 20, 20);

  int colW = 220, rowH = 90, gapX = 20, gapY = 20, startY = 80;
  for (int i = 0; i < MAX_PEOPLE; i++) {
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), startY + row * (rowH + gapY), colW, rowH};
    personPickerRects[i] = r;
    if (strlen(peopleNames[i]) > 0) {
      char shown[24];
      formatDisplayName(peopleNames[i], shown, sizeof(shown));
      drawButton(r, shown, COLOR_PANEL_LIT, true);
    }
  }
}

void handlePersonPickerTouch(int x, int y) {
  for (int i = 0; i < MAX_PEOPLE; i++) {
    if (strlen(peopleNames[i]) == 0 || !touchInRect(x, y, personPickerRects[i])) continue;

    activePersonIndex = i;
    saveActivePerson();
    loadFavorites();

    char shown[24];
    formatDisplayName(peopleNames[i], shown, sizeof(shown));
    char greeting[40];
    snprintf(greeting, sizeof(greeting), "Hi, %s!", shown);
    tft.fillScreen(COLOR_BG);
    tft.setFreeFont(FONT_XL);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(greeting, 240, 160);
    delay(1200);

    screen = SCR_CATEGORY;
    return;
  }
}

// ---------------------------------------------------------------------
// Screen: Manage People (Home mode only, via Settings) - add up to 4
// people, remove one (tap-twice-confirm, same pattern as Factory Reset).
// Removing someone deletes their favorites too, and logs them out if
// they were the active person.
// ---------------------------------------------------------------------
Rect managePeopleRects[MAX_PEOPLE];
Rect btnAddPerson = {20, 254, 440, 46};
int personToRemove = -1;
unsigned long personRemoveArmedAt = 0;

void removePerson(int idx) {
  peopleNames[idx][0] = 0;
  savePeopleNames();
  Preferences p;
  char ns[10];
  snprintf(ns, sizeof(ns), "favs_%d", idx);
  p.begin(ns, false);
  p.clear();
  p.end();
  if (activePersonIndex == idx) {
    activePersonIndex = -1;
    saveActivePerson();
    loadFavorites(); // falls back to the shared "favs" namespace
  }
}

void drawManagePeopleScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Manage People", 20, 8);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap a name twice to remove them.", 20, 36);

  if (personToRemove >= 0 && millis() - personRemoveArmedAt > 5000) {
    personToRemove = -1; // auto-disarm after the confirm window elapses
  }

  int colW = 220, rowH = 80, gapX = 20, gapY = 16, startY = 62;
  for (int i = 0; i < MAX_PEOPLE; i++) {
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), startY + row * (rowH + gapY), colW, rowH};
    managePeopleRects[i] = r;
    if (strlen(peopleNames[i]) > 0) {
      char shown[24];
      formatDisplayName(peopleNames[i], shown, sizeof(shown));
      bool armed = (personToRemove == i);
      char label[32];
      if (armed) snprintf(label, sizeof(label), "Remove %s?", shown);
      else strncpy(label, shown, sizeof(label) - 1);
      drawButton(r, label, armed ? COLOR_DANGER : COLOR_PANEL_LIT, true);
    } else {
      drawButtonFast(r, "(empty)", COLOR_MUTED);
    }
  }

  if (peopleCount() < MAX_PEOPLE) {
    drawButton(btnAddPerson, "+ Add Person", COLOR_GOOD);
  }
}

void handleManagePeopleTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    personToRemove = -1;
    return;
  }
  for (int i = 0; i < MAX_PEOPLE; i++) {
    if (strlen(peopleNames[i]) == 0 || !touchInRect(x, y, managePeopleRects[i])) continue;
    if (personToRemove == i) {
      removePerson(i);
      personToRemove = -1;
    } else {
      personToRemove = i;
      personRemoveArmedAt = millis();
    }
    screen = SCR_MANAGE_PEOPLE;
    return;
  }
  if (peopleCount() < MAX_PEOPLE && touchInRect(x, y, btnAddPerson)) {
    for (int i = 0; i < MAX_PEOPLE; i++) {
      if (strlen(peopleNames[i]) == 0) {
        startTextEntry(peopleNames[i], "Person's name", SCR_MANAGE_PEOPLE, false);
        break;
      }
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
  drawFittedText(30, 32, 420, textEntryLen > 0 ? shown : "-", FONT_LG, TFT_WHITE, COLOR_PANEL);

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
    savePeopleNames(); // harmless no-op unless this entry was actually a person's name
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
Rect clientCheckboxRect = {20, 190, 40, 40};
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
  for (int i = 0; i < pinLen; i++) {
    // Dev Mode PIN isn't protecting sensitive personal data - show the
    // actual digits so it's possible to notice a mis-registered tap
    // while typing, rather than only finding out after "OK" fails with
    // no way to tell why. The two login-related PIN purposes stay
    // masked, since those matter more for privacy.
    mask[i] = (pinPurpose == PIN_DEV_MODE) ? pinBuf[i] : '*';
  }
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

bool factoryResetArmed = false;
bool wifiForgetArmed = false;
unsigned long wifiForgetArmedAt = 0;
unsigned long factoryResetArmedAt = 0;
static const unsigned long FACTORY_RESET_ARM_WINDOW_MS = 5000;

void performFactoryReset() {
  Preferences p;
  p.begin("favs", false); p.clear(); p.end();
  p.begin("log", false);  p.clear(); p.end();
  p.begin("setup", false); p.clear(); p.end();
  p.begin("people", false); p.clear(); p.end();
  for (int i = 0; i < MAX_PEOPLE; i++) {
    char ns[10];
    snprintf(ns, sizeof(ns), "favs_%d", i);
    p.begin(ns, false); p.clear(); p.end();
  }
  // "tftcal" (touch calibration) and "roomcfg" (room label) deliberately
  // NOT cleared - those are characteristics of this physical unit and its
  // placement, not resettable user settings.
  if (wifitime_isConfigured()) wifitime_forgetNetwork();
  btaudio_endSession(); // clean BT teardown before restarting - see the note in bt_audio.cpp
  ESP.restart();
}

Screen soundscapesOrigin = SCR_SETTINGS; // where Soundscapes' "Back" returns to - SCR_RUN when entered mid-session instead. Declared here (moved from near the Soundscapes screen code) because handleSettingsItemTap() below uses it - a real "used before declared" build failure otherwise.

// ---------------------------------------------------------------------
// Settings screen - data-driven, paginated grid of uniform-size items,
// rather than individually hand-placed buttons of inconsistent sizes
// (a real, confirmed complaint - some were 28px tall, others 48px).
// Conditional items (Switch User, Soundscapes) are simply left out of
// the list when they don't apply, rather than needing their own
// special-cased layout math. SettingsItemId itself lives in ui_types.h,
// not here - see that file's comment for why.
// ---------------------------------------------------------------------
int buildVisibleSettingsItems(SettingsItemId* out) {
  int n = 0;
  out[n++] = SET_DEV_MODE;
  out[n++] = SET_GAMES;
  out[n++] = SET_CHECK_UPDATES;
  out[n++] = SET_BT_DEVICE;
  out[n++] = SET_ROOM_OR_PEOPLE;
  if (!isWellnessCenter && peopleCount() > 0) out[n++] = SET_SWITCH_USER;
  out[n++] = SET_WIFI;
  out[n++] = SET_VIEW_LOG;
  if (sdmedia_isAvailable() && sdmedia_soundscapeCount() > 0) out[n++] = SET_SOUNDSCAPES;
  out[n++] = SET_FACTORY_RESET;
  return n;
}

void getSettingsItemDisplay(SettingsItemId id, char* labelOut, size_t labelLen, uint16_t* colorOut, bool* activeOut) {
  *colorOut = COLOR_MUTED;
  *activeOut = false;
  switch (id) {
    case SET_DEV_MODE:
      if (devModeUnlocked) {
        unsigned long remainMin = DEV_MODE_TIMEOUT_MIN - ((millis() - devModeStartMillis) / 60000UL);
        snprintf(labelOut, labelLen, "Dev Mode: ON (%lum left)", remainMin);
        *colorOut = COLOR_GOOD; *activeOut = true;
      } else {
        snprintf(labelOut, labelLen, "Unlock Dev Mode");
      }
      break;
    case SET_GAMES:
      snprintf(labelOut, labelLen, "Games");
      break;
    case SET_CHECK_UPDATES:
      snprintf(labelOut, labelLen, "Check for Updates");
      break;
    case SET_BT_DEVICE:
      if (strlen(btDeviceName) > 0) snprintf(labelOut, labelLen, "BT: %s", btDeviceName);
      else snprintf(labelOut, labelLen, "Set Up Bluetooth");
      break;
    case SET_ROOM_OR_PEOPLE:
      if (isWellnessCenter) {
        if (strlen(roomLabel) > 0) {
          char shownRoom[24];
          formatDisplayName(roomLabel, shownRoom, sizeof(shownRoom));
          snprintf(labelOut, labelLen, "Room: %s", shownRoom);
        } else {
          snprintf(labelOut, labelLen, "Set Room Label");
        }
      } else {
        snprintf(labelOut, labelLen, "Manage People");
      }
      break;
    case SET_SWITCH_USER: {
      if (activePersonIndex >= 0) {
        char shown[24];
        formatDisplayName(peopleNames[activePersonIndex], shown, sizeof(shown));
        snprintf(labelOut, labelLen, "Switch User (%s)", shown);
      } else {
        snprintf(labelOut, labelLen, "Switch User");
      }
      break;
    }
    case SET_WIFI:
      if (wifiForgetArmed && millis() - wifiForgetArmedAt > FACTORY_RESET_ARM_WINDOW_MS) wifiForgetArmed = false;
      if (wifitime_isConfigured()) {
        snprintf(labelOut, labelLen, wifiForgetArmed ? "Tap again to forget" : "WiFi: Forget Network");
        *colorOut = wifiForgetArmed ? COLOR_DANGER : COLOR_MUTED; *activeOut = wifiForgetArmed;
      } else {
        snprintf(labelOut, labelLen, "Set Up WiFi (for timestamps)");
      }
      break;
    case SET_VIEW_LOG:
      snprintf(labelOut, labelLen, "Session Log");
      break;
    case SET_SOUNDSCAPES:
      if (audio_isSoundscapePlaying()) snprintf(labelOut, labelLen, "Soundscapes: Playing");
      else snprintf(labelOut, labelLen, "Soundscapes (%d available)", sdmedia_soundscapeCount());
      *activeOut = audio_isSoundscapePlaying();
      break;
    case SET_FACTORY_RESET:
      if (factoryResetArmed && millis() - factoryResetArmedAt > FACTORY_RESET_ARM_WINDOW_MS) factoryResetArmed = false;
      snprintf(labelOut, labelLen, factoryResetArmed ? "Tap again to confirm" : "Factory Reset");
      *colorOut = factoryResetArmed ? COLOR_DANGER : COLOR_MUTED; *activeOut = factoryResetArmed;
      break;
    default:
      labelOut[0] = 0;
      break;
  }
}

void handleSettingsItemTap(SettingsItemId id) {
  switch (id) {
    case SET_DEV_MODE:
      if (devModeUnlocked) { relockDevMode(); screen = SCR_SETTINGS; }
      else startPinEntry(PIN_DEV_MODE);
      break;
    case SET_GAMES:
      screen = SCR_GAMES_MENU;
      break;
    case SET_CHECK_UPDATES:
      screen = SCR_UPDATE;
      break;
    case SET_BT_DEVICE:
      screen = SCR_BT_SCAN;
      break;
    case SET_ROOM_OR_PEOPLE:
      if (isWellnessCenter) startTextEntry(roomLabel, "Room label", SCR_SETTINGS, false);
      else screen = SCR_MANAGE_PEOPLE;
      break;
    case SET_SWITCH_USER:
      activePersonIndex = -1;
      saveActivePerson();
      screen = SCR_PERSON_PICKER;
      break;
    case SET_WIFI:
      if (wifitime_isConfigured()) {
        if (wifiForgetArmed) {
          wifitime_forgetNetwork();
          wifiForgetArmed = false;
          screen = SCR_SETTINGS;
        } else {
          wifiForgetArmed = true;
          wifiForgetArmedAt = millis();
          screen = SCR_SETTINGS;
        }
      } else {
        wifitime_beginSetupPortal();
        screen = SCR_WIFI_SETUP;
      }
      break;
    case SET_VIEW_LOG:
      screen = SCR_LOG;
      break;
    case SET_SOUNDSCAPES:
      soundscapesOrigin = SCR_SETTINGS;
      screen = SCR_SOUNDSCAPES;
      break;
    case SET_FACTORY_RESET:
      if (factoryResetArmed) performFactoryReset();
      else { factoryResetArmed = true; factoryResetArmedAt = millis(); screen = SCR_SETTINGS; }
      break;
    default:
      break;
  }
}

int settingsPage = 0;
const int SETTINGS_PER_PAGE = 6;
Rect settingsItemRects[SETTINGS_PER_PAGE];
SettingsItemId settingsVisibleItems[SET_ITEM_COUNT];
int settingsVisibleCount = 0;
Rect btnSetPrev = {20, 254, 220, 46};
Rect btnSetNext = {260, 254, 200, 46};

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
  drawFittedText(20, 34, 340, tierBuf, FONT_SM, COLOR_TEXT_DIM, COLOR_BG); // constrained so a long name can't run into the Home button

  drawHomeButton();

  settingsVisibleCount = buildVisibleSettingsItems(settingsVisibleItems);
  int totalPages = (settingsVisibleCount + SETTINGS_PER_PAGE - 1) / SETTINGS_PER_PAGE;
  if (totalPages < 1) totalPages = 1;
  if (settingsPage >= totalPages) settingsPage = 0; // safety, if fewer items are visible now than before

  if (totalPages > 1) {
    char pageBuf[20];
    snprintf(pageBuf, sizeof(pageBuf), "Page %d of %d", settingsPage + 1, totalPages);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pageBuf, 460, 16);
    tft.setTextDatum(TL_DATUM);
  }

  int start = settingsPage * SETTINGS_PER_PAGE;
  int colW = 220, rowH = 48, gapX = 20, gapY = 10;
  for (int i = 0; i < SETTINGS_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 64 + row * (rowH + gapY), colW, rowH};
    settingsItemRects[i] = r;
    if (idx < settingsVisibleCount) {
      char label[48];
      uint16_t color;
      bool active;
      getSettingsItemDisplay(settingsVisibleItems[idx], label, sizeof(label), &color, &active);
      drawButton(r, label, color, active);
    }
  }

  if (totalPages > 1) {
    drawButton(btnSetPrev, "< Prev");
    drawButton(btnSetNext, "Next >");
  }
}

void handleSettingsTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  int start = settingsPage * SETTINGS_PER_PAGE;
  for (int i = 0; i < SETTINGS_PER_PAGE; i++) {
    int idx = start + i;
    if (idx < settingsVisibleCount && touchInRect(x, y, settingsItemRects[i])) {
      handleSettingsItemTap(settingsVisibleItems[idx]);
      return;
    }
  }
  int totalPages = (settingsVisibleCount + SETTINGS_PER_PAGE - 1) / SETTINGS_PER_PAGE;
  if (totalPages > 1) {
    if (touchInRect(x, y, btnSetPrev)) {
      if (settingsPage > 0) settingsPage--;
      screen = SCR_SETTINGS;
    } else if (touchInRect(x, y, btnSetNext)) {
      if (settingsPage + 1 < totalPages) settingsPage++;
      screen = SCR_SETTINGS;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: WiFi setup portal in progress - a real Cancel/Home path while
// waiting for the person to pick a network and enter a password on
// their phone. See the design note in wifi_time.cpp: a fully-blocking
// version of this had no way to back out of it at all.
// ---------------------------------------------------------------------
Rect btnWifiCancel = {150, 240, 180, 46};

void drawWifiSetupScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Connect your phone to:", 240, 90);
  tft.setFreeFont(FONT_XL);
  tft.drawString("MADD-PEMF-Setup", 240, 125);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Password: maddpemf2026", 240, 160);
  tft.drawString("Then pick your WiFi network on your phone.", 240, 182);
  tft.setTextDatum(TL_DATUM);
  drawButton(btnWifiCancel, "Cancel", COLOR_MUTED);
}

void handleWifiSetupTouch(int x, int y) {
  if (touchInRect(x, y, btnWifiCancel)) {
    wifitime_cancelPortal();
    screen = SCR_SETTINGS;
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
    drawFittedText(x, y, 220, sessionLog[i].name, FONT_SM, TFT_WHITE, COLOR_BG); // constrained - a long preset name could otherwise run into the next column
    char buf[48];
    snprintf(buf, sizeof(buf), "%.0f Hz - %d min", sessionLog[i].freqHz, sessionLog[i].durationMin);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString(buf, x, y + 22);
    if (sessionLog[i].timestamp > 0) {
      struct tm* t = localtime(&sessionLog[i].timestamp);
      char dateBuf[24];
      strftime(dateBuf, sizeof(dateBuf), "%b %d, %Y %H:%M", t);
      tft.drawString(dateBuf, x, y + 42);
    }
  }
}

void handleLogTouch(int x, int y) {
  handleHomeTouch(x, y);
}

// ---------------------------------------------------------------------
// Screen: Bluetooth device scan/pick
// ---------------------------------------------------------------------
Rect btnScanToggle = {20, 36, 440, 42};
Rect btnHeadphonesToggle = {20, 88, 220, 42};
Rect btnForgetBt = {260, 88, 200, 42};
bool btForgetArmed = false;
unsigned long btForgetArmedAt = 0;
Rect btScanResultRects[6];

bool scanRequested = false; // true the instant the button is tapped, before the BT stack actually starts discovering - gives immediate feedback instead of an unexplained gap

void drawBtScanScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Bluetooth Device", 20, 6);
  drawHomeButton();

  bool scanning = btaudio_isScanning();
  const char* scanLabel = scanning ? "Scanning... (tap to stop)"
                        : scanRequested ? "Starting scan..."
                        : "Scan for Devices";
  drawButton(btnScanToggle, scanLabel, (scanning || scanRequested) ? COLOR_WARN : COLOR_GOOD, scanning || scanRequested);

  int gridStartY = 88;
  if (strlen(btDeviceName) > 0) {
    // Auto-disarm the forget confirmation if the window has elapsed,
    // same pattern as WiFi's forget-network and Factory Reset.
    if (btForgetArmed && millis() - btForgetArmedAt > FACTORY_RESET_ARM_WINDOW_MS) {
      btForgetArmed = false;
    }
    // Only relevant once a device is actually connected - true binaural
    // beats need two ears each hearing a different tone, which only
    // headphones can deliver; a speaker mixes both channels together
    // in the air, defeating the effect. No reliable way to detect this
    // automatically, so it's set explicitly here.
    drawButton(btnHeadphonesToggle, btHeadphonesMode ? "This is: Headphones" : "This is: A Speaker",
               COLOR_MUTED, btHeadphonesMode);
    drawButton(btnForgetBt, btForgetArmed ? "Tap again to forget" : "Forget This Device",
               btForgetArmed ? COLOR_DANGER : COLOR_MUTED, btForgetArmed);
    gridStartY = 138;
  }

  int count = btaudio_scanResultCount();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  if (count == 0) {
    tft.drawString((scanning || scanRequested) ? "Looking for nearby devices..." : "No devices found yet - tap Scan.", 20, gridStartY + 4);
  }

  // 2-column x 3-row grid, same pattern as the other list screens.
  int shown = count < 6 ? count : 6;
  int colW = 220, rowH = 42, gapX = 20, gapY = 8;
  for (int i = 0; i < 6; i++) {
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), gridStartY + row * (rowH + gapY), colW, rowH};
    btScanResultRects[i] = r;
    if (i < shown) {
      drawButtonFast(r, btaudio_scanResultName(i));
    }
  }
  if (count > 6) {
    tft.drawString("More devices found - power off nearby", 20, gridStartY + 152);
    tft.drawString("ones you don't want to narrow it down.", 20, gridStartY + 172);
  }
}

void handleBtScanTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    if (btaudio_isScanning()) btaudio_stopScan();
    scanRequested = false;
    return;
  }
  if (touchInRect(x, y, btnScanToggle)) {
    if (btaudio_isScanning() || scanRequested) {
      btaudio_stopScan();
      scanRequested = false;
    } else {
      btaudio_startScan();
      scanRequested = true;
    }
    screen = SCR_BT_SCAN;
    return;
  }
  if (strlen(btDeviceName) > 0 && touchInRect(x, y, btnHeadphonesToggle)) {
    btHeadphonesMode = !btHeadphonesMode;
    btaudio_setHeadphonesMode(btHeadphonesMode);
    saveSetupInfo();
    screen = SCR_BT_SCAN;
    return;
  }
  if (strlen(btDeviceName) > 0 && touchInRect(x, y, btnForgetBt)) {
    if (btForgetArmed) {
      btDeviceName[0] = 0; // now empty, so audioUsingBluetooth() correctly falls back to the local speaker
      btHeadphonesMode = false;
      btForgetArmed = false;
      saveSetupInfo();
    } else {
      btForgetArmed = true;
      btForgetArmedAt = millis();
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
      btaudio_endSession(); // clean BT teardown before restarting - see the note in bt_audio.cpp
      // The actual connection only happens cleanly on a fresh boot - see
      // the comment in btaudio_connectToScanResult(). Show a brief message
      // so this doesn't look like a freeze right before it restarts.
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Saved - restarting...", 240, 160);
      delay(800); // trimmed from 1200ms - still readable, shaves a little off the total wait
      ESP.restart();
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
char customFreqBuf[8] = "";
int customFreqLen = 0;
WaveShape customFreqWave = WAVE_SQUARE;
static const int CUSTOM_FREQ_MAX = 1000;
static const int CUSTOM_FREQ_MAX_CHARS = 7; // e.g. "999.99" plus room

Rect cfKeyRects[12];
const char* CF_KEY_LABELS[12] = {"1","2","3","4","5","6","7","8","9",".","0","<"};
Rect btnCfSquare = {20, 100, 85, 44};
Rect btnCfSine   = {115, 100, 85, 44};
Rect btnCfStart  = {20, 154, 180, 44};

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
  snprintf(rangeBuf, sizeof(rangeBuf), "Enter 1-%d Hz (decimals OK)", CUSTOM_FREQ_MAX);
  tft.drawString(rangeBuf, 20, 34);

  // Clear the FULL value-display area first, not just the new text's own
  // width - otherwise a shorter number typed after a longer one (or a
  // backspace) leaves stale digit remnants behind, which read as
  // garbled/cut-off text.
  tft.fillRect(18, 52, 200, 40, COLOR_BG);
  char shown[20];
  snprintf(shown, sizeof(shown), "%s Hz", customFreqLen > 0 ? customFreqBuf : "-");
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(shown, 20, 58);

  drawButton(btnCfSquare, "Square", 0xFFFF, customFreqWave == WAVE_SQUARE);
  drawButton(btnCfSine, "Sine", 0xFFFF, customFreqWave == WAVE_SINE);
  drawButton(btnCfStart, "Start", customFreqLen > 0 ? COLOR_GOOD : COLOR_MUTED, customFreqLen > 0);

  int gx = 220, gy = 48, cellW = 70, cellH = 44, gap = 8; // gy=48 clears the enlarged Home button (ends y=44)
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 3; c++) {
      int idx = r * 3 + c;
      Rect rct = {gx + c * (cellW + gap), gy + r * (cellH + gap), cellW, cellH};
      cfKeyRects[idx] = rct;
      drawButtonFast(rct, CF_KEY_LABELS[idx]);
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
  if (touchInRect(x, y, btnCfStart)) {
    if (customFreqLen == 0) { screen = SCR_CUSTOM_FREQ; return; }
    float val = atof(customFreqBuf);
    if (val < 1) val = 1;
    if (val > CUSTOM_FREQ_MAX) val = CUSTOM_FREQ_MAX;
    selectedIndex = -1; // not a preset - favoriting doesn't apply
    selName = "Custom Frequency";
    selFreq = val;
    selWave = customFreqWave;
    selCategoryName = "Custom";
    pendingSequenceIndex = -1; // this is a custom frequency pick, not a Sequence
    customFreqLen = 0; customFreqBuf[0] = 0;
    runScreenOrigin = SCR_CUSTOM_FREQ;
    screen = isWellnessCenter ? SCR_CLIENT_CONFIRM : SCR_RUN;
    return;
  }

  for (int i = 0; i < 12; i++) {
    if (!touchInRect(x, y, cfKeyRects[i])) continue;
    char c = CF_KEY_LABELS[i][0];
    if (c == '<') {
      if (customFreqLen > 0) customFreqBuf[--customFreqLen] = 0;
    } else if (c == '.') {
      // Only one decimal point, and not as the very first character
      bool alreadyHasDot = (strchr(customFreqBuf, '.') != nullptr);
      if (!alreadyHasDot && customFreqLen > 0 && customFreqLen < CUSTOM_FREQ_MAX_CHARS) {
        customFreqBuf[customFreqLen++] = '.';
        customFreqBuf[customFreqLen] = 0;
      }
    } else if (customFreqLen < CUSTOM_FREQ_MAX_CHARS) {
      customFreqBuf[customFreqLen++] = c;
      customFreqBuf[customFreqLen] = 0;
    }
    screen = SCR_CUSTOM_FREQ;
    return;
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
  int qy = 60; // shifted down from 20 - real bug found: at 20 it overlapped the enlarged Home button (ends y=44)

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

Rect btnOtaAction = {20, 180, 220, 44};
bool otaChecked = false;
bool otaUpdateAvailable = false;
bool otaChecking = false;

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
  tft.drawString("Update over WiFi, or scan with", 20, 66);
  tft.drawString("your phone to install by cable:", 20, 86);

  esp_qrcode_config_t qrCfg = ESP_QRCODE_CONFIG_DEFAULT();
  qrCfg.display_func = drawUpdateQrCode;
  qrCfg.max_qrcode_version = 5;
  qrCfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;
  esp_qrcode_generate(&qrCfg, UPDATE_URL);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  // Manually wrapped - the URL is too long for one line at this font size.
  tft.drawString("briasim-star.github.io/", 20, 106);
  tft.drawString("ESP32-4-/install.html", 20, 128);

  if (!otaChecked) {
    drawButton(btnOtaAction, "Check for Updates", COLOR_GOOD);
  } else if (otaUpdateAvailable) {
    char label[32];
    snprintf(label, sizeof(label), "Install v%s Now", ota_latestVersionString());
    drawButton(btnOtaAction, label, COLOR_WARN);
  } else {
    drawButton(btnOtaAction, "Up to date!", COLOR_MUTED, true);
  }
}

void handleUpdateTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    otaChecked = false; // reset so re-entering this screen starts fresh
    return;
  }
  if (otaChecking) return; // ignore taps while a blocking check/install is underway

  if (touchInRect(x, y, btnOtaAction)) {
    if (!otaChecked) {
      otaChecking = true;
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Checking for updates...", 240, 150);
      // Hardware watchdog as a hard backstop - a real hang here once
      // required a battery pull to recover from, and software timeouts
      // alone have a documented reliability quirk (see ota_update.cpp),
      // so this guarantees automatic recovery within 45s no matter what.
      esp_task_wdt_init(45, true);
      esp_task_wdt_add(NULL);
      bool ok = ota_checkForUpdate();
      esp_task_wdt_delete(NULL);
      otaChecking = false;
      if (!ok && strlen(ota_lastErrorMessage()) > 0) {
        tft.fillScreen(COLOR_BG);
        tft.drawString("Couldn't check for updates", 240, 140);
        tft.setFreeFont(FONT_SM);
        tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
        tft.drawString(ota_lastErrorMessage(), 240, 175);
        delay(2000);
      } else {
        otaChecked = true;
        otaUpdateAvailable = ok;
      }
      screen = SCR_UPDATE;
    } else if (otaUpdateAvailable) {
      otaChecking = true;
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Downloading update...", 240, 140);
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
      tft.drawString("The device will restart when done.", 240, 175);
      esp_task_wdt_init(45, true);
      esp_task_wdt_add(NULL);
      bool ok = ota_downloadAndInstall(); // restarts the device on success - only returns on failure
      esp_task_wdt_delete(NULL);
      otaChecking = false;
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Update failed", 240, 140);
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
      tft.drawString(ota_lastErrorMessage(), 240, 175);
      tft.drawString("Nothing was changed - try again later.", 240, 195);
      delay(2500);
      screen = SCR_UPDATE;
    }
  }
}

// ---------------------------------------------------------------------
// Screen: category picker
// ---------------------------------------------------------------------
Rect catButtonRects[CATEGORY_COUNT];
Rect btnFavorites = {20, 40, 220, 48};
Rect btnSettings = {260, 40, 200, 48};
Rect btnCustomFreq   = {20, 274, 220, 40};
Rect btnSequences    = {260, 274, 200, 40};
bool viewingFavorites = false;
// pendingSequenceIndex declared earlier (near the other mode flags) - see note there.

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

Rect titleTapZone = {140, 0, 200, 40}; // widened - same lesson as the checkbox fix: small tap targets read as "doesn't work"
int titleTapCount = 0;
unsigned long titleTapWindowStart = 0;
bool showDeviceStats = false;

void drawCategoryScreen() {
  // Same splash-background treatment as the Welcome screen - falls back
  // to the plain solid color if there's no SD card. This screen's own
  // buttons already have their own solid panel fills (readable either
  // way); only the plain title text needs its own backing strip.
  if (!sdmedia_showSplash()) {
    tft.fillScreen(COLOR_BG);
  }
  tft.fillRect(0, 0, 480, 36, COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("MADD PEMF", 240, 8);
  tft.setTextDatum(TL_DATUM);

  drawButtonFast(btnFavorites, "* Favorites", COLOR_MUTED);
  drawButton(btnSettings, "Settings", COLOR_MUTED);

  int colW = 220, rowH = 48, gapX = 20, gapY = 10;
  for (int i = 0; i < CATEGORY_COUNT; i++) {
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 100 + row * (rowH + gapY), colW, rowH};
    catButtonRects[i] = r;
    drawButtonFast(r, CATEGORY_NAMES[i]);
  }

  drawButtonFast(btnCustomFreq, "Custom Freq", COLOR_MUTED);
  drawButton(btnSequences, "Harmonics", COLOR_MUTED);

  if (showDeviceStats) {
    int px = 60, py = 70, pw = 360, ph = 180;
    int radius = 24;
    tft.fillRoundRect(px, py, pw, ph, radius, COLOR_PANEL_LIT);
    tft.drawRoundRect(px, py, pw, ph, radius, COLOR_ACCENT);
    tft.setFreeFont(FONT_LG);
    tft.setTextColor(TFT_WHITE, COLOR_PANEL_LIT);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("You found it!", 240, py + 16);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_PANEL_LIT);
    char buf[48];
    snprintf(buf, sizeof(buf), "%lu sessions run, all-time", lifetimeSessionCount);
    tft.drawString(buf, 240, py + 56);
    unsigned long hrs = lifetimeMinutes / 60;
    snprintf(buf, sizeof(buf), "%lu hours, %lu minutes of therapy delivered", hrs, lifetimeMinutes % 60);
    tft.drawString(buf, 240, py + 78);
    unsigned long upHrs = millis() / 3600000UL;
    unsigned long upMin = (millis() / 60000UL) % 60;
    snprintf(buf, sizeof(buf), "Up for %luh %lum since last power-on", upHrs, upMin);
    tft.drawString(buf, 240, py + 100);
    tft.setTextColor(COLOR_ACCENT, COLOR_PANEL_LIT);
    tft.drawString("Tap anywhere to close", 240, py + 140);
    tft.setTextDatum(TL_DATUM);
  }
}

void handleCategoryTouch(int x, int y) {
  if (showDeviceStats) {
    showDeviceStats = false; // tap anywhere to dismiss
    screen = SCR_CATEGORY;
    return;
  }
  if (touchInRect(x, y, titleTapZone)) {
    unsigned long now = millis();
    if (now - titleTapWindowStart > 4000) { titleTapCount = 0; titleTapWindowStart = now; }
    titleTapCount++;
    if (titleTapCount >= 5) {
      titleTapCount = 0;
      showDeviceStats = true;
    }
    screen = SCR_CATEGORY;
    return;
  }
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
  if (touchInRect(x, y, btnSequences)) {
    screen = SCR_SEQUENCES;
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
// Screen: Sequences - browse and pick from the named multi-stage
// programs (SEQUENCES[] in presets.h). Same 2-col x 3-row paginated
// list pattern as the preset List screen.
// ---------------------------------------------------------------------
int sequencesPage = 0;
const int SEQUENCES_PER_PAGE = 6;
Rect sequenceItemRects[SEQUENCES_PER_PAGE];
Rect btnSeqPrevPage = {20, 254, 140, 46};
Rect btnSeqBack = {180, 254, 120, 46};
Rect btnSeqNextPage = {320, 254, 140, 46};

// Compact "X Hz -> Y Hz" preview using the sequence's first and last
// step frequencies, so a person can see roughly what range it moves
// through before picking it, rather than just a name.
void getSequenceFreqRange(int idx, char* buf, size_t bufLen) {
  const Program& p = SEQUENCES[idx];
  float first = p.steps[0].freqHz;
  float last = p.steps[p.stepCount - 1].freqHz;
  float diff = (first > last) ? (first - last) : (last - first); // avoid needing fabsf/math.h here
  if (diff < 0.5f) {
    snprintf(buf, bufLen, "%.2g Hz", first);
  } else {
    snprintf(buf, bufLen, "%.2g -> %.2g Hz", first, last);
  }
}

void drawSequencesScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Harmonics", 20, 8);
  drawHomeButton();

  int totalSeqPages = (NUM_SEQUENCES + SEQUENCES_PER_PAGE - 1) / SEQUENCES_PER_PAGE;
  if (totalSeqPages > 1) {
    // Placed next to the title rather than below the grid/nav row -
    // there wasn't enough vertical room down there without it crowding
    // into the Prev/Back/Next buttons.
    char pageBuf[20];
    snprintf(pageBuf, sizeof(pageBuf), "Page %d of %d", sequencesPage + 1, totalSeqPages);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pageBuf, 460, 16);
    tft.setTextDatum(TL_DATUM);
  }

  int start = sequencesPage * SEQUENCES_PER_PAGE;
  int colW = 220, rowH = 60, gapX = 20, gapY = 8;
  for (int i = 0; i < SEQUENCES_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 48 + row * (rowH + gapY), colW, rowH};
    sequenceItemRects[i] = r;
    if (idx < NUM_SEQUENCES) {
      int radius = min(r.w, r.h) / 2;
      tft.fillRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_PANEL);
      tft.drawRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_ACCENT);
      // Name on its own line, then a dimmer frequency-range line below.
      // Both are width-constrained with truncation - a real bug before:
      // plain drawString() here had no width limit at all, so longer
      // names genuinely ran outside the box.
      int maxTextW = r.w - 16;
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(TFT_WHITE, COLOR_PANEL);
      tft.setTextDatum(TC_DATUM);
      if (tft.textWidth(SEQUENCES[idx].name) <= maxTextW) {
        tft.drawString(SEQUENCES[idx].name, r.x + r.w / 2, r.y + 8);
      } else {
        char trimmed[24];
        strncpy(trimmed, SEQUENCES[idx].name, sizeof(trimmed) - 1);
        trimmed[sizeof(trimmed) - 1] = 0;
        int len = strlen(trimmed);
        while (len > 1) {
          trimmed[len - 1] = 0;
          char tryBuf[26];
          snprintf(tryBuf, sizeof(tryBuf), "%s..", trimmed);
          if (tft.textWidth(tryBuf) <= maxTextW) {
            tft.drawString(tryBuf, r.x + r.w / 2, r.y + 8);
            break;
          }
          len--;
        }
      }
      char rangeBuf[24];
      getSequenceFreqRange(idx, rangeBuf, sizeof(rangeBuf));
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_PANEL);
      tft.drawString(rangeBuf, r.x + r.w / 2, r.y + 32); // always short (e.g. "10 -> 2 Hz"), never needs truncation
      tft.setTextDatum(TL_DATUM);
    }
  }
  drawButton(btnSeqPrevPage, "< Prev");
  drawButton(btnSeqBack, "Back", COLOR_MUTED);
  drawButton(btnSeqNextPage, "Next >");
}

void handleSequencesTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  int start = sequencesPage * SEQUENCES_PER_PAGE;
  for (int i = 0; i < SEQUENCES_PER_PAGE; i++) {
    int idx = start + i;
    if (idx < NUM_SEQUENCES && touchInRect(x, y, sequenceItemRects[i])) {
      const Program& p = SEQUENCES[idx];
      selName = p.name;
      selFreq = p.steps[0].freqHz;
      selWave = p.steps[0].wave;
      selCategoryName = "Harmonics";
      selectedIndex = -1;
      pendingSequenceIndex = idx;
      runScreenOrigin = SCR_SEQUENCES;
      // Match the session timer to this specific sequence's own total
      // duration (the researched/agreed step timings already built into
      // it), rather than leaving whatever generic timer value was last
      // set - a sequence should auto-stop right when it naturally
      // finishes, not get cut short or keep running past its own end.
      int totalSec = 0;
      for (int s = 0; s < p.stepCount; s++) totalSec += p.steps[s].durationSec;
      int totalMin = (totalSec + 30) / 60; // round to nearest minute
      if (totalMin < 1) totalMin = 1;
      if (totalMin > 60) totalMin = 60; // hard cap, matches the stepper's own max
      timerMinutes = totalMin;
      screen = isWellnessCenter ? SCR_CLIENT_CONFIRM : SCR_RUN;
      return;
    }
  }
  if (touchInRect(x, y, btnSeqPrevPage)) {
    if (sequencesPage > 0) sequencesPage--;
    screen = SCR_SEQUENCES;
  } else if (touchInRect(x, y, btnSeqNextPage)) {
    if ((sequencesPage + 1) * SEQUENCES_PER_PAGE < NUM_SEQUENCES) sequencesPage++;
    screen = SCR_SEQUENCES;
  } else if (touchInRect(x, y, btnSeqBack)) {
    screen = SCR_CATEGORY;
  }
}

// ---------------------------------------------------------------------
// Screen: Soundscapes (SD card only) - browse and toggle ambient sound
// playback. Uses the same Bluetooth audio output as the tone-sync
// feature (mutually exclusive with it - only one plays at a time).
// ---------------------------------------------------------------------
int soundscapesPage = 0;
const int SOUNDSCAPES_PER_PAGE = 6;
Rect soundscapeItemRects[SOUNDSCAPES_PER_PAGE];
Rect btnSndPrevPage = {20, 216, 140, 46};
Rect btnSndBack = {180, 216, 120, 46};
Rect btnSndNextPage = {320, 216, 140, 46};
int nowPlayingSoundscapeIndex = -1;

// ---------------------------------------------------------------------
// Still Point - hold your finger as steady as possible; the calmer the
// touch, the more a ripple grows. No losing state, no timer. Polls
// touch independently here rather than through the normal tap-dispatch
// pattern, since this needs continuous position while held, not just a
// discrete tap.
// ---------------------------------------------------------------------
bool stillHeld = false;
int stillLastX = 0, stillLastY = 0;
float stillSteadiness = 0; // 0..1 - grows while still, decays when it moves
float stillLastRadius = -1;

void drawGameStillScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Still Point", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Hold your finger as still as you can", 20, 40);
  stillHeld = false;
  stillSteadiness = 0;
  stillLastRadius = -1;
}

void handleGameStillTouch(int x, int y) {
  handleHomeTouch(x, y); // the steadiness tracking itself happens in updateGameStill(), not here
}

void updateGameStill() {
  uint16_t tx, ty;
  bool touched = tft.getTouch(&tx, &ty);
  if (touched && ty > 60) {
    if (stillHeld) {
      int dx = (int)tx - stillLastX, dy = (int)ty - stillLastY;
      int distSq = dx * dx + dy * dy;
      if (distSq < 25) { // moved less than ~5px since last frame - counts as steady
        stillSteadiness += 0.02f;
        if (stillSteadiness > 1.0f) stillSteadiness = 1.0f;
      } else {
        stillSteadiness -= 0.05f;
        if (stillSteadiness < 0) stillSteadiness = 0;
      }
    }
    stillHeld = true;
    stillLastX = tx;
    stillLastY = ty;

    float radius = 20 + stillSteadiness * 80;
    if (stillLastRadius >= 0) {
      tft.drawCircle(stillLastX, stillLastY, (int)stillLastRadius, COLOR_BG);
    }
    tft.drawCircle(tx, ty, (int)radius, COLOR_ACCENT);
    stillLastRadius = radius;
  } else if (stillHeld) {
    if (stillLastRadius >= 0) {
      tft.drawCircle(stillLastX, stillLastY, (int)stillLastRadius, COLOR_BG);
    }
    stillHeld = false;
    stillSteadiness = 0;
    stillLastRadius = -1;
  }
}

// ---------------------------------------------------------------------
// Color Flow - colored dots drift across the screen; tap them in the
// right color order. No rush, no fail state - a wrong-color tap is
// simply ignored rather than penalized.
// ---------------------------------------------------------------------
struct FlowDot {
  float x, y, vx;
  uint16_t color;
  bool active;
};
static const int MAX_FLOW_DOTS = 6;
FlowDot flowDots[MAX_FLOW_DOTS];
uint16_t flowPalette[3];
int flowNextExpected = 0;
unsigned long flowLastSpawn = 0;

void drawGameColorFlowScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Color Flow", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap the dots in order: red, blue, green", 20, 40);

  flowPalette[0] = tft.color565(220, 70, 70);
  flowPalette[1] = tft.color565(70, 140, 220);
  flowPalette[2] = tft.color565(90, 200, 120);
  for (int i = 0; i < MAX_FLOW_DOTS; i++) flowDots[i].active = false;
  flowNextExpected = 0;
  flowLastSpawn = millis();
}

void spawnFlowDot() {
  for (int i = 0; i < MAX_FLOW_DOTS; i++) {
    if (!flowDots[i].active) {
      flowDots[i].x = -10;
      flowDots[i].y = 70 + random(0, 180);
      flowDots[i].vx = 0.6f + (random(0, 40) / 100.0f);
      flowDots[i].color = flowPalette[random(0, 3)];
      flowDots[i].active = true;
      return;
    }
  }
}

void handleGameColorFlowTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  for (int i = 0; i < MAX_FLOW_DOTS; i++) {
    if (!flowDots[i].active) continue;
    int dx = x - (int)flowDots[i].x, dy = y - (int)flowDots[i].y;
    if (dx * dx + dy * dy < 400) { // within ~20px
      if (flowDots[i].color == flowPalette[flowNextExpected]) {
        tft.fillCircle((int)flowDots[i].x, (int)flowDots[i].y, 14, COLOR_BG);
        flowDots[i].active = false;
        flowNextExpected = (flowNextExpected + 1) % 3;
      }
      return; // wrong-color tap: ignored, no penalty
    }
  }
}

void updateGameColorFlow() {
  if (millis() - flowLastSpawn > 1500) {
    spawnFlowDot();
    flowLastSpawn = millis();
  }
  for (int i = 0; i < MAX_FLOW_DOTS; i++) {
    if (!flowDots[i].active) continue;
    tft.fillCircle((int)flowDots[i].x, (int)flowDots[i].y, 14, COLOR_BG);
    flowDots[i].x += flowDots[i].vx;
    if (flowDots[i].x > 500) {
      flowDots[i].active = false;
      continue;
    }
    tft.fillCircle((int)flowDots[i].x, (int)flowDots[i].y, 14, flowDots[i].color);
  }
}

// ---------------------------------------------------------------------
// Pulse Match - a gentle pulse appears at a calm, steady tempo; tap
// along with it. Forgiving timing window, and a missed beat just resets
// the streak count rather than showing any kind of fail state.
// ---------------------------------------------------------------------
unsigned long pulseLastBeat = 0;
static const unsigned long PULSE_INTERVAL_MS = 1500; // 40 BPM
int pulseStreak = 0;
bool pulseFlash = false;
unsigned long pulseFlashStart = 0;
bool pulseWasFlashing = false;

void drawGamePulseScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Pulse Match", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap anywhere in time with the pulse", 20, 40);
  tft.drawCircle(240, 170, 50, COLOR_ACCENT);
  pulseLastBeat = millis();
  pulseStreak = 0;
  pulseFlash = false;
  pulseWasFlashing = false;
}

void handleGamePulseTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (y < 60) return;
  unsigned long sinceBeat = millis() - pulseLastBeat;
  unsigned long distToBeat = sinceBeat < (PULSE_INTERVAL_MS / 2) ? sinceBeat : (PULSE_INTERVAL_MS - sinceBeat);
  if (distToBeat < 300) { // forgiving +-300ms window
    pulseStreak++;
  } else {
    pulseStreak = 0; // gentle reset - just starts the streak count over, no fail screen
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "Streak: %d", pulseStreak);
  tft.fillRect(20, 260, 200, 24, COLOR_BG);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_ACCENT, COLOR_BG);
  tft.drawString(buf, 20, 260);
}

void updateGamePulse() {
  unsigned long elapsed = millis() - pulseLastBeat;
  if (elapsed >= PULSE_INTERVAL_MS) {
    pulseLastBeat = millis();
    pulseFlash = true;
    pulseFlashStart = millis();
  }
  bool shouldFlash = pulseFlash && (millis() - pulseFlashStart < 200);
  if (shouldFlash != pulseWasFlashing) {
    tft.fillCircle(240, 170, 50, shouldFlash ? COLOR_ACCENT : COLOR_BG);
    if (!shouldFlash) tft.drawCircle(240, 170, 50, COLOR_ACCENT); // resting outline when not flashing
    pulseWasFlashing = shouldFlash;
  }
  if (pulseFlash && millis() - pulseFlashStart >= 200) pulseFlash = false;
}

// ---------------------------------------------------------------------
// Zen Garden - tap to place a small bloom; they accumulate into a
// pattern over the session rather than fading like the Ripple engine's
// shapes. No losing, no timer - purely additive, meditative play.
// ---------------------------------------------------------------------
struct ZenBloom { int x, y; uint16_t color; bool active; };
static const int MAX_ZEN_BLOOMS = 40;
ZenBloom zenBlooms[MAX_ZEN_BLOOMS];
int zenNextSlot = 0;
uint16_t zenPetalColors[4];

void drawGameZenScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Zen Garden", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap to grow a garden", 20, 40);
  zenPetalColors[0] = tft.color565(230, 180, 200);
  zenPetalColors[1] = tft.color565(200, 210, 160);
  zenPetalColors[2] = tft.color565(180, 200, 230);
  zenPetalColors[3] = tft.color565(230, 200, 160);
  for (int i = 0; i < MAX_ZEN_BLOOMS; i++) zenBlooms[i].active = false;
  zenNextSlot = 0;
}

void handleGameZenTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (y < 60) return;
  int slot = zenNextSlot;
  zenBlooms[slot].x = x;
  zenBlooms[slot].y = y;
  zenBlooms[slot].color = zenPetalColors[slot % 4];
  zenBlooms[slot].active = true;
  tft.fillCircle(x, y, 8, zenBlooms[slot].color);
  tft.drawCircle(x, y, 8, COLOR_ACCENT);
  zenNextSlot = (zenNextSlot + 1) % MAX_ZEN_BLOOMS; // wraps - the garden itself is never erased, this just bounds the tracking array
}

// ---------------------------------------------------------------------
// Bubble Pop - bubbles rise slowly; tap to pop them. No timer, no fail
// state - a bubble that reaches the top just quietly disappears.
// ---------------------------------------------------------------------
struct Bubble { float x, y, vy, r; bool active; };
static const int MAX_BUBBLES = 8;
Bubble bubbles[MAX_BUBBLES];
unsigned long bubbleLastSpawn = 0;

void drawGameBubbleScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Bubble Pop", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap the bubbles as they rise", 20, 40);
  for (int i = 0; i < MAX_BUBBLES; i++) bubbles[i].active = false;
  bubbleLastSpawn = millis();
}

void spawnBubble() {
  for (int i = 0; i < MAX_BUBBLES; i++) {
    if (!bubbles[i].active) {
      bubbles[i].x = 40 + random(0, 400);
      bubbles[i].y = 320;
      bubbles[i].vy = 0.4f + (random(0, 40) / 100.0f);
      bubbles[i].r = 12 + random(0, 10);
      bubbles[i].active = true;
      return;
    }
  }
}

void handleGameBubbleTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  for (int i = 0; i < MAX_BUBBLES; i++) {
    if (!bubbles[i].active) continue;
    float dx = x - bubbles[i].x, dy = y - bubbles[i].y;
    if (dx * dx + dy * dy < bubbles[i].r * bubbles[i].r * 2.5f) {
      tft.fillCircle((int)bubbles[i].x, (int)bubbles[i].y, (int)bubbles[i].r + 1, COLOR_BG);
      bubbles[i].active = false;
      return;
    }
  }
}

void updateGameBubbles() {
  if (millis() - bubbleLastSpawn > 1200) {
    spawnBubble();
    bubbleLastSpawn = millis();
  }
  for (int i = 0; i < MAX_BUBBLES; i++) {
    if (!bubbles[i].active) continue;
    tft.fillCircle((int)bubbles[i].x, (int)bubbles[i].y, (int)bubbles[i].r + 1, COLOR_BG);
    bubbles[i].y -= bubbles[i].vy;
    if (bubbles[i].y < 60) {
      bubbles[i].active = false;
      continue;
    }
    tft.drawCircle((int)bubbles[i].x, (int)bubbles[i].y, (int)bubbles[i].r, COLOR_ACCENT);
  }
}

// ---------------------------------------------------------------------
// Match Two - classic memory-matching, deliberately untimed with no
// penalty for a wrong guess - mismatched tiles just flip back after a
// short pause.
// ---------------------------------------------------------------------
static const int MATCH_TILE_COUNT = 12; // 6 pairs
int matchValues[MATCH_TILE_COUNT];
bool matchRevealed[MATCH_TILE_COUNT];
bool matchMatched[MATCH_TILE_COUNT];
int matchFirstPick = -1, matchSecondPick = -1;
unsigned long matchMismatchShownAt = 0;
Rect matchTileRects[MATCH_TILE_COUNT];
uint16_t matchTileColors[6];

void drawMatchTile(int i) {
  Rect r = matchTileRects[i];
  int radius = min(r.w, r.h) / 2;
  if (matchMatched[i]) {
    tft.fillRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_BG);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, radius, matchTileColors[matchValues[i]]);
  } else if (matchRevealed[i]) {
    tft.fillRoundRect(r.x, r.y, r.w, r.h, radius, matchTileColors[matchValues[i]]);
  } else {
    tft.fillRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, radius, COLOR_ACCENT);
  }
}

void drawGameMatchScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Match Two", 20, 8);
  drawHomeButton();

  matchTileColors[0] = tft.color565(220, 90, 90);
  matchTileColors[1] = tft.color565(90, 170, 220);
  matchTileColors[2] = tft.color565(100, 200, 130);
  matchTileColors[3] = tft.color565(230, 190, 90);
  matchTileColors[4] = tft.color565(190, 120, 220);
  matchTileColors[5] = tft.color565(240, 150, 90);

  int vals[MATCH_TILE_COUNT];
  for (int i = 0; i < 6; i++) { vals[i * 2] = i; vals[i * 2 + 1] = i; }
  for (int i = MATCH_TILE_COUNT - 1; i > 0; i--) {
    int j = random(0, i + 1);
    int tmp = vals[i]; vals[i] = vals[j]; vals[j] = tmp;
  }
  for (int i = 0; i < MATCH_TILE_COUNT; i++) {
    matchValues[i] = vals[i];
    matchRevealed[i] = false;
    matchMatched[i] = false;
  }
  matchFirstPick = -1;
  matchSecondPick = -1;
  matchMismatchShownAt = 0;

  int cols = 4, rows = 3, tileW = 100, tileH = 70, gapX = 10, gapY = 10;
  int gridW = cols * tileW + (cols - 1) * gapX;
  int startX = (480 - gridW) / 2;
  for (int i = 0; i < MATCH_TILE_COUNT; i++) {
    int col = i % cols, row = i / cols;
    matchTileRects[i].x = startX + col * (tileW + gapX);
    matchTileRects[i].y = 50 + row * (tileH + gapY);
    matchTileRects[i].w = tileW;
    matchTileRects[i].h = tileH;
    drawMatchTile(i);
  }
}

void handleGameMatchTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (matchSecondPick >= 0) return; // waiting for the mismatch pause - see updateGameMatch()
  for (int i = 0; i < MATCH_TILE_COUNT; i++) {
    if (matchMatched[i] || matchRevealed[i]) continue;
    if (touchInRect(x, y, matchTileRects[i])) {
      matchRevealed[i] = true;
      drawMatchTile(i);
      if (matchFirstPick < 0) {
        matchFirstPick = i;
      } else {
        matchSecondPick = i;
        if (matchValues[matchFirstPick] == matchValues[matchSecondPick]) {
          matchMatched[matchFirstPick] = true;
          matchMatched[matchSecondPick] = true;
          drawMatchTile(matchFirstPick);
          drawMatchTile(matchSecondPick);
          matchFirstPick = -1;
          matchSecondPick = -1;
        } else {
          matchMismatchShownAt = millis();
        }
      }
      return;
    }
  }
}

void updateGameMatch() {
  if (matchSecondPick >= 0 && millis() - matchMismatchShownAt > 800) {
    matchRevealed[matchFirstPick] = false;
    matchRevealed[matchSecondPick] = false;
    drawMatchTile(matchFirstPick);
    drawMatchTile(matchSecondPick);
    matchFirstPick = -1;
    matchSecondPick = -1;
  }
}

// ---------------------------------------------------------------------
// Sand Draw - drag your finger to draw a flowing line that fades away
// after a few seconds, like drawing in sand. Pure sensory, creative
// play - polls touch continuously here, same pattern as Still Point.
// ---------------------------------------------------------------------
struct SandPoint { int x, y; unsigned long t; bool valid; };
static const int MAX_SAND_POINTS = 150;
SandPoint sandPoints[MAX_SAND_POINTS];
int sandHead = 0;
int sandTail = 0;
bool sandWasDown = false;
static const unsigned long SAND_FADE_MS = 3000;

void drawGameSandScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Sand Draw", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Drag to draw - it fades like sand", 20, 40);
  for (int i = 0; i < MAX_SAND_POINTS; i++) sandPoints[i].valid = false;
  sandHead = 0;
  sandTail = 0;
  sandWasDown = false;
}

void handleGameSandTouch(int x, int y) {
  handleHomeTouch(x, y); // the drawing itself happens in updateGameSand() below
}

void updateGameSand() {
  uint16_t tx, ty;
  bool touched = tft.getTouch(&tx, &ty);
  if (touched && ty > 60) {
    int prevIdx = (sandHead - 1 + MAX_SAND_POINTS) % MAX_SAND_POINTS;
    if (sandWasDown && sandPoints[prevIdx].valid) {
      tft.drawLine(sandPoints[prevIdx].x, sandPoints[prevIdx].y, tx, ty, COLOR_ACCENT);
    }
    sandPoints[sandHead].x = tx;
    sandPoints[sandHead].y = ty;
    sandPoints[sandHead].t = millis();
    sandPoints[sandHead].valid = true;
    sandHead = (sandHead + 1) % MAX_SAND_POINTS;
    if (sandHead == sandTail) sandTail = (sandTail + 1) % MAX_SAND_POINTS;
    sandWasDown = true;
  } else {
    sandWasDown = false;
  }

  while (sandTail != sandHead && sandPoints[sandTail].valid && millis() - sandPoints[sandTail].t > SAND_FADE_MS) {
    int nextIdx = (sandTail + 1) % MAX_SAND_POINTS;
    if (sandPoints[nextIdx].valid) {
      tft.drawLine(sandPoints[sandTail].x, sandPoints[sandTail].y, sandPoints[nextIdx].x, sandPoints[nextIdx].y, COLOR_BG);
    }
    sandPoints[sandTail].valid = false;
    sandTail = nextIdx;
  }
}

// ---------------------------------------------------------------------
// Petal Count - a quieter companion to Breath Bubble: each full paced
// breath cycle adds one petal to a slowly growing arrangement, with no
// tapping required - just breathe along and watch it grow. Shares the
// same phase length as Breath Bubble (BREATH_PHASE_MS, declared here
// since this code sits earlier in the file than Breath Bubble's own
// block - both use it).
// ---------------------------------------------------------------------
static const unsigned long BREATH_PHASE_MS = 4000;
unsigned long petalPhaseStart = 0;
int petalPhase = 0;
int petalCount = 0;
uint16_t petalColor;

void drawPetalCountLabel() {
  char buf[24];
  snprintf(buf, sizeof(buf), "Petals grown: %d", petalCount);
  tft.fillRect(0, 60, 480, 24, COLOR_BG);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(buf, 240, 66);
  tft.setTextDatum(TL_DATUM);
}

void drawGamePetalScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Petal Count", 20, 8);
  drawHomeButton();
  petalColor = tft.color565(230, 180, 200);
  petalPhaseStart = millis();
  petalPhase = 0;
  petalCount = 0;
  drawPetalCountLabel();
}

void handleGamePetalTouch(int x, int y) {
  handleHomeTouch(x, y); // purely automatic otherwise - no tapping needed
}

void updateGamePetal() {
  unsigned long elapsed = millis() - petalPhaseStart;
  if (elapsed >= BREATH_PHASE_MS) {
    petalPhase++;
    petalPhaseStart = millis();
    if (petalPhase >= 4) {
      petalPhase = 0;
      // one full breath cycle completed - place a new petal in a
      // slowly-growing ring pattern
      float angle = (petalCount % 12) * (2 * PI / 12.0f);
      int ring = petalCount / 12;
      float r = 40 + ring * 25;
      int px = 240 + (int)(cos(angle) * r);
      int py = 200 + (int)(sin(angle) * r);
      if (px > 20 && px < 460 && py > 90 && py < 300) {
        tft.fillCircle(px, py, 8, petalColor);
        tft.drawCircle(px, py, 8, COLOR_ACCENT);
        petalCount++;
        drawPetalCountLabel();
      }
    }
  }
}

// ---------------------------------------------------------------------
// Games menu - picks which simple game to play. All games share the
// same exit pattern (Home button) and, where relevant, the Ripple
// engine's animated-shape primitives below.
// ---------------------------------------------------------------------
const char* GAME_NAMES[10] = {
  "Ripple Garden", "Breath Bubble", "Still Point", "Color Flow", "Pulse Match",
  "Zen Garden", "Bubble Pop", "Match Two", "Sand Draw", "Petal Count"
};
const Screen GAME_SCREENS[10] = {
  SCR_GAME_RIPPLE, SCR_GAME_BREATH, SCR_GAME_STILL, SCR_GAME_COLORFLOW, SCR_GAME_PULSE,
  SCR_GAME_ZEN, SCR_GAME_BUBBLE, SCR_GAME_MATCH, SCR_GAME_SAND, SCR_GAME_PETAL
};
Rect gameMenuItemRects[6];
int gamesMenuPage = 0;
const int GAMES_PER_PAGE = 6;

// ---------------------------------------------------------------------
// Breath Bubble - paced box-breathing exercise (inhale/hold/exhale/hold,
// 4 seconds each phase). No scoring, no failure - purely a visual pace
// to follow, the circle growing and shrinking with the breath.
// ---------------------------------------------------------------------
unsigned long breathPhaseStart = 0;
int breathPhase = 0; // 0=inhale, 1=hold, 2=exhale, 3=hold
float breathLastRadius = -1;
const char* BREATH_PHASE_LABELS[4] = {"Breathe in...", "Hold...", "Breathe out...", "Hold..."};

void drawBreathLabel() {
  tft.fillRect(0, 60, 480, 30, COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(BREATH_PHASE_LABELS[breathPhase], 240, 70);
  tft.setTextDatum(TL_DATUM);
}

void drawGameBreathScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Breath Bubble", 20, 8);
  drawHomeButton();
  breathPhaseStart = millis();
  breathPhase = 0;
  breathLastRadius = -1;
  drawBreathLabel();
}

void handleGameBreathTouch(int x, int y) {
  handleHomeTouch(x, y);
}

void updateGameBreath() {
  unsigned long elapsed = millis() - breathPhaseStart;
  if (elapsed >= BREATH_PHASE_MS) {
    breathPhase = (breathPhase + 1) % 4;
    breathPhaseStart = millis();
    elapsed = 0;
    drawBreathLabel();
  }
  float t = (float)elapsed / (float)BREATH_PHASE_MS; // 0..1 within this phase
  float minR = 40, maxR = 100, radius;
  if (breathPhase == 0) radius = minR + (maxR - minR) * t;      // inhale: grow
  else if (breathPhase == 1) radius = maxR;                     // hold big
  else if (breathPhase == 2) radius = maxR - (maxR - minR) * t; // exhale: shrink
  else radius = minR;                                            // hold small

  if (breathLastRadius >= 0) {
    tft.drawCircle(240, 210, (int)breathLastRadius, COLOR_BG);
    tft.drawCircle(240, 210, (int)breathLastRadius - 1, COLOR_BG);
  }
  tft.drawCircle(240, 210, (int)radius, COLOR_ACCENT);
  tft.drawCircle(240, 210, (int)radius - 1, COLOR_ACCENT);
  breathLastRadius = radius;
}


void drawGamesMenuScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Games", 20, 8);
  drawHomeButton();

  int start = gamesMenuPage * GAMES_PER_PAGE;
  int colW = 220, rowH = 44, gapX = 20, gapY = 8;
  for (int i = 0; i < GAMES_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 48 + row * (rowH + gapY), colW, rowH};
    gameMenuItemRects[i] = r;
    if (idx < 10) drawButtonFast(r, GAME_NAMES[idx]);
  }
  Rect btnGamesPrev = {20, 216, 140, 46};
  Rect btnGamesBack = {180, 216, 120, 46};
  Rect btnGamesNext = {320, 216, 140, 46};
  drawButton(btnGamesPrev, "< Prev");
  drawButton(btnGamesBack, "Back", COLOR_MUTED);
  drawButton(btnGamesNext, "Next >");
}

void handleGamesMenuTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  int start = gamesMenuPage * GAMES_PER_PAGE;
  for (int i = 0; i < GAMES_PER_PAGE; i++) {
    int idx = start + i;
    if (idx < 10 && touchInRect(x, y, gameMenuItemRects[i])) {
      screen = GAME_SCREENS[idx];
      return;
    }
  }
  Rect btnGamesPrev = {20, 216, 140, 46};
  Rect btnGamesBack = {180, 216, 120, 46};
  Rect btnGamesNext = {320, 216, 140, 46};
  if (touchInRect(x, y, btnGamesPrev)) {
    if (gamesMenuPage > 0) gamesMenuPage--;
    screen = SCR_GAMES_MENU;
  } else if (touchInRect(x, y, btnGamesNext)) {
    if ((gamesMenuPage + 1) * GAMES_PER_PAGE < 10) gamesMenuPage++;
    screen = SCR_GAMES_MENU;
  } else if (touchInRect(x, y, btnGamesBack)) {
    screen = SCR_SETTINGS;
  }
}


// it: spawn something at a touch point, grow/fade it over time, expire
// it. A bubble popping, a petal blooming, or a fading drawn line are all
// the same underlying pattern with a different shape and trigger - this
// is meant as the foundation the other simple games build on, not a
// one-off.
// ---------------------------------------------------------------------
struct Ripple {
  int x, y;
  float radius;
  bool active;
};

static const int MAX_RIPPLES = 8;
Ripple ripples[MAX_RIPPLES];
static const float RIPPLE_MAX_RADIUS = 60.0f;
static const float RIPPLE_GROWTH_PER_FRAME = 1.5f;

// Linear blend between two RGB565 colors, channel by channel - used to
// fade a ripple's ring color toward the background as it grows, since
// this display has no real alpha transparency to draw with.
uint16_t blendColor565(uint16_t c1, uint16_t c2, float t) {
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  uint8_t r1 = (c1 >> 11) & 0x1F, g1 = (c1 >> 5) & 0x3F, b1 = c1 & 0x1F;
  uint8_t r2 = (c2 >> 11) & 0x1F, g2 = (c2 >> 5) & 0x3F, b2 = c2 & 0x1F;
  uint8_t r = r1 + (uint8_t)((r2 - r1) * t);
  uint8_t g = g1 + (uint8_t)((g2 - g1) * t);
  uint8_t b = b1 + (uint8_t)((b2 - b1) * t);
  return (r << 11) | (g << 5) | b;
}

void spawnRipple(int x, int y) {
  // Reuse the first free slot; if every slot is already active, steal
  // the first one rather than silently dropping the new tap.
  for (int i = 0; i < MAX_RIPPLES; i++) {
    if (!ripples[i].active) {
      ripples[i].x = x;
      ripples[i].y = y;
      ripples[i].radius = 0;
      ripples[i].active = true;
      return;
    }
  }
  ripples[0].x = x;
  ripples[0].y = y;
  ripples[0].radius = 0;
  ripples[0].active = true;
}

void drawGameRippleScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Ripple Garden", 20, 8);
  drawHomeButton();
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap anywhere to make ripples", 20, 40);
  for (int i = 0; i < MAX_RIPPLES; i++) ripples[i].active = false; // fresh start each time this screen is entered
}

void handleGameRippleTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (y < 60) return; // keep the title/instructions area clear of ripples
  spawnRipple(x, y);
}

// Called every loop() iteration while this screen is active. Erases
// each ripple's ring at its current radius before redrawing it at the
// new, larger radius - touching only those specific pixels rather than
// clearing the whole screen, so the animation is smooth instead of
// flashing (the same lesson learned from the countdown-timer flash
// regression earlier).
void updateGameRipples() {
  for (int i = 0; i < MAX_RIPPLES; i++) {
    if (!ripples[i].active) continue;
    if (ripples[i].radius > 0) {
      tft.drawCircle(ripples[i].x, ripples[i].y, (int)ripples[i].radius, COLOR_BG);
    }
    ripples[i].radius += RIPPLE_GROWTH_PER_FRAME;
    if (ripples[i].radius >= RIPPLE_MAX_RADIUS) {
      ripples[i].active = false;
      continue;
    }
    float t = ripples[i].radius / RIPPLE_MAX_RADIUS;
    uint16_t ringColor = blendColor565(COLOR_ACCENT, COLOR_BG, t);
    tft.drawCircle(ripples[i].x, ripples[i].y, (int)ripples[i].radius, ringColor);
  }
}


void drawSoundscapesScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Soundscapes", 20, 8);
  drawHomeButton();

  int count = sdmedia_soundscapeCount();
  int totalSndPages = (count + SOUNDSCAPES_PER_PAGE - 1) / SOUNDSCAPES_PER_PAGE;
  if (totalSndPages > 1) {
    char pageBuf[20];
    snprintf(pageBuf, sizeof(pageBuf), "Page %d of %d", soundscapesPage + 1, totalSndPages);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pageBuf, 460, 16);
    tft.setTextDatum(TL_DATUM);
  }

  int start = soundscapesPage * SOUNDSCAPES_PER_PAGE;
  int colW = 220, rowH = 44, gapX = 20, gapY = 8;
  for (int i = 0; i < SOUNDSCAPES_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {20 + col * (colW + gapX), 48 + row * (rowH + gapY), colW, rowH};
    soundscapeItemRects[i] = r;
    if (idx < count) {
      bool playing = audio_isSoundscapePlaying() && nowPlayingSoundscapeIndex == idx;
      drawButtonFast(r, sdmedia_soundscapeName(idx), playing ? COLOR_GOOD : 0xFFFF, playing);
    }
  }
  drawButton(btnSndPrevPage, "< Prev");
  const char* sndBackLabel = (audio_isSoundscapePlaying() && soundscapesOrigin != SCR_RUN) ? "Stop & Back" : "Back";
  drawButton(btnSndBack, sndBackLabel, COLOR_MUTED);
  drawButton(btnSndNextPage, "Next >");
}

void handleSoundscapesTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    audio_stopSoundscape();
    audio_setEnabled(btAudioOn); // restore to whatever the Run screen's own toggle says, not left stuck on
    return;
  }
  int count = sdmedia_soundscapeCount();
  int start = soundscapesPage * SOUNDSCAPES_PER_PAGE;
  for (int i = 0; i < SOUNDSCAPES_PER_PAGE; i++) {
    int idx = start + i;
    if (idx >= count || !touchInRect(x, y, soundscapeItemRects[i])) continue;

    if (audio_isSoundscapePlaying() && nowPlayingSoundscapeIndex == idx) {
      audio_stopSoundscape();
      audio_setEnabled(btAudioOn); // restore to whatever the Run screen's own toggle says, not left stuck on
      nowPlayingSoundscapeIndex = -1;
      screen = SCR_SOUNDSCAPES;
      return;
    }
    audio_stopSoundscape();
    audio_startSoundscape(sdmedia_soundscapePath(idx));
    nowPlayingSoundscapeIndex = idx;
    if (!audio_isEnabled()) {
      if (audioUsingBluetooth()) {
        tft.fillScreen(COLOR_BG);
        tft.setFreeFont(FONT_LG);
        tft.setTextColor(TFT_WHITE, COLOR_BG);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Connecting to speaker...", 240, 140);
        tft.setFreeFont(FONT_SM);
        tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
        tft.drawString("This can take up to a minute.", 240, 175);
      }
      // The local wired speaker has no real "connecting" delay - just
      // enabling the amp - so no need for a wait message on that path.
      audio_setEnabled(true);
    }
    screen = SCR_SOUNDSCAPES;
    return;
  }
  if (touchInRect(x, y, btnSndPrevPage)) {
    if (soundscapesPage > 0) soundscapesPage--;
    screen = SCR_SOUNDSCAPES;
  } else if (touchInRect(x, y, btnSndNextPage)) {
    if ((soundscapesPage + 1) * SOUNDSCAPES_PER_PAGE < count) soundscapesPage++;
    screen = SCR_SOUNDSCAPES;
  } else if (touchInRect(x, y, btnSndBack)) {
    if (soundscapesOrigin != SCR_RUN) {
      // Only stop it when returning to standalone browsing (Settings) -
      // returning to an active session should keep it playing, since
      // picking one from mid-session is the whole point of this path.
      audio_stopSoundscape();
      audio_setEnabled(btAudioOn); // restore to whatever the Run screen's own toggle says, not left stuck on
    }
    screen = soundscapesOrigin;
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
  // Just the name, not "(XX Hz)" too - several of the current preset
  // names are long enough on their own that adding the frequency here
  // caused real truncation/cut-off text. Frequency shows clearly on the
  // very next screen (Run) right after tapping, so nothing is lost.
  snprintf(buf, bufLen, "%s", BASE_PRESETS[realIdx].name);
}

// 2-column x 3-row grid, same pattern as the Category screen.
void drawListScreen() {
  // Same background treatment as Category/Welcome/Run - this screen
  // only redraws on navigation/pagination, never continuously, so it's
  // just as safe as those.
  if (!sdmedia_showSplash()) {
    tft.fillScreen(COLOR_BG);
  }
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(viewingFavorites ? "Favorites" : CATEGORY_NAMES[currentCategory], 20, 8);
  drawHomeButton();

  int total = listCount();
  int totalPages = (total + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
  if (totalPages > 1) {
    // Next to the title, same placement as the Harmonics screen's page
    // indicator - avoids squeezing it into the tight gap near the nav
    // buttons, which was overlapping them before.
    char pageBuf[20];
    snprintf(pageBuf, sizeof(pageBuf), "Page %d of %d", listPage + 1, totalPages);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pageBuf, 460, 16);
    tft.setTextDatum(TL_DATUM);
  }

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
  pendingSequenceIndex = -1; // this is a normal preset pick, not a Sequence
  runScreenOrigin = SCR_LIST;
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
Rect btnStartStop   = {20, 100, 220, 44};
Rect btnFavToggle   = {145, 28, 95, 36};
Rect btnBtAudio     = {20, 152, 220, 44};
Rect btnBackFromRun = {20, 204, 220, 44};
Rect btnRunSoundscape = {20, 256, 220, 44};

Stepper powerStepper  = {"Power",  260, 48, 10, 1, 100, 1, formatPercentValue}; // y=48 clears the enlarged Home button (ends y=44)
Stepper timerStepper  = {"Session timer", 260, 130, 30, 0, 60, 5, formatTimerValue};
Stepper volumeStepper = {"BT Volume", 260, 220, 60, 0, 100, 5, formatPercentValue};

// Session auto-stop bookkeeping
unsigned long sessionStartMillis = 0;
bool sessionTimerArmed = false;

// Pause/Resume - the coil output actually stops while paused (same as a
// full stop), but sessionStartMillis and the active program's own step
// timing are preserved so resuming continues from where it left off
// rather than restarting. Every place that computes elapsed session time
// uses sessionEffectiveMillis() instead of raw millis(), so the
// countdown, auto-stop, and logged duration all correctly freeze while
// paused instead of continuing to advance in the background.
bool sessionPaused = false;
unsigned long pauseStartMillis = 0;
float pausedFreq = 0; // captured before waveform_stop() (which zeroes its own internal frequency), so Resume knows what to restart at

unsigned long sessionEffectiveMillis() {
  return sessionPaused ? pauseStartMillis : millis();
}

// ---------------------------------------------------------------------
// Program execution (Sequences: ramps and holds through multiple stages) - a
// multi-step sequence that runs on top of the same waveform engine used
// for single-frequency presets. Ramp interpolation is throttled to once
// per second, since this is a magnetic-field application, not audio -
// no need for anything faster, and it's much cheaper on the sine mode's
// table rebuild.
bool programActive = false;
// pendingSequenceIndex declared earlier (near the other mode flags) - see note there.
Program currentProgram;
int programStepIndex = 0;
unsigned long programStepStartMillis = 0;
float programStepStartFreq = 0;
unsigned long lastProgramUpdateMillis = 0;

void startProgram(const Program& prog) {
  currentProgram = prog;
  programStepIndex = 0;
  programActive = true;
  ProgramStep& s0 = currentProgram.steps[0];
  programStepStartFreq = s0.freqHz;
  waveform_start(s0.freqHz, s0.wave, actualIntensityPercent());
  programStepStartMillis = millis();
}

void stopProgram() {
  programActive = false;
}

void updateProgram() {
  if (!programActive || sessionPaused) return; // frozen entirely while paused - no step timing advances
  if (millis() - lastProgramUpdateMillis < 1000) return;
  lastProgramUpdateMillis = millis();

  ProgramStep& s = currentProgram.steps[programStepIndex];
  unsigned long elapsedMs = millis() - programStepStartMillis;
  unsigned long durMs = (unsigned long)s.durationSec * 1000UL;

  if (s.kind == STEP_RAMP) {
    float t = (float)elapsedMs / (float)durMs;
    if (t > 1.0f) t = 1.0f;
    float freqNow = programStepStartFreq + (s.freqHz - programStepStartFreq) * t;
    waveform_setFrequency(freqNow);
    audio_setTargetFrequency(freqNow); // audio tone follows along too, safely capped - see DIRECT_TONE_MAX_HZ in bt_audio.cpp
  }

  if (elapsedMs >= durMs) {
    programStepIndex++;
    if (programStepIndex >= currentProgram.stepCount) {
      stopProgram();
      waveform_stop();
      return;
    }
    ProgramStep& next = currentProgram.steps[programStepIndex];
    programStepStartFreq = waveform_currentFreq();
    if (next.kind == STEP_HOLD) {
      waveform_setFrequency(next.freqHz);
      audio_setTargetFrequency(next.freqHz);
    }
    programStepStartMillis = millis();
  }
}

void drawCountdownOnly() {
  // Small, targeted redraw of just the countdown text - clears only its
  // own area first, not the whole screen. Used for the once-a-second
  // live update so the display doesn't visibly flash every second (a
  // real regression from calling the full drawRunScreen() for this).
  tft.fillRect(18, 82, 200, 20, COLOR_BG);
  if (sessionTimerArmed && (waveform_isRunning() || sessionPaused)) {
    unsigned long elapsedSec = (sessionEffectiveMillis() - sessionStartMillis) / 1000UL;
    long remainingSec = (long)timerMinutes * 60 - (long)elapsedSec;
    if (remainingSec < 0) remainingSec = 0;
    char timeBuf[24];
    snprintf(timeBuf, sizeof(timeBuf), sessionPaused ? "Paused: %ld:%02ld" : "Time left: %ld:%02ld",
             remainingSec / 60, remainingSec % 60);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(timeBuf, 20, 88);
  }
}

// Two-column landscape layout: left column is info + Start/Stop + BT
// toggle + Back, right column is the three steppers.
void drawRunScreen() {
  // Same background treatment as Category/Welcome - falls back to solid
  // color with no SD card. Confirmed safe to add here too: the once-
  // per-second countdown update (drawCountdownOnly()) only ever touches
  // its own small area, never the whole screen, so this draws once on
  // entry and stays static rather than needing to redraw repeatedly.
  // Every text element here already specifies an opaque background
  // color when drawn, which gives each one its own solid backing
  // automatically - no separate backing rectangles needed the way the
  // Welcome screen's dense disclaimer block needed one.
  if (!sdmedia_showSplash()) {
    tft.fillScreen(COLOR_BG);
  }
  drawHomeButton();

  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  // Long preset names (several of the current general-wellness names run
  // longer than the originals did) can exceed FONT_LG's width here even
  // with drawFittedText's truncation, which read as text getting cut
  // off. Drop to FONT_SM instead for anything that doesn't fit at the
  // normal size - shows the full name rather than truncating it.
  const GFXfont* titleFont = (tft.textWidth(selName) <= 210) ? FONT_LG : FONT_SM;
  drawFittedText(20, 8, 210, selName, titleFont, TFT_WHITE, COLOR_BG);

  char buf[48];
  if (programActive) {
    snprintf(buf, sizeof(buf), "%.0f Hz (step %d/%d)", waveform_currentFreq(),
             programStepIndex + 1, currentProgram.stepCount);
  } else {
    snprintf(buf, sizeof(buf), "%.0f Hz", selFreq);
  }
  tft.drawString(buf, 20, 40);

  bool isFav = (selectedIndex >= 0) && favoriteBits[selectedIndex];
  drawButtonFast(btnFavToggle, isFav ? "* ON" : "* Fav", isFav ? COLOR_WARN : COLOR_MUTED);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  snprintf(buf, sizeof(buf), "%s - %s",
           selWave == WAVE_SQUARE ? "Square" : "Sine", selCategoryName);
  tft.drawString(buf, 20, 70);

  bool running = waveform_isRunning();

  if (sessionTimerArmed && (running || sessionPaused)) {
    unsigned long elapsedSec = (sessionEffectiveMillis() - sessionStartMillis) / 1000UL;
    long remainingSec = (long)timerMinutes * 60 - (long)elapsedSec;
    if (remainingSec < 0) remainingSec = 0;
    char timeBuf[24];
    snprintf(timeBuf, sizeof(timeBuf), sessionPaused ? "Paused: %ld:%02ld" : "Time left: %ld:%02ld",
             remainingSec / 60, remainingSec % 60);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    tft.drawString(timeBuf, 20, 88);
  }

  const char* startLabel = sessionPaused ? "RESUME"
                          : running ? "PAUSE"
                          : (pendingSequenceIndex >= 0 ? "START (Harmonics)" : "START");
  uint16_t startColor = sessionPaused ? COLOR_GOOD : running ? COLOR_WARN : COLOR_GOOD;
  drawButton(btnStartStop, startLabel, startColor);

  const char* audioBtnLabel = audioUsingBluetooth()
    ? (btAudioOn ? "BT Audio: ON" : "BT Audio: OFF")
    : (btAudioOn ? "Speaker Audio: ON" : "Speaker Audio: OFF");
  drawButton(btnBtAudio, audioBtnLabel, 0xFFFF, btAudioOn);
  drawButton(btnBackFromRun, "Stop & Back", COLOR_MUTED);

  if (sdmedia_isAvailable() && sdmedia_soundscapeCount() > 0) {
    char sndLabel[48];
    if (audio_isSoundscapePlaying()) {
      snprintf(sndLabel, sizeof(sndLabel), "Stop: %s", sdmedia_soundscapeName(nowPlayingSoundscapeIndex));
    } else {
      snprintf(sndLabel, sizeof(sndLabel), "Choose Soundscape");
    }
    drawButton(btnRunSoundscape, sndLabel, COLOR_MUTED, audio_isSoundscapePlaying());
  }

  powerStepper.value = powerDisplay;
  timerStepper.value = timerMinutes;
  volumeStepper.value = volumePercent;
  volumeStepper.label = audioUsingBluetooth() ? "BT Volume" : "Speaker Volume"; // was stuck on "BT Volume" even in speaker mode - real bug, now dynamic
  drawStepper(powerStepper);
  drawStepper(timerStepper);
  drawStepper(volumeStepper);
}

void handleRunTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    if (waveform_isRunning() || sessionPaused) {
      int mins = (int)((sessionEffectiveMillis() - sessionStartMillis) / 60000UL);
      addLogEntry(selName, selFreq, mins);
    }
    stopProgram();
    waveform_stop();
    sessionTimerArmed = false;
    sessionPaused = false;
    btAudioOn = false;
    audio_setEnabled(false);
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
    audio_setVolume((uint8_t)volumePercent);
    screen = SCR_RUN;
    return;
  }
  if (touchInRect(x, y, btnStartStop)) {
    if (sessionPaused) {
      // Resume - shift the reference timestamps forward by however long
      // we were paused, so elapsed-time math picks up right where it
      // left off instead of counting the paused time as active.
      unsigned long pauseDuration = millis() - pauseStartMillis;
      sessionStartMillis += pauseDuration;
      if (programActive) programStepStartMillis += pauseDuration;
      sessionPaused = false;
      waveform_start(pausedFreq, selWave, actualIntensityPercent());
    } else if (waveform_isRunning()) {
      // Pause - not a full stop. Halts the coil output but deliberately
      // leaves sessionStartMillis/programStepIndex/etc. untouched so
      // Resume can continue cleanly rather than restarting. Frequency
      // is captured first since waveform_stop() zeroes it internally.
      pausedFreq = waveform_currentFreq();
      sessionPaused = true;
      pauseStartMillis = millis();
      waveform_stop();
    } else if (pendingSequenceIndex >= 0) {
      startProgram(SEQUENCES[pendingSequenceIndex]);
      pendingSequenceIndex = -1; // one-shot
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    } else {
      waveform_start(selFreq, selWave, actualIntensityPercent());
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    screen = SCR_RUN;
  } else if (touchInRect(x, y, btnBtAudio)) {
    btAudioOn = !btAudioOn;
    if (btAudioOn && audioUsingBluetooth()) {
      // The actual connect call blocks for a while (can take 30-60+
      // seconds doing a fresh name scan) - show feedback immediately so
      // this doesn't look like a freeze while it works. The local wired
      // speaker has no equivalent delay, so no message needed there.
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Connecting to speaker...", 240, 140);
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
      tft.drawString("This can take up to a minute.", 240, 175);
    }
    audio_setTargetFrequency(selFreq);
    audio_setVolume((uint8_t)volumePercent);
    audio_setEnabled(btAudioOn);
    screen = SCR_RUN;
  } else if (sdmedia_isAvailable() && sdmedia_soundscapeCount() > 0 && touchInRect(x, y, btnRunSoundscape)) {
    if (audio_isSoundscapePlaying()) {
      // Direct stop, right from the session - no need to navigate back
      // to the Soundscapes picker just to turn it off again.
      audio_stopSoundscape();
      audio_setEnabled(btAudioOn); // restore to whatever the Run screen's own toggle says, not left stuck on
      nowPlayingSoundscapeIndex = -1;
      screen = SCR_RUN;
    } else {
      // Doesn't stop the session - the coil keeps running throughout,
      // this just navigates to pick a soundscape and comes right back.
      soundscapesOrigin = SCR_RUN;
      screen = SCR_SOUNDSCAPES;
    }
  } else if (touchInRect(x, y, btnBackFromRun)) {
    if (waveform_isRunning() || sessionPaused) {
      int mins = (int)((sessionEffectiveMillis() - sessionStartMillis) / 60000UL);
      addLogEntry(selName, selFreq, mins);
    }
    stopProgram();
    waveform_stop();
    sessionTimerArmed = false;
    sessionPaused = false;
    btAudioOn = false;
    audio_setEnabled(false);
    screen = runScreenOrigin;
  }
}

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------
Screen lastDrawnScreen = SCR_WELCOME;
int lastDrawnPage = -1;
int lastDrawnSequencesPage = -1;
int lastDrawnSoundscapesPage = -1;
int lastDrawnGamesMenuPage = -1;
int lastDrawnSettingsPage = -1;
bool lastDrawnShowDeviceStats = false;

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // backlight on

  tft.init();
  tft.setRotation(1); // landscape, 480x320. If USB ends up on the left
                       // instead of the right, change this to 3.
  initTheme();

  // ---- SD card (optional) - splash screen + soundscape file scan.
  // Both gracefully do nothing if there's no card, or the card has
  // neither file - nothing here can block or fail the rest of boot.
  bool sdOk = sdmedia_begin();
  Serial.print("SD card mount: ");
  Serial.println(sdOk ? "SUCCESS" : "FAILED");
  if (sdOk) {
    bool splashShown = sdmedia_showSplash();
    Serial.print("Splash image found: ");
    Serial.println(splashShown ? "YES" : "NO (no splash.bmp at card root, or card mounted but unreadable)");
    if (splashShown) delay(1800); // let it actually be seen before moving on
  }
  int soundscapeCount = sdmedia_scanSoundscapes();
  Serial.print("Soundscape files found: ");
  Serial.println(soundscapeCount);

  // ---- Touch calibration (tied to rotation; auto-invalidates if rotation
  // changes, then reused every boot after that) ----
  // A resistive touch panel's raw readings don't map directly to screen
  // pixels, and that mapping is specific to whichever rotation was active
  // when it was calibrated. Changing rotation (e.g. portrait->landscape)
  // without recalibrating causes exactly the "have to tap it repeatedly"
  // symptom - this stores which rotation the saved calibration belongs to
  // and automatically re-runs it if that doesn't match the current one,
  // rather than needing a manual flash erase.
  static const uint8_t CURRENT_ROTATION = 1;
  uint16_t calData[5];
  Preferences prefs;
  prefs.begin("tftcal", false);
  bool haveValidCal = false;
  if (prefs.isKey("calData") && prefs.isKey("calRotation")) {
    uint8_t savedRotation = prefs.getUChar("calRotation", 255);
    if (savedRotation == CURRENT_ROTATION) {
      prefs.getBytes("calData", calData, sizeof(calData));
      tft.setTouch(calData);
      haveValidCal = true;
    }
  }
  if (!haveValidCal) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Touch each corner as it appears", 240, 150, 2);
    tft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 15);
    prefs.putBytes("calData", calData, sizeof(calData));
    prefs.putUChar("calRotation", CURRENT_ROTATION);
  }
  prefs.end();

  waveform_begin();
  loadPeople();
  loadFavorites();
  loadSessionLog();
  loadSetupInfo();
  wifitime_begin(); // fast no-op if WiFi was never set up; brief reconnect+NTP resync otherwise
  btaudio_setSavedDeviceName(btDeviceName);
  btaudio_setHeadphonesMode(btHeadphonesMode);
  localaudio_begin(); // installs the I2S driver for the local wired speaker - always available regardless of Bluetooth config
  audio_setVolume((uint8_t)volumePercent);

#if ENABLE_BT_AUDIO
  btaudio_begin("PEMF-Headset");
  // Deliberately NOT pre-connecting to a saved speaker here at boot
  // anymore. That was tried, but real documented reliability issues
  // exist in this Bluetooth library's connection lifecycle (background
  // reconnect-retry behavior that doesn't always resolve cleanly) - and
  // if that ever caused setup() to stall, the device's touchscreen would
  // never become responsive at all, since the main loop (where touches
  // are actually read) never starts until setup() finishes. That's a
  // far worse outcome than the mid-session connect wait this was trying
  // to avoid. BT audio now connects on-demand, mid-session, same as the
  // proven-working design from before - see the Run screen's BT toggle.
#endif

  drawWelcomeScreen();
}

void loop() {
  uint16_t tx, ty;
  bool touched = tft.getTouch(&tx, &ty);

  // TEMPORARY diagnostic - prints every touch read to Serial so we can
  // see exactly what's happening when a tap doesn't seem to register.
  // Safe to leave in during troubleshooting; remove once this is sorted.
  static bool wasTouchedDebug = false;
  if (touched && !wasTouchedDebug) {
    Serial.print("TOUCH at raw x=");
    Serial.print(tx);
    Serial.print(" y=");
    Serial.print(ty);
    Serial.print("  screen=");
    Serial.println((int)screen);
  }
  wasTouchedDebug = touched;

  // Advance the WiFi setup portal (non-blocking) if it's active, and
  // move on once it finishes (connected or timed out) or was cancelled.
  if (screen == SCR_WIFI_SETUP && wifitime_isPortalActive()) {
    if (wifitime_processPortal()) {
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString(wifitime_isConfigured() ? "WiFi connected!" : "Setup timed out - try again.", 240, 160);
      delay(1500);
      screen = SCR_SETTINGS;
    }
  }

  // Animate the active game screen, where one needs continuous motion
  // rather than just redrawing on touch. Zen Garden is deliberately
  // absent here - it's purely tap-driven with no animation needed.
  if (screen == SCR_GAME_RIPPLE || screen == SCR_GAME_BREATH || screen == SCR_GAME_STILL ||
      screen == SCR_GAME_COLORFLOW || screen == SCR_GAME_PULSE || screen == SCR_GAME_BUBBLE ||
      screen == SCR_GAME_MATCH || screen == SCR_GAME_SAND || screen == SCR_GAME_PETAL) {
    static unsigned long lastGameFrame = 0;
    if (millis() - lastGameFrame >= 40) {
      switch (screen) {
        case SCR_GAME_RIPPLE:    updateGameRipples(); break;
        case SCR_GAME_BREATH:    updateGameBreath(); break;
        case SCR_GAME_STILL:     updateGameStill(); break;
        case SCR_GAME_COLORFLOW: updateGameColorFlow(); break;
        case SCR_GAME_PULSE:     updateGamePulse(); break;
        case SCR_GAME_BUBBLE:    updateGameBubbles(); break;
        case SCR_GAME_MATCH:     updateGameMatch(); break;
        case SCR_GAME_SAND:      updateGameSand(); break;
        case SCR_GAME_PETAL:     updateGamePetal(); break;
        default: break;
      }
      lastGameFrame = millis();
    }
  }

  // Feed the local speaker's I2S buffer, if that's the active output.
  // Runs every iteration regardless of which screen is showing, since
  // audio needs to keep playing in the background (e.g. a soundscape
  // playing while browsing Settings) - cheap to call when nothing is
  // actually enabled.
  if (!audioUsingBluetooth()) {
    localaudio_update();
  } else {
#if ENABLE_BT_AUDIO
    btaudio_update(); // finishes the volume-set once the connection is actually confirmed established
#endif
  }

  // Advance any active program (ramp interpolation, step transitions)
  updateProgram();

  // Auto-stop when the session timer elapses
  if (sessionTimerArmed && waveform_isRunning()) {
    unsigned long elapsedMin = (millis() - sessionStartMillis) / 60000UL;
    if ((int)elapsedMin >= timerMinutes) {
      addLogEntry(selName, selFreq, (int)elapsedMin);
      stopProgram();
      waveform_stop();
      sessionTimerArmed = false;
      if (screen == SCR_RUN) drawRunScreen();
    } else if (screen == SCR_RUN) {
      // Live-update just the countdown text once a second - drawCountdownOnly()
      // targets a small rect, avoiding the full-screen flash a complete
      // drawRunScreen() redraw was causing every single second.
      static unsigned long lastCountdownRedraw = 0;
      if (millis() - lastCountdownRedraw >= 1000) {
        drawCountdownOnly();
        lastCountdownRedraw = millis();
      }
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
  if (screen != lastDrawnScreen || (screen == SCR_LIST && listPage != lastDrawnPage) ||
      (screen == SCR_SEQUENCES && sequencesPage != lastDrawnSequencesPage) ||
      (screen == SCR_SOUNDSCAPES && soundscapesPage != lastDrawnSoundscapesPage) ||
      (screen == SCR_GAMES_MENU && gamesMenuPage != lastDrawnGamesMenuPage) ||
      (screen == SCR_CATEGORY && showDeviceStats != lastDrawnShowDeviceStats) ||
      (screen == SCR_SETTINGS && settingsPage != lastDrawnSettingsPage)) {
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
      case SCR_PERSON_PICKER:      drawPersonPickerScreen(); break;
      case SCR_MANAGE_PEOPLE:      drawManagePeopleScreen(); break;
      case SCR_SEQUENCES:          drawSequencesScreen(); break;
      case SCR_SOUNDSCAPES:        drawSoundscapesScreen(); break;
      case SCR_WIFI_SETUP:         drawWifiSetupScreen(); break;
      case SCR_GAME_RIPPLE:        drawGameRippleScreen(); break;
      case SCR_GAMES_MENU:         drawGamesMenuScreen(); break;
      case SCR_GAME_BREATH:        drawGameBreathScreen(); break;
      case SCR_GAME_STILL:         drawGameStillScreen(); break;
      case SCR_GAME_COLORFLOW:     drawGameColorFlowScreen(); break;
      case SCR_GAME_PULSE:         drawGamePulseScreen(); break;
      case SCR_GAME_ZEN:           drawGameZenScreen(); break;
      case SCR_GAME_BUBBLE:        drawGameBubbleScreen(); break;
      case SCR_GAME_MATCH:         drawGameMatchScreen(); break;
      case SCR_GAME_SAND:          drawGameSandScreen(); break;
      case SCR_GAME_PETAL:         drawGamePetalScreen(); break;
    }
    lastDrawnScreen = screen;
    lastDrawnPage = listPage;
    lastDrawnSequencesPage = sequencesPage;
    lastDrawnSoundscapesPage = soundscapesPage;
    lastDrawnGamesMenuPage = gamesMenuPage;
    lastDrawnShowDeviceStats = showDeviceStats;
    lastDrawnSettingsPage = settingsPage;
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
      case SCR_PERSON_PICKER:      handlePersonPickerTouch(tx, ty); break;
      case SCR_MANAGE_PEOPLE:      handleManagePeopleTouch(tx, ty); break;
      case SCR_SEQUENCES:          handleSequencesTouch(tx, ty); break;
      case SCR_SOUNDSCAPES:        handleSoundscapesTouch(tx, ty); break;
      case SCR_WIFI_SETUP:         handleWifiSetupTouch(tx, ty); break;
      case SCR_GAME_RIPPLE:        handleGameRippleTouch(tx, ty); break;
      case SCR_GAMES_MENU:         handleGamesMenuTouch(tx, ty); break;
      case SCR_GAME_BREATH:        handleGameBreathTouch(tx, ty); break;
      case SCR_GAME_STILL:         handleGameStillTouch(tx, ty); break;
      case SCR_GAME_COLORFLOW:     handleGameColorFlowTouch(tx, ty); break;
      case SCR_GAME_PULSE:         handleGamePulseTouch(tx, ty); break;
      case SCR_GAME_ZEN:           handleGameZenTouch(tx, ty); break;
      case SCR_GAME_BUBBLE:        handleGameBubbleTouch(tx, ty); break;
      case SCR_GAME_MATCH:         handleGameMatchTouch(tx, ty); break;
      case SCR_GAME_SAND:          handleGameSandTouch(tx, ty); break;
      case SCR_GAME_PETAL:         handleGamePetalTouch(tx, ty); break;
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
    if (screen == SCR_MANAGE_PEOPLE)  drawManagePeopleScreen();
    if (screen == SCR_SOUNDSCAPES)    drawSoundscapesScreen();
  }
  wasTouched = touched;

  // Live-refresh the BT scan screen while a scan is running, so newly
  // discovered devices appear without needing a touch - but only redraw
  // when something actually changed (a new device found, or the "Starting
  // scan..." -> "Scanning..." transition), not on a blind timer. A full-
  // screen redraw competing with an in-progress tap (e.g. tapping the
  // scan button to stop it) was a real, if intermittent, cause of taps
  // needing to be repeated on this screen.
  static int lastBtScanRedrawCount = -1;
  static bool lastBtScanActiveState = false;
  if (screen == SCR_BT_SCAN && (btaudio_isScanning() || scanRequested)) {
    bool nowActive = btaudio_isScanning();
    if (nowActive) scanRequested = false; // real scanning has started - drop the "Starting..." label
    int currentCount = btaudio_scanResultCount();
    if (currentCount != lastBtScanRedrawCount || nowActive != lastBtScanActiveState) {
      drawBtScanScreen();
      lastBtScanRedrawCount = currentCount;
      lastBtScanActiveState = nowActive;
    }
  } else {
    lastBtScanRedrawCount = -1; // reset so the next scan starts fresh
    lastBtScanActiveState = false;
  }

  delay(20); // simple debounce; touch is polled, not interrupt-driven
}

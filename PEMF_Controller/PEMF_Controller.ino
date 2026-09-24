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

const char* FIRMWARE_VERSION = "1.4.2"; // not static - ota_update.cpp reads this via extern. Bumped again from 1.1.0 for the local-audio write-failure fix - check this on Settings -> Check for Updates before reporting a symptom, so we know whether it's from this build or an earlier one.
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
uint16_t MADD_PANEL, MADD_EDGE, MADD_MAGENTA, MADD_CYAN, MADD_COIL2, MADD_TEXT, MADD_DIM;
uint16_t MADD_SPECTRUM[6]; // orange, coral, pink, magenta, violet, blue
static const int AURORA_BANDS = 16;
uint16_t AURORA[AURORA_BANDS];

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

  // MADD brand palette - from the hex badge and Tesla-coil art
  MADD_PANEL   = tft.color565(18, 10, 36);    // panel fill over the aurora
  MADD_EDGE    = tft.color565(74, 53, 112);   // quiet panel outline
  MADD_MAGENTA = tft.color565(232, 62, 156);  // badge neon
  MADD_CYAN    = tft.color565(63, 216, 255);  // coil rings
  MADD_COIL2   = tft.color565(91, 124, 255);
  MADD_TEXT    = tft.color565(244, 238, 255);
  MADD_DIM     = tft.color565(183, 169, 214);
  const uint8_t spec[6][3] = {{255,166,43},{255,107,69},{232,69,122},{181,60,201},{123,70,230},{47,139,255}};
  for (int i = 0; i < 6; i++) MADD_SPECTRUM[i] = tft.color565(spec[i][0], spec[i][1], spec[i][2]);
  // Aurora: deep indigo at the top blending to plum at the bottom
  for (int i = 0; i < AURORA_BANDS; i++) {
    float t = (float)i / (AURORA_BANDS - 1);
    AURORA[i] = tft.color565(13 + (int)(35 * t), 8 + (int)(3 * t), 38 + (int)(13 * t));
  }
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
int volumePercent = 80;      // audio volume (speaker or BT), independent of coil output

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
// Audio choices (engine lives in bt_audio.cpp and runs in its own task)
//   audioOutputPref  - Speaker or Bluetooth, chosen in Settings (saved)
//   sessionSoundMode - what plays during a session: Off / Tone / Soundscape
//                      (saved - it's the pre-selection for every session)
//   sessionSoundscapeIndex - which soundscape; auto-picked per session to
//                      suit its category, changeable from the Run screen
//   previewSoundscapeIndex - a soundscape being previewed from Settings
// ---------------------------------------------------------------------
AudioOutput audioOutputPref = AUDIO_OUT_SPEAKER;
AudioSource sessionSoundMode = AUDIO_SRC_TONE;
int sessionSoundscapeIndex = -1;
int previewSoundscapeIndex = -1;

bool audioUsingBluetooth() {
#if ENABLE_BT_AUDIO
  return audioOutputPref == AUDIO_OUT_BLUETOOTH && strlen(btDeviceName) > 0;
#else
  return false;
#endif
}

// Pushes the saved output choice into the engine (and starts the BT
// connection in the background when Bluetooth is chosen - non-blocking).
void applyAudioOutput() {
  if (audioUsingBluetooth()) {
    audio_setOutput(AUDIO_OUT_BLUETOOTH);
    audio_btConnect();
  } else {
    audio_setOutput(AUDIO_OUT_SPEAKER);
  }
}

// Plain-language Bluetooth status, so nobody is left wondering whether it works.
void btStatusText(char* buf, size_t len) {
  switch (audio_btStatus()) {
    case BT_STATUS_SEARCHING:    snprintf(buf, len, "Looking for %s...", btDeviceName); break;
    case BT_STATUS_CONNECTING:   snprintf(buf, len, "Connecting to %s...", btDeviceName); break;
    case BT_STATUS_CONNECTED:    snprintf(buf, len, "Connected: %s", btDeviceName); break;
    case BT_STATUS_RECONNECTING: snprintf(buf, len, "Reconnecting to %s...", btDeviceName); break;
    case BT_STATUS_NOT_FOUND:    snprintf(buf, len, "%s not found - tap to retry", btDeviceName); break;
    default:                     snprintf(buf, len, audioUsingBluetooth() ? "Bluetooth starting..." : "Sound: onboard speaker"); break;
  }
}

uint16_t btStatusColor() {
  switch (audio_btStatus()) {
    case BT_STATUS_CONNECTED: return COLOR_GOOD;
    case BT_STATUS_NOT_FOUND: return COLOR_WARN;
    default:                  return COLOR_ACCENT;
  }
}

// One status line, used on the Bluetooth screen.
void drawBtStatusLine(int x, int y, int w) {
  tft.fillRect(x - 2, y - 2, w + 4, 18, COLOR_BG);
  if (!audioUsingBluetooth()) {
    drawFittedText(x, y, w, "Sound plays on the onboard speaker", FONT_SM, COLOR_TEXT_DIM, COLOR_BG);
    return;
  }
  char buf[48];
  btStatusText(buf, sizeof(buf));
  drawFittedText(x, y, w, buf, FONT_SM, btStatusColor(), COLOR_BG);
}

// Case-insensitive "does name contain keyword"
bool nameHas(const char* name, const char* kw) {
  size_t n = strlen(name), k = strlen(kw);
  for (size_t i = 0; i + k <= n; i++) {
    size_t j = 0;
    while (j < k && tolower(name[i + j]) == tolower(kw[j])) j++;
    if (j == k) return true;
  }
  return false;
}

// Picks a soundscape that suits the session - matched by file name, so
// it works with whatever is on the card (rain, ocean_waves, forest,
// fireplace, thunder, city_ambience...). Falls back to the first file.
int defaultSoundscapeFor(const char* categoryName, float freqHz) {
  int n = sdmedia_soundscapeCount();
  if (n == 0) return -1;
  const char* sleepy[]   = {"rain", "ocean", "fire"};
  const char* focus[]    = {"forest", "fire", "rain"};
  const char* athletic[] = {"thunder", "forest", "ocean"};
  const char* body[]     = {"ocean", "rain", "forest"};
  const char* general[]  = {"forest", "ocean", "rain"};
  const char** prefs = general;
  if (nameHas(categoryName, "sleep") || nameHas(categoryName, "relax") || freqHz <= 4.0f) prefs = sleepy;
  else if (nameHas(categoryName, "focus")) prefs = focus;
  else if (nameHas(categoryName, "athletic")) prefs = athletic;
  else if (nameHas(categoryName, "body") || nameHas(categoryName, "skin")) prefs = body;
  for (int p = 0; p < 3; p++)
    for (int i = 0; i < n; i++)
      if (nameHas(sdmedia_soundscapeName(i), prefs[p])) return i;
  return 0;
}

// "ocean_waves" -> "Ocean Waves"
void prettySoundName(int idx, char* out, size_t outLen) {
  const char* raw = sdmedia_soundscapeName(idx);
  size_t i = 0;
  bool cap = true;
  for (; raw[i] && i < outLen - 1; i++) {
    char c = (raw[i] == '_' || raw[i] == '-') ? ' ' : raw[i];
    out[i] = cap ? toupper(c) : c;
    cap = (c == ' ');
  }
  out[i] = 0;
}

// Splash as a background on the important screens (Welcome, Home, Run).
// Right after boot the splash is already on screen, so the first screen
// reuses it instead of drawing the same image a second time.
bool splashOnScreen = false;
void drawSplashBackground() {
  if (splashOnScreen) { splashOnScreen = false; return; }
  if (!sdmedia_showSplash()) tft.fillScreen(COLOR_BG);
}

// =====================================================================
// MADD look: aurora background, cut-corner panels, hex badge, top bar.
// Everything is simple shapes, so it draws fast on this display.
// =====================================================================
uint16_t auroraAt(int y) {
  int b = y * AURORA_BANDS / 320;
  if (b < 0) b = 0;
  if (b >= AURORA_BANDS) b = AURORA_BANDS - 1;
  return AURORA[b];
}

// Repaints just the background inside a rectangle (used to erase text).
void fillAurora(int x, int y, int w, int h) {
  const int bandH = 320 / AURORA_BANDS;
  int yEnd = y + h;
  while (y < yEnd) {
    int next = ((y / bandH) + 1) * bandH;
    if (next > yEnd) next = yEnd;
    tft.fillRect(x, y, w, next - y, auroraAt(y));
    y = next;
  }
}

void drawAuroraBackground() {
  fillAurora(0, 0, 480, 320);
  // faint magnetic field lines
  uint16_t f1 = tft.color565(40, 30, 88), f2 = tft.color565(52, 30, 96);
  tft.drawEllipse(240, 176, 150, 190, f1);
  tft.drawEllipse(240, 176, 230, 215, f2);
  // sparkles
  const int16_t sp[][2] = {{70,52},{418,140},{310,222},{36,212},{455,64},{165,130},{388,300},{120,300},{260,52}};
  for (auto& p : sp) tft.drawPixel(p[0], p[1], MADD_TEXT);
  // your spectrum waves along the bottom
  for (int w = 0; w < 3; w++) {
    uint16_t c = MADD_SPECTRUM[w == 0 ? 0 : (w == 1 ? 2 : 5)];
    int base = 302 + w * 6, px = 0, py = base;
    for (int x = 8; x <= 480; x += 8) {
      int yy = base + (int)(5.0f * sinf((x + w * 40) * 0.02f));
      tft.drawLine(px, py, x, yy, c);
      px = x; py = yy;
    }
  }
}

// Panel with cut corners - the "lab instrument" shape.
void drawChamfer(Rect r, uint16_t fill, uint16_t edge, int c = 10) {
  tft.fillRect(r.x + c, r.y, r.w - 2 * c, r.h, fill);
  tft.fillRect(r.x, r.y + c, c, r.h - 2 * c, fill);
  tft.fillRect(r.x + r.w - c, r.y + c, c, r.h - 2 * c, fill);
  tft.fillTriangle(r.x, r.y + c, r.x + c, r.y, r.x + c, r.y + c, fill);
  tft.fillTriangle(r.x + r.w - 1, r.y + c, r.x + r.w - 1 - c, r.y, r.x + r.w - 1 - c, r.y + c, fill);
  tft.fillTriangle(r.x, r.y + r.h - 1 - c, r.x + c, r.y + r.h - 1, r.x + c, r.y + r.h - 1 - c, fill);
  tft.fillTriangle(r.x + r.w - 1, r.y + r.h - 1 - c, r.x + r.w - 1 - c, r.y + r.h - 1, r.x + r.w - 1 - c, r.y + r.h - 1 - c, fill);
  int x0 = r.x, y0 = r.y, x1 = r.x + r.w - 1, y1 = r.y + r.h - 1;
  tft.drawLine(x0 + c, y0, x1 - c, y0, edge);
  tft.drawLine(x1 - c, y0, x1, y0 + c, edge);
  tft.drawLine(x1, y0 + c, x1, y1 - c, edge);
  tft.drawLine(x1, y1 - c, x1 - c, y1, edge);
  tft.drawLine(x1 - c, y1, x0 + c, y1, edge);
  tft.drawLine(x0 + c, y1, x0, y1 - c, edge);
  tft.drawLine(x0, y1 - c, x0, y0 + c, edge);
  tft.drawLine(x0, y0 + c, x0 + c, y0, edge);
}

void drawChamferButton(Rect r, const char* label, uint16_t fill, uint16_t edge, uint16_t textColor, const GFXfont* font = nullptr) {
  drawChamfer(r, fill, edge, r.h >= 40 ? 10 : 7);
  if (!font) font = FONT_SM;
  tft.setFreeFont(font);
  tft.setTextColor(textColor, fill);
  tft.setTextDatum(MC_DATUM);
  char buf[40];
  strncpy(buf, label, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  while (strlen(buf) > 1 && tft.textWidth(buf) > r.w - 14) buf[strlen(buf) - 1] = 0; // never spill past the edge
  tft.drawString(buf, r.x + r.w / 2, r.y + r.h / 2);
  tft.setTextDatum(TL_DATUM);
}

// Pointy-top hexagon, filled, with a double neon outline.
void drawHex(int cx, int cy, int r, uint16_t fill, uint16_t edge) {
  int px[6], py[6];
  for (int i = 0; i < 6; i++) {
    float a = (60.0f * i - 90.0f) * 0.0174533f;
    px[i] = cx + (int)(r * cosf(a));
    py[i] = cy + (int)(r * sinf(a));
  }
  for (int i = 0; i < 6; i++) tft.fillTriangle(cx, cy, px[i], py[i], px[(i + 1) % 6], py[(i + 1) % 6], fill);
  for (int i = 0; i < 6; i++) {
    tft.drawLine(px[i], py[i], px[(i + 1) % 6], py[(i + 1) % 6], edge);
    int j = (i + 1) % 6;
    tft.drawLine(px[i] + (cx - px[i]) / 30, py[i] + (cy - py[i]) / 30, px[j] + (cx - px[j]) / 30, py[j] + (cy - py[j]) / 30, edge);
  }
}

// The MADD badge: neon hexagon holding your color bands.
void drawHexBadge(int cx, int cy, int r) {
  drawHex(cx, cy, r, MADD_PANEL, MADD_MAGENTA);
  int bw = r, bh = (r * 3 / 2) / 4;
  for (int i = 0; i < 4; i++) {
    tft.fillRect(cx - bw / 2, cy - (bh * 2) + i * bh + 1, bw, bh - 1, MADD_SPECTRUM[i == 0 ? 0 : (i == 1 ? 2 : (i == 2 ? 4 : 5))]);
  }
}

void drawSpectrumStripe(int y) {
  for (int i = 0; i < 6; i++) tft.fillRect(i * 80, y, 80, 3, MADD_SPECTRUM[i]);
}

// Short status for the top-right corner of the Home and Session screens.
void topStatusText(char* buf, size_t len) {
  if (!audioUsingBluetooth()) { snprintf(buf, len, "Speaker"); return; }
  switch (audio_btStatus()) {
    case BT_STATUS_CONNECTED:    snprintf(buf, len, "BT connected"); break;
    case BT_STATUS_CONNECTING:   snprintf(buf, len, "BT connecting..."); break;
    case BT_STATUS_NOT_FOUND:    snprintf(buf, len, "BT not found"); break;
    case BT_STATUS_RECONNECTING: snprintf(buf, len, "BT reconnecting..."); break;
    default:                     snprintf(buf, len, "BT searching..."); break;
  }
}

char lastTopStatus[24] = "";
void drawTopStatus(bool force) {
  char buf[24];
  topStatusText(buf, sizeof(buf));
  if (!force && strcmp(buf, lastTopStatus) == 0) return;
  strcpy(lastTopStatus, buf);
  fillAurora(300, 6, 164, 24);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(audioUsingBluetooth() ? btStatusColor() : MADD_DIM);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(buf, 460, 12);
  tft.setTextDatum(TL_DATUM);
}

// Top bar: badge + title on the left, live status on the right, spectrum stripe under it.
void drawTopBar(const char* title, bool showBack, bool showHome = false) {
  drawHexBadge(34, 20, 14);
  char t[40];
  snprintf(t, sizeof(t), showBack ? "< %s" : "%s", title);
  tft.setFreeFont(FONT_LG);
  const GFXfont* f = (tft.textWidth(t) <= 240) ? FONT_LG : FONT_SM;
  drawFittedText(56, f == FONT_LG ? 8 : 12, 240, t, f, MADD_TEXT, MADD_PANEL);
  if (showHome) drawChamferButton(btnHome, "Home", MADD_PANEL, MADD_EDGE, MADD_TEXT);
  else drawTopStatus(true);
  drawSpectrumStripe(38);
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
  audioOutputPref = (AudioOutput)p.getUChar("audioOut", AUDIO_OUT_SPEAKER);
  sessionSoundMode = (AudioSource)p.getUChar("sndMode", AUDIO_SRC_TONE);
  volumePercent = p.getInt("volume", 80);
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
  p.putUChar("audioOut", (uint8_t)audioOutputPref);
  p.putUChar("sndMode", (uint8_t)sessionSoundMode);
  p.putInt("volume", volumePercent);
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

Rect btnHome = {358, 10, 106, 38}; // inset from the bezel

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

// Word-wraps text to fit maxW, one line every lineH pixels. Returns the y
// just below the last line drawn.
int drawWrappedText(int x, int y, int maxW, const char* text, const GFXfont* font, uint16_t color, uint16_t bg, int lineH) {
  tft.setFreeFont(font);
  tft.setTextColor(color, bg);
  tft.setTextDatum(TL_DATUM);
  char line[64] = "";
  const char* p = text;
  while (*p) {
    const char* wordEnd = p;
    while (*wordEnd && *wordEnd != ' ') wordEnd++;
    char trial[64];
    snprintf(trial, sizeof(trial), "%s%s%.*s", line, line[0] ? " " : "", (int)(wordEnd - p), p);
    if (line[0] && tft.textWidth(trial) > maxW) {
      tft.drawString(line, x, y);
      y += lineH;
      snprintf(line, sizeof(line), "%.*s", (int)(wordEnd - p), p);
    } else {
      strncpy(line, trial, sizeof(line) - 1);
      line[sizeof(line) - 1] = 0;
    }
    p = *wordEnd ? wordEnd + 1 : wordEnd;
  }
  if (line[0]) { tft.drawString(line, x, y); y += lineH; }
  return y;
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
  drawSplashBackground();
  tft.fillRect(0, 0, 480, 178, COLOR_BG);

  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("MADD PEMF", 20, 10);

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_WARN, COLOR_BG);
  tft.drawString("Wellness device - not a medical device", 20, 38);

  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  const char* lines[] = {
    "Not intended to diagnose, treat, cure,",
    "or prevent any disease.",
    "",
    "Do NOT use if you have a pacemaker,",
    "insulin pump, or other implanted",
    "electronic device, or if pregnant."
  };
  int y = 60;
  for (int i = 0; i < 6; i++) {
    tft.drawString(lines[i], 20, y);
    y += 19;
  }

  // Checkbox - the visual square stays small and fixed; checkboxRect
  // itself is intentionally wider (see its declaration) to cover the
  // whole row as the actual tap target, so only the square is drawn at
  // its own fixed size here rather than stretching to match.
  refreshWelcomeCheckbox();

}

// The checkbox and Continue button are the only things on this screen
// that change from a touch - everything else (title, disclaimer text,
// diagnostics) is set once and doesn't need to redraw on every tap.
// Separated out so loop()'s "state may have changed" redraw can call
// just this instead of the full drawWelcomeScreen() (background image
// included) on every single touch - the same fix applied to the Run
// screen, for the same reason (a real, confirmed unnecessary full
// redraw on every touch, visibly slow once this screen got a
// background image).
void refreshWelcomeCheckbox() {
  const int checkSquareSize = 36;
  tft.drawRect(checkboxRect.x, checkboxRect.y, checkSquareSize, checkSquareSize, COLOR_ACCENT);
  if (wellnessAck) {
    tft.fillRect(checkboxRect.x + 4, checkboxRect.y + 4, checkSquareSize - 8, checkSquareSize - 8, COLOR_ACCENT);
  } else {
    tft.fillRect(checkboxRect.x + 4, checkboxRect.y + 4, checkSquareSize - 8, checkSquareSize - 8, COLOR_BG); // clear a previously-checked mark
  }
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("I understand and agree", checkboxRect.x + checkSquareSize + 10, checkboxRect.y + 4);

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
  tft.drawString("Manage People", 20, 14);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.drawString("Tap a name twice to remove them.", 20, 46);

  if (personToRemove >= 0 && millis() - personRemoveArmedAt > 5000) {
    personToRemove = -1; // auto-disarm after the confirm window elapses
  }

  int colW = 220, rowH = 78, gapX = 20, gapY = 12, startY = 72;
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
  int y = 74, rowH = 32, gap = 3;

  const char* rows[4] = { KB_ROW1, KB_ROW2, KB_ROW3, KB_ROW4 };
  for (int r = 0; r < 4; r++) {
    int n = strlen(rows[r]);
    int w = (444 - (n - 1) * gap) / n;
    for (int i = 0; i < n; i++) {
      kbKeyRects[kbKeyCount] = {18 + i * (w + gap), y, w, rowH};
      kbKeyChars[kbKeyCount] = rows[r][i];
      kbKeyCount++;
    }
    y += rowH + gap;
  }

  y += 4;
  btnKbSpace     = {18, y, 252, 38};
  btnKbBackspace = {276, y, 90, 38};
  btnKbOk        = {372, y, 90, 38};
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
  tft.drawString(textEntryPrompt, 20, 8);

  char shown[24];
  formatDisplayName(textEntryBuf, shown, sizeof(shown));
  tft.fillRect(20, 36, 440, 32, COLOR_PANEL);
  tft.drawRect(20, 36, 440, 32, COLOR_ACCENT);
  drawFittedText(30, 42, 420, textEntryLen > 0 ? shown : "-", FONT_LG, TFT_WHITE, COLOR_PANEL);

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
Rect btnClientContinue = {60, 250, 200, 48};

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
Rect pinCancelBtn = {20, 200, 200, 46};
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
  const GFXfont* titleFont = (tft.textWidth(title) <= 226) ? FONT_LG : FONT_SM;
  drawFittedText(20, 16, 226, title, titleFont, TFT_WHITE, COLOR_BG);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  const char* subtitle = (pinPurpose == PIN_DEV_MODE) ? "Required to exceed the 45% ceiling" :
                         (pinPurpose == PIN_SET_LOGIN) ? "Choose a PIN staff will use to log in" :
                                                          "Staff PIN required to use this device";
  drawWrappedText(20, 48, 226, subtitle, FONT_SM, COLOR_TEXT_DIM, COLOR_BG, 20);

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
  tft.drawString(pinLen > 0 ? mask : "-", 20, 104);
  pinWrongFlash = false;

  int gx = 256, gy = 24, cellW = 64, cellH = 44, gap = 8; // right column; left column stays clear for the title
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

bool restartAfterWifiSetup = false; // set when Bluetooth had to be shut down for the WiFi setup page
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
  audio_btEnd(); // clean BT teardown before restarting
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
  out[n++] = SET_AUDIO_OUT;
  out[n++] = SET_CHECK_UPDATES;
  out[n++] = SET_BT_DEVICE;
  out[n++] = SET_ROOM_OR_PEOPLE;
  if (!isWellnessCenter && peopleCount() > 0) out[n++] = SET_SWITCH_USER;
  out[n++] = SET_WIFI;
  out[n++] = SET_VIEW_LOG;
  if (sdmedia_isAvailable() && sdmedia_soundscapeCount() > 0) out[n++] = SET_SOUNDSCAPES;
  out[n++] = SET_RECAL_TOUCH;
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
    case SET_AUDIO_OUT:
      snprintf(labelOut, labelLen, audioUsingBluetooth() ? "Sound Out: Bluetooth" : "Sound Out: Speaker");
      *activeOut = true;
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
      snprintf(labelOut, labelLen, "Soundscapes (%d)", sdmedia_soundscapeCount());
      break;
    case SET_RECAL_TOUCH:
      snprintf(labelOut, labelLen, "Recalibrate Touch");
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
    case SET_AUDIO_OUT:
      if (audioUsingBluetooth()) {
        audioOutputPref = AUDIO_OUT_SPEAKER;       // BT -> onboard speaker
        saveSetupInfo();
        applyAudioOutput();
        screen = SCR_SETTINGS;
      } else if (strlen(btDeviceName) > 0) {
        audioOutputPref = AUDIO_OUT_BLUETOOTH;     // speaker -> saved BT device
        saveSetupInfo();
        applyAudioOutput();
        screen = SCR_SETTINGS;
      } else {
        screen = SCR_BT_SCAN;                      // no BT device yet - pick one first
      }
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
        // The WiFi setup page (its own hotspot + web page) needs the memory
        // Bluetooth is holding - free it first. If Bluetooth was running,
        // the device restarts when setup finishes so Bluetooth comes back.
        audio_setSource(AUDIO_SRC_OFF);
        if (audio_btStarted()) { audio_btEnd(true); restartAfterWifiSetup = true; }
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
    case SET_RECAL_TOUCH:
      runTouchCalibration(); // blocking; Settings redraws right after
      screen = SCR_SETTINGS;
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
  tft.drawString("Settings", 20, 12);

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
  drawFittedText(20, 50, 330, tierBuf, FONT_SM, COLOR_TEXT_DIM, COLOR_BG); // constrained so a long name can't run into the Home button

  drawHomeButton();

  settingsVisibleCount = buildVisibleSettingsItems(settingsVisibleItems);
  int totalPages = (settingsVisibleCount + SETTINGS_PER_PAGE - 1) / SETTINGS_PER_PAGE;
  if (totalPages < 1) totalPages = 1;
  if (settingsPage >= totalPages) settingsPage = 0; // safety, if fewer items are visible now than before

  if (totalPages > 1) {
    char pageBuf[20];
    snprintf(pageBuf, sizeof(pageBuf), "Page %d of %d", settingsPage + 1, totalPages);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pageBuf, 348, 20);
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
      drawButtonFast(r, label, color, active); // smaller font so long labels fit
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
    if (restartAfterWifiSetup) ESP.restart(); // brings Bluetooth back
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
  tft.drawString("Session Log", 20, 12);
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
Rect btnScanToggle = {20, 54, 440, 42};
Rect btnHeadphonesToggle = {20, 104, 220, 42};
Rect btnForgetBt = {260, 104, 200, 42};
bool btForgetArmed = false;
unsigned long btForgetArmedAt = 0;
Rect btScanResultRects[6];

unsigned long btPickAt = 0; // when a device was tapped in the scan list (0 = none pending)
bool scanRequested = false; // true the instant the button is tapped, before the BT stack actually starts discovering - gives immediate feedback instead of an unexplained gap

void drawBtScanScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Bluetooth Device", 20, 12);
  drawHomeButton();
  drawBtStatusLine(20, 38, 330);

  bool scanning = audio_btIsScanning();
  const char* scanLabel = scanning ? "Scanning... (tap to stop)"
                        : scanRequested ? "Starting scan..."
                        : "Scan for Devices";
  drawButton(btnScanToggle, scanLabel, (scanning || scanRequested) ? COLOR_WARN : COLOR_GOOD, scanning || scanRequested);

  int gridStartY = 104;
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
    gridStartY = 154;
  }

  int count = audio_btScanResultCount();
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
      drawButtonFast(r, audio_btScanResultName(i));
    }
  }
  if (count > 6 && gridStartY < 150) {
    tft.drawString("More devices found - power off nearby", 20, gridStartY + 152);
    tft.drawString("ones you don't want to narrow it down.", 20, gridStartY + 172);
  }
}

void handleBtScanTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    if (audio_btIsScanning()) audio_btStopScan();
    scanRequested = false;
    return;
  }
  if (touchInRect(x, y, btnScanToggle)) {
    if (audio_btIsScanning() || scanRequested) {
      audio_btStopScan();
      scanRequested = false;
    } else {
      audio_btStartScan();
      scanRequested = true;
    }
    screen = SCR_BT_SCAN;
    return;
  }
  if (strlen(btDeviceName) > 0 && touchInRect(x, y, btnHeadphonesToggle)) {
    btHeadphonesMode = !btHeadphonesMode;
    audio_setHeadphonesMode(btHeadphonesMode);
    saveSetupInfo();
    screen = SCR_BT_SCAN;
    return;
  }
  if (strlen(btDeviceName) > 0 && touchInRect(x, y, btnForgetBt)) {
    if (btForgetArmed) {
      btDeviceName[0] = 0;
      btHeadphonesMode = false;
      btForgetArmed = false;
      audioOutputPref = AUDIO_OUT_SPEAKER; // back to the onboard speaker
      saveSetupInfo();
      applyAudioOutput();
    } else {
      btForgetArmed = true;
      btForgetArmedAt = millis();
    }
    screen = SCR_BT_SCAN;
    return;
  }
  int count = audio_btScanResultCount();
  int shown = count < 6 ? count : 6;
  for (int i = 0; i < shown; i++) {
    if (touchInRect(x, y, btScanResultRects[i])) {
      strncpy(btDeviceName, audio_btScanResultName(i), sizeof(btDeviceName) - 1);
      btDeviceName[sizeof(btDeviceName) - 1] = 0;
      audioOutputPref = AUDIO_OUT_BLUETOOTH; // picking a device means "use Bluetooth"
      saveSetupInfo();
      // Connect right now - no restart. The status line above the list
      // shows Connecting... then Connected.
      audio_setBtDeviceName(btDeviceName);
      audio_setOutput(AUDIO_OUT_BLUETOOTH);
      audio_btConnectToScanResult(i);
      btPickAt = millis();
      scanRequested = false;
      screen = SCR_BT_SCAN;
      return;
    }
  }
  // Tapping the status line when a device wasn't found retries right away.
  if (audioUsingBluetooth() && y >= 36 && y < 54 && audio_btStatus() == BT_STATUS_NOT_FOUND) {
    audio_btRetry();
    screen = SCR_BT_SCAN;
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
Rect btnCfSquare = {20, 112, 85, 44};
Rect btnCfSine   = {115, 112, 85, 44};
Rect btnCfStart  = {20, 166, 180, 44};

void drawCustomFreqScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Custom Frequency", 20, 14);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  char rangeBuf[32];
  snprintf(rangeBuf, sizeof(rangeBuf), "Enter 1-%d Hz (decimals OK)", CUSTOM_FREQ_MAX);
  tft.drawString(rangeBuf, 20, 44);

  // Clear the FULL value-display area first, not just the new text's own
  // width - otherwise a shorter number typed after a longer one (or a
  // backspace) leaves stale digit remnants behind, which read as
  // garbled/cut-off text.
  tft.fillRect(18, 62, 200, 44, COLOR_BG);
  char shown[20];
  snprintf(shown, sizeof(shown), "%s Hz", customFreqLen > 0 ? customFreqBuf : "-");
  tft.setFreeFont(FONT_XL);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(shown, 20, 68);

  drawButton(btnCfSquare, "Square", 0xFFFF, customFreqWave == WAVE_SQUARE);
  drawButton(btnCfSine, "Sine", 0xFFFF, customFreqWave == WAVE_SINE);
  drawButton(btnCfStart, "Start", customFreqLen > 0 ? COLOR_GOOD : COLOR_MUTED, customFreqLen > 0);

  int gx = 220, gy = 56, cellW = 70, cellH = 44, gap = 8; // gy=48 clears the enlarged Home button (ends y=44)
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
  tft.drawString("Check for Updates", 20, 14);
  drawHomeButton();

  tft.setFreeFont(FONT_SM);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  char verBuf[32];
  snprintf(verBuf, sizeof(verBuf), "Current version: %s", FIRMWARE_VERSION);
  tft.drawString(verBuf, 20, 44);
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
    if (!wifitime_isConfigured()) {
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Set up WiFi first", 240, 140);
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
      tft.drawString("Settings -> Set Up WiFi, then try again.", 240, 175);
      delay(2500);
      screen = SCR_UPDATE;
      return;
    }
    // Updates run at startup, before Bluetooth or a session has taken any
    // memory - checking mid-run (with Bluetooth up) could crash the board.
    // One tap: restart -> check -> install if newer -> restart into it.
    endSession();
    audio_setSource(AUDIO_SRC_OFF);
    Preferences p;
    p.begin("ota", false);
    p.putBool("pending", true);
    p.end();
    tft.fillScreen(COLOR_BG);
    tft.setFreeFont(FONT_LG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Restarting to check for updates", 240, 140);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("This takes about 30 seconds.", 240, 175);
    tft.setTextDatum(TL_DATUM);
    delay(1500);
    audio_btEnd();
    ESP.restart();
  }
}

// Runs from setup() when "Check for Updates" was tapped: check, and install
// if a newer version is published. Shows plain progress the whole time.
void runPendingUpdateCheck() {
  Preferences p;
  p.begin("ota", false);
  bool pending = p.getBool("pending", false);
  if (pending) p.putBool("pending", false); // one attempt only - never a restart loop
  p.end();
  if (!pending) return;

  auto show = [](const char* big, const char* small) {
    tft.fillScreen(COLOR_BG);
    tft.setFreeFont(FONT_LG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(big, 240, 140);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString(small, 240, 175);
    tft.setTextDatum(TL_DATUM);
  };

  show("Checking for updates...", "Connecting to WiFi");
  esp_task_wdt_init(45, true);
  esp_task_wdt_add(NULL);
  bool newer = ota_checkForUpdate();
  esp_task_wdt_delete(NULL);
  if (!newer) {
    if (strlen(ota_lastErrorMessage()) > 0) show("Couldn't check for updates", ota_lastErrorMessage());
    else {
      char v[40];
      snprintf(v, sizeof(v), "You have the latest version (%s)", FIRMWARE_VERSION);
      show("You're up to date", v);
    }
    delay(3000);
    return;
  }
  char msg[48];
  snprintf(msg, sizeof(msg), "Installing version %s", ota_latestVersionString());
  show(msg, "Please keep the power on - about 1 minute.");
  esp_task_wdt_init(45, true);
  esp_task_wdt_add(NULL);
  ota_downloadAndInstall(); // restarts into the new version on success
  esp_task_wdt_delete(NULL);
  show("Update didn't finish", ota_lastErrorMessage());
  delay(3500);
}

// ---------------------------------------------------------------------
// Screen: category picker
// ---------------------------------------------------------------------
Rect catButtonRects[CATEGORY_COUNT];
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

Rect titleTapZone = {50, 0, 230, 36}; // tap the title 5 times for the hidden stats panel
int titleTapCount = 0;
unsigned long titleTapWindowStart = 0;
bool showDeviceStats = false;

// ---------------------------------------------------------------------
// Home screen: quick start, favorites, and nine color-coded tiles.
// ---------------------------------------------------------------------
Rect btnQuickStart = {18, 48, 290, 70};
Rect btnFavorites  = {318, 48, 144, 70};

enum HomeTile : uint8_t { TILE_CATEGORY, TILE_PROGRAMS, TILE_CUSTOM, TILE_SETTINGS };
struct HomeTileDef { const char* label; HomeTile kind; Category cat; int colorIdx; };
// colorIdx: 0 orange, 1 coral, 2 pink, 3 magenta, 4 violet, 5 blue, -1 cyan, -2 dim
static const HomeTileDef HOME_TILES[9] = {
  {"Sleep",    TILE_CATEGORY, CAT_HEART_CIRC,       5},
  {"Focus",    TILE_CATEGORY, CAT_MENTAL_COGNITIVE, 4},
  {"Wellness", TILE_CATEGORY, CAT_CHRONIC_SYSTEMIC, 2},
  {"Athletic", TILE_CATEGORY, CAT_PAIN_RECOVERY,    0},
  {"Body",     TILE_CATEGORY, CAT_BONE_JOINT,       1},
  {"Skin",     TILE_CATEGORY, CAT_SKIN_WOUND,       3},
  {"Programs", TILE_PROGRAMS, CAT_BONE_JOINT,      -1},
  {"Custom",   TILE_CUSTOM,   CAT_BONE_JOINT,      -2},
  {"Settings", TILE_SETTINGS, CAT_BONE_JOINT,      -2},
};
Rect homeTileRects[9];

uint16_t tileColor(int idx) {
  if (idx >= 0) return MADD_SPECTRUM[idx];
  return idx == -1 ? MADD_CYAN : MADD_DIM;
}

// Quick start = the last preset used (from the session log), or Deep Relaxation.
int quickStartPresetIndex() {
  if (logCount > 0) {
    for (int i = 0; i < NUM_BASE_PRESETS; i++)
      if (strcmp(BASE_PRESETS[i].name, sessionLog[0].name) == 0) return i;
  }
  for (int i = 0; i < NUM_BASE_PRESETS; i++)
    if (BASE_PRESETS[i].category == CAT_HEART_CIRC) return i;
  return 0;
}

void openPresetByIndex(int realIdx, Screen origin) {
  selectedIndex = realIdx;
  selName = BASE_PRESETS[realIdx].name;
  selFreq = BASE_PRESETS[realIdx].freqHz;
  selWave = BASE_PRESETS[realIdx].wave;
  selCategoryName = CATEGORY_NAMES[BASE_PRESETS[realIdx].category];
  pendingSequenceIndex = -1;
  runScreenOrigin = origin;
  screen = isWellnessCenter ? SCR_CLIENT_CONFIRM : SCR_RUN;
}

void drawCoilIcon(int cx, int topY, bool bright) {
  uint16_t a = bright ? MADD_CYAN : MADD_EDGE, b = bright ? MADD_COIL2 : MADD_EDGE;
  tft.drawEllipse(cx, topY,      10, 3, a);
  tft.drawEllipse(cx, topY + 10, 15, 4, b);
  tft.drawEllipse(cx, topY + 21, 20, 5, a);
  tft.drawEllipse(cx, topY + 33, 25, 6, b);
}

void drawCategoryScreen() {
  drawAuroraBackground();
  drawTopBar("MADD PEMF", false);

  // Quick start card
  drawChamfer(btnQuickStart, MADD_PANEL, MADD_MAGENTA);
  drawCoilIcon(58, 60, true);
  int q = quickStartPresetIndex();
  drawFittedText(96, 58, 200, "Quick start", FONT_LG, MADD_TEXT, MADD_PANEL);
  char f[16], line[48];
  formatFreq(BASE_PRESETS[q].freqHz, f, sizeof(f));
  snprintf(line, sizeof(line), "%s  %s", BASE_PRESETS[q].name, f);
  drawFittedText(96, 90, 200, line, FONT_SM, MADD_CYAN, MADD_PANEL);

  drawChamfer(btnFavorites, MADD_PANEL, MADD_EDGE);
  drawFittedText(btnFavorites.x + 22, btnFavorites.y + 24, 110, "Favorites", FONT_LG, MADD_TEXT, MADD_PANEL);

  // 3 x 3 tiles, each with its own color edge
  for (int i = 0; i < 9; i++) {
    int col = i % 3, row = i / 3;
    Rect r = {18 + col * 150, 126 + row * 56, 144, 50};
    homeTileRects[i] = r;
    drawChamfer(r, MADD_PANEL, MADD_EDGE, 8);
    tft.fillRect(r.x, r.y + 8, 4, r.h - 16, tileColor(HOME_TILES[i].colorIdx));
    drawFittedText(r.x + 16, r.y + 15, r.w - 24, HOME_TILES[i].label, FONT_LG, MADD_TEXT, MADD_PANEL);
  }

  if (showDeviceStats) {
    Rect p = {60, 70, 360, 180};
    drawChamfer(p, MADD_PANEL, MADD_MAGENTA, 14);
    tft.setFreeFont(FONT_LG);
    tft.setTextColor(MADD_TEXT);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("You found it!", 240, p.y + 16);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(MADD_DIM);
    char buf[48];
    snprintf(buf, sizeof(buf), "%lu sessions run, all-time", lifetimeSessionCount);
    tft.drawString(buf, 240, p.y + 56);
    snprintf(buf, sizeof(buf), "%lu h %lu min of session time", lifetimeMinutes / 60, lifetimeMinutes % 60);
    tft.drawString(buf, 240, p.y + 78);
    snprintf(buf, sizeof(buf), "Up %luh %lum since power-on", millis() / 3600000UL, (millis() / 60000UL) % 60);
    tft.drawString(buf, 240, p.y + 100);
    tft.setTextColor(MADD_CYAN);
    tft.drawString("Tap anywhere to close", 240, p.y + 140);
    tft.setTextDatum(TL_DATUM);
  }
}

void handleCategoryTouch(int x, int y) {
  if (showDeviceStats) {
    showDeviceStats = false; // tap anywhere to dismiss
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
    return;
  }
  if (touchInRect(x, y, btnQuickStart)) {
    openPresetByIndex(quickStartPresetIndex(), SCR_CATEGORY);
    return;
  }
  if (touchInRect(x, y, btnFavorites)) {
    viewingFavorites = true;
    buildFavoritesIndex();
    listPage = 0;
    screen = SCR_LIST;
    return;
  }
  for (int i = 0; i < 9; i++) {
    if (!touchInRect(x, y, homeTileRects[i])) continue;
    switch (HOME_TILES[i].kind) {
      case TILE_CATEGORY:
        viewingFavorites = false;
        currentCategory = HOME_TILES[i].cat;
        buildCategoryIndex(currentCategory);
        listPage = 0;
        screen = SCR_LIST;
        break;
      case TILE_PROGRAMS: screen = SCR_SEQUENCES; break;
      case TILE_CUSTOM:   screen = SCR_CUSTOM_FREQ; break;
      case TILE_SETTINGS: screen = SCR_SETTINGS; break;
    }
    return;
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
Rect btnSeqPrevPage = {18, 222, 140, 42};
Rect btnSeqBack = {170, 222, 140, 42};
Rect btnSeqNextPage = {322, 222, 140, 42};

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
  drawAuroraBackground();
  drawTopBar("Programs", true, true);

  int start = sequencesPage * SEQUENCES_PER_PAGE;
  for (int i = 0; i < SEQUENCES_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {18 + col * 225, 52 + row * 54, 219, 48};
    sequenceItemRects[i] = r;
    if (idx < NUM_SEQUENCES) {
      drawChamfer(r, MADD_PANEL, MADD_EDGE, 8);
      tft.fillRect(r.x, r.y + 8, 4, r.h - 16, MADD_CYAN);
      drawFittedText(r.x + 14, r.y + 8, r.w - 22, SEQUENCES[idx].name, FONT_SM, MADD_TEXT, MADD_PANEL);
      char rangeBuf[24];
      getSequenceFreqRange(idx, rangeBuf, sizeof(rangeBuf));
      drawFittedText(r.x + 14, r.y + 27, r.w - 22, rangeBuf, FONT_SM, MADD_CYAN, MADD_PANEL);
    }
  }
  drawPagerRow(btnSeqPrevPage, btnSeqBack, btnSeqNextPage, sequencesPage, (NUM_SEQUENCES + SEQUENCES_PER_PAGE - 1) / SEQUENCES_PER_PAGE);
}

void handleSequencesTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (y < 36 && x < 300) { screen = SCR_CATEGORY; return; } // "< title" = back
  int start = sequencesPage * SEQUENCES_PER_PAGE;
  for (int i = 0; i < SEQUENCES_PER_PAGE; i++) {
    int idx = start + i;
    if (idx < NUM_SEQUENCES && touchInRect(x, y, sequenceItemRects[i])) {
      const Program& p = SEQUENCES[idx];
      selName = p.name;
      selFreq = p.steps[0].freqHz;
      selWave = p.steps[0].wave;
      selCategoryName = "Programs";
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
Rect btnSndPrevPage = {18, 222, 140, 42};
Rect btnSndBack = {170, 222, 140, 42};
Rect btnSndNextPage = {322, 222, 140, 42};


// ---------------------------------------------------------------------
// Soundscapes screen - two uses:
//   from the Run screen  -> tap one to make it this session's sound
//                           (returns straight to the session)
//   from Settings        -> tap to preview / tap again to stop
// ---------------------------------------------------------------------
void drawSoundscapesScreen() {
  drawAuroraBackground();
  drawTopBar(soundscapesOrigin == SCR_RUN ? "Pick a sound" : "Soundscapes", true, soundscapesOrigin != SCR_RUN);

  int count = sdmedia_soundscapeCount();
  int highlighted = (soundscapesOrigin == SCR_RUN) ? sessionSoundscapeIndex : previewSoundscapeIndex;
  int start = soundscapesPage * SOUNDSCAPES_PER_PAGE;
  for (int i = 0; i < SOUNDSCAPES_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {18 + col * 225, 52 + row * 54, 219, 48};
    soundscapeItemRects[i] = r;
    if (idx < count) {
      char name[32];
      prettySoundName(idx, name, sizeof(name));
      bool on = (idx == highlighted);
      drawChamfer(r, on ? tft.color565(12, 58, 68) : MADD_PANEL, on ? MADD_CYAN : MADD_EDGE, 8);
      tft.fillRect(r.x, r.y + 8, 4, r.h - 16, on ? MADD_CYAN : MADD_SPECTRUM[(idx + 2) % 6]);
      drawFittedText(r.x + 14, r.y + 15, r.w - 22, name, FONT_LG, MADD_TEXT, MADD_PANEL);
    }
  }
  drawPagerRow(btnSndPrevPage, btnSndBack, btnSndNextPage, soundscapesPage, (count + SOUNDSCAPES_PER_PAGE - 1) / SOUNDSCAPES_PER_PAGE);
  if (soundscapesOrigin != SCR_RUN) {
    drawFittedText(18, 272, 444, "Tap to preview, tap again to stop.", FONT_SM, MADD_DIM, MADD_PANEL);
  }
}

void stopPreview() { previewSoundscapeIndex = -1; }

void handleSoundscapesTouch(int x, int y) {
  if (handleHomeTouch(x, y)) {
    stopPreview();
    if (soundscapesOrigin == SCR_RUN) screen = SCR_RUN; // never abandon a running session via Home here
    return;
  }
  if (y < 36 && x < 300) { stopPreview(); screen = soundscapesOrigin; return; } // "< title" = back
  int count = sdmedia_soundscapeCount();
  int start = soundscapesPage * SOUNDSCAPES_PER_PAGE;
  for (int i = 0; i < SOUNDSCAPES_PER_PAGE; i++) {
    int idx = start + i;
    if (idx >= count || !touchInRect(x, y, soundscapeItemRects[i])) continue;
    if (soundscapesOrigin == SCR_RUN) {
      sessionSoundscapeIndex = idx;
      sessionSoundMode = AUDIO_SRC_SOUNDSCAPE;
      saveSetupInfo();
      screen = SCR_RUN;
    } else {
      previewSoundscapeIndex = (previewSoundscapeIndex == idx) ? -1 : idx;
    }
    return;
  }
  if (touchInRect(x, y, btnSndPrevPage)) {
    if (soundscapesPage > 0) soundscapesPage--;
  } else if (touchInRect(x, y, btnSndNextPage)) {
    if ((soundscapesPage + 1) * SOUNDSCAPES_PER_PAGE < count) soundscapesPage++;
  } else if (touchInRect(x, y, btnSndBack)) {
    stopPreview();
    screen = soundscapesOrigin;
  }
}

// ---------------------------------------------------------------------
// Screen: preset list (paged)
// ---------------------------------------------------------------------
Rect itemRects[ITEMS_PER_PAGE];
Rect btnPrevPage = {18, 222, 140, 42};
Rect btnBackFromList = {170, 222, 140, 42};
Rect btnNextPage = {322, 222, 140, 42};

int listCount() {
  return catCount;
}

void getListLabel(int posInCategory, char* buf, size_t bufLen) {
  int realIdx = catIndices[posInCategory];
  snprintf(buf, bufLen, "%s", BASE_PRESETS[realIdx].name);
}

// 2-column x 3-row grid, same pattern as the Category screen. Plain
// background (no splash) - this screen changes on every page flip.
// Prev / Back / Next row used by every list screen, with a page count.
void drawPagerRow(Rect prev, Rect back, Rect next, int page, int pages) {
  if (pages < 1) pages = 1;
  drawChamferButton(prev, "< Prev", MADD_PANEL, page > 0 ? MADD_EDGE : tft.color565(40, 28, 60), page > 0 ? MADD_TEXT : MADD_EDGE, FONT_LG);
  drawChamferButton(back, "Back", MADD_PANEL, MADD_MAGENTA, MADD_TEXT, FONT_LG);
  drawChamferButton(next, "Next >", MADD_PANEL, page + 1 < pages ? MADD_EDGE : tft.color565(40, 28, 60), page + 1 < pages ? MADD_TEXT : MADD_EDGE, FONT_LG);
  if (pages > 1) {
    char buf[20];
    snprintf(buf, sizeof(buf), "Page %d of %d", page + 1, pages);
    tft.setFreeFont(FONT_SM);
    tft.setTextColor(MADD_DIM);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, 462, 272);
    tft.setTextDatum(TL_DATUM);
  }
}

// The color a category uses on the Home tiles, reused on its list.
uint16_t categoryColor(Category c) {
  for (int i = 0; i < 9; i++)
    if (HOME_TILES[i].kind == TILE_CATEGORY && HOME_TILES[i].cat == c) return tileColor(HOME_TILES[i].colorIdx);
  return MADD_MAGENTA;
}
// 2 x 3 grid of presets on the MADD background, each with its category color.
void drawListScreen() {
  drawAuroraBackground();
  drawTopBar(viewingFavorites ? "Favorites" : CATEGORY_NAMES[currentCategory], true, true);

  int total = listCount();
  if (total == 0 && viewingFavorites) {
    Rect note = {18, 60, 444, 70};
    drawChamfer(note, MADD_PANEL, MADD_EDGE);
    drawFittedText(34, 74, 410, "No favorites yet.", FONT_LG, MADD_TEXT, MADD_PANEL);
    drawFittedText(34, 104, 410, "Tap the star on a session screen to add one.", FONT_SM, MADD_DIM, MADD_PANEL);
  }
  int start = listPage * ITEMS_PER_PAGE;
  char buf[64];
  for (int i = 0; i < ITEMS_PER_PAGE; i++) {
    int idx = start + i;
    int col = i % 2, row = i / 2;
    Rect r = {18 + col * 225, 52 + row * 54, 219, 48};
    itemRects[i] = r;
    if (idx < total) {
      getListLabel(idx, buf, sizeof(buf));
      drawChamfer(r, MADD_PANEL, MADD_EDGE, 8);
      tft.fillRect(r.x, r.y + 8, 4, r.h - 16, categoryColor(BASE_PRESETS[catIndices[idx]].category));
      drawFittedText(r.x + 14, r.y + 8, r.w - 22, buf, FONT_SM, MADD_TEXT, MADD_PANEL);
      char f[16];
      formatFreq(BASE_PRESETS[catIndices[idx]].freqHz, f, sizeof(f));
      drawFittedText(r.x + 14, r.y + 27, r.w - 22, f, FONT_SM, MADD_CYAN, MADD_PANEL);
    }
  }
  drawPagerRow(btnPrevPage, btnBackFromList, btnNextPage, listPage, (total + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE);
}

void openRunScreenForIndex(int posInCategory) {
  int realIdx = catIndices[posInCategory];
  selectedIndex = realIdx;
  selName = BASE_PRESETS[realIdx].name;
  selFreq = BASE_PRESETS[realIdx].freqHz;
  selWave = BASE_PRESETS[realIdx].wave;
  selCategoryName = CATEGORY_NAMES[BASE_PRESETS[realIdx].category];
  pendingSequenceIndex = -1;
  runScreenOrigin = SCR_LIST;
  screen = isWellnessCenter ? SCR_CLIENT_CONFIRM : SCR_RUN;
}

void handleListTouch(int x, int y) {
  if (handleHomeTouch(x, y)) return;
  if (y < 36 && x < 300) { screen = SCR_CATEGORY; return; } // "< title" = back
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
  } else if (touchInRect(x, y, btnNextPage)) {
    if ((listPage + 1) * ITEMS_PER_PAGE < total) listPage++;
  } else if (touchInRect(x, y, btnBackFromList)) {
    screen = SCR_CATEGORY;
  }
}

// ---------------------------------------------------------------------
// Screen: running / detail view
//   Left column : name, frequency, Start/Pause, Sound, Pick Sound, Stop & Back
//   Right column: Power, Session timer, Volume, sound-output status
// ---------------------------------------------------------------------
// Left: coil rings + hexagon countdown, then Start/Pause, Stop, Favorite.
Rect btnRunTitle      = {18, 0, 280, 36};     // "< Name" in the top bar = stop and go back
Rect btnStartStop     = {18, 258, 96, 40};
Rect btnBackFromRun   = {120, 258, 66, 40};   // Stop
Rect btnFavToggle     = {192, 258, 32, 40};
// Right panel: sound choice, power, volume, timer, status.
Rect runPanel         = {230, 46, 234, 254};
Rect btnSndOff        = {240, 70, 62, 34};
Rect btnSndTone       = {308, 70, 68, 34};
Rect btnSndScape      = {382, 70, 74, 34};
Rect btnPickSound     = {240, 108, 216, 22};
Rect btnPwrMinus      = {240, 136, 48, 34};
Rect btnPwrPlus       = {408, 136, 48, 34};
Rect btnVolMinus      = {240, 194, 48, 34};
Rect btnVolPlus       = {408, 194, 48, 34};
Rect btnTmrMinus      = {240, 234, 48, 34};
Rect btnTmrPlus       = {408, 234, 48, 34};
static const int RUN_HEX_X = 112, RUN_HEX_Y = 142, RUN_HEX_R = 58;

unsigned long sessionStartMillis = 0;
bool sessionTimerArmed = false;

bool sessionPaused = false;
unsigned long pauseStartMillis = 0;
float pausedFreq = 0;

unsigned long sessionEffectiveMillis() {
  return sessionPaused ? pauseStartMillis : millis();
}

bool programActive = false;
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
  if (!programActive || sessionPaused) return;
  if (millis() - lastProgramUpdateMillis < 1000) return;
  lastProgramUpdateMillis = millis();

  ProgramStep& s = currentProgram.steps[programStepIndex];
  unsigned long elapsedMs = millis() - programStepStartMillis;
  unsigned long durMs = (unsigned long)s.durationSec * 1000UL;

  if (s.kind == STEP_RAMP) {
    float t = (float)elapsedMs / (float)durMs;
    if (t > 1.0f) t = 1.0f;
    waveform_setFrequency(programStepStartFreq + (s.freqHz - programStepStartFreq) * t);
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
    if (next.kind == STEP_HOLD) waveform_setFrequency(next.freqHz);
    programStepStartMillis = millis();
  }
}

// The frequency actually on the coil right now (follows Sequence ramps).
float liveFrequency() {
  if (waveform_isRunning()) return waveform_currentFreq();
  if (sessionPaused) return pausedFreq;
  return selFreq;
}

void formatFreq(float f, char* buf, size_t len) {
  if (f == (int)f) snprintf(buf, len, "%d Hz", (int)f);
  else snprintf(buf, len, "%.2f Hz", f);
}

// ---------------------------------------------------------------------
// Keeps the audio engine in step with the session. Called every loop():
// audio plays while the coil is running (and during a Settings preview),
// the tone always tracks the live coil frequency, and the soundscape is
// whichever one this session has selected.
// ---------------------------------------------------------------------
void syncAudio() {
  AudioSource src = AUDIO_SRC_OFF;
  int scapeIdx = -1;
  // Sound plays the whole time the session screen is open (and while
  // picking a soundscape from it) - not only after START - so choosing a
  // sound gives instant feedback. Leaving the session stops it.
  bool sessionOn = (screen == SCR_RUN) || (screen == SCR_SOUNDSCAPES && soundscapesOrigin == SCR_RUN);

  if (sessionOn) {
    src = sessionSoundMode;
    scapeIdx = sessionSoundscapeIndex;
    if (src == AUDIO_SRC_SOUNDSCAPE && scapeIdx < 0) src = AUDIO_SRC_TONE;
  } else if (screen == SCR_SOUNDSCAPES && previewSoundscapeIndex >= 0) {
    src = AUDIO_SRC_SOUNDSCAPE;
    scapeIdx = previewSoundscapeIndex;
  }

  if (src == AUDIO_SRC_SOUNDSCAPE) audio_setSoundscapeFile(sdmedia_soundscapePath(scapeIdx));
  audio_setToneFrequency(liveFrequency());
  audio_setSource(src);
}

// Session setup that has to happen once each time a NEW preset/sequence/
// custom frequency is opened: pick the matching default soundscape.
const char* preparedForName = nullptr;
float preparedForFreq = -1;

void prepareSessionAudioIfNew() {
  if (selName == preparedForName && selFreq == preparedForFreq) return;
  preparedForName = selName;
  preparedForFreq = selFreq;
  sessionSoundscapeIndex = defaultSoundscapeFor(selCategoryName, selFreq);
}

const char* soundModeLabel(char* buf, size_t len) {
  if (sessionSoundMode == AUDIO_SRC_OFF) {
    snprintf(buf, len, "Sound: Off");
  } else if (sessionSoundMode == AUDIO_SRC_SOUNDSCAPE && sessionSoundscapeIndex >= 0) {
    char name[24];
    prettySoundName(sessionSoundscapeIndex, name, sizeof(name));
    snprintf(buf, len, "Sound: %s", name);
  } else {
    snprintf(buf, len, "Sound: Tone");
  }
  return buf;
}

void outputStatusText(char* buf, size_t len) {
  if (audioUsingBluetooth()) {
    // Once a second: is the BT stack actually pulling audio, and is it silent?
    static uint32_t lastFrames = 0;
    static unsigned long lastCheck = 0;
    static bool flowing = false;
    if (millis() - lastCheck >= 900) {
      uint32_t fr = audio_btFramesSent();
      flowing = (fr != lastFrames);
      lastFrames = fr;
      lastCheck = millis();
    }
    // Plain words on screen; the technical detail goes to the USB cable only.
    static int lastDiag = -1;
    int diag = !audio_btIsConnected() ? 0 : !flowing ? 1 : (audio_getSource() != AUDIO_SRC_OFF && audio_btLastPeak() == 0) ? 2 : 3;
    if (diag != lastDiag) {
      const char* names[] = {"not connected", "connected, no audio requested", "streaming silence", "streaming audio"};
      Serial.printf("[BT] %s\n", names[diag]);
      lastDiag = diag;
    }
    if (!audio_btIsConnected()) btStatusText(buf, len);
    else snprintf(buf, len, "Playing on %s", btDeviceName);
  } else {
    snprintf(buf, len, "Playing on onboard speaker");
  }
}

// Small targeted redraws - called once a second. Each piece only repaints
// when its text actually changed, so nothing on screen flickers.
char lastCountdownText[24] = "";
char lastFreqText[32] = "";
char lastStatusText[32] = "";

// Tesla-coil rings above and below the hexagon; they pulse with the session.
void drawRunCoils(bool bright) {
  uint16_t a = bright ? MADD_CYAN : MADD_EDGE, b = bright ? MADD_COIL2 : tft.color565(44, 36, 86);
  tft.drawEllipse(RUN_HEX_X, 66, 26, 5, a);
  tft.drawEllipse(RUN_HEX_X, 214, 84, 13, a);
  tft.drawEllipse(RUN_HEX_X, 233, 96, 15, b);
}

void drawRunHexContents(const char* timeText, const char* freqText) {
  drawHex(RUN_HEX_X, RUN_HEX_Y, RUN_HEX_R, MADD_PANEL, MADD_MAGENTA);
  tft.setTextDatum(MC_DATUM);
  tft.setFreeFont(FONT_LG);
  tft.setTextColor(MADD_TEXT);
  tft.drawString(timeText, RUN_HEX_X, RUN_HEX_Y - 10);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(MADD_CYAN);
  tft.drawString(freqText, RUN_HEX_X, RUN_HEX_Y + 18);
  tft.setTextDatum(TL_DATUM);
}

void drawCountdownOnly() {
  char timeBuf[24];
  if (sessionTimerArmed && (waveform_isRunning() || sessionPaused)) {
    unsigned long elapsedSec = (sessionEffectiveMillis() - sessionStartMillis) / 1000UL;
    long remainingSec = (long)timerMinutes * 60 - (long)elapsedSec;
    if (remainingSec < 0) remainingSec = 0;
    snprintf(timeBuf, sizeof(timeBuf), "%ld:%02ld", remainingSec / 60, remainingSec % 60);
  } else if (sessionPaused) {
    snprintf(timeBuf, sizeof(timeBuf), "Paused");
  } else if (waveform_isRunning()) {
    snprintf(timeBuf, sizeof(timeBuf), "On");       // timer set to Off - runs until stopped
  } else {
    if (timerMinutes > 0) snprintf(timeBuf, sizeof(timeBuf), "%d:00", timerMinutes);
    else snprintf(timeBuf, sizeof(timeBuf), "Ready");
  }

  char freqBuf[32], f[16];
  formatFreq(liveFrequency(), f, sizeof(f));
  if (programActive) snprintf(freqBuf, sizeof(freqBuf), "%s  %d/%d", f, programStepIndex + 1, currentProgram.stepCount);
  else snprintf(freqBuf, sizeof(freqBuf), "%s", f);

  if (strcmp(timeBuf, lastCountdownText) != 0 || strcmp(freqBuf, lastFreqText) != 0) {
    drawRunHexContents(timeBuf, freqBuf);
    strcpy(lastCountdownText, timeBuf);
    strcpy(lastFreqText, freqBuf);
  }

  char status[32];
  outputStatusText(status, sizeof(status));
  if (strcmp(status, lastStatusText) != 0) {
    tft.fillRect(240, 276, 218, 18, MADD_PANEL);
    drawFittedText(240, 278, 216, status, FONT_SM, audioUsingBluetooth() ? btStatusColor() : MADD_DIM, MADD_PANEL);
    strcpy(lastStatusText, status);
  }
  drawTopStatus(false);
}

void drawRunScreen() {
  prepareSessionAudioIfNew();
  drawAuroraBackground();
  drawTopBar(selName, true);
  drawRunCoils(true);
  drawChamfer(runPanel, MADD_PANEL, MADD_EDGE, 12);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(MADD_DIM);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Sound", 240, 52);

  // force the once-a-second pieces to repaint on this fresh screen
  lastCountdownText[0] = 0;
  lastFreqText[0] = 0;
  lastStatusText[0] = 0;
  refreshRunControls();
}

// One "-  Label value  +" row inside the right panel.
void drawValueRow(Rect minus, Rect plus, const char* text) {
  drawChamferButton(minus, "-", MADD_PANEL, MADD_EDGE, MADD_TEXT, FONT_LG);
  drawChamferButton(plus,  "+", MADD_PANEL, MADD_EDGE, MADD_TEXT, FONT_LG);
  int x = minus.x + minus.w + 4, w = plus.x - x - 4;
  tft.fillRect(x, minus.y, w, minus.h, MADD_PANEL);
  tft.setFreeFont(FONT_SM);
  tft.setTextColor(MADD_TEXT);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, x + w / 2, minus.y + minus.h / 2);
  tft.setTextDatum(TL_DATUM);
}

// Power meter that fills through your spectrum colors.
void drawPowerMeter() {
  int lit = (powerDisplay + 9) / 10; // 1-10 segments
  for (int i = 0; i < 10; i++) {
    uint16_t c = (i < lit) ? MADD_SPECTRUM[i * 6 / 10] : tft.color565(42, 26, 64);
    tft.fillRect(240 + i * 22, 174, 20, 12, c);
  }
}

// Everything that can change from a tap while staying on the Run screen.
void refreshRunControls() {
  bool running = waveform_isRunning();
  if (sessionPaused)      drawChamferButton(btnStartStop, "Resume", tft.color565(12, 58, 40), COLOR_GOOD, MADD_TEXT, FONT_LG);
  else if (running)       drawChamferButton(btnStartStop, "Pause", tft.color565(90, 54, 6), MADD_SPECTRUM[0], MADD_TEXT, FONT_LG);
  else                    drawChamferButton(btnStartStop, "Start", tft.color565(12, 58, 40), COLOR_GOOD, MADD_TEXT, FONT_LG);
  drawChamferButton(btnBackFromRun, "Stop", tft.color565(58, 13, 26), MADD_SPECTRUM[2], MADD_TEXT);
  bool isFav = (selectedIndex >= 0) && favoriteBits[selectedIndex];
  if (selectedIndex >= 0) {
    drawChamferButton(btnFavToggle, "*", isFav ? tft.color565(90, 54, 6) : MADD_PANEL, isFav ? MADD_SPECTRUM[0] : MADD_EDGE, MADD_TEXT, FONT_LG);
  }

  // Sound choice - the lit one is what's playing
  bool haveScapes = sdmedia_soundscapeCount() > 0;
  uint16_t onFill = tft.color565(12, 58, 68);
  drawChamferButton(btnSndOff, "Off", sessionSoundMode == AUDIO_SRC_OFF ? tft.color565(40, 30, 64) : MADD_PANEL,
                    sessionSoundMode == AUDIO_SRC_OFF ? MADD_DIM : MADD_EDGE, MADD_TEXT);
  drawChamferButton(btnSndTone, "Tone", sessionSoundMode == AUDIO_SRC_TONE ? onFill : MADD_PANEL,
                    sessionSoundMode == AUDIO_SRC_TONE ? MADD_CYAN : MADD_EDGE, MADD_TEXT);
  drawChamferButton(btnSndScape, "Nature", sessionSoundMode == AUDIO_SRC_SOUNDSCAPE ? onFill : MADD_PANEL,
                    sessionSoundMode == AUDIO_SRC_SOUNDSCAPE ? MADD_CYAN : MADD_EDGE, haveScapes ? MADD_TEXT : MADD_EDGE);

  char nowBuf[40];
  if (sessionSoundMode == AUDIO_SRC_SOUNDSCAPE && sessionSoundscapeIndex >= 0) {
    char name[24];
    prettySoundName(sessionSoundscapeIndex, name, sizeof(name));
    snprintf(nowBuf, sizeof(nowBuf), "%s  (tap to change)", name);
  } else if (sessionSoundMode == AUDIO_SRC_TONE) {
    char f[16];
    formatFreq(liveFrequency(), f, sizeof(f));
    snprintf(nowBuf, sizeof(nowBuf), "Tone matched to %s", f);
  } else {
    snprintf(nowBuf, sizeof(nowBuf), "Sound off");
  }
  tft.fillRect(btnPickSound.x, btnPickSound.y, btnPickSound.w, btnPickSound.h, MADD_PANEL);
  drawFittedText(btnPickSound.x, btnPickSound.y + 4, btnPickSound.w, nowBuf, FONT_SM, MADD_CYAN, MADD_PANEL);

  char buf[24];
  snprintf(buf, sizeof(buf), "Power %d%%", powerDisplay);
  drawValueRow(btnPwrMinus, btnPwrPlus, buf);
  drawPowerMeter();
  snprintf(buf, sizeof(buf), "%s %d%%", audioUsingBluetooth() ? "BT vol" : "Volume", volumePercent);
  drawValueRow(btnVolMinus, btnVolPlus, buf);
  if (timerMinutes > 0) snprintf(buf, sizeof(buf), "Timer %d min", timerMinutes);
  else snprintf(buf, sizeof(buf), "Timer off");
  drawValueRow(btnTmrMinus, btnTmrPlus, buf);

  drawCountdownOnly();
}

// Pulses the coil rings in step with the session (visible up to ~4 Hz).
void updateRunPulse() {
  static bool lastBright = true;
  bool bright = true;
  if (waveform_isRunning()) {
    float hz = liveFrequency();
    if (hz > 4.0f) hz = 4.0f;
    if (hz < 0.5f) hz = 0.5f;
    unsigned long period = (unsigned long)(1000.0f / hz);
    bright = (millis() % period) < period / 2;
  }
  if (bright != lastBright) {
    drawRunCoils(bright);
    lastBright = bright;
  }
}

void endSession() {
  if (waveform_isRunning() || sessionPaused) {
    int mins = (int)((sessionEffectiveMillis() - sessionStartMillis) / 60000UL);
    addLogEntry(selName, selFreq, mins);
  }
  stopProgram();
  waveform_stop();
  sessionTimerArmed = false;
  sessionPaused = false;
}

void handleRunTouch(int x, int y) {
  if (touchInRect(x, y, btnRunTitle) || touchInRect(x, y, btnBackFromRun)) {
    endSession();
    screen = runScreenOrigin;
    return;
  }
  if (selectedIndex >= 0 && touchInRect(x, y, btnFavToggle)) {
    setFavorite(selectedIndex, !favoriteBits[selectedIndex]);
    return;
  }
  if (touchInRect(x, y, btnPwrMinus) || touchInRect(x, y, btnPwrPlus)) {
    bool up = touchInRect(x, y, btnPwrPlus);
    int step = (powerDisplay < 10 || (!up && powerDisplay <= 10)) ? 1 : 5;
    powerDisplay += up ? step : -step;
    if (powerDisplay < 1) powerDisplay = 1;
    if (powerDisplay > 100) powerDisplay = 100;
    waveform_setIntensity(actualIntensityPercent());
    return;
  }
  if (touchInRect(x, y, btnVolMinus) || touchInRect(x, y, btnVolPlus)) {
    volumePercent += touchInRect(x, y, btnVolPlus) ? 5 : -5;
    if (volumePercent < 0) volumePercent = 0;
    if (volumePercent > 100) volumePercent = 100;
    audio_setVolume((uint8_t)volumePercent);
    saveSetupInfo();
    return;
  }
  if (touchInRect(x, y, btnTmrMinus) || touchInRect(x, y, btnTmrPlus)) {
    timerMinutes += touchInRect(x, y, btnTmrPlus) ? 5 : -5;
    if (timerMinutes < 0) timerMinutes = 0;
    if (timerMinutes > 60) timerMinutes = 60;
    if (waveform_isRunning()) {
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    return;
  }
  if (touchInRect(x, y, btnStartStop)) {
    if (sessionPaused) {
      unsigned long pauseDuration = millis() - pauseStartMillis;
      sessionStartMillis += pauseDuration;
      if (programActive) programStepStartMillis += pauseDuration;
      sessionPaused = false;
      waveform_start(pausedFreq, selWave, actualIntensityPercent());
    } else if (waveform_isRunning()) {
      pausedFreq = waveform_currentFreq();
      sessionPaused = true;
      pauseStartMillis = millis();
      waveform_stop();
    } else if (pendingSequenceIndex >= 0) {
      startProgram(SEQUENCES[pendingSequenceIndex]);
      pendingSequenceIndex = -1;
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    } else {
      waveform_start(selFreq, selWave, actualIntensityPercent());
      sessionStartMillis = millis();
      sessionTimerArmed = (timerMinutes > 0);
    }
    return;
  }
  if (touchInRect(x, y, btnSndOff))  { sessionSoundMode = AUDIO_SRC_OFF;  saveSetupInfo(); return; }
  if (touchInRect(x, y, btnSndTone)) { sessionSoundMode = AUDIO_SRC_TONE; saveSetupInfo(); return; }
  if (sdmedia_soundscapeCount() > 0 && touchInRect(x, y, btnSndScape)) {
    sessionSoundMode = AUDIO_SRC_SOUNDSCAPE;
    if (sessionSoundscapeIndex < 0) sessionSoundscapeIndex = defaultSoundscapeFor(selCategoryName, selFreq);
    saveSetupInfo();
    return;
  }
  if (sdmedia_soundscapeCount() > 0 && sessionSoundMode == AUDIO_SRC_SOUNDSCAPE && touchInRect(x, y, btnPickSound)) {
    soundscapesOrigin = SCR_RUN;
    if (sessionSoundscapeIndex >= 0) soundscapesPage = sessionSoundscapeIndex / SOUNDSCAPES_PER_PAGE;
    screen = SCR_SOUNDSCAPES; // the session keeps running while picking
    return;
  }
  // Tapping the status line retries Bluetooth when it wasn't found.
  if (audioUsingBluetooth() && x >= 240 && y >= 272 && y < 300 && audio_btStatus() == BT_STATUS_NOT_FOUND) {
    audio_btRetry();
  }
}

// ---------------------------------------------------------------------
// Screen drawing dispatch
// ---------------------------------------------------------------------
void drawScreen(Screen s) {
  switch (s) {
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
  }
}

// After a tap that kept us on the same screen: repaint only what can
// change. Screens that are cheap (plain background) just redraw fully;
// the splash-background screens only touch their controls.
void refreshScreen(Screen s) {
  switch (s) {
    case SCR_WELCOME:  refreshWelcomeCheckbox(); break;
    case SCR_RUN:      refreshRunControls(); break;
    case SCR_CATEGORY: break; // only the hidden stats panel changes, tracked separately
    case SCR_LIST: case SCR_SEQUENCES: case SCR_LOG: case SCR_PERSON_PICKER: case SCR_WIFI_SETUP:
      break;          // page changes are tracked separately; nothing else changes on tap
    default:           drawScreen(s); break;
  }
}

// ---------------------------------------------------------------------
// Touch calibration with targets INSET from the edges, so it works with
// a bezel covering the screen border. (The library's built-in routine
// puts its arrows in the extreme corners - under the bezel.) The raw
// readings at the four inset targets are extended out to the screen
// edges, producing the exact same calibration format tft.setTouch() uses.
// ---------------------------------------------------------------------
static const int CAL_INSET = 48;

static void drawCalTarget(int x, int y, uint16_t color) {
  tft.drawCircle(x, y, 14, color);
  tft.drawCircle(x, y, 13, color);
  tft.drawFastHLine(x - 22, y, 45, color);
  tft.drawFastVLine(x, y - 22, 45, color);
}

// Waits for a firm press, averages 8 raw readings, then waits for release.
static void readCalPoint(int32_t& rx, int32_t& ry) {
  const uint16_t Z_MIN = 350;
  while (tft.getTouchRawZ() > Z_MIN) delay(10); // release from any previous press
  delay(150);
  int32_t sx = 0, sy = 0;
  int n = 0;
  while (n < 8) {
    if (tft.getTouchRawZ() > Z_MIN) {
      uint16_t x, y;
      tft.getTouchRaw(&x, &y);
      sx += x; sy += y; n++;
      delay(15);
    } else {
      n = 0; sx = 0; sy = 0; // finger lifted mid-sample - start this point over
      delay(10);
    }
  }
  rx = sx / 8; ry = sy / 8;
  while (tft.getTouchRawZ() > Z_MIN) delay(10);
}

void runTouchCalibration() {
  const int W = tft.width(), H = tft.height(), M = CAL_INSET;
  // Same order as the library: top-left, bottom-left, top-right, bottom-right
  const int px[4] = { M, M, W - 1 - M, W - 1 - M };
  const int py[4] = { M, H - 1 - M, M, H - 1 - M };
  int32_t v[8];

  for (int i = 0; i < 4; i++) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(FONT_SM);
    tft.drawString("Touch Setup", W / 2, H / 2 - 22);
    tft.setTextColor(COLOR_TEXT_DIM, TFT_BLACK);
    tft.drawString("Press and hold the center of the target", W / 2, H / 2 + 4);
    char step[16];
    snprintf(step, sizeof(step), "%d of 4", i + 1);
    tft.drawString(step, W / 2, H / 2 + 28);
    drawCalTarget(px[i], py[i], TFT_WHITE);
    readCalPoint(v[i * 2], v[i * 2 + 1]);
    drawCalTarget(px[i], py[i], TFT_GREEN);
    delay(250);
  }

  // Which raw axis follows screen X? TL->BL moves along screen Y, so if raw
  // X changed more than raw Y there, the axes are swapped.
  bool rotate = abs(v[0] - v[2]) > abs(v[1] - v[3]);
  float nearX, farX, nearY, farY;
  if (!rotate) {
    nearX = (v[0] + v[2]) / 2.0f; farX = (v[4] + v[6]) / 2.0f;
    nearY = (v[1] + v[5]) / 2.0f; farY = (v[3] + v[7]) / 2.0f;
  } else {
    nearX = (v[1] + v[3]) / 2.0f; farX = (v[5] + v[7]) / 2.0f;
    nearY = (v[0] + v[4]) / 2.0f; farY = (v[2] + v[6]) / 2.0f;
  }
  // Extend from the inset targets out to pixel 0 and pixel W/H.
  float sx = (farX - nearX) / (float)(W - 1 - 2 * M);
  float sy = (farY - nearY) / (float)(H - 1 - 2 * M);
  float x0 = nearX - M * sx, x1 = nearX + (W - M) * sx;
  float y0 = nearY - M * sy, y1 = nearY + (H - M) * sy;

  bool invX = false, invY = false;
  if (x0 > x1) { float t = x0; x0 = x1; x1 = t; invX = true; }
  if (y0 > y1) { float t = y0; y0 = y1; y1 = t; invY = true; }
  if (x0 < 1) x0 = 1;
  if (y0 < 1) y0 = 1;

  uint16_t cal[5];
  cal[0] = (uint16_t)x0;
  cal[1] = (uint16_t)max(1.0f, x1 - x0);
  cal[2] = (uint16_t)y0;
  cal[3] = (uint16_t)max(1.0f, y1 - y0);
  cal[4] = (rotate ? 1 : 0) | (invX ? 2 : 0) | (invY ? 4 : 0);
  tft.setTouch(cal);

  Preferences p;
  p.begin("tftcal", false);
  p.putBytes("calInset", cal, sizeof(cal));
  p.end();

  // Quick check so the person can see it worked before continuing
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setFreeFont(FONT_SM);
  tft.drawString("Done! Tap anywhere to test - a dot", W / 2, 120);
  tft.drawString("should appear under your finger.", W / 2, 144);
  tft.setTextColor(COLOR_TEXT_DIM, TFT_BLACK);
  tft.drawString("(continues in 6 seconds)", W / 2, 180);
  unsigned long start = millis();
  while (millis() - start < 6000) {
    uint16_t tx, ty;
    if (tft.getTouch(&tx, &ty)) tft.fillCircle(tx, ty, 4, TFT_GREEN);
    delay(15);
  }
}

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------
Screen lastDrawnScreen = SCR_WELCOME;
bool fullRedrawRequested = true; // first loop pass draws the Welcome screen
int lastDrawnPage = -1;
int lastDrawnSequencesPage = -1;
int lastDrawnSoundscapesPage = -1;
int lastDrawnSettingsPage = -1;
bool lastDrawnShowDeviceStats = false;

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(1); // landscape 480x320
  initTheme();
  tft.fillScreen(COLOR_BG);

  // Audio engine first: it claims the speaker DAC (IO26) and then hands
  // IO25 back so waveform_begin() below can own it for the coil PWM.
  audio_begin();
  waveform_begin();

  // SD card: splash + soundscape list. Missing card = plain screens, no sounds.
  bool sdOk = sdmedia_begin();
  Serial.printf("SD card mount: %s\n", sdOk ? "OK" : "FAILED");
  if (sdOk && sdmedia_showSplash()) {
    splashOnScreen = true;
    delay(1500);
  }
  Serial.printf("Soundscapes found: %d\n", sdmedia_scanSoundscapes());

  // Touch calibration. Uses its own "calInset" key so every unit re-runs
  // the new bezel-friendly calibration once. Holding a finger on the
  // screen during the splash also forces a recalibration.
  uint16_t calData[5];
  Preferences prefs;
  prefs.begin("tftcal", true);
  bool haveValidCal = prefs.isKey("calInset");
  if (haveValidCal) prefs.getBytes("calInset", calData, sizeof(calData));
  prefs.end();
  bool forceCal = tft.getTouchRawZ() > 350;
  if (haveValidCal && !forceCal) {
    tft.setTouch(calData);
  } else {
    splashOnScreen = false;
    runTouchCalibration();
  }

  loadPeople();
  loadFavorites();
  loadSessionLog();
  loadSetupInfo();
  wifitime_begin(); // brief NTP sync only if WiFi was set up; WiFi is off again afterwards

  audio_setVolume((uint8_t)volumePercent);
  audio_setHeadphonesMode(btHeadphonesMode);
  audio_setBtDeviceName(btDeviceName);
  runPendingUpdateCheck(); // before Bluetooth starts, so the download has the memory it needs
  applyAudioOutput(); // Bluetooth connects in the background if it's the chosen output
}

void loop() {
  uint16_t tx = 0, ty = 0;
  bool touched = tft.getTouch(&tx, &ty);

  if (screen == SCR_WIFI_SETUP && wifitime_isPortalActive()) {
    if (wifitime_processPortal()) {
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString(wifitime_isConfigured() ? "WiFi connected!" : "Setup timed out - try again.", 240, 160);
      delay(1500);
      if (restartAfterWifiSetup) ESP.restart(); // brings Bluetooth back
      screen = SCR_SETTINGS;
      fullRedrawRequested = true;
    }
  }

  updateProgram();

  // Auto-stop when the session timer elapses
  bool runNeedsRefresh = false;
  if (sessionTimerArmed && waveform_isRunning()) {
    unsigned long elapsedMin = (millis() - sessionStartMillis) / 60000UL;
    if ((int)elapsedMin >= timerMinutes) {
      endSession();
      runNeedsRefresh = true;
    }
  }
  // A Sequence can also finish on its own
  static bool wasRunning = false;
  if (wasRunning && !waveform_isRunning() && !sessionPaused) runNeedsRefresh = true;
  wasRunning = waveform_isRunning();

  if (devModeUnlocked && (millis() - devModeStartMillis) / 60000UL >= DEV_MODE_TIMEOUT_MIN) {
    relockDevMode();
    if (screen == SCR_SETTINGS) fullRedrawRequested = true;
  }

  // ---- touch: one handler call per new press ----
  bool needsRefresh = false;
  static bool wasTouched = false;
  if (touched && !wasTouched) {
    Screen before = screen;
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
    }
    if (screen == before) needsRefresh = true;
    // Handlers that painted a temporary message (greeting, "checking for
    // updates...") leave the screen dirty - repaint those fully.
    if (before == SCR_PERSON_PICKER || before == SCR_UPDATE) fullRedrawRequested = true;
  }
  wasTouched = touched;

  syncAudio();

  // ---- drawing: at most ONE full draw per pass ----
  bool changed = fullRedrawRequested || screen != lastDrawnScreen ||
                 (screen == SCR_LIST && listPage != lastDrawnPage) ||
                 (screen == SCR_SEQUENCES && sequencesPage != lastDrawnSequencesPage) ||
                 (screen == SCR_SOUNDSCAPES && soundscapesPage != lastDrawnSoundscapesPage) ||
                 (screen == SCR_CATEGORY && showDeviceStats != lastDrawnShowDeviceStats) ||
                 (screen == SCR_SETTINGS && settingsPage != lastDrawnSettingsPage);
  if (changed) {
    drawScreen(screen);
    lastDrawnScreen = screen;
    lastDrawnPage = listPage;
    lastDrawnSequencesPage = sequencesPage;
    lastDrawnSoundscapesPage = soundscapesPage;
    lastDrawnShowDeviceStats = showDeviceStats;
    lastDrawnSettingsPage = settingsPage;
    fullRedrawRequested = false;
  } else if (needsRefresh) {
    refreshScreen(screen);
  } else if (runNeedsRefresh && screen == SCR_RUN) {
    refreshRunControls();
  }

  if (screen == SCR_RUN) updateRunPulse();
  if (screen == SCR_CATEGORY) { static unsigned long lastTopTick = 0; if (millis() - lastTopTick > 1000) { drawTopStatus(false); lastTopTick = millis(); } }
  // Once-a-second small updates on the Run screen (countdown, live
  // frequency, BT status) - each only repaints if its text changed.
  if (screen == SCR_RUN) {
    static unsigned long lastTick = 0;
    if (millis() - lastTick >= 1000) {
      drawCountdownOnly();
      lastTick = millis();
    }
  }

  // Bluetooth pop-up: whenever Bluetooth connects or drops, show a short
  // banner at the bottom of whatever screen is up, then tidy it away.
  static uint32_t lastBtChanges = 0;
  static unsigned long toastUntil = 0;
  static BtStatus lastShownBt = BT_STATUS_OFF;
  if (audioUsingBluetooth()) {
    BtStatus st = audio_btStatus();
    bool edge = (audio_btStatusChanges() != lastBtChanges) ||
                (st == BT_STATUS_NOT_FOUND && lastShownBt != BT_STATUS_NOT_FOUND);
    if (edge && (st == BT_STATUS_CONNECTED || st == BT_STATUS_RECONNECTING || st == BT_STATUS_NOT_FOUND) && st != lastShownBt) {
      char msg[48];
      if (st == BT_STATUS_CONNECTED) snprintf(msg, sizeof(msg), "Bluetooth connected: %s", btDeviceName);
      else if (st == BT_STATUS_RECONNECTING) snprintf(msg, sizeof(msg), "Bluetooth dropped - reconnecting...");
      else snprintf(msg, sizeof(msg), "%s not found. Is it on?", btDeviceName);
      Rect t = {40, 258, 400, 40};
      tft.fillRoundRect(t.x, t.y, t.w, t.h, 12, COLOR_PANEL);
      tft.drawRoundRect(t.x, t.y, t.w, t.h, 12, st == BT_STATUS_CONNECTED ? COLOR_GOOD : COLOR_WARN);
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(TFT_WHITE, COLOR_PANEL);
      tft.setTextDatum(MC_DATUM);
      tft.drawString(msg, 240, t.y + t.h / 2);
      tft.setTextDatum(TL_DATUM);
      toastUntil = millis() + 2500;
      lastShownBt = st;
    }
    lastBtChanges = audio_btStatusChanges();
    if (st == BT_STATUS_SEARCHING || st == BT_STATUS_CONNECTING) lastShownBt = st;
  }
  if (toastUntil && millis() > toastUntil) {
    toastUntil = 0;
    fullRedrawRequested = true; // repaint the screen underneath the banner
  }
  // A picked device should connect within ~25 s. If it hasn't, fall back to
  // the proven path: it's already saved, so restart once and connect at boot.
  if (btPickAt) {
    if (audio_btIsConnected()) btPickAt = 0;
    else if (millis() - btPickAt > 25000) {
      tft.fillScreen(COLOR_BG);
      tft.setFreeFont(FONT_LG);
      tft.setTextColor(TFT_WHITE, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Finishing Bluetooth setup", 240, 140);
      tft.setFreeFont(FONT_SM);
      tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
      tft.drawString("Restarting once to connect - about 10 seconds.", 240, 175);
      tft.setTextDatum(TL_DATUM);
      delay(1500);
      audio_btEnd();
      ESP.restart();
    }
  }

  // The Bluetooth screen's status line follows along live.
  if (screen == SCR_BT_SCAN) {
    static unsigned long lastBtLine = 0;
    if (millis() - lastBtLine > 1000) { drawBtStatusLine(20, 38, 330); lastBtLine = millis(); }
  }

  // Live BT scan list - redraw only when something actually changed.
  static int lastBtScanRedrawCount = -1;
  static bool lastBtScanActiveState = false;
  if (screen == SCR_BT_SCAN && (audio_btIsScanning() || scanRequested)) {
    bool nowActive = audio_btIsScanning();
    if (nowActive) scanRequested = false;
    int currentCount = audio_btScanResultCount();
    if (currentCount != lastBtScanRedrawCount || nowActive != lastBtScanActiveState) {
      drawBtScanScreen();
      lastBtScanRedrawCount = currentCount;
      lastBtScanActiveState = nowActive;
    }
  } else {
    lastBtScanRedrawCount = -1;
    lastBtScanActiveState = false;
  }

  delay(15); // touch is polled; audio runs in its own task so this no longer matters for sound
}
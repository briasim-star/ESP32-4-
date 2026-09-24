#include "wifi_time.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>

// The temporary setup network's name/password - shown on the device's
// own screen during setup so only someone standing at the device can
// join it. WPA2-protected (not open), per the security review.
static const char* SETUP_AP_NAME = "MADD-PEMF-Setup";
static const char* SETUP_AP_PASSWORD = "maddpemf2026";

static const char* WIFI_NS = "wifitime";
static bool g_timeSynced = false;

// The WiFiManager instance needs to persist across multiple loop()
// iterations for non-blocking operation (see wifitime_processPortal()),
// so it can't be a local variable inside one function call like the
// rest of this file's style - it's created once, on first use.
static WiFiManager* wm = nullptr;
static bool portalActive = false;

static bool markedConfigured() {
  Preferences p;
  p.begin(WIFI_NS, true);
  bool v = p.getBool("configured", false);
  p.end();
  return v;
}

static void setMarkedConfigured(bool val) {
  Preferences p;
  p.begin(WIFI_NS, false);
  p.putBool("configured", val);
  p.end();
}

// Time zones (Settings -> Time zone). POSIX rules include daylight saving.
static const char* TZ_NAMES[] = {"Eastern", "Central", "Mountain", "Arizona", "Pacific", "Alaska", "Hawaii",
                                 "Atlantic", "UTC", "UK", "Central Europe", "Australia East"};
static const char* TZ_RULES[] = {"EST5EDT,M3.2.0,M11.1.0", "CST6CDT,M3.2.0,M11.1.0", "MST7MDT,M3.2.0,M11.1.0", "MST7",
                                 "PST8PDT,M3.2.0,M11.1.0", "AKST9AKDT,M3.2.0,M11.1.0", "HST10", "AST4ADT,M3.2.0,M11.1.0",
                                 "UTC0", "GMT0BST,M3.5.0/1,M10.5.0", "CET-1CEST,M3.5.0,M10.5.0/3", "AEST-10AEDT,M10.1.0,M4.1.0/3"};
static const int TZ_COUNT = sizeof(TZ_NAMES) / sizeof(TZ_NAMES[0]);
static int g_tz = 0;

int wifitime_tzCount() { return TZ_COUNT; }
int wifitime_tzIndex() { return g_tz; }
const char* wifitime_tzName(int i) { return (i >= 0 && i < TZ_COUNT) ? TZ_NAMES[i] : "UTC"; }
void wifitime_setTz(int i) {
  if (i < 0 || i >= TZ_COUNT) i = 0;
  g_tz = i;
  setenv("TZ", TZ_RULES[i], 1);
  tzset();
  Preferences p;
  p.begin(WIFI_NS, false);
  p.putInt("tz", i);
  p.end();
}
void wifitime_loadTz() {
  Preferences p;
  p.begin(WIFI_NS, true);
  g_tz = p.getInt("tz", 0);
  p.end();
  if (g_tz < 0 || g_tz >= TZ_COUNT) g_tz = 0;
  setenv("TZ", TZ_RULES[g_tz], 1);
  tzset();
}

static bool doNtpSync(unsigned long timeoutMs) {
  configTzTime(TZ_RULES[g_tz], "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  unsigned long start = millis();
  while (!getLocalTime(&timeinfo, 100)) {
    if (millis() - start > timeoutMs) return false;
  }
  return true;
}

bool wifitime_isConfigured() {
  return markedConfigured();
}

void wifitime_begin() {
  if (!markedConfigured()) return; // never touch WiFi until explicitly set up

  WiFi.mode(WIFI_STA);
  WiFi.begin(); // reconnects using the ESP32's own saved credentials
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 6000) { // runs before Bluetooth starts - keep it short
    delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    g_timeSynced = doNtpSync(3000);
  }
  Serial.printf("[TIME] clock %s after %lu ms\n", g_timeSynced ? "set" : "not set", millis() - start);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ---------------------------------------------------------------------
// Setup portal - non-blocking. A fully-blocking autoConnect() call was
// tried first, but confirmed a real problem: there was no way to cancel
// out of it once started (up to the full 3-minute timeout, no Back
// button could work, since our own touch-polling loop never got a
// chance to run while blocked inside that one call). This version lets
// our main loop keep running (and the Home button work) while waiting
// for someone to actually pick a network and enter a password on their
// phone. One real, documented limit worth knowing: once credentials ARE
// submitted, the library's own connection attempt is still a blocking
// call internally (a confirmed limitation of the library itself, not
// something we can avoid) - so Cancel works up until that specific
// moment, after which the actual connect attempt (typically a few
// seconds, not minutes) has to finish on its own either way.
// ---------------------------------------------------------------------
static void (*connectingCallback)() = nullptr;
void wifitime_onConnecting(void (*cb)()) { connectingCallback = cb; }

// Fired by WiFiManager right after the person submits their network and
// password, just before the (blocking) connection attempt - so the screen
// can say "Connecting..." instead of looking frozen.
static void onPreSave() {
  if (connectingCallback) connectingCallback();
}

void wifitime_beginSetupPortal() {
  if (wm == nullptr) wm = new WiFiManager();
  wm->setConfigPortalBlocking(false);
  wm->setConfigPortalTimeout(180);
  wm->setConnectTimeout(12);          // a wrong password fails in ~12 s instead of hanging
  wm->setPreSaveConfigCallback(onPreSave);
  portalActive = true;
  // startConfigPortal (not autoConnect): open the setup hotspot right away.
  // autoConnect first retried the OLD saved network and made the screen
  // stall before the hotspot even appeared.
  wm->startConfigPortal(SETUP_AP_NAME, SETUP_AP_PASSWORD);
}

// Call every loop() iteration while wifitime_isPortalActive() is true.
// Returns true once the portal has finished (connected or timed out),
// so the caller knows to check wifitime_isConfigured() and move on.
bool wifitime_processPortal() {
  if (!portalActive || wm == nullptr) return true;
  wm->process();
  if (!wm->getConfigPortalActive()) {
    // Finished - either connected successfully or timed out
    portalActive = false;
    if (WiFi.status() == WL_CONNECTED) {
      setMarkedConfigured(true);
      g_timeSynced = doNtpSync(3000); // quicker clock sync after setup
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return true;
  }
  return false;
}

bool wifitime_isPortalActive() { return portalActive; }

void wifitime_cancelPortal() {
  if (wm != nullptr && portalActive) {
    wm->stopConfigPortal();
  }
  portalActive = false;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// Erases the saved network WITHOUT turning the WiFi radio on. The old way
// (WiFiManager::resetSettings) switched WiFi on, which could run the board
// out of memory while Bluetooth was running and crash it.
void wifitime_forgetNetwork() {
  Preferences p;
  p.begin("nvs.net80211", false); // the WiFi driver's own saved-network store
  p.clear();
  p.end();
  setMarkedConfigured(false);
  g_timeSynced = false;
}

bool wifitime_hasRealTime() { return g_timeSynced; }

time_t wifitime_now() {
  if (!g_timeSynced) return 0;
  time_t now;
  time(&now);
  return now;
}

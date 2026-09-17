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

// UTC - session log timestamps are relative anyway (date + rough time),
// so timezone offset isn't wired in here to keep this simple; can be
// added later as a Settings field if that turns out to matter.
static bool doNtpSync(unsigned long timeoutMs) {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
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
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    g_timeSynced = doNtpSync(5000);
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void wifitime_startSetupPortal() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // gives up automatically after 3 minutes
  bool ok = wm.autoConnect(SETUP_AP_NAME, SETUP_AP_PASSWORD);
  if (ok) {
    setMarkedConfigured(true);
    g_timeSynced = doNtpSync(5000);
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void wifitime_forgetNetwork() {
  WiFiManager wm;
  wm.resetSettings();
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

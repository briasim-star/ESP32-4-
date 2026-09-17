#pragma once
#include <Arduino.h>
#include <time.h>

// ---------------------------------------------------------------------
// Brief, on-demand WiFi - purely to sync the real date/time via NTP, so
// session log entries can show an actual date instead of just "started
// 47 minutes ago". The device is NEVER on WiFi except for these short,
// explicit windows:
//   - once at boot, IF WiFi has already been set up (quick reconnect +
//     resync, times out fast and gives up if the network isn't in range)
//   - during the captive-portal setup flow (Settings -> Set Up WiFi)
// It never listens for connections, accepts remote commands, or does
// anything else on the network. Credentials are never hardcoded or
// touched by our own code - they're entered by the person, once, through
// the captive portal, and stored by the WiFiManager library / the
// ESP32's own WiFi driver.
// ---------------------------------------------------------------------

void wifitime_begin();               // call once from setup() - fast, safe no-op if never configured
bool wifitime_isConfigured();
void wifitime_startSetupPortal();    // blocking - shows the captive portal, returns once done or timed out
void wifitime_forgetNetwork();       // clears saved credentials, same tap-twice-confirm pattern as elsewhere
bool wifitime_hasRealTime();         // true once NTP sync has succeeded this session
time_t wifitime_now();               // current real epoch time, or 0 if never synced

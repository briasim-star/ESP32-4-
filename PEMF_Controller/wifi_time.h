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

// Setup portal - non-blocking (see the design note in wifi_time.cpp for
// why: a fully-blocking version had a confirmed real bug where there was
// no way to cancel out of it once started).
void wifitime_beginSetupPortal();    // starts the portal, returns immediately
bool wifitime_processPortal();       // call every loop() iteration while active; returns true once finished
bool wifitime_isPortalActive();
void wifitime_cancelPortal();
void wifitime_onConnecting(void (*cb)()); // called just before connecting to the chosen network (show a message)        // call if the person backs out before finishing

void wifitime_forgetNetwork();       // clears saved credentials, same tap-twice-confirm pattern as elsewhere
bool wifitime_hasRealTime();         // true once NTP sync has succeeded this session
time_t wifitime_now();               // current real epoch time, or 0 if never synced

// Time zone (Settings -> Time zone), saved; applies daylight saving automatically.
void wifitime_loadTz();              // call once early in setup()
int wifitime_tzCount();
int wifitime_tzIndex();
const char* wifitime_tzName(int i);
void wifitime_setTz(int i);

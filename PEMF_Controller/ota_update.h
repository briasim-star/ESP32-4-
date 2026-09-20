#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// Wireless firmware updates - checks a small version file hosted on our
// own GitHub Pages site, and if a newer version is available, downloads
// and installs it over WiFi, no USB cable needed.
//
// Security: downloads happen over HTTPS, validated against a full
// bundle of standard trusted certificate authorities (the same kind of
// list a web browser uses) - not a single pinned certificate. A single
// pinned certificate is the more common shortcut, but has a real,
// documented failure mode: if the hosting provider ever rotates which
// certificate authority they use, every already-deployed device would
// silently and permanently lose the ability to receive further updates
// (the very mechanism meant to fix problems remotely breaks itself,
// with no remote recovery). The bundle approach avoids that.
// ---------------------------------------------------------------------

bool ota_checkForUpdate();              // blocking - contacts the server, returns true if a newer version is available
const char* ota_latestVersionString();  // valid only after ota_checkForUpdate() returns true
bool ota_downloadAndInstall();          // blocking - downloads, flashes, and reboots on success; returns false on failure (device is untouched)
const char* ota_lastErrorMessage();     // human-readable reason for the last failure, if any

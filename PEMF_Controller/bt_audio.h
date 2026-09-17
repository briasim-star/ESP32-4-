#pragma once
#include <Arduino.h>

// Optional: streams an audible representation of the current PEMF
// frequency to a paired Bluetooth headset/speaker over classic
// Bluetooth A2DP (uses the ESP32-A2DP library by pschatzmann).
//
// - If the target frequency is itself audible (>= 100 Hz), it plays that
//   frequency directly as a sine tone.
// - If the target frequency is below audible range, it plays a fixed
//   audible carrier tone (default 200 Hz) whose volume is pulsed
//   ("isochronic") at the target rate, so you can hear the timing even
//   though the raw frequency itself is silent.
//
// This is entirely separate from the coil output - turning it on/off
// does not affect what's being sent to the MD10C.

void btaudio_begin(const char* deviceNameToConnect); // e.g. your headset's BT name
void btaudio_setTargetFrequency(float freqHz);
void btaudio_setVolume(uint8_t percent); // 0-100
void btaudio_setEnabled(bool on);
bool btaudio_isEnabled();
bool btaudio_isConnected();

// ---------------------------------------------------------------------
// Device scan - uses the real ESP32-A2DP library API (verified against
// its actual source): set_ssid_callback() fires once per discovered
// device during a scan; this collects unique names into a small list
// (rejecting each one so scanning keeps going) rather than auto-connecting
// to the first thing found. The person then picks one from the UI; the
// actual connection only happens on the next boot (see the .ino's
// handleBtScanTouch) - redirecting an already-running A2DP session to a
// new target mid-session was tried and confirmed not to work reliably.
// ---------------------------------------------------------------------
static const int BT_SCAN_MAX_RESULTS = 10;

void btaudio_startScan();          // begins an open discovery scan
void btaudio_stopScan();           // cancels an in-progress scan
void btaudio_endSession();         // full teardown - call before any ESP.restart()

// ---------------------------------------------------------------------
// Soundscapes - streams a WAV file from the SD card as the BT audio
// output, in place of the tone-generation modes above. See sd_media.h
// for scanning the SD card's /sounds/ folder for available files.
// ---------------------------------------------------------------------
bool btaudio_startSoundscape(const char* path); // opens and begins looping playback
void btaudio_stopSoundscape();
bool btaudio_isSoundscapePlaying();
bool btaudio_isScanning();
int btaudio_scanResultCount();
const char* btaudio_scanResultName(int idx);
void btaudio_connectToScanResult(int idx); // saves the chosen name only - see note above
void btaudio_setSavedDeviceName(const char* name); // restore a previously-chosen name at boot, no scan needed


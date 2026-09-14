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
void btaudio_setEnabled(bool on);
bool btaudio_isEnabled();
bool btaudio_isConnected();

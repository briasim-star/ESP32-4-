#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// Local speaker output - a physical speaker wired directly to the
// board's onboard amplifier (AUDIO_ENABLE/AUDIO_DAC pins, see pins.h),
// using the ESP32's built-in DAC via I2S. This is a completely separate
// signal path from Bluetooth audio (bt_audio.h) - the two never run at
// once, selected by the user in Settings.
//
// Deliberately a standalone module rather than sharing code with
// bt_audio.cpp: that file has been through a lot of hard-won debugging
// this project (watchdog protection, cert bundles, fade timing, volume
// fixes), and refactoring it to serve two outputs risked disturbing
// something that already works. Some tone-generation logic is
// duplicated here as a result - a deliberate, low-risk tradeoff.
// ---------------------------------------------------------------------

void localaudio_begin();                    // call once from setup() - installs the I2S driver, safe no-op if called again
void localaudio_setEnabled(bool on);         // mirrors btaudio_setEnabled()'s shape
bool localaudio_isEnabled();
void localaudio_setTargetFrequency(float hz); // mirrors btaudio_setTargetFrequency()
float localaudio_getTargetFrequency(); // diagnostic - so the Welcome screen can show the raw value directly, not just whether writes are happening
void localaudio_setVolume(uint8_t percent);   // 0-100, this module's own scaling - no AVRCP-equivalent for a wired speaker

// Soundscapes - mirrors the soundscape half of bt_audio.h's interface
bool localaudio_startSoundscape(const char* path);
void localaudio_stopSoundscape();
bool localaudio_isSoundscapePlaying();

// Called every loop() iteration while this output is selected and
// something should be playing - generates/reads the next chunk of
// samples and feeds them to the I2S DMA buffer. Cheap to call when
// nothing is active (returns immediately).
void localaudio_update();

// Diagnostics - exposed so it's possible to actually confirm what's
// happening on real hardware instead of assuming it works.
bool localaudio_didInstallSucceed();
bool localaudio_didInstallFail();
unsigned long localaudio_totalSamplesWritten();
unsigned long localaudio_totalWriteFailures();

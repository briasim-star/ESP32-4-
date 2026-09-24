#pragma once
#include <Arduino.h>

// ============================================================================
// bt_audio.h - the ONE audio engine for the whole device
// ============================================================================
// (File name kept from the old Bluetooth-only module so the GitHub upload
// simply overwrites it. local_audio.cpp/.h are now intentionally empty.)
//
// Two independent choices:
//   OUTPUT  - where sound goes:   onboard speaker  OR  Bluetooth speaker/headset
//   SOURCE  - what is playing:    nothing, the PEMF-matched tone, or a
//                                 soundscape WAV from the SD card /sounds folder
//
// All audio work happens in a dedicated FreeRTOS task, NOT in loop().
// That is the core fix for "the onboard speaker never works" and for
// soundscape stutter: previously audio was only fed from loop(), which
// sleeps 20ms every pass and stalls for hundreds of ms during screen
// redraws, so the speaker DMA buffer was empty most of the time.
//
// Onboard speaker hardware (E32R40T spec): IO26 = DAC audio signal into the
// on-board amplifier, IO4 = amplifier enable (LOW = on). IO25 is the OTHER
// DAC pin and is used by our coil PWM (PIN_MD10C_PWM) - this engine never
// enables DAC1/IO25 (the old code accidentally did, via i2s_set_pin(NULL)).
//
// Soundscape WAV files: 16-bit PCM, 44.1kHz, mono or stereo.
// ============================================================================

enum AudioOutput : uint8_t { AUDIO_OUT_SPEAKER = 0, AUDIO_OUT_BLUETOOTH = 1 };
enum AudioSource : uint8_t { AUDIO_SRC_OFF = 0, AUDIO_SRC_TONE = 1, AUDIO_SRC_SOUNDSCAPE = 2 };

void audio_begin();                        // call once in setup(), BEFORE waveform_begin()

// ---- output selection ----
void audio_setOutput(AudioOutput out);
AudioOutput audio_getOutput();

// ---- what's playing ----
void audio_setSource(AudioSource src);     // OFF / TONE / SOUNDSCAPE (fades smoothly)
AudioSource audio_getSource();
void audio_setToneFrequency(float hz);     // follows the PEMF frequency
bool audio_setSoundscapeFile(const char* path); // opens the WAV (does not start playing by itself)
const char* audio_soundscapeFile();
void audio_setVolume(uint8_t percent);     // 0-100
void audio_chime(float hz, uint16_t ms);   // soft bell on top of whatever is playing (session start/end)
void audio_setNightShape(float gain, float warmth); // Sleep Night: 0-1 loudness, 0-1 warmth (1/0 = normal)
void audio_setHeadphonesMode(bool on);     // true = real binaural beats (headphones only)
bool audio_isHeadphonesMode();

// ---- Bluetooth ----
void audio_setBtDeviceName(const char* name); // saved device to connect to
const char* audio_btDeviceName();
void audio_btConnect();                    // non-blocking; starts the A2DP stack toward the saved name

// What the UI shows the person about Bluetooth.
enum BtStatus : uint8_t {
  BT_STATUS_OFF,          // Bluetooth not in use
  BT_STATUS_SEARCHING,    // looking for the saved device
  BT_STATUS_CONNECTING,   // found it, handshake in progress
  BT_STATUS_CONNECTED,
  BT_STATUS_RECONNECTING, // was connected, dropped, trying again
  BT_STATUS_NOT_FOUND     // 45 s with no luck - still trying in the background
};
BtStatus audio_btStatus();
uint32_t audio_btStatusChanges();          // increments on every connect/disconnect - lets the UI notice changes
void audio_btRetry();                      // "tap to retry" - goes straight to the remembered device
void audio_btForget();                     // erase the paired device everywhere (ours + the library's memory)
void audio_btConnectToScanResult(int idx); // connect to a device picked from the scan list (no restart)
bool audio_btStarted();
bool audio_btIsConnected();
// Full teardown - call before ESP.restart(). releaseMemory=true also frees the
// Bluetooth controller's RAM (needed for the HTTPS update download); BT then
// can't restart until the next reboot.
void audio_btEnd(bool releaseMemory = false);

// Device discovery (open scan). Picking a result saves the name; the
// connection itself is made after a restart (the one pattern that has
// proven reliable with this library).
static const int BT_SCAN_MAX_RESULTS = 10;
void audio_btStartScan();
void audio_btStopScan();
bool audio_btIsScanning();
int audio_btScanResultCount();
const char* audio_btScanResultName(int idx);

// ---- diagnostics ----
bool audio_speakerReady();                 // I2S/DAC driver installed OK
uint32_t audio_btFramesSent();             // audio frames the BT stack has pulled from us
int audio_btLastPeak();                    // loudness of the last BT block (0 = silence)
unsigned long audio_underruns();           // soundscape buffer ran dry (should stay ~0)

#include "bt_audio.h"
#include "BluetoothA2DPSource.h"
#include <math.h>

// ----------------------------------------------------------------------
// NOTE: The exact callback signature in the ESP32-A2DP library (by
// pschatzmann) has changed slightly across versions - it's always some
// form of "int32_t callback(Frame* data, int32_t frameCount)" where Frame
// has two int16_t channels, but the type name has been "Frame" or
// "Channels" depending on version. If this doesn't compile, open
// File > Examples > ESP32-A2DP > bt_music_sender_sinus in the Arduino IDE
// after installing the library and match this file's callback signature
// to whatever that example uses - it's a two-line fix.
// ----------------------------------------------------------------------
//
// AUDIO TECHNIQUE SELECTION:
// This picks its audio-entrainment technique based on the target PEMF
// frequency, since research on binaural beats consistently finds that
// true left/right binaural perception breaks down once the frequency
// difference between ears exceeds roughly 30 Hz (Licklider et al. 1950;
// Perrott & Nelson) - above that, listeners just hear two separate tones
// rather than a beat. Worth noting: even within its effective range,
// controlled studies on binaural beats' actual physiological effects are
// mixed (e.g. a 2017 Frontiers study found no significant EEG change) -
// this is offered as an audio companion to the session, not a verified
// clinical effect.
//
//   - target <= 30 Hz  : true binaural - carrier tone in the left ear,
//                        carrier+target in the right ear. The two tones
//                        differ only in the right channel's frequency.
//   - 30-100 Hz        : isochronic-style - a single audible carrier tone,
//                        identical in both ears, amplitude-modulated
//                        (pulsed) at the target rate.
//   - target >= 100 Hz : direct tone - the target frequency itself is
//                        already audible, so it's played straight,
//                        identical in both ears.
// ----------------------------------------------------------------------

static BluetoothA2DPSource a2dp_source;
static const uint32_t SAMPLE_RATE = 44100;

static volatile bool enabled = false;
static volatile float targetFreqHz = 0;
static volatile float g_volumeFrac = 1.0f; // 0.0 - 1.0, set via btaudio_setVolume()

static const float BINAURAL_CARRIER_HZ   = 200.0f; // left-ear tone for binaural mode
static const float BINAURAL_MAX_DIFF_HZ  = 30.0f;  // research-supported ceiling for beat perception
static const float ISOCHRONIC_CARRIER_HZ = 200.0f; // audible carrier for the AM fallback
static const float AUDIBLE_THRESHOLD_HZ  = 100.0f;

static double leftPhase = 0;
static double rightPhase = 0;
static double envelopePhase = 0;

enum AudioMode { MODE_BINAURAL, MODE_ISOCHRONIC, MODE_DIRECT };

static AudioMode currentAudioMode(float f) {
  if (f <= BINAURAL_MAX_DIFF_HZ) return MODE_BINAURAL;
  if (f < AUDIBLE_THRESHOLD_HZ) return MODE_ISOCHRONIC;
  return MODE_DIRECT;
}

// Generates one audio frame using whichever technique fits the current
// target frequency - see the technique-selection note above.
int32_t get_sound_data(Frame* data, int32_t frameCount) {
  if (!enabled || targetFreqHz <= 0) {
    for (int32_t i = 0; i < frameCount; i++) {
      data[i].channel1 = 0;
      data[i].channel2 = 0;
    }
    return frameCount;
  }

  AudioMode mode = currentAudioMode(targetFreqHz);

  for (int32_t i = 0; i < frameCount; i++) {
    int16_t leftSample, rightSample;

    if (mode == MODE_BINAURAL) {
      double l = sin(leftPhase);
      double r = sin(rightPhase);
      leftSample  = (int16_t)(l * 12000.0 * g_volumeFrac);
      rightSample = (int16_t)(r * 12000.0 * g_volumeFrac);

      leftPhase  += 2.0 * M_PI * BINAURAL_CARRIER_HZ / SAMPLE_RATE;
      rightPhase += 2.0 * M_PI * (BINAURAL_CARRIER_HZ + targetFreqHz) / SAMPLE_RATE;
      if (leftPhase  > 2.0 * M_PI) leftPhase  -= 2.0 * M_PI;
      if (rightPhase > 2.0 * M_PI) rightPhase -= 2.0 * M_PI;

    } else if (mode == MODE_ISOCHRONIC) {
      double carrier = sin(leftPhase);
      double envelope = 0.5 + 0.5 * sin(envelopePhase);
      int16_t sample = (int16_t)(carrier * envelope * 12000.0 * g_volumeFrac);
      leftSample = rightSample = sample;

      leftPhase += 2.0 * M_PI * ISOCHRONIC_CARRIER_HZ / SAMPLE_RATE;
      if (leftPhase > 2.0 * M_PI) leftPhase -= 2.0 * M_PI;
      envelopePhase += 2.0 * M_PI * targetFreqHz / SAMPLE_RATE;
      if (envelopePhase > 2.0 * M_PI) envelopePhase -= 2.0 * M_PI;

    } else { // MODE_DIRECT
      double carrier = sin(leftPhase);
      int16_t sample = (int16_t)(carrier * 12000.0 * g_volumeFrac);
      leftSample = rightSample = sample;

      leftPhase += 2.0 * M_PI * targetFreqHz / SAMPLE_RATE;
      if (leftPhase > 2.0 * M_PI) leftPhase -= 2.0 * M_PI;
    }

    data[i].channel1 = leftSample;
    data[i].channel2 = rightSample;
  }
  return frameCount;
}

void btaudio_begin(const char* deviceNameToConnect) {
  // Does not connect yet - actual connection only happens once
  // btaudio_setEnabled(true) is called, so the coil output can be used
  // without a headset paired at all.
  static const char* nameHolder = deviceNameToConnect;
  (void)nameHolder;
}

void btaudio_setTargetFrequency(float freqHz) {
  targetFreqHz = freqHz;
}

void btaudio_setVolume(uint8_t percent) {
  if (percent > 100) percent = 100;
  g_volumeFrac = percent / 100.0f;
}

static char g_deviceName[64] = "";
static bool g_started = false;
static bool a2dpSourceStarted = false; // true once ANY a2dp_source.start() call has happened - scan or connect

void btaudio_setEnabled(bool on) {
  enabled = on;
  if (on && !g_started && strlen(g_deviceName) > 0) {
    a2dp_source.start(g_deviceName, get_sound_data);
    a2dpSourceStarted = true;
    g_started = true;
  }
}

bool btaudio_isEnabled() { return enabled; }

bool btaudio_isConnected() {
  return g_started && a2dp_source.is_connected();
}

// ---------------------------------------------------------------------
// Device scan
// ---------------------------------------------------------------------
static char scanResults[BT_SCAN_MAX_RESULTS][32];
static int scanResultCount = 0;
static volatile bool scanningActive = false;

// Fires once per discovered device during a2dp_source.start() with no
// name given. Always returns false (never auto-accept) so the scan keeps
// running and collecting names - the person picks manually from the UI.
static bool onSsidFound(const char* ssid, esp_bd_addr_t address, int rssi) {
  if (!ssid || strlen(ssid) == 0) return false;
  if (scanResultCount >= BT_SCAN_MAX_RESULTS) return false;
  for (int i = 0; i < scanResultCount; i++) {
    if (strcmp(scanResults[i], ssid) == 0) return false; // already have it
  }
  strncpy(scanResults[scanResultCount], ssid, sizeof(scanResults[0]) - 1);
  scanResults[scanResultCount][sizeof(scanResults[0]) - 1] = 0;
  scanResultCount++;
  return false;
}

static void onDiscoveryModeChanged(esp_bt_gap_discovery_state_t mode) {
  scanningActive = (mode == ESP_BT_GAP_DISCOVERY_STARTED);
}

void btaudio_startScan() {
  scanResultCount = 0;
  a2dp_source.set_ssid_callback(onSsidFound);
  a2dp_source.set_discovery_mode_callback(onDiscoveryModeChanged);
  a2dp_source.start(); // empty name list -> open discovery, per the library's own API
  a2dpSourceStarted = true;
}

void btaudio_stopScan() {
  a2dp_source.cancel_discovery();
}

bool btaudio_isScanning() { return scanningActive; }
int btaudio_scanResultCount() { return scanResultCount; }

const char* btaudio_scanResultName(int idx) {
  if (idx < 0 || idx >= scanResultCount) return "";
  return scanResults[idx];
}

void btaudio_connectToScanResult(int idx) {
  if (idx < 0 || idx >= scanResultCount) return;
  strncpy(g_deviceName, scanResults[idx], sizeof(g_deviceName) - 1);
  g_deviceName[sizeof(g_deviceName) - 1] = 0;
  // The A2DP source was already running (from the discovery scan) - it
  // needs a proper teardown via end() before a fresh start() targeting
  // this specific device actually takes effect. Without this, the
  // library's internal state machine is still in its original discovery
  // session and a second start() call doesn't cleanly redirect it.
  if (a2dpSourceStarted) {
    a2dp_source.end();
  }
  g_started = false;
  a2dp_source.start(g_deviceName, get_sound_data);
  a2dpSourceStarted = true;
  g_started = true;
}

void btaudio_setSavedDeviceName(const char* name) {
  strncpy(g_deviceName, name, sizeof(g_deviceName) - 1);
  g_deviceName[sizeof(g_deviceName) - 1] = 0;
}

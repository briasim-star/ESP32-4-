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

static BluetoothA2DPSource a2dp_source;
static const uint32_t SAMPLE_RATE = 44100;

static volatile bool enabled = false;
static volatile float targetFreqHz = 0;
static const float AUDIBLE_CARRIER_HZ = 200.0f; // used when targetFreq is sub-audible
static const float AUDIBLE_THRESHOLD_HZ = 100.0f;

static double carrierPhase = 0;
static double envelopePhase = 0;

// Generates one audio frame: either a direct tone at targetFreqHz (if it's
// already audible) or an audible carrier tone amplitude-modulated
// ("isochronic") at targetFreqHz (if targetFreqHz is sub-audible).
int32_t get_sound_data(Frame* data, int32_t frameCount) {
  if (!enabled || targetFreqHz <= 0) {
    for (int32_t i = 0; i < frameCount; i++) {
      data[i].channel1 = 0;
      data[i].channel2 = 0;
    }
    return frameCount;
  }

  bool directTone = targetFreqHz >= AUDIBLE_THRESHOLD_HZ;
  double carrierFreq = directTone ? targetFreqHz : AUDIBLE_CARRIER_HZ;
  double envFreq = directTone ? 0.0 : targetFreqHz;

  for (int32_t i = 0; i < frameCount; i++) {
    double carrier = sin(carrierPhase);
    double envelope = directTone ? 1.0 : (0.5 + 0.5 * sin(envelopePhase));
    int16_t sample = (int16_t)(carrier * envelope * 12000.0); // headroom, not full-scale

    data[i].channel1 = sample;
    data[i].channel2 = sample;

    carrierPhase += 2.0 * M_PI * carrierFreq / SAMPLE_RATE;
    if (carrierPhase > 2.0 * M_PI) carrierPhase -= 2.0 * M_PI;

    if (!directTone) {
      envelopePhase += 2.0 * M_PI * envFreq / SAMPLE_RATE;
      if (envelopePhase > 2.0 * M_PI) envelopePhase -= 2.0 * M_PI;
    }
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

static const char* g_deviceName = "PEMF-Headset";
static bool g_started = false;

void btaudio_setEnabled(bool on) {
  enabled = on;
  if (on && !g_started) {
    a2dp_source.start(g_deviceName, get_sound_data);
    g_started = true;
  }
}

bool btaudio_isEnabled() { return enabled; }

bool btaudio_isConnected() {
  return g_started && a2dp_source.is_connected();
}

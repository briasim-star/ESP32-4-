#include "bt_audio.h"
#include "BluetoothA2DPSource.h"
#include <math.h>
#include <SD.h>

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
// Ceiling on how high the LITERAL audible tone is ever allowed to go, kept
// completely independent of the coil's own frequency. The coil can safely
// run up to 20kHz (e.g. the Multi-Frequency Sequence's 10,000Hz step), but
// a raw 10kHz *audio* tone would be genuinely harsh/piercing to listen to,
// not just "technically" in the audible range. Above this ceiling the
// audio simply holds at the ceiling tone instead of climbing with the coil.
static const float DIRECT_TONE_MAX_HZ    = 1000.0f;

// Fade-in/fade-out gain, applied on top of any audio source (tone or
// soundscape) - a sudden jump from silence to full amplitude, or back,
// is exactly what causes an audible click/pop when audio is turned on
// or off. This ramps smoothly over ~30ms instead, sample by sample.
// Confirmed by real report: an audible, startling click when toggling
// BT audio on/off.
static float fadeGain = 0.0f;
static const float FADE_STEP = 1.0f / (SAMPLE_RATE * 0.03f); // ~30ms fade

// ----------------------------------------------------------------------
// Precomputed sine lookup table (with linear interpolation), used instead
// of calling sin()/sinf() per audio sample. This callback runs inside the
// Bluetooth stack's own task at 44.1kHz - a slow callback there was
// starving other tasks badly enough to trip the watchdog and crash the
// device (confirmed via serial log: "Task watchdog... CPU 0: BTC_TASK").
// Also using float throughout rather than double, since the ESP32's FPU
// is single-precision only - double math silently falls back to much
// slower software emulation.
// ----------------------------------------------------------------------
static const int SIN_TABLE_SIZE = 256; // power of 2, for cheap wraparound via bitmask
static float sinTable[SIN_TABLE_SIZE];
static bool sinTableReady = false;

static void ensureSinTable() {
  if (sinTableReady) return;
  for (int i = 0; i < SIN_TABLE_SIZE; i++) {
    sinTable[i] = sinf(2.0f * (float)M_PI * i / SIN_TABLE_SIZE);
  }
  sinTableReady = true;
}

// phase is in radians, 0..2*PI (wraps). Table lookup + linear interpolation.
static inline float fastSin(float phase) {
  float normalized = phase * ((float)SIN_TABLE_SIZE / (2.0f * (float)M_PI));
  int idx0 = (int)normalized;
  float frac = normalized - (float)idx0;
  idx0 &= (SIN_TABLE_SIZE - 1);
  int idx1 = (idx0 + 1) & (SIN_TABLE_SIZE - 1);
  return sinTable[idx0] + frac * (sinTable[idx1] - sinTable[idx0]);
}

static float leftPhase = 0;
static float rightPhase = 0;
static float envelopePhase = 0;
static const float TWO_PI_F = 2.0f * (float)M_PI;

enum AudioMode { MODE_BINAURAL, MODE_ISOCHRONIC, MODE_DIRECT };

static AudioMode currentAudioMode(float f) {
  if (f <= BINAURAL_MAX_DIFF_HZ) return MODE_BINAURAL;
  if (f < AUDIBLE_THRESHOLD_HZ) return MODE_ISOCHRONIC;
  return MODE_DIRECT;
}

// ---------------------------------------------------------------------
// Soundscapes - streams raw PCM directly from an SD-card WAV file into
// the same audio callback used for tone generation. Since our WAV files
// are already prepared at 44.1kHz/16-bit/stereo (matching this pipeline
// exactly), this is a direct byte-for-byte read into the Frame buffer -
// no decoding or resampling needed. Pattern verified against a real,
// community-confirmed example of exactly this SD+A2DP combination.
// ---------------------------------------------------------------------
static fs::File soundscapeFile;
static bool soundscapePlaying = false;
static bool soundscapeStopping = false; // true while fading out, file still open
static uint32_t soundscapeDataStart = 0; // byte offset where PCM data begins
static uint32_t soundscapeDataSize = 0;  // size of the PCM data chunk, in bytes

bool btaudio_startSoundscape(const char* path) {
  if (soundscapePlaying) btaudio_stopSoundscape();
  soundscapeFile = SD.open(path, FILE_READ);
  if (!soundscapeFile) return false;

  // Walk the WAV's chunks to find "data" properly, rather than assuming
  // a fixed 44-byte offset - some encoders (including ffmpeg, which made
  // these files) can add extra chunks before it.
  soundscapeFile.seek(12); // past "RIFF" + size(4) + "WAVE"
  soundscapeDataStart = 0;
  soundscapeDataSize = 0;
  while (soundscapeFile.available()) {
    char chunkId[4];
    if (soundscapeFile.read((uint8_t*)chunkId, 4) != 4) break;
    uint8_t sizeBytes[4];
    if (soundscapeFile.read(sizeBytes, 4) != 4) break;
    uint32_t chunkSize = sizeBytes[0] | (sizeBytes[1] << 8) | (sizeBytes[2] << 16) | (sizeBytes[3] << 24);

    if (memcmp(chunkId, "data", 4) == 0) {
      soundscapeDataStart = soundscapeFile.position();
      soundscapeDataSize = chunkSize;
      break;
    }
    soundscapeFile.seek(soundscapeFile.position() + chunkSize);
  }

  if (soundscapeDataSize == 0) {
    soundscapeFile.close();
    return false;
  }
  soundscapePlaying = true;
  return true;
}

void btaudio_stopSoundscape() {
  // Defer actually closing the file until the fade-out completes (see
  // get_sound_data()) - closing it immediately would cut the audio off
  // abruptly, causing the same kind of click this whole fade system
  // exists to prevent.
  if (soundscapePlaying) soundscapeStopping = true;
  soundscapePlaying = false;
}

bool btaudio_isSoundscapePlaying() { return soundscapePlaying; }

// Generates one audio frame using whichever technique fits the current
// target frequency - see the technique-selection note above.
int32_t get_sound_data(Frame* data, int32_t frameCount) {
  bool wantAudio = soundscapePlaying || (enabled && targetFreqHz > 0);

  if (soundscapePlaying || soundscapeStopping) {
    static const int FRAME_SIZE_BYTES = sizeof(int16_t) * 2; // 2 channels
    int32_t bytesNeeded = frameCount * FRAME_SIZE_BYTES;
    int32_t bytesRead = soundscapeFile.read((uint8_t*)data, bytesNeeded);
    if (bytesRead < bytesNeeded) {
      // Hit end of file mid-buffer - loop back to the start of the PCM
      // data and fill the remainder, so the loop point has no gap.
      soundscapeFile.seek(soundscapeDataStart);
      int32_t remaining = bytesNeeded - bytesRead;
      soundscapeFile.read(((uint8_t*)data) + bytesRead, remaining);
    }
    for (int32_t i = 0; i < frameCount; i++) {
      float target = soundscapePlaying ? 1.0f : 0.0f;
      if (fadeGain < target) {
        fadeGain += FADE_STEP;
        if (fadeGain > target) fadeGain = target;
      } else if (fadeGain > target) {
        fadeGain -= FADE_STEP;
        if (fadeGain < target) fadeGain = target;
      }
      data[i].channel1 = (int16_t)(data[i].channel1 * g_volumeFrac * fadeGain);
      data[i].channel2 = (int16_t)(data[i].channel2 * g_volumeFrac * fadeGain);
    }
    if (soundscapeStopping && fadeGain <= 0.0f) {
      soundscapeFile.close();
      soundscapeStopping = false;
    }
    return frameCount;
  }

  if (!wantAudio && fadeGain <= 0.0f) {
    for (int32_t i = 0; i < frameCount; i++) {
      data[i].channel1 = 0;
      data[i].channel2 = 0;
    }
    return frameCount;
  }

  // Either actively wanted, or still fading out from just being turned
  // off - keep generating the tone either way, multiplied by fadeGain,
  // which ramps toward 1 (turning on) or 0 (turning off) a little each
  // sample instead of jumping instantly.
  ensureSinTable();
  float useFreq = (targetFreqHz > 0) ? targetFreqHz : 10.0f; // fallback only for a fade-out tail if targetFreqHz was ever cleared
  AudioMode mode = currentAudioMode(useFreq);

  for (int32_t i = 0; i < frameCount; i++) {
    float target = wantAudio ? 1.0f : 0.0f;
    if (fadeGain < target) {
      fadeGain += FADE_STEP;
      if (fadeGain > target) fadeGain = target;
    } else if (fadeGain > target) {
      fadeGain -= FADE_STEP;
      if (fadeGain < target) fadeGain = target;
    }

    int16_t leftSample, rightSample;

    if (mode == MODE_BINAURAL) {
      float l = fastSin(leftPhase);
      float r = fastSin(rightPhase);
      leftSample  = (int16_t)(l * 12000.0f * g_volumeFrac * fadeGain);
      rightSample = (int16_t)(r * 12000.0f * g_volumeFrac * fadeGain);

      leftPhase  += TWO_PI_F * BINAURAL_CARRIER_HZ / SAMPLE_RATE;
      rightPhase += TWO_PI_F * (BINAURAL_CARRIER_HZ + useFreq) / SAMPLE_RATE;
      if (leftPhase  > TWO_PI_F) leftPhase  -= TWO_PI_F;
      if (rightPhase > TWO_PI_F) rightPhase -= TWO_PI_F;

    } else if (mode == MODE_ISOCHRONIC) {
      float carrier = fastSin(leftPhase);
      float envelope = 0.5f + 0.5f * fastSin(envelopePhase);
      int16_t sample = (int16_t)(carrier * envelope * 12000.0f * g_volumeFrac * fadeGain);
      leftSample = rightSample = sample;

      leftPhase += TWO_PI_F * ISOCHRONIC_CARRIER_HZ / SAMPLE_RATE;
      if (leftPhase > TWO_PI_F) leftPhase -= TWO_PI_F;
      envelopePhase += TWO_PI_F * useFreq / SAMPLE_RATE;
      if (envelopePhase > TWO_PI_F) envelopePhase -= TWO_PI_F;

    } else { // MODE_DIRECT
      float audibleFreq = useFreq > DIRECT_TONE_MAX_HZ ? DIRECT_TONE_MAX_HZ : useFreq;
      float carrier = fastSin(leftPhase);
      int16_t sample = (int16_t)(carrier * 12000.0f * g_volumeFrac * fadeGain);
      leftSample = rightSample = sample;

      leftPhase += TWO_PI_F * audibleFreq / SAMPLE_RATE;
      if (leftPhase > TWO_PI_F) leftPhase -= TWO_PI_F;
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
    // Explicitly command the connected device's own hardware volume via
    // AVRCP, once, right after connecting - this applies identically to
    // headphones as to a standalone speaker (the protocol doesn't
    // distinguish between them; a device either honors this command or
    // it doesn't, regardless of category). Many Bluetooth audio devices
    // default to a low volume on a fresh connection unless told
    // otherwise - without this, our own volume control below is scaling
    // a signal the device itself is quietly attenuating on top of.
    //
    // Deliberately NOT the absolute max (127) though - headphones sit
    // right at the ear, so fully opening a device's own hardware volume
    // and relying entirely on our own scaling to bring it back down is
    // a real hearing-safety consideration for headphones specifically,
    // not just an audibility one. ~80% of the device's own range still
    // fixes the "too quiet by default" problem while leaving a genuine
    // safety margin rather than sitting wide open at max. Our own
    // volumePercent (below) remains the real, meaningful control either
    // way - this is just ensuring it isn't fighting a device that's
    // quietly holding itself back.
    a2dp_source.set_volume(100); // ~80% of the library's 0-127 range
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

// Full teardown of the Bluetooth stack - call this before any ESP.restart(),
// so a software reset doesn't abruptly cut off a still-active BT session.
// Confirmed via testing: skipping this caused touch/SPI flakiness on the
// first screen(s) shown right after such a restart (the touchscreen shares
// the SPI bus, and an uncleanly-terminated radio session left it briefly
// unsettled) - a normal power-on doesn't have this issue since it's a true
// hardware reset, not a software one.
void btaudio_endSession() {
  if (a2dpSourceStarted) {
    a2dp_source.end();
    a2dpSourceStarted = false;
    g_started = false;
  }
}

bool btaudio_isScanning() { return scanningActive; }
int btaudio_scanResultCount() { return scanResultCount; }

const char* btaudio_scanResultName(int idx) {
  if (idx < 0 || idx >= scanResultCount) return "";
  return scanResults[idx];
}

// After two attempts to redirect an already-running A2DP session mid-flight
// (both confirmed not to work by real device testing), this just records
// the chosen name. The actual connection happens on the next boot, calling
// start() exactly once, fresh - the one pattern every official example of
// this library uses successfully. See handleBtScanTouch() in the .ino,
// which saves this name and restarts the device right after calling this.
void btaudio_connectToScanResult(int idx) {
  if (idx < 0 || idx >= scanResultCount) return;
  strncpy(g_deviceName, scanResults[idx], sizeof(g_deviceName) - 1);
  g_deviceName[sizeof(g_deviceName) - 1] = 0;
}

void btaudio_setSavedDeviceName(const char* name) {
  strncpy(g_deviceName, name, sizeof(g_deviceName) - 1);
  g_deviceName[sizeof(g_deviceName) - 1] = 0;
}

#include "local_audio.h"
#include "pins.h"
#include <driver/i2s.h>
#include <SD.h>

// ---------------------------------------------------------------------
// Technical notes, since these are the kind of details that silently
// produce garbled or absent audio if gotten wrong:
//
// - The ESP32's built-in DAC only reads the top 8 bits of each 16-bit
//   sample, and expects them UNSIGNED (0-255 after that truncation).
//   Our tone/soundscape math produces SIGNED 16-bit samples (-32768 to
//   32767, standard PCM), so +32768 is added to every sample before
//   writing - confirmed via real ESP32 forum reports of exactly this
//   conversion being required.
// - There's a documented word-swap bug specific to MONO built-in-DAC
//   I2S output (samples come out in the wrong order in pairs). Reports
//   confirm STEREO mode - writing every sample to both channels, even
//   though only one GPIO is physically wired to our amp - avoids this
//   bug entirely. That's what this uses, rather than mono.
// ---------------------------------------------------------------------

static const uint32_t LOCAL_SAMPLE_RATE = 44100;
static const i2s_port_t LOCAL_I2S_PORT = I2S_NUM_0; // built-in DAC mode only works on I2S0

static bool i2sInstalled = false;
static bool enabled = false;
static float targetFreqHz = 0;
static float g_volumeFrac = 0.6f;

static const float BINAURAL_CARRIER_HZ = 200.0f;
static const float ISOCHRONIC_CARRIER_HZ = 200.0f;
static const float BINAURAL_MAX_DIFF_HZ = 30.0f;
static const float AUDIBLE_THRESHOLD_HZ = 20.0f;
static const float DIRECT_TONE_MAX_HZ = 1000.0f;
static const float TWO_PI_F = 6.28318530718f;

static float leftPhase = 0, rightPhase = 0, envelopePhase = 0;

// Small sine lookup table, same approach as bt_audio.cpp's fastSin() -
// duplicated deliberately rather than shared, per the design note in
// local_audio.h.
static const int SIN_TABLE_SIZE = 256;
static float sinTable[SIN_TABLE_SIZE];
static bool sinTableReady = false;

static void ensureSinTable() {
  if (sinTableReady) return;
  for (int i = 0; i < SIN_TABLE_SIZE; i++) {
    sinTable[i] = sinf((float)i / SIN_TABLE_SIZE * TWO_PI_F);
  }
  sinTableReady = true;
}

static float fastSin(float phase) {
  while (phase < 0) phase += TWO_PI_F;
  while (phase >= TWO_PI_F) phase -= TWO_PI_F;
  int idx = (int)(phase / TWO_PI_F * SIN_TABLE_SIZE) % SIN_TABLE_SIZE;
  return sinTable[idx];
}

enum AudioMode { MODE_BINAURAL, MODE_ISOCHRONIC, MODE_DIRECT };
static AudioMode currentAudioMode(float f) {
  if (f <= BINAURAL_MAX_DIFF_HZ) return MODE_BINAURAL;
  if (f < AUDIBLE_THRESHOLD_HZ) return MODE_ISOCHRONIC;
  return MODE_DIRECT;
}

// Soundscape playback state
static fs::File soundscapeFile;
static bool soundscapePlaying = false;
static bool soundscapeStopping = false;
static uint32_t soundscapeDataStart = 0;
static uint32_t soundscapeDataSize = 0;
static float fadeGain = 0.0f;
static const float FADE_STEP = 1.0f / (LOCAL_SAMPLE_RATE * 0.03f); // ~30ms fade, same reasoning as bt_audio.cpp's click fix

static bool i2sInstallFailed = false; // diagnostic - exposed so we can actually confirm success/failure instead of assuming

void localaudio_begin() {
  if (i2sInstalled) return;

  pinMode(AUDIO_ENABLE, OUTPUT);
  digitalWrite(AUDIO_ENABLE, HIGH); // amp disabled until actually needed

  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = LOCAL_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo - see the word-swap note above
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false
  };
  if (i2s_driver_install(LOCAL_I2S_PORT, &i2sConfig, 0, NULL) != ESP_OK) {
    i2sInstallFailed = true;
    return;
  }
  i2s_set_pin(LOCAL_I2S_PORT, NULL); // NULL = route to the internal DAC pins (GPIO25/26)
  // Real conflict found and fixed here: GPIO25 (DAC1) is also this
  // board's PIN_MD10C_PWM - the coil's own PWM pin (see pins.h). Using
  // BOTH_EN drove GPIO25 for audio too, fighting the coil for the same
  // pin - almost certainly why no sound came out at all. GPIO26 (DAC2,
  // "LEFT" in this API) is the only pin actually wired to the physical
  // speaker anyway, so enabling only that one fixes the conflict
  // entirely, with no rewiring needed. The channel_format above stays
  // stereo (both buffer slots filled) specifically because that's what
  // avoids the documented mono word-swap bug - only the DAC_MODE below
  // (which pins actually get driven) changes.
  i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); // GPIO26 only - never touches GPIO25
  i2sInstalled = true;
}

bool localaudio_didInstallSucceed() { return i2sInstalled; }
bool localaudio_didInstallFail() { return i2sInstallFailed; }

void localaudio_setEnabled(bool on) {
  enabled = on;
  digitalWrite(AUDIO_ENABLE, on ? LOW : HIGH); // LOW = amplifier enabled, per pins.h
}

bool localaudio_isEnabled() { return enabled; }

void localaudio_setTargetFrequency(float hz) { targetFreqHz = hz; }

void localaudio_setVolume(uint8_t percent) {
  if (percent > 100) percent = 100;
  g_volumeFrac = percent / 100.0f;
}

bool localaudio_startSoundscape(const char* path) {
  if (soundscapePlaying) localaudio_stopSoundscape();
  soundscapeFile = SD.open(path, FILE_READ);
  if (!soundscapeFile) return false;

  soundscapeFile.seek(12);
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

void localaudio_stopSoundscape() {
  if (soundscapePlaying) soundscapeStopping = true;
  soundscapePlaying = false;
}

bool localaudio_isSoundscapePlaying() { return soundscapePlaying; }

// Writes one interleaved stereo sample (same value on both channels,
// converted from signed to the DAC's expected unsigned range) to the
// I2S DMA buffer.
static unsigned long totalSamplesWritten = 0;
static unsigned long totalWriteFailures = 0; // write() returned less than expected - diagnostic, not currently used to change behavior

static void writeSample(int16_t sample) {
  uint16_t unsignedSample = (uint16_t)((int32_t)sample + 32768);
  uint16_t frame[2] = { unsignedSample, unsignedSample }; // left, right - identical
  size_t written;
  i2s_write(LOCAL_I2S_PORT, frame, sizeof(frame), &written, 0); // 0 ticks wait - never blocks loop()
  if (written == sizeof(frame)) {
    totalSamplesWritten++;
  } else {
    totalWriteFailures++;
  }
}

unsigned long localaudio_totalSamplesWritten() { return totalSamplesWritten; }
unsigned long localaudio_totalWriteFailures() { return totalWriteFailures; }

static unsigned long lastUpdateMicros = 0;

void localaudio_update() {
  if (!i2sInstalled) return;

  // A real, confirmed bug lived here: this used to generate a small
  // fixed number of samples per call (64 for tones, 256 for
  // soundscapes), but loop() only calls this roughly every 20-40ms
  // (there's an explicit delay(20) at the end of every iteration, plus
  // everything else loop() does). At 44.1kHz, 64 samples is only ~1.5ms
  // of audio - meaning the DMA buffer was starved almost the entire
  // time between calls, which sounds like near-total silence, not a
  // real tone. This now generates however many samples actually
  // correspond to the real time that elapsed since the last call,
  // capped to a sane maximum so a long delay (e.g. right after boot)
  // doesn't try to generate an enormous burst all at once.
  unsigned long nowMicros = micros();
  if (lastUpdateMicros == 0) lastUpdateMicros = nowMicros; // first call - nothing elapsed yet
  unsigned long elapsedMicros = nowMicros - lastUpdateMicros;
  int samplesNeeded = (int)((unsigned long long)elapsedMicros * LOCAL_SAMPLE_RATE / 1000000ULL);
  if (samplesNeeded > 4000) samplesNeeded = 4000; // cap - avoid a huge catch-up burst after any long pause
  if (samplesNeeded <= 0) return; // called again too soon to owe any new samples yet
  lastUpdateMicros = nowMicros;

  if (soundscapePlaying || soundscapeStopping) {
    // Each source sample is 4 bytes (2 channels x 2 bytes), and we only
    // consume every other one (left channel) below, so read 2 bytes of
    // source data per sample actually needed.
    int bytesToRead = samplesNeeded * 2;
    static uint8_t buf[8192]; // large enough for the capped max (4000 samples x 2 bytes)
    if (bytesToRead > (int)sizeof(buf)) bytesToRead = sizeof(buf);
    int bytesRead = soundscapeFile.read(buf, bytesToRead);
    if (bytesRead <= 0) {
      soundscapeFile.seek(soundscapeDataStart);
      bytesRead = soundscapeFile.read(buf, bytesToRead);
    }
    int16_t* samples = (int16_t*)buf;
    int sampleCount = bytesRead / 2;
    // Source WAV is stereo already (see the SD-card prep notes elsewhere
    // in this project) - take every other sample (left channel) to keep
    // this simple, since the DAC path is mono-quality anyway.
    for (int i = 0; i < sampleCount; i += 2) {
      float target = soundscapePlaying ? 1.0f : 0.0f;
      if (fadeGain < target) { fadeGain += FADE_STEP; if (fadeGain > target) fadeGain = target; }
      else if (fadeGain > target) { fadeGain -= FADE_STEP; if (fadeGain < target) fadeGain = target; }
      writeSample((int16_t)(samples[i] * g_volumeFrac * fadeGain));
    }
    if (soundscapeStopping && fadeGain <= 0.0f) {
      soundscapeFile.close();
      soundscapeStopping = false;
    }
    return;
  }

  if (!enabled || targetFreqHz <= 0) return;

  ensureSinTable();
  AudioMode mode = currentAudioMode(targetFreqHz);

  for (int i = 0; i < samplesNeeded; i++) {
    int16_t sample;
    if (mode == MODE_BINAURAL) {
      // Wired to one physical speaker, not headphones - true binaural
      // beating needs two ears each hearing a different tone, which a
      // single speaker can't deliver. Falls back to isochronic-style
      // amplitude pulsing instead, which works correctly over one
      // speaker and still conveys the same target frequency.
      float carrier = fastSin(leftPhase);
      float envelope = 0.5f + 0.5f * fastSin(envelopePhase);
      sample = (int16_t)(carrier * envelope * 12000.0f * g_volumeFrac);
      leftPhase += TWO_PI_F * BINAURAL_CARRIER_HZ / LOCAL_SAMPLE_RATE;
      if (leftPhase > TWO_PI_F) leftPhase -= TWO_PI_F;
      envelopePhase += TWO_PI_F * targetFreqHz / LOCAL_SAMPLE_RATE;
      if (envelopePhase > TWO_PI_F) envelopePhase -= TWO_PI_F;
    } else if (mode == MODE_ISOCHRONIC) {
      float carrier = fastSin(leftPhase);
      float envelope = 0.5f + 0.5f * fastSin(envelopePhase);
      sample = (int16_t)(carrier * envelope * 12000.0f * g_volumeFrac);
      leftPhase += TWO_PI_F * ISOCHRONIC_CARRIER_HZ / LOCAL_SAMPLE_RATE;
      if (leftPhase > TWO_PI_F) leftPhase -= TWO_PI_F;
      envelopePhase += TWO_PI_F * targetFreqHz / LOCAL_SAMPLE_RATE;
      if (envelopePhase > TWO_PI_F) envelopePhase -= TWO_PI_F;
    } else {
      float audibleFreq = targetFreqHz > DIRECT_TONE_MAX_HZ ? DIRECT_TONE_MAX_HZ : targetFreqHz;
      float carrier = fastSin(leftPhase);
      sample = (int16_t)(carrier * 12000.0f * g_volumeFrac);
      leftPhase += TWO_PI_F * audibleFreq / LOCAL_SAMPLE_RATE;
      if (leftPhase > TWO_PI_F) leftPhase -= TWO_PI_F;
    }
    writeSample(sample);
  }
}

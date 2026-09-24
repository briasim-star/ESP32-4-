#include "bt_audio.h"
#include "pins.h"
#include "sd_media.h"
#include "BluetoothA2DPSource.h"
#include <driver/i2s.h>
#include <driver/dac.h>
#include <driver/rtc_io.h>
#include <SD.h>
#include <Preferences.h>
#include <esp_gap_bt_api.h>
#include <math.h>

// ============================================================================
// Unified audio engine - see bt_audio.h for the overview.
//
// Data flow:
//   audioTask (own FreeRTOS task, core 1, above loop() priority)
//     - opens/parses the chosen soundscape WAV and keeps a RAM sndRing buffer
//       topped up from the SD card
//     - when OUTPUT = speaker: renders audio and pushes it to the I2S DMA
//       (i2s_write blocks until there is room, so it is naturally paced)
//   Bluetooth A2DP callback (runs in the BT stack's task)
//     - when OUTPUT = bluetooth: renders audio straight into the BT buffer
//   renderFrames() is the single place tone/soundscape audio is produced,
//   so both outputs always sound the same.
// ============================================================================

static const uint32_t AUD_SAMPLE_RATE = 44100;
static const i2s_port_t SPK_I2S_PORT = I2S_NUM_0; // built-in DAC only works on I2S0
static const float TWO_PI_F = 6.28318530718f;

struct StereoFrame { int16_t l; int16_t r; };
static_assert(sizeof(StereoFrame) == sizeof(Frame), "Frame layout must be two int16 channels");

// ---------------------------------------------------------------------------
// Shared state (written by loop(), read by the audio task / BT callback)
// ---------------------------------------------------------------------------
static volatile AudioOutput g_output = AUDIO_OUT_SPEAKER;
static volatile AudioSource g_source = AUDIO_SRC_OFF;
static volatile float g_toneHz = 10.0f;
static volatile float g_volume = 0.6f;
static volatile bool g_headphones = false;

static unsigned long g_underruns = 0;
static bool g_speakerReady = false;

// ---------------------------------------------------------------------------
// Tone generation (same technique selection as before - this part already
// sounded right):
//   <= 30 Hz : binaural beat on headphones, isochronic pulse otherwise
//   30-100 Hz: isochronic pulse on a 200 Hz carrier
//   >= 100 Hz: the frequency itself as a sine, capped at 1 kHz
// ---------------------------------------------------------------------------
static const float CARRIER_HZ = 200.0f;
static const float BINAURAL_MAX_HZ = 30.0f;
static const float AUDIBLE_HZ = 100.0f;
static const float DIRECT_MAX_HZ = 1000.0f;
static const float TONE_LEVEL = 0.45f; // of full scale - a pure tone near full level makes small speakers distort (sounds crunchy)

static const int SIN_N = 256;
static float sinTable[SIN_N];

static inline float fastSin(float phase) {
  float x = phase * (SIN_N / TWO_PI_F);
  int i0 = (int)x;
  float frac = x - i0;
  i0 &= (SIN_N - 1);
  int i1 = (i0 + 1) & (SIN_N - 1);
  return sinTable[i0] + frac * (sinTable[i1] - sinTable[i0]);
}

static float phL = 0, phR = 0, phEnv = 0;

static inline void advancePhase(float& ph, float hz) {
  ph += TWO_PI_F * hz / AUD_SAMPLE_RATE;
  if (ph >= TWO_PI_F) ph -= TWO_PI_F;
}

// Carrier pitch for pulsed tones: each session frequency maps to its own
// note between G3 (196 Hz) and G4 (392 Hz) on a log scale, so different
// sessions sound different, and the pitch stays above the range where
// small speakers rattle.
static float carrierFor(float f) {
  if (f < 0.5f) f = 0.5f;
  if (f > 100.0f) f = 100.0f;
  float t = logf(f / 0.5f) / logf(200.0f); // 0.5 Hz -> 0, 100 Hz -> 1
  return 196.0f * powf(2.0f, t);
}

static void renderTone(StereoFrame* out, int n) {
  float f = g_toneHz;
  if (f <= 0) f = 10.0f;
  const float CARRIER_HZ = carrierFor(f);
  bool binaural = (f <= BINAURAL_MAX_HZ) && g_headphones && g_output == AUDIO_OUT_BLUETOOTH;
  const float amp = 32767.0f * TONE_LEVEL;
  for (int i = 0; i < n; i++) {
    float l, r;
    if (binaural) {
      l = fastSin(phL);
      r = fastSin(phR);
      advancePhase(phL, CARRIER_HZ);
      advancePhase(phR, CARRIER_HZ + f);
    } else if (f < AUDIBLE_HZ) {
      float env = 0.5f + 0.5f * fastSin(phEnv);
      l = r = fastSin(phL) * env;
      advancePhase(phL, CARRIER_HZ);
      advancePhase(phEnv, f);
    } else {
      l = r = fastSin(phL);
      advancePhase(phL, f > DIRECT_MAX_HZ ? DIRECT_MAX_HZ : f);
    }
    out[i].l = (int16_t)(l * amp);
    out[i].r = (int16_t)(r * amp);
  }
}

// ---------------------------------------------------------------------------
// Soundscape sndRing buffer (stereo frames). Producer = audioTask only.
// Consumer = whichever output is active. Index updates are guarded by a
// spinlock; a generation counter makes a file switch safe mid-read.
// ---------------------------------------------------------------------------
static const int RING_FRAMES = 8192; // ~186 ms of cushion for Bluetooth hiccups and slow SD reads
static StereoFrame sndRing[RING_FRAMES];
static int ringWritePos = 0, ringReadPos = 0, ringFilled = 0;
static uint32_t ringGen = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

static int ringPull(StereoFrame* out, int n) {
  portENTER_CRITICAL(&ringMux);
  int filled = ringFilled, rp = ringReadPos;
  uint32_t gen = ringGen;
  portEXIT_CRITICAL(&ringMux);

  int take = n < filled ? n : filled;
  int first = RING_FRAMES - rp;
  if (first > take) first = take;
  memcpy(out, sndRing + rp, first * sizeof(StereoFrame));
  if (take > first) memcpy(out + first, sndRing, (take - first) * sizeof(StereoFrame));

  portENTER_CRITICAL(&ringMux);
  if (gen == ringGen) {
    ringReadPos = (rp + take) % RING_FRAMES;
    ringFilled -= take;
  }
  portEXIT_CRITICAL(&ringMux);

  if (take < n) {
    memset(out + take, 0, (n - take) * sizeof(StereoFrame));
    g_underruns++;
  }
  return take;
}

static void ringReset() {
  portENTER_CRITICAL(&ringMux);
  ringWritePos = ringReadPos = ringFilled = 0;
  ringGen++;
  portEXIT_CRITICAL(&ringMux);
}

// ---------------------------------------------------------------------------
// WAV file handling (audioTask only)
// ---------------------------------------------------------------------------
static fs::File sndFile;
static uint32_t sndDataStart = 0, sndDataSize = 0, sndRemaining = 0;
static uint16_t sndChannels = 2;

static char currentPath[64] = "";
static char pendingPath[64] = "";
static volatile bool pendingOpen = false;
static portMUX_TYPE pathMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t rd32(const uint8_t* b) { return b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t)b[3] << 24); }

static bool openWav(const char* path) {
  if (sndFile) sndFile.close();
  sndDataSize = 0;
  if (!path || !path[0] || !sdmedia_isAvailable()) return false;
  sndFile = SD.open(path, FILE_READ);
  if (!sndFile) return false;

  uint8_t hdr[12];
  if (sndFile.read(hdr, 12) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) { sndFile.close(); return false; }
  uint16_t bits = 16;
  sndChannels = 2;
  while (sndFile.available()) {
    uint8_t ch[8];
    if (sndFile.read(ch, 8) != 8) break;
    uint32_t size = rd32(ch + 4);
    if (!memcmp(ch, "fmt ", 4)) {
      uint8_t fmt[16];
      if (sndFile.read(fmt, 16) != 16) break;
      sndChannels = fmt[2] | (fmt[3] << 8);
      bits = fmt[14] | (fmt[15] << 8);
      if (size > 16) sndFile.seek(sndFile.position() + (size - 16) + (size & 1));
    } else if (!memcmp(ch, "data", 4)) {
      sndDataStart = sndFile.position();
      sndDataSize = size;
      break;
    } else {
      sndFile.seek(sndFile.position() + size + (size & 1));
    }
  }
  if (sndDataSize == 0 || bits != 16 || (sndChannels != 1 && sndChannels != 2)) {
    sndFile.close();
    sndDataSize = 0;
    return false;
  }
  sndRemaining = sndDataSize;
  return true;
}

static uint8_t readBuf[4096];

static void refillRing() {
  if (!sndFile || sndDataSize == 0) return;
  const int bpf = sndChannels * 2;
  for (int guard = 0; guard < 8; guard++) {
    portENTER_CRITICAL(&ringMux);
    int freeFrames = RING_FRAMES - ringFilled;
    int wp = ringWritePos;
    uint32_t gen = ringGen;
    portEXIT_CRITICAL(&ringMux);
    if (freeFrames < 512) return;

    int chunk = freeFrames;
    if (chunk > RING_FRAMES - wp) chunk = RING_FRAMES - wp;
    if (chunk > (int)sizeof(readBuf) / bpf) chunk = sizeof(readBuf) / bpf;
    if (sndRemaining < (uint32_t)bpf) { // loop the file seamlessly
      sndFile.seek(sndDataStart);
      sndRemaining = sndDataSize;
    }
    if ((uint32_t)(chunk * bpf) > sndRemaining) chunk = sndRemaining / bpf;

    sdmedia_lock();
    int got = sndFile.read(readBuf, chunk * bpf) / bpf;
    sdmedia_unlock();
    if (got <= 0) { sndFile.seek(sndDataStart); sndRemaining = sndDataSize; return; }
    sndRemaining -= got * bpf;

    const int16_t* s = (const int16_t*)readBuf;
    StereoFrame* d = sndRing + wp;
    if (sndChannels == 2) {
      memcpy(d, s, got * sizeof(StereoFrame));
    } else {
      for (int i = 0; i < got; i++) { d[i].l = d[i].r = s[i]; }
    }

    portENTER_CRITICAL(&ringMux);
    if (gen == ringGen) {
      ringWritePos = (wp + got) % RING_FRAMES;
      ringFilled += got;
    }
    portEXIT_CRITICAL(&ringMux);
  }
}

static void serviceSoundscapeFile() {
  if (pendingOpen) {
    char path[64];
    portENTER_CRITICAL(&pathMux);
    strncpy(path, pendingPath, sizeof(path));
    pendingOpen = false;
    portEXIT_CRITICAL(&pathMux);
    ringReset();
    sdmedia_lock();
    openWav(path);
    sdmedia_unlock();
  }
  refillRing();
}

// ---------------------------------------------------------------------------
// The one renderer. Handles smooth fades whenever the source changes
// (Off <-> Tone <-> Soundscape) so there are no clicks.
// ---------------------------------------------------------------------------
static AudioSource renderingSource = AUDIO_SRC_OFF;
static const float SCAPE_GAIN = 1.0f; // no boost - boosting clipped the loud parts (rain hits, thunder) into static
// Keeps boosted audio from wrapping around (which sounds like loud crackles).
static inline int16_t clip16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)v;
}
static float fadeGain = 0.0f;
static const float FADE_STEP = 1.0f / (AUD_SAMPLE_RATE * 0.04f); // ~40 ms

static void renderFrames(StereoFrame* out, int n) {
  AudioSource want = g_source;

  if (renderingSource == AUDIO_SRC_SOUNDSCAPE) ringPull(out, n);
  else if (renderingSource == AUDIO_SRC_TONE) renderTone(out, n);
  else memset(out, 0, n * sizeof(StereoFrame));

  float vol = g_volume;
  // Soundscape files are mastered quieter than our tone - lift them to match.
  float srcGain = (renderingSource == AUDIO_SRC_SOUNDSCAPE) ? SCAPE_GAIN : 1.0f;
  bool switching = (want != renderingSource);
  for (int i = 0; i < n; i++) {
    if (switching) { fadeGain -= FADE_STEP; if (fadeGain < 0) fadeGain = 0; }
    else if (fadeGain < 1.0f) { fadeGain += FADE_STEP; if (fadeGain > 1.0f) fadeGain = 1.0f; }
    float g = fadeGain * vol * srcGain;
    out[i].l = clip16(out[i].l * g);
    out[i].r = clip16(out[i].r * g);
  }
  if (switching && fadeGain <= 0.0f) {
    renderingSource = want;
    phL = phR = phEnv = 0;
  }
}

// ---------------------------------------------------------------------------
// Onboard speaker (I2S -> built-in DAC2 on IO26 -> onboard amp)
// ---------------------------------------------------------------------------
static void speakerInit() {
  pinMode(AUDIO_ENABLE, OUTPUT);
  digitalWrite(AUDIO_ENABLE, HIGH); // amp off until there is sound

  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
  cfg.sample_rate = AUD_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB; // correct format for the built-in DAC
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 512;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;

  if (i2s_driver_install(SPK_I2S_PORT, &cfg, 0, NULL) != ESP_OK) return;

  // IMPORTANT: do NOT call i2s_set_pin(port, NULL) - that enables BOTH DAC
  // pins, and IO25 is the coil's PWM pin. Enable only DAC2 (IO26, speaker).
  i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
  dac_output_disable(DAC_CHANNEL_1);
  rtc_gpio_deinit(GPIO_NUM_25);        // hand IO25 back to the digital matrix for LEDC
  i2s_zero_dma_buffer(SPK_I2S_PORT);
  g_speakerReady = true;
}

static uint16_t dacBuf[256 * 2];
static StereoFrame spkFrames[256];

static void speakerPump() {
  static unsigned long silentSince = 0;
  static bool ampOn = false;

  renderFrames(spkFrames, 256);

  bool audible = (renderingSource != AUDIO_SRC_OFF) || (g_source != AUDIO_SRC_OFF);
  if (audible) {
    silentSince = 0;
    if (!ampOn) { digitalWrite(AUDIO_ENABLE, LOW); ampOn = true; }
  } else if (ampOn) {
    if (silentSince == 0) silentSince = millis();
    else if (millis() - silentSince > 800) { digitalWrite(AUDIO_ENABLE, HIGH); ampOn = false; }
  }

  // Built-in DAC takes the top 8 bits, unsigned. Mix L+R to mono for the
  // single speaker, then offset to unsigned mid-scale. Dropping to 8 bits by
  // plain truncation sounds grainy/crunchy on quiet passages, so each sample
  // is rounded and gets a tiny random "dither" first - the standard fix.
  static uint32_t lcg = 0x1234567u;
  for (int i = 0; i < 256; i++) {
    int32_t m = ((int32_t)spkFrames[i].l + spkFrames[i].r) / 2;
    lcg = lcg * 1664525u + 1013904223u; int32_t r1 = (int32_t)(lcg >> 24);
    lcg = lcg * 1664525u + 1013904223u; int32_t r2 = (int32_t)(lcg >> 24);
    m += (r1 - r2) + 128;                 // +/- one 8-bit step of dither, plus rounding
    if (m > 32767) m = 32767;
    if (m < -32768) m = -32768;
    uint16_t u = (uint16_t)(m + 32768);
    dacBuf[2 * i] = u;
    dacBuf[2 * i + 1] = u;
  }
  size_t written = 0;
  i2s_write(SPK_I2S_PORT, dacBuf, sizeof(dacBuf), &written, portMAX_DELAY);
}

static void speakerQuiet() {
  static bool quieted = false;
  if (g_output == AUDIO_OUT_SPEAKER) { quieted = false; return; }
  if (!quieted) {
    digitalWrite(AUDIO_ENABLE, HIGH);
    if (g_speakerReady) i2s_zero_dma_buffer(SPK_I2S_PORT);
    quieted = true;
  }
}

// ---------------------------------------------------------------------------
// Bluetooth (A2DP source)
// ---------------------------------------------------------------------------
static BluetoothA2DPSource a2dp;
static char btName[64] = "";
static bool g_btStarted = false;
static bool btVolumeSent = false;

// Diagnostics: proves whether the BT stack is actually pulling audio from us,
// and whether what we hand it is silent or not.
static volatile uint32_t btFramesSent = 0;
static volatile int16_t btLastPeak = 0;

static int32_t btDataCallback(Frame* data, int32_t len) {
  if (len <= 0) return 0;
  if (g_output != AUDIO_OUT_BLUETOOTH) {
    memset(data, 0, len * sizeof(Frame));
  } else {
    renderFrames((StereoFrame*)data, len);
  }
  int16_t peak = 0;
  StereoFrame* f = (StereoFrame*)data;
  for (int32_t i = 0; i < len; i += 8) {
    int16_t a = f[i].l < 0 ? -f[i].l : f[i].l;
    if (a > peak) peak = a;
  }
  btLastPeak = peak;
  btFramesSent += len;
  return len;
}

// ---- Connection tracking (for on-screen status) ----------------------------
// The library reports each state change through this callback; the UI reads
// audio_btStatus() to show Searching / Connecting / Connected / Not found.
static volatile esp_a2d_connection_state_t btConnState = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
static volatile bool btEverConnected = false;
static volatile uint32_t btStatusChanges = 0;
static unsigned long btAttemptStartMs = 0;
static bool btSavedAddrValid = false;
static uint8_t btSavedAddr[6];
static bool btQuickConnectDone = false;
static const unsigned long BT_NOT_FOUND_MS = 45000;

// The paired device's address is remembered, so later boots reconnect to it
// directly (a couple of seconds) instead of searching for its name (slow).
static void btLoadSavedAddr() {
  Preferences p;
  p.begin("btaddr", true);
  String n = p.getString("name", "");
  btSavedAddrValid = (n.length() > 0 && n == String(btName) && p.getBytes("addr", btSavedAddr, 6) == 6);
  p.end();
}
static void btSaveAddr(const uint8_t* addr) {
  memcpy(btSavedAddr, addr, 6);
  btSavedAddrValid = true;
  Preferences p;
  p.begin("btaddr", false);
  p.putString("name", btName);
  p.putBytes("addr", btSavedAddr, 6);
  p.end();
}

static void onBtConnectionState(esp_a2d_connection_state_t state, void*) {
  btConnState = state;
  btStatusChanges++;
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) btEverConnected = true;
  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) btAttemptStartMs = millis(); // library keeps retrying; restart the "not found" clock
}

// While connecting by name: accept the device whose name matches ours.
static bool onSsidMatchSaved(const char* ssid, esp_bd_addr_t address, int rssi) {
  if (ssid && btName[0] && strcmp(ssid, btName) == 0) {
    btSaveAddr(address);
    return true;
  }
  return false;
}

static void btService() {
  if (!g_btStarted) return;
  if (!btVolumeSent && a2dp.is_connected()) {
    a2dp.set_volume(100); // ~80% of the speaker's range - higher drives small speakers into distortion
    btVolumeSent = true;
  }
  if (!a2dp.is_connected()) btVolumeSent = false;

  // Note: never call a2dp.connect_to() from here. The library only starts
  // the audio stream for connections its own state machine made; a
  // connection made around it shows "connected" but plays nothing.
}

// ---------------------------------------------------------------------------
// The audio task
// ---------------------------------------------------------------------------
static void audioTask(void*) {
  for (;;) {
    serviceSoundscapeFile();
    btService();
    speakerQuiet();
    if (g_output == AUDIO_OUT_SPEAKER && g_speakerReady) {
      speakerPump(); // blocks in i2s_write -> paces this loop (~5.8 ms per pass)
    } else {
      vTaskDelay(pdMS_TO_TICKS(2)); // Bluetooth pulls audio itself - just keep the soundscape buffer full
    }
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void audio_begin() {
  static bool begun = false;
  if (begun) return;
  begun = true;
  for (int i = 0; i < SIN_N; i++) sinTable[i] = sinf(TWO_PI_F * i / SIN_N);
  pinMode(AUDIO_ENABLE, OUTPUT);
  digitalWrite(AUDIO_ENABLE, HIGH); // amp off
  // The speaker's I2S driver is installed only when the speaker is actually
  // the chosen output (see audio_setOutput) - it holds ~16 KB of DMA memory
  // that the Bluetooth stack needs more when Bluetooth is in use.
  xTaskCreatePinnedToCore(audioTask, "audio", 6144, NULL, 3, NULL, 1);
}

void audio_setOutput(AudioOutput out) {
  if (out == AUDIO_OUT_SPEAKER && !g_speakerReady) speakerInit();
  g_output = out;
}
AudioOutput audio_getOutput() { return g_output; }

void audio_setSource(AudioSource src) { g_source = src; }
AudioSource audio_getSource() { return g_source; }

void audio_setToneFrequency(float hz) { g_toneHz = hz; }

bool audio_setSoundscapeFile(const char* path) {
  if (!path || !path[0]) return false;
  if (strcmp(path, currentPath) == 0) return true; // already loaded
  strncpy(currentPath, path, sizeof(currentPath) - 1);
  currentPath[sizeof(currentPath) - 1] = 0;
  portENTER_CRITICAL(&pathMux);
  strncpy(pendingPath, path, sizeof(pendingPath) - 1);
  pendingPath[sizeof(pendingPath) - 1] = 0;
  pendingOpen = true;
  portEXIT_CRITICAL(&pathMux);
  return true;
}
const char* audio_soundscapeFile() { return currentPath; }

void audio_setVolume(uint8_t percent) {
  if (percent > 100) percent = 100;
  // Ears hear loudness on a curve, not a straight line - this makes 50% sound
  // like half volume instead of nearly full.
  g_volume = powf(percent / 100.0f, 1.8f);
}

void audio_setHeadphonesMode(bool on) { g_headphones = on; }
bool audio_isHeadphonesMode() { return g_headphones; }

void audio_setBtDeviceName(const char* name) {
  strncpy(btName, name ? name : "", sizeof(btName) - 1);
  btName[sizeof(btName) - 1] = 0;
}
const char* audio_btDeviceName() { return btName; }

void audio_btConnect() {
  if (g_btStarted || btName[0] == 0) return;
  btLoadSavedAddr();
  a2dp.set_on_connection_state_changed(onBtConnectionState);
  a2dp.set_ssid_callback(onSsidMatchSaved);
  a2dp.set_auto_reconnect(true); // library-native fast reconnect to the last device
  btAttemptStartMs = millis();
  btQuickConnectDone = true;
  // Same call the earlier (working) firmware used for Bluetooth tone output.
  a2dp.start(btName, btDataCallback); // returns quickly; the connection completes in the background
  g_btStarted = true;
  btVolumeSent = false;
}

BtStatus audio_btStatus() {
  if (!g_btStarted) return BT_STATUS_OFF;
  if (btConnState == ESP_A2D_CONNECTION_STATE_CONNECTED) return BT_STATUS_CONNECTED;
  if (btConnState == ESP_A2D_CONNECTION_STATE_CONNECTING) return BT_STATUS_CONNECTING;
  if (millis() - btAttemptStartMs > BT_NOT_FOUND_MS) return BT_STATUS_NOT_FOUND;
  return btEverConnected ? BT_STATUS_RECONNECTING : BT_STATUS_SEARCHING;
}
uint32_t audio_btStatusChanges() { return btStatusChanges; }

void audio_btRetry() {
  if (!g_btStarted || a2dp.is_connected()) return;
  btAttemptStartMs = millis();
  a2dp.cancel_discovery();
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0); // library connects when it sees the saved name
}
bool audio_btStarted() { return g_btStarted; }
bool audio_btIsConnected() { return g_btStarted && a2dp.is_connected(); }

void audio_btEnd(bool releaseMemory) {
  if (g_btStarted) {
    a2dp.end(releaseMemory);
    g_btStarted = false;
  }
}

// ---- scan ----
static char scanResults[BT_SCAN_MAX_RESULTS][32];
static uint8_t scanAddrs[BT_SCAN_MAX_RESULTS][6];
static volatile int scanCount = 0;
static volatile bool g_scanning = false;

static bool onSsidFound(const char* ssid, esp_bd_addr_t address, int rssi) {
  if (!ssid || !ssid[0] || scanCount >= BT_SCAN_MAX_RESULTS) return false;
  for (int i = 0; i < scanCount; i++) if (strcmp(scanResults[i], ssid) == 0) return false;
  strncpy(scanResults[scanCount], ssid, sizeof(scanResults[0]) - 1);
  scanResults[scanCount][sizeof(scanResults[0]) - 1] = 0;
  memcpy(scanAddrs[scanCount], address, 6);
  scanCount++;
  return false; // never auto-connect - the person picks from the list
}

static void onDiscoveryModeChanged(esp_bt_gap_discovery_state_t mode) {
  g_scanning = (mode == ESP_BT_GAP_DISCOVERY_STARTED);
}

void audio_btStartScan() {
  scanCount = 0;
  a2dp.set_ssid_callback(onSsidFound);
  a2dp.set_discovery_mode_callback(onDiscoveryModeChanged);
  a2dp.set_on_connection_state_changed(onBtConnectionState);
  a2dp.set_data_callback_in_frames(btDataCallback);
  if (!g_btStarted) {
    a2dp.start(); // no name -> open discovery
    g_btStarted = true;
  } else {
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0); // stack already up - just search again
  }
}

// Connect straight to a device picked from the scan list - no restart needed.
void audio_btConnectToScanResult(int idx) {
  if (idx < 0 || idx >= scanCount) return;
  a2dp.cancel_discovery();
  strncpy(btName, scanResults[idx], sizeof(btName) - 1);
  btName[sizeof(btName) - 1] = 0;
  btSaveAddr(scanAddrs[idx]);
  // Let the library make the connection itself: search again, and the
  // name-match callback tells it "this one" when the device shows up.
  a2dp.set_ssid_callback(onSsidMatchSaved);
  a2dp.set_auto_reconnect(true);
  btAttemptStartMs = millis();
  btQuickConnectDone = true;
  btEverConnected = false;
  delay(300); // let the cancelled search wind down
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}
void audio_btStopScan() { if (g_btStarted) a2dp.cancel_discovery(); }
bool audio_btIsScanning() { return g_scanning; }
int audio_btScanResultCount() { return scanCount; }
const char* audio_btScanResultName(int idx) {
  if (idx < 0 || idx >= scanCount) return "";
  return scanResults[idx];
}

// Forget the paired device everywhere it is remembered: our fast-reconnect
// memory AND the Bluetooth library's own "last device" (otherwise the library
// quietly reconnects to the old speaker).
void audio_btForget() {
  if (g_btStarted && a2dp.is_connected()) a2dp.disconnect();
  a2dp.clean_last_connection();
  Preferences p;
  p.begin("btaddr", false);
  p.clear();
  p.end();
  btSavedAddrValid = false;
  btName[0] = 0;
}

uint32_t audio_btFramesSent() { return btFramesSent; }
int audio_btLastPeak() { return btLastPeak; }

bool audio_speakerReady() { return g_speakerReady; }
unsigned long audio_underruns() { return g_underruns; }

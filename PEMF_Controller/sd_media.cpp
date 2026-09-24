#include "sd_media.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include "pins.h"

extern TFT_eSPI tft; // the one instance, declared in the main .ino

static bool sdAvailable = false;

// Trying a genuinely separate SPI peripheral for the SD card, rather
// than sharing the display's. The earlier shared-instance approach
// (SD.begin(SD_CS, tft.getSPIinstance())) was the right fix for a real,
// confirmed problem - SD breaking touch entirely - but the SD card
// still won't mount even with that fix in place, which raises a real
// possibility: sharing one bus between two peripherals with different
// timing/mode needs could itself be preventing SD from initializing
// correctly, even though it stopped the worse symptom (broken touch).
// ESP32 has two general-purpose SPI peripherals (HSPI and VSPI); the
// display appears to already be using VSPI (that's what the sharing fix
// was built around), so this gives the SD card its own independent
// HSPI connection instead, using its actual wired pins via the GPIO
// matrix (any pin can route to either peripheral) - genuinely separate
// hardware, not just separate pin numbers.
static SPIClass sdSPI(HSPI);
static SemaphoreHandle_t sdMutex = nullptr;

void sdmedia_lock() {
  if (!sdMutex) sdMutex = xSemaphoreCreateRecursiveMutex();
  xSemaphoreTakeRecursive(sdMutex, portMAX_DELAY);
}
void sdmedia_unlock() {
  if (sdMutex) xSemaphoreGiveRecursive(sdMutex);
}

bool sdmedia_begin() {
  sdmedia_lock();
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  sdAvailable = SD.begin(SD_CS, sdSPI, 20000000); // faster clock = quicker splash + easy soundscape streaming
  if (!sdAvailable) sdAvailable = SD.begin(SD_CS, sdSPI); // fall back to the library's safe default clock
  sdmedia_unlock();
  return sdAvailable;
}

bool sdmedia_isAvailable() { return sdAvailable; }

// ---------------------------------------------------------------------
// BMP drawing - adapted from TFT_eSPI's own verified example
// (TFT_SPIFFS_BMP/BMP_functions.ino), swapped from SPIFFS to SD. Only
// handles 24-bit uncompressed BMP, which is what we export the splash
// image as. BMP rows are stored bottom-up, hence the y-- as we go.
// ---------------------------------------------------------------------
static uint16_t read16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read();
  ((uint8_t *)&result)[1] = f.read();
  return result;
}

static uint32_t read32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read();
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read();
  return result;
}

// Reads ROWS_PER_CHUNK rows per SD read (one big read is far faster than
// hundreds of tiny ones) and releases the SD lock between chunks so a
// playing soundscape never starves while the splash is being drawn.
static const int ROWS_PER_CHUNK = 4; // ~5.8 KB temporary buffer - leaves RAM for Bluetooth

static bool drawBmpFromSd(const char *filename, int16_t x, int16_t y) {
  if ((x >= tft.width()) || (y >= tft.height())) return false;
  sdmedia_lock();
  fs::File bmpFS = SD.open(filename, FILE_READ);
  if (!bmpFS) { sdmedia_unlock(); return false; }

  bool ok = false;
  uint16_t w = 0, h = 0;
  uint32_t seekOffset = 0;
  bool valid = false;
  if (read16(bmpFS) == 0x4D42) { // "BM" signature
    read32(bmpFS);               // file size (unused)
    read32(bmpFS);               // reserved
    seekOffset = read32(bmpFS);
    read32(bmpFS);               // header size (unused)
    w = read32(bmpFS);
    h = read32(bmpFS);
    valid = (read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0) && w > 0 && w <= 480;
  }
  sdmedia_unlock();

  if (valid) {
    uint16_t padding = (4 - ((w * 3) & 3)) & 3;
    uint32_t rowBytes = w * 3 + padding;
    uint8_t* buf = (uint8_t*)malloc(rowBytes * ROWS_PER_CHUNK);
    if (buf) {
      bool oldSwapBytes = tft.getSwapBytes();
      tft.setSwapBytes(true);
      int16_t rowY = y + h - 1; // BMP rows are stored bottom-up
      uint32_t filePos = seekOffset;
      for (uint16_t row = 0; row < h; row += ROWS_PER_CHUNK) {
        int rows = (h - row < ROWS_PER_CHUNK) ? (h - row) : ROWS_PER_CHUNK;
        sdmedia_lock();
        bmpFS.seek(filePos);
        bmpFS.read(buf, rowBytes * rows);
        sdmedia_unlock();
        filePos += rowBytes * rows;
        for (int r = 0; r < rows; r++) {
          uint8_t* bptr = buf + r * rowBytes;
          uint16_t* tptr = (uint16_t*)bptr; // converted in place (2 bytes out per 3 in)
          for (uint16_t col = 0; col < w; col++) {
            uint8_t b = *bptr++, g = *bptr++, rr = *bptr++;
            *tptr++ = ((rr & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
          }
          tft.pushImage(x, rowY--, w, 1, (uint16_t*)(buf + r * rowBytes));
        }
      }
      tft.setSwapBytes(oldSwapBytes);
      free(buf);
      ok = true;
    }
  }
  sdmedia_lock();
  bmpFS.close();
  sdmedia_unlock();
  return ok;
}

static int splashState = -1; // -1 unknown, 0 missing, 1 present - checked once, not on every screen

bool sdmedia_showSplash() {
  if (!sdAvailable) return false;
  if (splashState < 0) {
    sdmedia_lock();
    splashState = SD.exists("/splash.bmp") ? 1 : 0;
    sdmedia_unlock();
  }
  if (splashState == 0) return false;
  return drawBmpFromSd("/splash.bmp", 0, 0);
}

// ---------------------------------------------------------------------
// Soundscape file scanning
// ---------------------------------------------------------------------
static char soundscapeNames[MAX_SOUNDSCAPES][32];
static char soundscapePaths[MAX_SOUNDSCAPES][48];
static int soundscapeCountVal = 0;

// Diagnostic counters, exposed so the Welcome screen can show exactly
// where this process succeeds or fails - real evidence instead of
// another guess, after two rounds of guessing didn't resolve this.
static bool lastScanDirOpened = false;
static int lastScanTotalEntries = 0;
static int lastScanFileEntries = 0;

int sdmedia_scanSoundscapes() {
  soundscapeCountVal = 0;
  lastScanDirOpened = false;
  lastScanTotalEntries = 0;
  lastScanFileEntries = 0;
  if (!sdAvailable) return 0;

  sdmedia_lock();
  struct Unlocker { ~Unlocker() { sdmedia_unlock(); } } unlocker; // released on every return path

  fs::File dir = SD.open("/sounds");
  if (!dir || !dir.isDirectory()) return 0;
  lastScanDirOpened = true;

  fs::File entry = dir.openNextFile();
  while (entry && soundscapeCountVal < MAX_SOUNDSCAPES) {
    lastScanTotalEntries++;
    if (!entry.isDirectory()) {
      lastScanFileEntries++;
      String rawName = entry.name(); // may be a bare filename or a full path, depending on core version
      String lower = rawName;
      lower.toLowerCase();
      if (lower.endsWith(".wav")) {
        String base = rawName;
        int slash = base.lastIndexOf('/');
        if (slash >= 0) base = base.substring(slash + 1);
        String displayName = base;
        displayName.replace(".wav", "");
        displayName.replace(".WAV", "");

        String fullPath = rawName;
        if (!fullPath.startsWith("/")) fullPath = "/sounds/" + fullPath;

        strncpy(soundscapeNames[soundscapeCountVal], displayName.c_str(), sizeof(soundscapeNames[0]) - 1);
        soundscapeNames[soundscapeCountVal][sizeof(soundscapeNames[0]) - 1] = 0;
        strncpy(soundscapePaths[soundscapeCountVal], fullPath.c_str(), sizeof(soundscapePaths[0]) - 1);
        soundscapePaths[soundscapeCountVal][sizeof(soundscapePaths[0]) - 1] = 0;
        soundscapeCountVal++;
      }
    }
    entry = dir.openNextFile();
  }
  return soundscapeCountVal;
}

bool sdmedia_lastScanDirOpened() { return lastScanDirOpened; }
int sdmedia_lastScanTotalEntries() { return lastScanTotalEntries; }
int sdmedia_lastScanFileEntries() { return lastScanFileEntries; }

int sdmedia_soundscapeCount() { return soundscapeCountVal; }

const char* sdmedia_soundscapeName(int idx) {
  if (idx < 0 || idx >= soundscapeCountVal) return "";
  return soundscapeNames[idx];
}

const char* sdmedia_soundscapePath(int idx) {
  if (idx < 0 || idx >= soundscapeCountVal) return "";
  return soundscapePaths[idx];
}

#include "sd_media.h"
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

bool sdmedia_begin() {
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  sdAvailable = SD.begin(SD_CS, sdSPI);
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

static bool drawBmpFromSd(const char *filename, int16_t x, int16_t y) {
  if ((x >= tft.width()) || (y >= tft.height())) return false;
  fs::File bmpFS = SD.open(filename, FILE_READ);
  if (!bmpFS) return false;

  bool ok = false;
  if (read16(bmpFS) == 0x4D42) { // "BM" signature
    read32(bmpFS);               // file size (unused)
    read32(bmpFS);                // reserved
    uint32_t seekOffset = read32(bmpFS);
    read32(bmpFS);                // header size (unused)
    uint16_t w = read32(bmpFS);
    uint16_t h = read32(bmpFS);

    if ((read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0)) {
      y += h - 1; // BMP rows are bottom-up
      bool oldSwapBytes = tft.getSwapBytes();
      tft.setSwapBytes(true);
      bmpFS.seek(seekOffset);
      uint16_t padding = (4 - ((w * 3) & 3)) & 3;
      uint8_t lineBuffer[w * 3 + padding];
      for (uint16_t row = 0; row < h; row++) {
        bmpFS.read(lineBuffer, sizeof(lineBuffer));
        uint8_t* bptr = lineBuffer;
        uint16_t* tptr = (uint16_t*)lineBuffer;
        for (uint16_t col = 0; col < w; col++) {
          uint8_t b = *bptr++, g = *bptr++, r = *bptr++;
          *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }
        tft.pushImage(x, y--, w, 1, (uint16_t*)lineBuffer);
      }
      tft.setSwapBytes(oldSwapBytes);
      ok = true;
    }
  }
  bmpFS.close();
  return ok;
}

bool sdmedia_showSplash() {
  if (!sdAvailable) return false;
  if (!SD.exists("/splash.bmp")) return false;
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

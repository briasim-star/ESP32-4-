// ============================================================================
// TFT_eSPI User_Setup.h - configured for the LCDWiki 4.0" ESP32-32E display
// ============================================================================
// This file is copied over the TFT_eSPI library's own User_Setup.h by the
// GitHub Actions build (see .github/workflows/build_and_publish.yml). You
// do not need to touch this by hand for the automated build to work.
//
// If you ever DO build with Arduino IDE locally instead, copy this file to:
//   <Arduino sketchbook>/libraries/TFT_eSPI/User_Setup.h
// (overwriting the one that ships with the library).
// ============================================================================

#define ST7796_DRIVER

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1   // shares reset with the ESP32 itself

#define TFT_BL   27
#define TFT_BACKLIGHT_ON HIGH

#define TOUCH_CS 33

#define LOAD_GLCD
#define LOAD_FONT2
// #define LOAD_FONT4   // not used by MADD PEMF - left out to keep the firmware small enough for wireless updates
// #define LOAD_FONT6   // not used by MADD PEMF - left out to keep the firmware small enough for wireless updates
// #define LOAD_FONT7   // not used by MADD PEMF - left out to keep the firmware small enough for wireless updates
// #define LOAD_FONT8   // not used by MADD PEMF - left out to keep the firmware small enough for wireless updates
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

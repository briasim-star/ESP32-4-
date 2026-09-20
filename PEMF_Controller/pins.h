#pragma once
// ============================================================================
// pins.h - Pin map for the LCDWiki 4.0" ESP32-32E (E32R40T) display module
// ============================================================================
// These come straight from the board's published pin assignment table.
// DO NOT reassign the onboard pins below - they are physically wired on the
// PCB and cannot be moved. The only pins we get to choose ourselves are
// PIN_MD10C_PWM and PIN_MD10C_DIR, which use the board's otherwise-unused
// I2C header pins (IO25 / IO32).
// ============================================================================

// ---- Our added wiring: MD10C motor driver -----------------------------
// We're using the board's onboard "I2C Peripheral Interface" header for
// this - it's a single 1.25mm 4-pin connector that already breaks out
// VCC, GND, and two free GPIOs (IO25/IO32), so one cable carries all
// four wires the MD10C needs. No I2C protocol is actually used - we're
// just repurposing the header's physical pins as plain digital I/O.
//
//   Board "I2C" header pin  ->  MD10C pin
//   ------------------------    ---------
//   VCC (3.3V)               -> VCC (logic)
//   GND                      -> GND
//   IO25 (labeled SCL)       -> PWM
//   IO32 (labeled SDA)       -> DIR
//
#define PIN_MD10C_PWM   25   // -> MD10C "PWM" pin (board's I2C header, SCL position)
#define PIN_MD10C_DIR   32   // -> MD10C "DIR" pin (board's I2C header, SDA position)
// MD10C VIN+/VIN-     -> your 14V/10A supply (NOT from the ESP32 board)
// MD10C OUT A/OUT B   -> your coil leads

// ---- Onboard LCD (ST7796, 4-wire SPI) ----------------------------------
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST   -1   // shares the EN/reset line with the ESP32 itself
#define TFT_SCLK  14
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_BL    27   // backlight, HIGH = on

// ---- Onboard resistive touch (XPT2046, shares the LCD SPI bus) --------
#define TOUCH_CS   33
#define TOUCH_IRQ  36

// ---- Onboard RGB status LED (common anode: LOW = on) -------------------
#define LED_R 22
#define LED_G 16
#define LED_B 17

// ---- Onboard speaker path (used only if BT audio is off and you want
//      local audible feedback instead - optional, not required) --------
#define AUDIO_ENABLE 4    // LOW = amplifier enabled
#define AUDIO_DAC    26   // onboard DAC output to the speaker amp

// ---- Onboard MicroSD card slot -----------------------------------------
// Uses the ESP32's standard VSPI pin numbers, but on a genuinely
// separate HSPI peripheral instance (see sd_media.cpp) - sharing the
// display's own SPI instance fixed a real, confirmed touch-breaking
// conflict, but the SD card still wasn't mounting even with that fix in
// place, so this tries fully independent hardware instead.
#define SD_CS      5    // SD card select, low level effective
#define SD_MOSI    23
#define SD_MISO    19
#define SD_SCLK    18

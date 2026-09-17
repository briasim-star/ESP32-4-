#pragma once
#include <Arduino.h>
#include "presets.h"

// Waveform engine driving the MD10C over PIN_MD10C_PWM / PIN_MD10C_DIR.
// - WAVE_SQUARE drives DIR with a hardware-PWM square wave at the target
//   frequency (50% duty = symmetric on/off time), while the PWM
//   (magnitude) pin's duty cycle sets the overall amplitude via the
//   intensity setting. Result: an exact, jitter-free bipolar square wave
//   at any frequency the LEDC hardware can generate, with adjustable
//   amplitude.
// - WAVE_SINE uses the MD10C's "sign-magnitude" mode: DIR carries the
//   polarity bit, PWM carries the instantaneous magnitude at a fixed
//   20kHz carrier. A timer ISR steps through a sine lookup table; the
//   coil's own inductance low-pass-filters the switching into a smooth
//   sine current.

void waveform_begin();
void waveform_start(float freqHz, WaveShape shape, uint8_t intensityPercent = 100);
void waveform_setIntensity(uint8_t intensityPercent); // live update while running
void waveform_setFrequency(float newFreqHz); // live update while running (for ramps/programs)
void waveform_stop();
bool waveform_isRunning();
float waveform_currentFreq();
WaveShape waveform_currentShape();

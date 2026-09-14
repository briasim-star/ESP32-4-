#pragma once
#include <Arduino.h>
// ============================================================================
// presets.h - Base frequency list + auto-generated harmonics
// ============================================================================
// IMPORTANT: These are commonly-cited reference numbers from PEMF literature,
// Schumann-resonance references, and general brainwave-band terminology.
// They are labeled by NAME/NUMBER ONLY - not by any health claim - because
// frequency-to-condition claims in this space are marketing, not settled
// science. Treat this as a signal generator with a curated frequency list,
// not a medical device.
// ============================================================================

enum WaveShape : uint8_t { WAVE_SQUARE = 0, WAVE_SINE = 1 };

struct Preset {
  const char* name;
  float freqHz;
  WaveShape wave;
  const char* claim; // entertainment-only association tag, shown with a
                      // disclaimer - not a medical claim, see README.
};

// The MD10C is only rated for PWM/switching up to 20 kHz - we hard-cap all
// generated output (including harmonics) at that ceiling.
static const float MAX_OUTPUT_FREQ_HZ = 20000.0f;

// The "claim" field is an entertainment-only association tag, of the same
// flavor commercial PEMF/wellness gadgets print on their presets (sleep,
// calm, energy, etc). It is NOT a medical claim, has no clinical backing
// tied to the specific frequency, and is displayed with an on-screen
// disclaimer in the UI. See README for the reasoning behind this choice.
static const Preset BASE_PRESETS[] = {
  // --- Schumann-resonance reference values ---
  {"Schumann fundamental", 7.83f,  WAVE_SINE,   "Grounding / calm"},
  {"Schumann 2nd",         14.3f,  WAVE_SINE,   "Alertness"},
  {"Schumann 3rd",         20.8f,  WAVE_SINE,   "Focus"},
  {"Schumann 4th",         27.3f,  WAVE_SINE,   "Energy"},
  {"Schumann 5th",         33.8f,  WAVE_SINE,   "Uplift"},

  // --- Brainwave-band reference values (one representative Hz per band) ---
  {"Delta band",            2.0f,  WAVE_SQUARE, "Deep sleep"},
  {"Theta band",            6.0f,  WAVE_SQUARE, "Relaxation"},
  {"Alpha band",           10.0f,  WAVE_SQUARE, "Calm focus"},
  {"Beta band",            20.0f,  WAVE_SQUARE, "Alert / active"},
  {"Gamma band",           40.0f,  WAVE_SQUARE, "Mental clarity"},

  // --- Generic low-frequency sweep steps used across consumer PEMF gear ---
  {"Sweep 1 Hz",            1.0f,  WAVE_SQUARE, "Wind-down"},
  {"Sweep 3 Hz",            3.0f,  WAVE_SQUARE, "Sleep support"},
  {"Sweep 5 Hz",            5.0f,  WAVE_SQUARE, "Relaxation"},
  {"Sweep 8 Hz",            8.0f,  WAVE_SQUARE, "Calm"},
  {"Sweep 15 Hz",          15.0f,  WAVE_SQUARE, "Recovery"},
  {"Sweep 25 Hz",          25.0f,  WAVE_SQUARE, "Recovery"},
  {"Sweep 50 Hz",          50.0f,  WAVE_SQUARE, "General wellness"},
  {"Sweep 75 Hz",          75.0f,  WAVE_SQUARE, "Energy"},
  {"Sweep 100 Hz",        100.0f,  WAVE_SQUARE, "Invigorate"},

  // --- Experimental / higher-frequency test points ---
  {"Experimental 250Hz",  250.0f,  WAVE_SINE,   "Experimental"},
  {"Experimental 500Hz",  500.0f,  WAVE_SINE,   "Experimental"},
  {"Experimental 1kHz",  1000.0f,  WAVE_SINE,   "Experimental"},
  {"Experimental 3kHz",  3000.0f,  WAVE_SINE,   "Experimental"},
};
static const int NUM_BASE_PRESETS = sizeof(BASE_PRESETS) / sizeof(BASE_PRESETS[0]);

struct HarmonicEntry {
  const char* baseName;
  int order;
  float freqHz;
  WaveShape wave;
  const char* claim; // inherited from the base preset it was derived from
};

// Generates 2x..6x harmonics of every base preset, stopping once a harmonic
// would exceed MAX_OUTPUT_FREQ_HZ. Returns the number of entries written.
inline int buildHarmonicList(HarmonicEntry* out, int maxOut) {
  int n = 0;
  for (int i = 0; i < NUM_BASE_PRESETS && n < maxOut; i++) {
    for (int order = 2; order <= 6 && n < maxOut; order++) {
      float f = BASE_PRESETS[i].freqHz * order;
      if (f > MAX_OUTPUT_FREQ_HZ) break;
      out[n].baseName = BASE_PRESETS[i].name;
      out[n].order = order;
      out[n].freqHz = f;
      out[n].wave = BASE_PRESETS[i].wave;
      out[n].claim = BASE_PRESETS[i].claim;
      n++;
    }
  }
  return n;
}

// Generous upper bound for a statically-sized harmonics buffer in the .ino
static const int MAX_HARMONIC_ENTRIES = NUM_BASE_PRESETS * 5 + 8;

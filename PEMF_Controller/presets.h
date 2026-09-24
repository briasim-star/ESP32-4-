#pragma once
#include <Arduino.h>
// ============================================================================
// presets.h - Condition/frequency reference list
// ============================================================================
// Frequencies and condition groupings sourced from a public PEMF reference
// chart. These are commonly-cited numbers from PEMF marketing/wellness
// literature - NOT clinically validated, and NOT a substitute for medical
// care. See the startup safety screen for contraindications (implanted
// electronic devices, pregnancy) before using this device on a person.
// ============================================================================

// Pulse shapes. SQUARE = the classic bipolar pulse. SAW = power ramps up
// then drops sharply, polarity flipping each pulse (a shape offered by
// several established devices; adds even harmonics). LAYERED = each slow
// pulse is a short burst of fast 500 Hz polarity flips (like the
// "background frequency" programs some mats offer). All are exploration
// settings - no health benefit is claimed for any shape.
enum WaveShape : uint8_t { WAVE_SQUARE = 0, WAVE_SINE = 1, WAVE_SAW = 2, WAVE_LAYERED = 3 };

// The MD10C is only rated for PWM/switching up to 20 kHz - hard-cap all
// generated output at that ceiling.
static const float MAX_OUTPUT_FREQ_HZ = 20000.0f;

enum Category : uint8_t {
  CAT_BONE_JOINT = 0,
  CAT_PAIN_RECOVERY,
  CAT_CHRONIC_SYSTEMIC,
  CAT_HEART_CIRC,
  CAT_MENTAL_COGNITIVE,
  CAT_SKIN_WOUND,
  CATEGORY_COUNT
};

static const char* CATEGORY_NAMES[CATEGORY_COUNT] = {
  "Body Comfort",
  "Athletic",
  "General Wellness",
  "Relaxation & Sleep",
  "Focus & Balance",
  "Skin Comfort"
};
// Display names are general-wellness language only - deliberately not
// organized around diagnosed medical conditions. The internal CAT_x
// identifiers below (CAT_BONE_JOINT etc.) are leftover names from an
// earlier version and no longer describe their category's actual theme -
// left as-is since they're never shown to the user and renaming them
// would mean touching every reference throughout the .ino for no
// user-facing benefit. Mapping: CAT_BONE_JOINT->Body Comfort,
// CAT_PAIN_RECOVERY->Athletic, CAT_CHRONIC_SYSTEMIC->General
// Wellness, CAT_HEART_CIRC->Relaxation & Sleep, CAT_MENTAL_COGNITIVE->
// Focus & Balance, CAT_SKIN_WOUND->Skin Comfort.

struct Preset {
  const char* name;
  float freqHz;
  WaveShape wave;
  Category category;
};

static const Preset BASE_PRESETS[] = {
  // --- Comfort & Recovery: general body/joint/muscle comfort ---
  {"General Body Comfort",     10.0f, WAVE_SQUARE, CAT_BONE_JOINT},
  {"Joint Comfort",            15.0f, WAVE_SQUARE, CAT_BONE_JOINT},
  {"Shoulder & Neck Comfort",   7.0f, WAVE_SQUARE, CAT_BONE_JOINT},
  {"Tension Ease",              8.0f, WAVE_SQUARE, CAT_BONE_JOINT},
  {"Head & Tension Ease",       4.0f, WAVE_SQUARE, CAT_BONE_JOINT},

  // --- Athletic & Movement: general fitness/recovery framing ---
  {"Muscle Comfort",           20.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Athletic Recovery",        22.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Athletic Performance",     28.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Post-Activity Ease",       12.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},

  // --- General Wellness: everyday vitality/energy framing ---
  {"General Vitality",         24.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Energy Balance",           19.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Everyday Wellness",         9.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Grounding (Schumann)",      7.83f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},

  // --- Relaxation & Sleep ---
  {"Deep Relaxation",           2.0f, WAVE_SQUARE, CAT_HEART_CIRC},
  {"Meditation",                6.0f, WAVE_SQUARE, CAT_HEART_CIRC},
  {"Better Rest",               1.5f, WAVE_SQUARE, CAT_HEART_CIRC},
  {"Stress Ease",               3.0f, WAVE_SQUARE, CAT_HEART_CIRC},

  // --- Focus & Balance ---
  {"Mental Clarity",           18.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Focus",                    14.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Emotional Balance",        33.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Inner Balance",            11.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},

  // --- Skin Comfort ---
  {"Skin Comfort",              5.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"General Skin Wellness",    17.0f, WAVE_SQUARE, CAT_SKIN_WOUND},

  // --- Cultural/wellness frequency association (see note below) ---
  {"Calm Focus",               13.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE}, // was "963 Hz" - now a Solfeggio SOUND (see Sounds); slot kept so saved favorites stay in place

  // --- Added 1.9.2 (always APPEND here: favorites are saved by position) ---
  // Frequencies common across established consumer devices (research
  // review, Sept 2026). Exploration settings only - no benefit claimed.
  {"Classic Rhythm",            9.6f, WAVE_SQUARE,  CAT_CHRONIC_SYSTEMIC}, // long-standing main frequency on one consumer device
  {"Steady Flow",              16.0f, WAVE_SQUARE,  CAT_CHRONIC_SYSTEMIC}, // used in several published low-intensity field studies
  {"Bright Tone",              25.0f, WAVE_SQUARE,  CAT_CHRONIC_SYSTEMIC}, // mid-range setting common across 1-30 Hz mats
  {"Sawtooth Wave",            10.0f, WAVE_SAW,     CAT_CHRONIC_SYSTEMIC}, // alternative pulse shape offered by several devices
  {"Deep Calm",                 1.0f, WAVE_SQUARE,  CAT_HEART_CIRC},       // used in overnight programs on some devices
  {"Stillness",                 0.5f, WAVE_SQUARE,  CAT_HEART_CIRC},       // lowest setting on several mats
  {"Layered Calm",              2.0f, WAVE_LAYERED, CAT_HEART_CIRC},       // slow pulse carrying a 500 Hz "background" layer
};
static const int NUM_BASE_PRESETS = sizeof(BASE_PRESETS) / sizeof(BASE_PRESETS[0]);
// This list was rewritten to use general-wellness language only, with
// every entry that named a diagnosed medical condition removed entirely
// (no renaming/softening - fully removed). The original condition-
// specific frequency chart still exists as reference material outside
// this firmware (the product's instruction manual / online reference),
// clearly sourced, but is deliberately not part of this device's own
// operating menu.
//
// "Grounding (Schumann)" at 7.83Hz is a real, measured resonance of
// Earth's own electromagnetic field (established physics) - the
// "grounding" wellness association layered on top of that real number is
// a popular one, not a clinical claim.
//
// 963 Hz ("God frequency") and the other Solfeggio numbers are AUDIO tones
// from a sound/meditation tradition, not PEMF pulse rates - since 2.0.1
// they live in the Sounds list (played as pure tones under any session),
// each shown with its traditional association and "no health effect is
// claimed". No pineal gland or decalcification
// claim is made anywhere in this codebase: real published research on
// frequency and the pineal gland (50-60Hz ELF-EMF exposure) points
// toward melatonin *disruption*, not a benefit, so that framing was
// deliberately not used here.


// ---------------------------------------------------------------------
// Programs - multi-step frequency sequences, distinct from the single-
// frequency presets above. Two kinds of step:
//   STEP_HOLD - stay at freqHz for durationSec
//   STEP_RAMP - smoothly glide from the previous step's frequency to
//               freqHz over durationSec (a rise or fall, depending on
//               direction)
// ---------------------------------------------------------------------
enum StepKind : uint8_t { STEP_HOLD = 0, STEP_RAMP = 1 };

struct ProgramStep {
  float freqHz;
  WaveShape wave;
  StepKind kind;
  int durationSec;
};

static const int MAX_PROGRAM_STEPS = 8;

struct Program {
  const char* name;
  const char* note; // brief context/source, shown to the user
  ProgramStep steps[MAX_PROGRAM_STEPS];
  int stepCount;
};

// ---------------------------------------------------------------------
// Sequences - named, multi-stage Programs the person can browse and pick,
// distinct from the auto-generated-harmonics idea this replaced (that
// concept wasn't grounded in anything real, per review - see chat
// history). Each sequence below is one of two honestly-different kinds:
//
//   1. Grounded in a real published source (cited in its note field)
//   2. Built from this device's own already-reviewed general-wellness
//      frequency list, sequenced the same way a person would manually
//      step through frequencies on a bench function generator - NOT an
//      independently published study. Labeled honestly as such.
//
// "Sleep Descent" and "Wake Ascent" follow the real alpha/theta/delta EEG
// band structure described in a published (feasibility-stage) sleep-
// entrainment study protocol - the bands themselves are established
// neuroscience; causal entrainment efficacy is still genuinely mixed in
// the literature, so these are labeled as "structured the same way a
// real study protocol was designed," not "proven effective."
// ---------------------------------------------------------------------
static const int NUM_SEQUENCES = 24; // 1.9.x: "High Frequency Sweep" (Rife-tradition numbers) removed
static const Program SEQUENCES[NUM_SEQUENCES] = {
  // --- Added 1.9.2: glides, cycles, sweeps and a harmonic ladder, modeled on
  // programs offered by established consumer devices. Exploration settings. ---
  { "Evening Wind-Down",
    "Glides from 9.6 Hz down to 1 Hz over 45 minutes, like the stepped-down evening programs on some established mats. An exploration setting.",
    { {9.6f, WAVE_SQUARE, STEP_HOLD, 120}, {1.0f, WAVE_SQUARE, STEP_RAMP, 2580} }, 2 },

  { "Gentle Cycle",
    "Drifts slowly between 3 Hz and 1 Hz, a pattern found in evening programs on some devices. An exploration setting.",
    { {3.0f, WAVE_SQUARE, STEP_HOLD, 180}, {1.0f, WAVE_SQUARE, STEP_RAMP, 180}, {3.0f, WAVE_SQUARE, STEP_RAMP, 180},
      {1.0f, WAVE_SQUARE, STEP_RAMP, 180}, {3.0f, WAVE_SQUARE, STEP_RAMP, 180}, {1.0f, WAVE_SQUARE, STEP_RAMP, 180} }, 6 },

  { "Harmonic Ladder",
    "Climbs the harmonic series in octaves - 1.25, 2.5, 5, 10, 20 Hz - then settles back down. Each step doubles the one before. An exploration setting.",
    { {1.25f, WAVE_SQUARE, STEP_HOLD, 180}, {2.5f, WAVE_SQUARE, STEP_HOLD, 180}, {5.0f, WAVE_SQUARE, STEP_HOLD, 180},
      {10.0f, WAVE_SQUARE, STEP_HOLD, 180}, {20.0f, WAVE_SQUARE, STEP_HOLD, 180}, {10.0f, WAVE_SQUARE, STEP_HOLD, 120},
      {5.0f, WAVE_SQUARE, STEP_HOLD, 120}, {2.5f, WAVE_SQUARE, STEP_HOLD, 120} }, 8 },

  { "8-11 Hz Band Glide",
    "Glides back and forth through 8-11 Hz, a range common across consumer mats. An exploration setting.",
    { {8.0f, WAVE_SQUARE, STEP_HOLD, 60}, {11.0f, WAVE_SQUARE, STEP_RAMP, 240}, {8.0f, WAVE_SQUARE, STEP_RAMP, 240},
      {11.0f, WAVE_SQUARE, STEP_RAMP, 240}, {8.0f, WAVE_SQUARE, STEP_RAMP, 240} }, 5 },

  { "28-31 Hz Band Glide",
    "Glides back and forth through 28-31 Hz, another range common across consumer mats. An exploration setting.",
    { {28.0f, WAVE_SQUARE, STEP_HOLD, 60}, {31.0f, WAVE_SQUARE, STEP_RAMP, 240}, {28.0f, WAVE_SQUARE, STEP_RAMP, 240},
      {31.0f, WAVE_SQUARE, STEP_RAMP, 240}, {28.0f, WAVE_SQUARE, STEP_RAMP, 240} }, 5 },

  { "Full Sweep 1-30 Hz",
    "A slow sweep up and back across the range most consumer mats use. An exploration setting.",
    { {1.0f, WAVE_SQUARE, STEP_HOLD, 60}, {30.0f, WAVE_SQUARE, STEP_RAMP, 900}, {1.0f, WAVE_SQUARE, STEP_RAMP, 900} }, 3 },

  { "Varied Steps 1-30 Hz",
    "Moves between different frequencies across 1-30 Hz every two minutes, like the varied programs on some mats. A fixed pattern, not random. An exploration setting.",
    { {7.0f, WAVE_SQUARE, STEP_HOLD, 120}, {22.0f, WAVE_SQUARE, STEP_HOLD, 120}, {3.0f, WAVE_SQUARE, STEP_HOLD, 120},
      {15.0f, WAVE_SQUARE, STEP_HOLD, 120}, {28.0f, WAVE_SQUARE, STEP_HOLD, 120}, {5.0f, WAVE_SQUARE, STEP_HOLD, 120},
      {12.0f, WAVE_SQUARE, STEP_HOLD, 120}, {1.0f, WAVE_SQUARE, STEP_HOLD, 120} }, 8 },

  { "Sleep Descent",
    "Steps down through the alpha, theta and delta ranges (8-12, 4-8, 0.5-4 Hz) the brain moves through when falling asleep. The ranges are established science; that a pulsed field can guide the brain through them has not been shown.",
    { {10.0f, WAVE_SQUARE, STEP_HOLD, 300}, {6.0f, WAVE_SQUARE, STEP_RAMP, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },

  { "Wake Ascent",
    "The reverse of Sleep Descent: delta, theta, alpha, then beta (13-30 Hz). Same established ranges; no effect is claimed.",
    { {2.0f, WAVE_SQUARE, STEP_HOLD, 120}, {6.0f, WAVE_SQUARE, STEP_RAMP, 300}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300}, {20.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 4 },

  { "Calm & Focus",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {3.0f, WAVE_SQUARE, STEP_HOLD, 300}, {10.0f, WAVE_SQUARE, STEP_RAMP, 600}, {14.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },

  { "Energy Reset",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {18.0f, WAVE_SQUARE, STEP_HOLD, 300}, {24.0f, WAVE_SQUARE, STEP_RAMP, 600}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Post-Activity Recovery",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {12.0f, WAVE_SQUARE, STEP_HOLD, 300}, {22.0f, WAVE_SQUARE, STEP_RAMP, 600}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Grounding & Balance",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {7.83f, WAVE_SQUARE, STEP_HOLD, 600}, {33.0f, WAVE_SQUARE, STEP_RAMP, 300}, {7.83f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Recovery Reset",
    "Built from this device's own general-wellness frequency list, not an independently published study. General recovery/comfort framing only.",
    { {24.0f, WAVE_SQUARE, STEP_HOLD, 300}, {18.0f, WAVE_SQUARE, STEP_RAMP, 600}, {10.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },

  { "Uplift",
    "Built from this device's own general-wellness frequency list, not an independently published study. General mood/energy wellness framing only.",
    { {33.0f, WAVE_SQUARE, STEP_HOLD, 600}, {24.0f, WAVE_SQUARE, STEP_RAMP, 600}, {18.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Meeting Ready",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {10.0f, WAVE_SQUARE, STEP_HOLD, 180}, {14.0f, WAVE_SQUARE, STEP_RAMP, 420}, {18.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Pre-Workout Prime",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {15.0f, WAVE_SQUARE, STEP_HOLD, 180}, {25.0f, WAVE_SQUARE, STEP_RAMP, 420}, {28.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Travel Ease",
    "Built from this device's own general-wellness frequency list, not an independently published study. General comfort framing only.",
    { {24.0f, WAVE_SQUARE, STEP_HOLD, 300}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Screen Break",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {8.0f, WAVE_SQUARE, STEP_HOLD, 300}, {12.0f, WAVE_SQUARE, STEP_RAMP, 300}, {14.0f, WAVE_SQUARE, STEP_HOLD, 300} }, 3 },

  { "Evening Unwind",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {3.0f, WAVE_SQUARE, STEP_HOLD, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 2 },

  { "Morning Meditation",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {7.83f, WAVE_SQUARE, STEP_HOLD, 600}, {6.0f, WAVE_SQUARE, STEP_RAMP, 600}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Deep Focus Session",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {10.0f, WAVE_SQUARE, STEP_HOLD, 300}, {14.0f, WAVE_SQUARE, STEP_RAMP, 600}, {14.0f, WAVE_SQUARE, STEP_HOLD, 900} }, 3 },

  { "Center & Ground",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {7.83f, WAVE_SQUARE, STEP_HOLD, 900}, {33.0f, WAVE_SQUARE, STEP_RAMP, 300}, {7.83f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Weekend Reset",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {10.0f, WAVE_SQUARE, STEP_HOLD, 600}, {15.0f, WAVE_SQUARE, STEP_RAMP, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },
};


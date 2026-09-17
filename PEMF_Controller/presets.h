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

enum WaveShape : uint8_t { WAVE_SQUARE = 0, WAVE_SINE = 1 };

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
  {"Muscle Comfort",           50.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Athletic Recovery",       100.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Athletic Performance",     50.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Post-Activity Ease",       12.0f, WAVE_SQUARE, CAT_PAIN_RECOVERY},

  // --- General Wellness: everyday vitality/energy framing ---
  {"General Vitality",         24.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Energy Balance",           18.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Everyday Wellness",        10.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Grounding (Schumann)",      7.83f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},

  // --- Relaxation & Sleep ---
  {"Deep Relaxation",           2.0f, WAVE_SQUARE, CAT_HEART_CIRC},
  {"Meditation",                2.0f, WAVE_SQUARE, CAT_HEART_CIRC},
  {"Better Rest",               2.0f, WAVE_SQUARE, CAT_HEART_CIRC},
  {"Stress Ease",               3.0f, WAVE_SQUARE, CAT_HEART_CIRC},

  // --- Focus & Balance ---
  {"Mental Clarity",          100.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Focus",                   100.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Emotional Balance",        33.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Chakra Alignment",         33.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},

  // --- Skin Comfort ---
  {"Skin Comfort",             10.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"General Skin Wellness",    15.0f, WAVE_SQUARE, CAT_SKIN_WOUND},

  // --- Cultural/wellness frequency association (see note below) ---
  {"963 Hz - God Frequency",   963.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
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
// "963 Hz - God Frequency" is a popular Solfeggio-tone/meditation cultural
// association (numerology-based, not a clinical claim) - same honest
// treatment as the Schumann entry above: real number, cultural
// association labeled as association. No pineal gland or decalcification
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

// Studied multi-frequency package: 220/727/880/10000 Hz, from a real
// (rat) study comparing this combination against a single 4Hz signal.
// The study didn't specify whether these were delivered as discrete
// steps or swept - built here as a gentle rise through each, then a ramp
// back down to the start, rather than abrupt jumps. Named descriptively
// rather than "Bone Healing" - the research citation belongs in the note
// field, not as a product name implying a treatment outcome. The C++
// symbol name below (BONE_HEALING_PROGRAM) is unchanged since it's
// referenced elsewhere in the firmware and isn't user-facing.
static const Program BONE_HEALING_PROGRAM = {
  "Multi-Frequency Sequence",
  "A studied multi-frequency package (220/727/880/10,000 Hz) from bone-fracture-healing research",
  {
    {220.0f,   WAVE_SQUARE, STEP_HOLD, 60},
    {727.0f,   WAVE_SQUARE, STEP_RAMP, 60},
    {880.0f,   WAVE_SQUARE, STEP_RAMP, 60},
    {10000.0f, WAVE_SQUARE, STEP_RAMP, 60},
    {220.0f,   WAVE_SQUARE, STEP_RAMP, 60}, // ramp back down to close the session gently
  },
  5
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
static const int NUM_SEQUENCES = 18;
static const Program SEQUENCES[NUM_SEQUENCES] = {
  BONE_HEALING_PROGRAM, // real cited rat study - see its own definition above

  { "Sleep Descent",
    "Structured like a published sleep-entrainment study design: follows the real alpha->theta->delta brainwave-band progression the body moves through when falling asleep. Band science is established; entrainment efficacy is still debated in the literature.",
    { {10.0f, WAVE_SQUARE, STEP_HOLD, 300}, {6.0f, WAVE_SQUARE, STEP_RAMP, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },

  { "Wake Ascent",
    "The reverse of Sleep Descent - delta->theta->alpha->beta, following the same real EEG band structure.",
    { {2.0f, WAVE_SQUARE, STEP_HOLD, 120}, {6.0f, WAVE_SQUARE, STEP_RAMP, 300}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300}, {20.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 4 },

  { "Calm & Focus",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {3.0f, WAVE_SQUARE, STEP_HOLD, 300}, {100.0f, WAVE_SQUARE, STEP_RAMP, 600}, {100.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },

  { "Energy Reset",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {18.0f, WAVE_SQUARE, STEP_HOLD, 300}, {24.0f, WAVE_SQUARE, STEP_RAMP, 600}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Post-Activity Recovery",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {12.0f, WAVE_SQUARE, STEP_HOLD, 300}, {50.0f, WAVE_SQUARE, STEP_RAMP, 600}, {10.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

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
    { {18.0f, WAVE_SQUARE, STEP_HOLD, 180}, {100.0f, WAVE_SQUARE, STEP_RAMP, 420}, {100.0f, WAVE_SQUARE, STEP_HOLD, 300} }, 3 },

  { "Pre-Workout Prime",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {18.0f, WAVE_SQUARE, STEP_HOLD, 180}, {50.0f, WAVE_SQUARE, STEP_RAMP, 420}, {50.0f, WAVE_SQUARE, STEP_HOLD, 300} }, 3 },

  { "Travel Ease",
    "Built from this device's own general-wellness frequency list, not an independently published study. General comfort framing only.",
    { {24.0f, WAVE_SQUARE, STEP_HOLD, 300}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Screen Break",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {8.0f, WAVE_SQUARE, STEP_HOLD, 300}, {100.0f, WAVE_SQUARE, STEP_RAMP, 300}, {100.0f, WAVE_SQUARE, STEP_HOLD, 300} }, 3 },

  { "Evening Unwind",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {3.0f, WAVE_SQUARE, STEP_HOLD, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 2 },

  { "Morning Meditation",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {7.83f, WAVE_SQUARE, STEP_HOLD, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600}, {33.0f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Deep Focus Session",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {8.0f, WAVE_SQUARE, STEP_HOLD, 300}, {100.0f, WAVE_SQUARE, STEP_RAMP, 600}, {100.0f, WAVE_SQUARE, STEP_HOLD, 900} }, 3 },

  { "Center & Ground",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {7.83f, WAVE_SQUARE, STEP_HOLD, 900}, {33.0f, WAVE_SQUARE, STEP_RAMP, 300}, {7.83f, WAVE_SQUARE, STEP_RAMP, 300} }, 3 },

  { "Weekend Reset",
    "Built from this device's own general-wellness frequency list, not an independently published study.",
    { {10.0f, WAVE_SQUARE, STEP_HOLD, 600}, {15.0f, WAVE_SQUARE, STEP_RAMP, 600}, {2.0f, WAVE_SQUARE, STEP_RAMP, 600} }, 3 },
};


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
  "Bone & Joint",
  "Pain & Recovery",
  "Chronic & Systemic",
  "Heart & Circulatory",
  "Mental & Cognitive",
  "Skin & Wound"
};

struct Preset {
  const char* name;
  float freqHz;
  WaveShape wave;
  Category category;
};

static const Preset BASE_PRESETS[] = {
  // --- Bone & Joint Health ---
  {"Osteoporosis",              9.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Bursitis",                  8.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Periostitis",               6.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Tendinitis",                8.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Fractures",                10.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Psoriatic arthritis",      18.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Pseudoarthrosis",          10.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Knee osteoarthritis",      15.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Bone regeneration",        16.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Rheumatoid arthritis",     20.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Frozen shoulder",           7.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Ligament injuries",         7.0f,  WAVE_SQUARE, CAT_BONE_JOINT},
  {"Nonunion fractures",       20.0f,  WAVE_SQUARE, CAT_BONE_JOINT},

  // --- Pain & Injury Recovery ---
  {"Knee pain",                10.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Hip pain",                 10.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Back pain",                10.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Plantar fasciitis",         8.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Tennis/golf elbow",         8.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Cervical vertebra pain",   18.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Shin splints",             10.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Headache",                  4.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Chronic pelvic pain",       5.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Carpal tunnel syndrome",    6.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Neuropathy",                6.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Dislocations/sprains",     10.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Strains",                  12.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Herniated disc",           17.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Sciatica",                 17.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Carpal tunnel",            20.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Nerve regeneration",       50.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Muscle healing",           50.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Athletic recovery",       100.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Athletic performance",     50.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},
  {"Neuralgia trigemini",      18.0f,  WAVE_SQUARE, CAT_PAIN_RECOVERY},

  // --- Chronic & Systemic Conditions ---
  {"Chronic fatigue syndrome",  5.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Rheumatic fever",           6.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Chronic sinusitis",         5.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Hyperthyroidism",          10.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Acute bronchitis",          4.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Allergies",                 5.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Constipation",               5.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Cystitis",                  5.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Erectile dysfunction",      6.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Asthma",                    7.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Stomach aches",            10.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Stomach/duodenal ulcers",  10.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Chronic bronchitis",       12.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Diabetes mellitus",        12.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Crohn's disease",          13.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Fibromyalgia",             15.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Diabetes",                 22.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Prostatitis",                6.0f, WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Hepatitis",                18.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Immune system support",    24.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},
  {"Metabolic rate",           18.0f,  WAVE_SQUARE, CAT_CHRONIC_SYSTEMIC},

  // --- Heart & Circulatory Health ---
  {"Varicose veins",           12.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Peripheral artery disease",12.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Venous insufficiency",     12.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Angina pectoris",           4.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Arrhythmia",                7.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Angina",                    7.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Arteriosclerosis",          7.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Circulatory dysfunction",   7.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Raynaud's syndrome",       15.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Hypertension",             20.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Low blood pressure",       20.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Poor circulation",          6.0f,  WAVE_SQUARE, CAT_HEART_CIRC},
  {"Lymphatic disorders",      18.0f,  WAVE_SQUARE, CAT_HEART_CIRC},

  // --- Mental & Cognitive Health ---
  {"Insomnia",                   5.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Memory impairment",          6.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"ADD/ADHD",                   6.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Mood disorders",            10.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Deep relaxation",            2.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Meditation",                 2.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Better rest",                2.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Stress relief",              3.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Anxiety",                    4.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Depression",                 4.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Alzheimer's disease",        4.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Dizziness",                 10.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Multiple sclerosis",        13.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Hyperactivity",             17.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Parkinson's disease",       17.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Stroke",                    20.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Cerebral palsy balance",    20.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Emotional balance",         33.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Chakra alignment",          33.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Cognitive enhancement",    100.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Focus improvement",        100.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Psychosomatic syndrome",    22.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Dystonia neurovegetativa",  20.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Weather sensitivity",       13.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},
  {"Glaucoma",                  15.0f, WAVE_SQUARE, CAT_MENTAL_COGNITIVE},

  // --- Skin & Wound Care ---
  {"Psoriasis",                 10.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Burns",                     15.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Chronic blepharitis",        1.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Acne",                      10.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Tinnitus",                  10.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Bruises",                   14.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Dental/oral support",       30.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Wound healing",             75.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Eczema",                    15.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Neurodermatitis",           15.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
  {"Skin allergies",            15.0f, WAVE_SQUARE, CAT_SKIN_WOUND},
};
static const int NUM_BASE_PRESETS = sizeof(BASE_PRESETS) / sizeof(BASE_PRESETS[0]);

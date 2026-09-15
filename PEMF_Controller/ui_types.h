#pragma once
// ============================================================================
// ui_types.h - shared UI geometry/widget types
// ============================================================================
// These live in their own header (rather than directly in the .ino) because
// the Arduino build system auto-generates forward declarations for every
// function in the sketch and inserts them at the very top of the file -
// before any types defined later in the .ino body exist yet. Keeping these
// types in a header that's #included early avoids that ordering bug.
// ============================================================================

struct Rect { int x, y, w, h; };

// Stepper: a "- value +" control, no track/bar. Replaces the old Slider
// widget entirely (no sliders anywhere in this UI).
struct Stepper {
  const char* label;
  int x, y;              // position of the "-" button; layout is - [value] +
  int value, minVal, maxVal, step;
  const char* (*formatFn)(int); // optional custom value formatter, may be null
};

// Screen and PinPurpose live here too, for the same reason: they're used as
// function-parameter types (e.g. startTextEntry(..., Screen returnTo, ...)),
// and Arduino's auto-generated prototypes are hoisted to right after the
// last #include - so these have to be defined before that point no matter
// where in the .ino body they'd otherwise naturally sit.
enum Screen { SCR_WELCOME, SCR_SETUP_MODE, SCR_SETUP_LOGIN_CHOICE, SCR_TEXT_ENTRY, SCR_CATEGORY, SCR_LIST, SCR_RUN,
              SCR_SETTINGS, SCR_PIN, SCR_LOG, SCR_UPDATE, SCR_CLIENT_CONFIRM };

enum PinPurpose { PIN_DEV_MODE, PIN_SET_LOGIN, PIN_CHECK_LOGIN };

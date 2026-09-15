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

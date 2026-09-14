#pragma once
// ============================================================================
// ui_types.h - shared UI geometry/widget types
// ============================================================================
// These live in their own header (rather than directly in the .ino) because
// the Arduino build system auto-generates forward declarations for every
// function in the sketch and inserts them at the very top of the file -
// before any types defined later in the .ino body exist yet. Keeping Rect
// and Slider in a header that's #included early avoids that ordering bug.
// ============================================================================

struct Rect { int x, y, w, h; };

struct Slider {
  const char* label;
  int x, y, w;          // bar geometry; buttons sit either side of it
  int value, minVal, maxVal, step;
  const char* (*formatFn)(int); // optional custom value formatter, may be null
};

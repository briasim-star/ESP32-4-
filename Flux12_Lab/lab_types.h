#pragma once
// UI helper types (in a header so the Arduino auto-prototypes see them first)
struct Rect { int x, y, w, h; };
struct Stepper { const char* label; int x, y; int value, minVal, maxVal, step; };
enum Mode { SQUARE, PULSE, STEADY, SWEEP, SPIN, HOVER, BOUNCE };

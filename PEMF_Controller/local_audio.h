#pragma once
// Intentionally empty. The onboard-speaker code now lives in the unified
// audio engine (bt_audio.h / bt_audio.cpp), which drives both the onboard
// speaker and Bluetooth from one background task. This file is kept only
// so older checkouts that still #include it continue to compile.

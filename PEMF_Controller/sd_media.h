#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// SD card handling - splash screen image and soundscape audio files.
// Everything here degrades gracefully: if no card is present (or it's
// present but empty/unreadable), sdmedia_begin() simply returns false
// and every other function here becomes a safe no-op. The rest of the
// firmware checks sdmedia_isAvailable() (and, for soundscapes, whether
// any files were actually found) before ever showing a menu entry for
// either feature - so there's nothing to "fail" at runtime if a card
// isn't inserted, the options just don't appear.
// ---------------------------------------------------------------------

bool sdmedia_begin();          // attempt to mount the card once, at boot
bool sdmedia_isAvailable();

// Splash screen - draws /splash.bmp full-screen if the card is mounted
// and that file exists. Returns false (and draws nothing) otherwise -
// the caller falls through to the normal Welcome screen either way.
bool sdmedia_showSplash();

// Soundscapes - scans /sounds/ for .wav files at boot.
static const int MAX_SOUNDSCAPES = 12;
int sdmedia_scanSoundscapes();      // returns how many were found
int sdmedia_soundscapeCount();
const char* sdmedia_soundscapeName(int idx); // display name
const char* sdmedia_soundscapePath(int idx); // full path, for playback

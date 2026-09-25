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

// The audio task streams soundscapes from the card while the UI may be
// drawing the splash image from it - every SD access takes this lock
// (recursive, so nested use is fine).
void sdmedia_lock();
void sdmedia_unlock();

// Splash screen - draws /splash.bmp full-screen if the card is mounted
// and that file exists. Returns false (and draws nothing) otherwise -
// the caller falls through to the normal Welcome screen either way.
bool sdmedia_showSplash();

// Soundscapes - scans /sounds/ for .wav files at boot.
// Only the name is stored (32 bytes each); the path is rebuilt from it on
// demand, so 30 entries use less RAM than the old 12 with stored paths.
static const int MAX_SOUNDSCAPES = 30;
int sdmedia_scanSoundscapes();      // returns how many were found
int sdmedia_soundscapeCount();
const char* sdmedia_soundscapeName(int idx); // display name
const char* sdmedia_soundscapePath(int idx); // full path, for playback

// Diagnostics for the last scan - shown on the Welcome screen to see
// exactly where the process succeeds or fails: whether /sounds itself
// opened as a directory at all, how many entries it contained in
// total, and how many of those were files (vs. subdirectories) -
// separate from how many actually matched the .wav filter.
bool sdmedia_lastScanDirOpened();
int sdmedia_lastScanTotalEntries();
int sdmedia_lastScanFileEntries();

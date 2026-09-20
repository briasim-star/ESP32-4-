#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// Checks whether THIS specific device currently has an active premium
// subscription, using its own unique built-in chip ID as its identity -
// no login or account setup needed on the device itself.
//
// This governs access to premium content ONLY (extra Sequences/
// Soundscapes added after the free set) - core device function, safety
// features, and bug fixes are never gated by this and never will be.
//
// ENTITLEMENT_CHECK_URL below is a placeholder - point it at your real
// backend once one exists (see backend/app.py for a minimal starting
// point). Until then, this safely does nothing: no backend configured
// means every check simply reports "not entitled", which is the
// correct, safe default - a missing or unreachable backend should never
// accidentally unlock (or lock out) anything.
// ---------------------------------------------------------------------

bool subscription_checkEntitlement(); // blocking - contacts the backend; returns the result, and caches it
bool subscription_isEntitled();       // cached result of the last check - safe to call often, does no network activity
String subscription_getDeviceId();    // this device's unique identifier, derived from its built-in chip ID

#include "subscription_check.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "esp32_cert_bundle.h"

// Placeholder - replace with your real backend's URL once one is
// deployed (see backend/app.py). Left pointing nowhere real on
// purpose: an unconfigured or unreachable backend must always resolve
// to "not entitled", never to an accidental unlock.
static const char* ENTITLEMENT_CHECK_URL = "https://your-backend-domain.example.com/check-entitlement";
static const unsigned long ENTITLEMENT_WIFI_TIMEOUT_MS = 8000;

static bool entitled = false;

String subscription_getDeviceId() {
  // ESP.getEfuseMac() reads the chip's own factory-burned unique ID -
  // fixed for the physical device's lifetime, needs no setup, and
  // doubles as a natural "device account" the backend can key off of.
  uint64_t chipId = ESP.getEfuseMac();
  char buf[17];
  snprintf(buf, sizeof(buf), "%04X%08X", (uint16_t)(chipId >> 32), (uint32_t)chipId);
  return String(buf);
}

bool subscription_checkEntitlement() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // reconnects using the ESP32's own saved credentials
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < ENTITLEMENT_WIFI_TIMEOUT_MS) {
      delay(100);
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    return entitled; // offline - keep whatever the last known result was, don't guess
  }

  WiFiClientSecure client;
  client.setCACertBundle(x509_crt_bundle, x509_crt_bundle_len);

  HTTPClient http;
  String url = String(ENTITLEMENT_CHECK_URL) + "?device_id=" + subscription_getDeviceId();
  if (!http.begin(client, url)) {
    return entitled;
  }
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String body = http.getString();
    body.trim();
    entitled = (body == "true");
  }
  // Any other response (including "can't reach the server") leaves
  // `entitled` at its previous value rather than assuming false - a
  // brief network hiccup shouldn't suddenly lock someone out of
  // content they're actually paying for.
  http.end();
  return entitled;
}

bool subscription_isEntitled() { return entitled; }

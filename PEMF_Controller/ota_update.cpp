#include "ota_update.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include "esp32_cert_bundle.h" // provides x509_crt_bundle / x509_crt_bundle_len - see build_and_publish.yml for how this library gets installed

static const char* VERSION_CHECK_URL = "https://briasim-star.github.io/ESP32-4-/firmware/version.txt";
static const char* FIRMWARE_BIN_URL  = "https://briasim-star.github.io/ESP32-4-/firmware/firmware.bin";
static const unsigned long OTA_WIFI_TIMEOUT_MS = 15000;

static char latestVersion[16] = "";
static char lastError[64] = "";

extern const char* FIRMWARE_VERSION; // defined in the main .ino

static bool ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(); // reconnects using the ESP32's own saved credentials, same as wifitime_begin()
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < OTA_WIFI_TIMEOUT_MS) {
    delay(100);
  }
  return WiFi.status() == WL_CONNECTED;
}

const char* ota_latestVersionString() { return latestVersion; }
const char* ota_lastErrorMessage() { return lastError; }

bool ota_checkForUpdate() {
  lastError[0] = 0;
  if (!ensureWifiConnected()) {
    strncpy(lastError, "Could not connect to WiFi", sizeof(lastError) - 1);
    return false;
  }

  WiFiClientSecure client;
  client.setCACertBundle(x509_crt_bundle, x509_crt_bundle_len);

  HTTPClient http;
  if (!http.begin(client, VERSION_CHECK_URL)) {
    strncpy(lastError, "Could not reach update server", sizeof(lastError) - 1);
    return false;
  }
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    snprintf(lastError, sizeof(lastError), "Server returned error %d", httpCode);
    http.end();
    return false;
  }
  String body = http.getString();
  body.trim();
  http.end();

  strncpy(latestVersion, body.c_str(), sizeof(latestVersion) - 1);
  latestVersion[sizeof(latestVersion) - 1] = 0;

  return strcmp(latestVersion, FIRMWARE_VERSION) != 0;
}

bool ota_downloadAndInstall() {
  lastError[0] = 0;
  if (!ensureWifiConnected()) {
    strncpy(lastError, "Could not connect to WiFi", sizeof(lastError) - 1);
    return false;
  }

  WiFiClientSecure client;
  client.setCACertBundle(x509_crt_bundle, x509_crt_bundle_len);

  HTTPClient http;
  if (!http.begin(client, FIRMWARE_BIN_URL)) {
    strncpy(lastError, "Could not reach update server", sizeof(lastError) - 1);
    return false;
  }
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    snprintf(lastError, sizeof(lastError), "Server returned error %d", httpCode);
    http.end();
    return false;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0) {
    strncpy(lastError, "Server did not report a file size", sizeof(lastError) - 1);
    http.end();
    return false;
  }

  if (!Update.begin(contentLength)) {
    strncpy(lastError, "Not enough free space for the update", sizeof(lastError) - 1);
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t written = Update.writeStream(*stream);
  http.end();

  if (written != (size_t)contentLength) {
    strncpy(lastError, "Download was incomplete", sizeof(lastError) - 1);
    Update.abort();
    return false;
  }

  if (!Update.end(true)) {
    snprintf(lastError, sizeof(lastError), "Install failed (code %d)", Update.getError());
    return false;
  }

  ESP.restart(); // new firmware takes over from here
  return true; // unreachable, but keeps the compiler happy
}

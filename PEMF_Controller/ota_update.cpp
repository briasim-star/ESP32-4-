#include "ota_update.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <esp_task_wdt.h>
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

static const unsigned long OTA_HTTP_TIMEOUT_MS = 15000; // request-level timeout
static const unsigned long OTA_STALL_TIMEOUT_MS = 15000; // used below during download - if no new bytes arrive for this long, abort rather than hang

static bool checkForUpdateImpl() {
  lastError[0] = 0;
  if (!ensureWifiConnected()) {
    strncpy(lastError, "Could not connect to WiFi", sizeof(lastError) - 1);
    return false;
  }

  WiFiClientSecure client;
  client.setCACertBundle(x509_crt_bundle); // this library's version takes just the bundle pointer, not a separate length - confirmed by the real compiler error, not guessed

  HTTPClient http;
  // Two explicit, bounded timeouts, on top of the manual stall-detection
  // in ota_downloadAndInstall() below - a real hang here once forced a
  // battery pull to recover from, so this isn't just a nicety.
  http.setConnectTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
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

static bool downloadAndInstallImpl() {
  lastError[0] = 0;
  if (!ensureWifiConnected()) {
    strncpy(lastError, "Could not connect to WiFi", sizeof(lastError) - 1);
    return false;
  }

  WiFiClientSecure client;
  client.setCACertBundle(x509_crt_bundle); // this library's version takes just the bundle pointer, not a separate length - confirmed by the real compiler error, not guessed

  HTTPClient http;
  http.setConnectTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
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

  // Manual chunked read instead of Update.writeStream() - that call
  // blocks until the stream ends, and a connection that stalls partway
  // through (rather than cleanly failing) could leave it waiting
  // indefinitely. This loop tracks how long it's been since the last
  // byte actually arrived and aborts if that ever exceeds
  // OTA_STALL_TIMEOUT_MS - the device is guaranteed to come back to a
  // usable state within a bounded time no matter what the network does,
  // rather than needing a battery pull to recover.
  WiFiClient* stream = http.getStreamPtr();
  size_t written = 0;
  uint8_t buf[1024];
  unsigned long lastDataMillis = millis();

  while (written < (size_t)contentLength) {
    size_t avail = stream->available();
    if (avail > 0) {
      size_t toRead = avail < sizeof(buf) ? avail : sizeof(buf);
      int readBytes = stream->readBytes(buf, toRead);
      if (readBytes > 0) {
        size_t wroteNow = Update.write(buf, readBytes);
        if (wroteNow != (size_t)readBytes) {
          strncpy(lastError, "Write to flash failed mid-download", sizeof(lastError) - 1);
          Update.abort();
          http.end();
          return false;
        }
        written += wroteNow;
        lastDataMillis = millis();
        esp_task_wdt_reset(); // feed the watchdog - real progress is being made, don't let it fire mid-download
      }
    } else {
      if (!client.connected()) {
        strncpy(lastError, "Connection lost during download", sizeof(lastError) - 1);
        Update.abort();
        http.end();
        return false;
      }
      if (millis() - lastDataMillis > OTA_STALL_TIMEOUT_MS) {
        strncpy(lastError, "Download stalled - no data received in time", sizeof(lastError) - 1);
        Update.abort();
        http.end();
        return false;
      }
      delay(10);
    }
  }
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

// WiFi and Bluetooth share one radio on the ESP32. Leaving WiFi on after an
// update check makes Bluetooth audio crackle, so it is always switched off.
static void wifiOff() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool ota_checkForUpdate() {
  bool r = checkForUpdateImpl();
  wifiOff();
  return r;
}

bool ota_downloadAndInstall() {
  bool r = downloadAndInstallImpl(); // restarts on success
  wifiOff();
  return r;
}

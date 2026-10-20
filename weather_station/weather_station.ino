/*
  ESP32-C3 Anemometer + BME280 + SD CSV logger (NTP + WiFiManager)
  + Web UI + REST API + CSV download + "last N days ZIP" download (streamed)

  Wind:
  - Pulse input: GPIO0 interrupt
  - Calibration: 20 pps = 1.75 m/s => m/s = pps * (1.75/20)

  Logging:
  - Bucket-aligned logs (configurable duration, NTP time)
  - /data/YYYYMMDD.csv (daily, retained for LogConfig::RETENTION_DAYS)

  Downloads:
  - List:     /api/files?dir=data
  - CSV:      /download?path=/data/20251214.csv
  - ZIP:      /download_zip?days=7  (streams ZIP of last N daily /data files, STORE mode)

  Notes:
  - GPIO0 is a boot strap pin on ESP32-C3; ensure sensor doesn't hold it LOW at boot.
  - SD pin mapping varies by board/module; set SDConfig::CS_PIN and optionally SPI.begin(...) pins.
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <time.h>

#include <Wire.h>

#include <SPI.h>
#include <LittleFS.h>
#ifdef SUPPORT_OTA
#include <ArduinoOTA.h>
#endif
#include <pgmspace.h>

#include <vector>
#include <algorithm>
#include "config.h"
#include "web_server.h"
#include "env_sensors.h"
#include "data_log.h"
#ifdef AIR_QUAL_SENSOR
  #include "air_quality.h"
#endif
#ifdef WIND_SENSOR
  #include "wind_sensor.h"
#endif

// ------------------- FORWARD DECLARATIONS -------------------

struct BucketSample;
struct DaySummary;

// UI and API password
static const char* API_PASSWORD = "ChangeMe";
//Over the air update password
static const char* OTA_PASSWORD = "PleaseChangeMe";

#ifdef RGB_BUILTIN

#include <Adafruit_NeoPixel.h>

#define RGB_PIN   10
#define NUMPIXELS  1   // Only 1 onboard NeoPixel

// Initialize the NeoPixel strip object with RGB color order
Adafruit_NeoPixel ws2812(NUMPIXELS, RGB_PIN, NEO_RGB | NEO_KHZ800);

void initRGB() {
  ws2812.begin();           // INITIALIZE NeoPixel strip object
  ws2812.setBrightness(30); // Set brightness (0 to 255)
  ws2812.clear();
  ws2812.show();
}

// Primary colors
#define RGB_BLACK   0x000000

#define RGB_RED     0x00FF00
#define RGB_GREEN   0xFF0000
#define RGB_BLUE    0x0000FF

#define RGB_WHITE   0xFFFFFF

// Secondary colors
#define RGB_YELLOW  0xFFFF00
#define RGB_MAGENTA 0x00FFFF
#define RGB_CYAN    0xFF00FF

void setRGB(uint32_t rgb)
{
  ws2812.setPixelColor(0, rgb);
  ws2812.show();
}

#endif // RGB_BUILTIN

void reportMessage(uint32_t rgb, const char* msg) {
  Serial.println(msg);
#ifdef RGB_BUILTIN
  setRGB(rgb);
#endif
}

void reportError(const char* errmsg) {
    reportMessage(RGB_RED, errmsg);
}

// Network & Time
namespace NetworkConfig {
  static const char* NTP_SERVER_1 = "pool.ntp.org";
  static const char* NTP_SERVER_2 = "time.google.com";
  static const char* NTP_SERVER_3 = "time.windows.com";
  static const char* TIMEZONE = "PST8PDT"; // Pacific Daylight Savings Time
  static const char* AP_NAME = "Weather-Setup";
  static constexpr uint32_t AP_TIMEOUT_S = 180;
}


// ------------------- HELPERS -------------------

static constexpr uint32_t FILES_CACHE_TTL_MS = 30000; // 30s reuse to avoid slow SD scans on refresh
static String gFilesCacheDataJson;
static uint32_t gFilesCacheDataMs = 0;

// Auth / rate limit for password-protected endpoints
static int      gPwAttempts = 0;
static time_t   gPwWindowStart = 0;

static void resetPwWindowIfNeeded() {
  time_t now = epochNow();
  if (!timeIsValid(gPwWindowStart) || (now - gPwWindowStart) >= 3600) {
    gPwWindowStart = now;
    gPwAttempts = 0;
  }
}

static bool verifyPassword(const String& pw, bool& rateLimited) {
  resetPwWindowIfNeeded();
  rateLimited = false;
  if (gPwAttempts >= 10) {
    rateLimited = true;
    return false;
  }
  if (pw == API_PASSWORD) {
    gPwAttempts = 0;
    gPwWindowStart = epochNow();
    return true;
  }
  gPwAttempts++;
  if (gPwAttempts >= 10) rateLimited = true;
  return false;
}


static bool deleteDirFiles(const char* dirPath) {
  File dir = LittleFS.open(dirPath);
  if (!dir) return false;
  bool ok = true;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    if (entry.isDirectory()) {
      entry.close();
      continue;
    }
    String name = entry.name();
    entry.close();
    if (!LittleFS.remove(name.c_str())) ok = false;
  }
  dir.close();
  return ok;
}

static void invalidateFilesCache() {
  gFilesCacheDataMs = 0;
  gFilesCacheDataJson = "";
}

// ------------------- BUCKETS / DAILY -------------------

void accumulateTodayFromBucket(const BucketSample& b) {
#ifdef WIND_SENSOR
  // Use full precision floats for daily summary
  updateWind(b.avgWind, b.maxWind);
#endif
  updateEnvMinMax(
#ifdef TEMP_SENSOR
    b.avgTempC
#else
    NAN
#endif
#ifdef HUMID_SENSOR
    , b.avgHumRH
#else
    , NAN
#endif
#ifdef PRESS_SENSOR
    , b.avgPressHpa
#else
    , NAN
#endif
    );
#ifdef AIR_QUAL_SENSOR
    updateAirQuality(b.avgPM1, b.avgPM25, b.avgPM10);
#endif
}

// ------------------- DOWNLOAD / FILE LIST -------------------

static bool isAllowedFilename(const String& filename) {
  // Validate filename - no path traversal, no slashes, must end with .csv
  if (filename.indexOf("..") >= 0) return false;
  if (filename.indexOf('/') >= 0) return false;
  if (filename.indexOf('\\') >= 0) return false;
  if (!filename.endsWith(".csv")) return false;
  return true;
}

void handleDownload() {
  if (!gSdOk) {
    server.send(503, "text/plain", "SD not available");
    return;
  }

  String filename = server.arg("filename");
  if (!isAllowedFilename(filename)) {
    server.send(400, "text/plain", "Invalid filename");
    return;
  }

  String path = "/data/" + filename;
  if (!LittleFS.exists(path.c_str())) {
    server.send(404, "text/plain", "Not found");
    return;
  }

  File f = LittleFS.open(path.c_str(), FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleApiFiles() {
  if (!gSdOk) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"sd_not_available\"}");
    return;
  }

  // Always use /data directory
  String dir = "data";
  String base = "/data";

  if (!LittleFS.exists(base.c_str())) {
    server.send(200, "application/json", String("{\"ok\":true,\"dir\":\"") + dir + "\",\"files\":[]}");
    return;
  }

  uint32_t nowMs = millis();
  if (gFilesCacheDataMs != 0 && (nowMs - gFilesCacheDataMs) < FILES_CACHE_TTL_MS && gFilesCacheDataJson.length() > 0) {
    server.send(200, "application/json", gFilesCacheDataJson);
    return;
  }

  File root = LittleFS.open(base.c_str());
  if (!root || !root.isDirectory()) {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"failed_to_open_dir\"}");
    return;
  }

  String out;
  out.reserve(4096);
  out += "{\"ok\":true,\"dir\":\"" + dir + "\",\"files\":[";
  bool first = true;

  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String name = f.name();
      // Skip internal cache files
      if (name.indexOf("day_summaries_cache") >= 0) {
        f.close();
        f = root.openNextFile();
        continue;
      }
      if (name.endsWith(".csv")) {
        if (!first) out += ",";
        first = false;
        out += "{";
        // Just the filename, directory is specified in "dir" field
        out += "\"path\":\"" + name + "\",";
        out += "\"size\":" + String((uint32_t)f.size());
        out += "}";
      }
    }
    f.close();
    f = root.openNextFile();
  }
  root.close();

  out += "]}";
  gFilesCacheDataJson = out;
  gFilesCacheDataMs = nowMs;
  server.send(200, "application/json", out);
}

// ------------------- ZIP (streamed, STORE method) -------------------

static uint32_t crc32_table[256];
static bool crc32_init_done = false;

static void crc32_init() {
  if (crc32_init_done) return;
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
    crc32_table[i] = c;
  }
  crc32_init_done = true;
}

static inline uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  uint32_t c = crc;
  for (size_t i = 0; i < len; i++) c = crc32_table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
  return c;
}

static void dosDateTime(time_t t, uint16_t &dosDate, uint16_t &dosTime) {
  struct tm tmLocal;
  localtime_r(&t, &tmLocal);
  int year = tmLocal.tm_year + 1900;
  if (year < 1980) year = 1980;
  dosDate = (uint16_t)(((year - 1980) << 9) | ((tmLocal.tm_mon + 1) << 5) | (tmLocal.tm_mday));
  dosTime = (uint16_t)((tmLocal.tm_hour << 11) | (tmLocal.tm_min << 5) | ((tmLocal.tm_sec / 2) & 0x1F));
}

struct ZipEntryInfo {
  String name;          // inside-zip name like "data/20251214.csv"
  String sdPath;        // "/data/20251214.csv"
  uint32_t size = 0;    // uncompressed size
  uint32_t crc = 0;     // computed while streaming
  uint32_t lho = 0;     // local header offset
  uint16_t dosDate = 0;
  uint16_t dosTime = 0;
};

static std::vector<ZipEntryInfo> buildLastNDaysList(int days) {
  std::vector<ZipEntryInfo> out;
  out.reserve(days);

  time_t nowE = epochNow();
  time_t todayMidnight = localMidnight(nowE);

  for (int i = 0; i < days; i++) {
    time_t dayMidnight = subtractDaysLocalMidnight(todayMidnight, i);
    String ymd = ymdString(dayMidnight);
    String sdPath = String("/data/") + ymd + ".csv";

    if (gSdOk && LittleFS.exists(sdPath.c_str())) {
      File f = LittleFS.open(sdPath.c_str(), FILE_READ);
      if (f) {
        ZipEntryInfo e;
        e.sdPath = sdPath;
        e.name = String("data/") + ymd + ".csv";
        e.size = (uint32_t)f.size();
        dosDateTime(dayMidnight, e.dosDate, e.dosTime);
        f.close();
        out.push_back(e);
      }
    }
  }

  // Oldest -> newest
  std::reverse(out.begin(), out.end());
  return out;
}

void handleDownloadZip() {
  if (!gSdOk) {
    server.send(503, "text/plain", "SD not available");
    return;
  }

  int days = server.arg("days").toInt();
  if (days <= 0) days = 7;
  // If LogConfig::RETENTION_DAYS is 0 (never delete), allow any number of days
  if (LogConfig::RETENTION_DAYS > 0 && days > LogConfig::RETENTION_DAYS) days = LogConfig::RETENTION_DAYS;
  if (days < 1) days = 1;

  auto entries = buildLastNDaysList(days);
  if (entries.empty()) {
    server.send(404, "text/plain", "No daily CSV files found");
    return;
  }

  crc32_init();

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Type", "application/zip");
  server.sendHeader("Content-Disposition", "attachment; filename=\"last_" + String(days) + "_days.zip\"");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200);

  uint32_t offset = 0;

  const size_t BUF_SZ = 512;
  uint8_t buf[BUF_SZ];

  // Helper to write data via sendContent
  auto sendBytes = [&](const uint8_t* data, size_t len) {
    server.sendContent((const char*)data, len);
    offset += len;
  };

  auto sendU16 = [&](uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF)};
    sendBytes(b, 2);
  };

  auto sendU32 = [&](uint32_t v) {
    uint8_t b[4] = {
      (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
      (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF)
    };
    sendBytes(b, 4);
  };

  // Local headers + data + data descriptors
  for (auto &e : entries) {
    File f = LittleFS.open(e.sdPath.c_str(), FILE_READ);
    if (!f) continue;

    e.lho = offset;

    // Local file header
    sendU32(0x04034b50);                      // signature
    sendU16(20);                              // version
    sendU16(0x0008);                          // flags: data descriptor present
    sendU16(0);                               // method: STORE
    sendU16(e.dosTime);
    sendU16(e.dosDate);
    sendU32(0);                               // crc
    sendU32(0);                               // comp size
    sendU32(0);                               // uncomp size
    sendU16((uint16_t)e.name.length());       // name len
    sendU16(0);                               // extra len

    sendBytes((const uint8_t*)e.name.c_str(), e.name.length());

    // Data stream + CRC32
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t sent = 0;
    while (f.available()) {
      int n = f.read(buf, BUF_SZ);
      if (n <= 0) break;
      sendBytes(buf, n);
      crc = crc32_update(crc, buf, (size_t)n);
      sent += (uint32_t)n;
      yield();
    }
    f.close();

    e.crc = crc ^ 0xFFFFFFFFUL;
    e.size = sent;

    // Data descriptor
    sendU32(0x08074b50);
    sendU32(e.crc);
    sendU32(e.size);
    sendU32(e.size);

    yield();
  }

  // Central directory
  uint32_t cdStart = offset;

  for (auto &e : entries) {
    sendU32(0x02014b50);                    // signature
    sendU16(20);                            // version made by
    sendU16(20);                            // version needed
    sendU16(0x0008);                        // flags
    sendU16(0);                             // method
    sendU16(e.dosTime);
    sendU16(e.dosDate);
    sendU32(e.crc);
    sendU32(e.size);
    sendU32(e.size);
    sendU16((uint16_t)e.name.length());     // name len
    sendU16(0);                             // extra len
    sendU16(0);                             // comment len
    sendU16(0);                             // disk start
    sendU16(0);                             // internal attrs
    sendU32(0);                             // external attrs
    sendU32(e.lho);                         // local header offset

    sendBytes((const uint8_t*)e.name.c_str(), e.name.length());
    yield();
  }

  uint32_t cdSize = offset - cdStart;

  // End of central directory
  sendU32(0x06054b50);
  sendU16(0);
  sendU16(0);
  sendU16((uint16_t)entries.size());
  sendU16((uint16_t)entries.size());
  sendU32(cdSize);
  sendU32(cdStart);
  sendU16(0);

  server.sendContent("");  // End chunked transfer
}

void handleApiClearData() {
  String pw = server.arg("pw");
  bool rateLimited = false;
  if (!verifyPassword(pw, rateLimited)) {
    server.send(rateLimited ? 429 : 401, "application/json",
                rateLimited ? "{\"ok\":false,\"error\":\"rate_limited\"}"
                            : "{\"ok\":false,\"error\":\"unauthorized\"}");
    return;
  }
#ifdef SD_CARD_ENABLE
  server.send(503, "application/json", "{\"ok\":false,\"error\":\"sd_not_available\"}");
#else
  bool ok = deleteDirFiles("/data");
  invalidateFilesCache();
  String out = String("{\"ok\":") + (ok ? "true" : "false") + "}";
  server.send(ok ? 200 : 500, "application/json", out);
#endif
}

void handleApiDelete() {
  String pw = server.arg("pw");
  bool rateLimited = false;
  if (!verifyPassword(pw, rateLimited)) {
    server.send(rateLimited ? 429 : 401, "application/json",
                rateLimited ? "{\"ok\":false,\"error\":\"rate_limited\"}"
                            : "{\"ok\":false,\"error\":\"unauthorized\"}");
    return;
  }
  if (!gSdOk) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"sd_not_available\"}");
    return;
  }
  String filename = server.arg("filename");
  if (!isAllowedFilename(filename)) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid_filename\"}");
    return;
  }
  String path = "/data/" + filename;
  if (!LittleFS.exists(path.c_str())) {
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
    return;
  }
  bool ok = LittleFS.remove(path.c_str());
  if (!ok) {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"delete_failed\"}");
    return;
  }
  invalidateFilesCache();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiReboot() {
  String pw = server.arg("pw");
  bool rateLimited = false;
  if (!verifyPassword(pw, rateLimited)) {
    server.send(rateLimited ? 429 : 401, "application/json",
                rateLimited ? "{\"ok\":false,\"error\":\"rate_limited\"}"
                            : "{\"ok\":false,\"error\":\"unauthorized\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
  delay(100);
  ESP.restart();
}

static inline String numOrNull(float v, int digits) {
  return isfinite(v) ? String(v, digits) : String("null");
}

static String buildBucketJson(const BucketSample& b) {
  // Full descriptive property names
#ifdef WIND_SENSOR
  float avg_wind = (b.avgWind != 255) ? ((float)b.avgWind / 2.0f) : NAN;
  float max_wind = (b.maxWind != 255) ? ((float)b.maxWind / 2.0f) : NAN;
#endif
#ifdef TEMP_SENSOR
  float temp_c = (b.avgTempC != -128) ? (float)b.avgTempC : NAN;
#endif
#ifdef HUMID_SENSOR
  float hum_rh = (b.avgHumRH != 255) ? (float)b.avgHumRH : NAN;
#endif
#ifdef PRESS_SENSOR
  float press_hpa = (b.avgPressHpa != -128) ? (1000.0f + (float)b.avgPressHpa) : NAN;
#endif

  String out = "{\"timestamp\":";
  out += String((uint32_t)b.startEpoch);
#ifdef WIND_SENSOR
  out += ",\"wind_speed_avg\":";
  out += numOrNull(avg_wind, 3);
  out += ",\"wind_speed_max\":";
  out += numOrNull(max_wind, 3);
  out += ",\"wind_speed_samples\":";
  out += String(b.samples);
#endif
#ifdef TEMP_SENSOR
  out += ",\"temperature\":";
  out += numOrNull(temp_c, 2);
#endif
#ifdef HUMID_SENSOR
  out += ",\"humidity\":";
  out += numOrNull(hum_rh, 2);
#endif
#ifdef PRESS_SENSOR
  out += ",\"pressure\":";
  out += numOrNull(press_hpa, 2);
#endif
#ifdef AIR_QUAL_SENSOR
  out += ",\"pm1\":";
  out += numOrNull(b.avgPM1, 1);
  out += ",\"pm25\":";
  out += numOrNull(b.avgPM25, 1);
  out += ",\"pm10\":";
  out += numOrNull(b.avgPM10, 1);
#endif
  out += "}";
  return out;
}

static String buildBucketJsonCompact(const BucketSample& b) {
  // Compact format for internal UI - array format with full precision
  // Format: [epoch, avgWind, maxWind, samples, tempC, humRH, pressHpa, pm1, pm25, pm10]
  if (!timeIsValid(b.startEpoch)) return "";

  // Use full precision floats directly
#ifdef WIND_SENSOR
  float avg_wind = b.avgWind;
  float max_wind = b.maxWind;
#endif
#ifdef TEMP_SENSOR
  float temp_c = b.avgTempC;
#endif
#ifdef HUMID_SENSOR
  float hum_rh = b.avgHumRH;
#endif
#ifdef PRESS_SENSOR
  float press_hpa = b.avgPressHpa;
#endif

  String out = "[";
  out += String((uint32_t)b.startEpoch);
  out += ",";
#ifdef WIND_SENSOR
  out += numOrNull(avg_wind, 3);
  out += ",";
  out += numOrNull(max_wind, 3);
  out += ",";
  out += String(b.samples);
#endif
#ifdef TEMP_SENSOR
  out += ",";
  out += numOrNull(temp_c, 2);
#endif
#ifdef HUMID_SENSOR
  out += ",";
  out += numOrNull(hum_rh, 2);
#endif
#ifdef PRESS_SENSOR
  out += ",";
  out += numOrNull(press_hpa, 2);
#endif
#ifdef AIR_QUAL_SENSOR
  out += ",";
  out += numOrNull(b.avgPM1, 1);
  out += ",";
  out += numOrNull(b.avgPM25, 1);
  out += ",";
  out += numOrNull(b.avgPM10, 1);
#endif
  out += "]";
  return out;
}

void handleApiBuckets() {
  time_t nowE = epochNow();
  time_t todayMidnight = localMidnight(nowE);

  // Use chunked transfer with batching for performance
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  String header = "{\"now_epoch\":";
  header += String((uint32_t)nowE);
  header += ",\"bucket_seconds\":";
  header += String(LogConfig::BUCKET_SECONDS);
  header += ",\"buckets\":[";
  server.sendContent(header);

  bool first = true;
  String batch = "";
  batch.reserve(12288); // Reserve 12KB for batching

  // Add finalized buckets from today only
  for (int i = 0; i < LogConfig::BUCKETS_24H; i++) {
    int idx = (gBucketWrite + i) % LogConfig::BUCKETS_24H;
    const BucketSample& b = gBuckets[idx];
    if (!timeIsValid(b.startEpoch)) continue;
    if (b.startEpoch < todayMidnight) continue;

    String bucketJson = buildBucketJson(b);
    if (bucketJson.length() > 0) {
      if (!first) batch += ",";
      first = false;
      batch += bucketJson;

      // Send batch when it reaches 10KB to avoid blocking too long
      if (batch.length() > 10000) {
        server.sendContent(batch);
        batch = "";
      }
    }
  }

  // Always append the current in-progress bucket
  BucketSample cur = currentBucketSnapshot();
  if (timeIsValid(cur.startEpoch) && cur.startEpoch >= todayMidnight) {
    bool alreadyFinalized = false;
    int lastIdx = (gBucketWrite - 1 + LogConfig::BUCKETS_24H) % LogConfig::BUCKETS_24H;
    if (timeIsValid(gBuckets[lastIdx].startEpoch) &&
        gBuckets[lastIdx].startEpoch == cur.startEpoch) {
      alreadyFinalized = true;
    }

    if (!alreadyFinalized) {
      String bucketJson = buildBucketJson(cur);
      if (bucketJson.length() > 0) {
        if (!first) batch += ",";
        batch += bucketJson;
      }
    }
  }

  // Send remaining batch
  if (batch.length() > 0) {
    server.sendContent(batch);
  }

  server.sendContent("]}");
}

void handleApiBucketsCompact() {
  // Compact format for internal UI use - saves bandwidth
  time_t nowE = epochNow();
  time_t todayMidnight = localMidnight(nowE);

  // Use chunked transfer with batching for performance
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  String header = "{\"now_epoch\":";
  header += String((uint32_t)nowE);
  header += ",\"bucket_seconds\":";
  header += String(LogConfig::BUCKET_SECONDS);
  header += ",\"buckets\":[";
  server.sendContent(header);

  bool first = true;
  String batch = "";
  batch.reserve(12288); // Reserve 12KB for batching

  // Add finalized buckets from last 24 hours
  time_t cutoff24h = nowE - 86400;  // 24 hours ago
  for (int i = 0; i < LogConfig::BUCKETS_24H; i++) {
    int idx = (gBucketWrite + i) % LogConfig::BUCKETS_24H;
    const BucketSample& b = gBuckets[idx];
    if (!timeIsValid(b.startEpoch)) continue;
    if (b.startEpoch < cutoff24h) continue;

    String bucketJson = buildBucketJsonCompact(b);
    if (bucketJson.length() > 0) {
      if (!first) batch += ",";
      first = false;
      batch += bucketJson;

      // Send batch when it reaches 10KB to avoid blocking too long
      if (batch.length() > 10000) {
        server.sendContent(batch);
        batch = "";
      }
    }
  }

  // Always append the current in-progress bucket
  BucketSample cur = currentBucketSnapshot();
  if (timeIsValid(cur.startEpoch) && cur.startEpoch >= cutoff24h) {
    bool alreadyFinalized = false;
    int lastIdx = (gBucketWrite - 1 + LogConfig::BUCKETS_24H) % LogConfig::BUCKETS_24H;
    if (timeIsValid(gBuckets[lastIdx].startEpoch) &&
        gBuckets[lastIdx].startEpoch == cur.startEpoch) {
      alreadyFinalized = true;
    }

    if (!alreadyFinalized) {
      String bucketJson = buildBucketJsonCompact(cur);
      if (bucketJson.length() > 0) {
        if (!first) batch += ",";
        batch += bucketJson;
      }
    }
  }

  // Send remaining batch
  if (batch.length() > 0) {
    server.sendContent(batch);
  }

  server.sendContent("]}");
}

static File gUploadFile;
static bool gUploadPasswordVerified = false;
static bool gUploadError = false;
static String gUploadErrorMsg = "";
static const char* TEMP_UPLOAD_FILE = "/web/.upload.tmp";

void handleUploadData() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    gUploadError = false;
    gUploadErrorMsg = "";
    gUploadPasswordVerified = false;

    if (!gSdOk) {
      gUploadError = true;
      gUploadErrorMsg = "sd_not_available";
      return;
    }

    if (!LittleFS.exists("/web")) {
      LittleFS.mkdir("/web");
    }

    // Delete any previous temp file
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) {
      LittleFS.remove(TEMP_UPLOAD_FILE);
    }

    // Save to temporary file first
    gUploadFile = LittleFS.open(TEMP_UPLOAD_FILE, FILE_WRITE);
    if (!gUploadFile) {
      gUploadError = true;
      gUploadErrorMsg = "failed_to_create_file";
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (gUploadFile && !gUploadError) {
      gUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (gUploadFile) {
      gUploadFile.close();
    }
  }
}

void handleUploadComplete() {
  // Check for upload errors first
  if (gUploadError) {
    // Delete temp file on error
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) {
      LittleFS.remove(TEMP_UPLOAD_FILE);
    }

    int statusCode = 500;
    if (gUploadErrorMsg == "unauthorized") statusCode = 401;
    else if (gUploadErrorMsg == "rate_limited") statusCode = 429;
    else if (gUploadErrorMsg == "forbidden_path") statusCode = 403;
    else if (gUploadErrorMsg == "sd_not_available") statusCode = 503;

    server.send(statusCode, "application/json",
                "{\"ok\":false,\"error\":\"" + gUploadErrorMsg + "\"}");

    gUploadPasswordVerified = false;
    gUploadError = false;
    gUploadErrorMsg = "";
    return;
  }

  // Get path and password (now available after multipart parsing)
  String path = server.arg("path");
  String pw = server.arg("pw");

  // Validate path
  if (path.length() == 0) {
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing_path\"}");
    return;
  }

  // Check for path traversal attempts
  if (path.indexOf("..") >= 0) {
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"path_traversal_detected\"}");
    return;
  }

  // Check for backslashes (Windows-style paths)
  if (path.indexOf('\\') >= 0) {
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"invalid_path_separator\"}");
    return;
  }

  // Ensure path starts with /web/
  if (!path.startsWith("/web/")) {
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"forbidden_path\"}");
    return;
  }

  // Validate that path doesn't have double slashes or other suspicious patterns
  if (path.indexOf("//") >= 0) {
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"invalid_path_format\"}");
    return;
  }

  // Extract filename from path and validate it
  int lastSlash = path.lastIndexOf('/');
  if (lastSlash < 0 || lastSlash == path.length() - 1) {
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"invalid_filename\"}");
    return;
  }

  String filename = path.substring(lastSlash + 1);
  // Validate filename has allowed characters (alphanumeric, dash, underscore, dot)
  for (unsigned int i = 0; i < filename.length(); i++) {
    char c = filename.charAt(i);
    if (!isalnum(c) && c != '-' && c != '_' && c != '.') {
      if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
      server.send(403, "application/json", "{\"ok\":false,\"error\":\"invalid_filename_characters\"}");
      return;
    }
  }

  // Verify password
  bool rateLimited = false;
  gUploadPasswordVerified = verifyPassword(pw, rateLimited);

  if (!gUploadPasswordVerified) {
    // Password failed - delete temp file
    if (LittleFS.exists(TEMP_UPLOAD_FILE)) {
      LittleFS.remove(TEMP_UPLOAD_FILE);
    }

    int statusCode = rateLimited ? 429 : 401;
    String errorMsg = rateLimited ? "rate_limited" : "unauthorized";
    server.send(statusCode, "application/json",
                "{\"ok\":false,\"error\":\"" + errorMsg + "\"}");
  } else {
    // Password OK - move temp file to final location
    // Delete existing file if present
    if (LittleFS.exists(path.c_str())) {
      LittleFS.remove(path.c_str());
    }

    // Rename temp file to final name
    if (LittleFS.rename(TEMP_UPLOAD_FILE, path.c_str())) {
      server.send(200, "application/json",
                  "{\"ok\":true,\"bytes\":" + String(server.upload().totalSize) + "}");
    } else {
      // Rename failed - try to delete temp file
      if (LittleFS.exists(TEMP_UPLOAD_FILE)) LittleFS.remove(TEMP_UPLOAD_FILE);
      server.send(500, "application/json", "{\"ok\":false,\"error\":\"rename_failed\"}");
    }
  }

  gUploadPasswordVerified = false;
  gUploadError = false;
  gUploadErrorMsg = "";
}

// ------------------- SETUP HELPERS -------------------

bool syncTimeWithNTP(uint32_t timeoutMs = 20000) {
  setenv("TZ", NetworkConfig::TIMEZONE, 1);
  tzset();
  configTzTime(NetworkConfig::TIMEZONE, NetworkConfig::NTP_SERVER_1, NetworkConfig::NTP_SERVER_2, NetworkConfig::NTP_SERVER_3);

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    time_t nowE = time(nullptr);
    if (timeIsValid(nowE)) return true;
    delay(250);
  }
  return false;
}

#define FORMAT_LITTLEFS_IF_FAILED true

bool initSD() {
#ifdef SD_CARD_ENABLE
  return LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
#else
  return false;
#endif
}

// ------------------- SETUP / LOOP -------------------
WiFiManager wm;

void initWiFiManager() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  setRGB(RGB_YELLOW);

  wm.setConfigPortalTimeout(120);
}

void processBucketCatchup(time_t nowE) {
  time_t aligned = floorToBucketBoundaryLocal(nowE);
  if (!timeIsValid(gCurrentBucketStart)) startBucketAt(aligned);

  // Limit catch-up work so we keep serving HTTP/OTA while filling missed buckets
  const int MAX_BUCKETS_PER_LOOP = 6;
  int bucketsProcessed = 0;
  while (gCurrentBucketStart < aligned && bucketsProcessed < MAX_BUCKETS_PER_LOOP) {
    finalizeCurrentBucket(gCurrentBucketStart);

    gCurrentBucketStart += LogConfig::BUCKET_SECONDS;
#ifdef WIND_SENSOR
    clearWindSamples();
#endif
    clearEnvSamples();
#ifdef AIR_QUAL_SENSOR
    resetAirQuality();
#endif
    bucketsProcessed++;
    if ((bucketsProcessed % 2) == 0) {
      server.handleClient();
#ifdef SUPPORT_OTA
      ArduinoOTA.handle();
#endif
      delay(0);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  initDataLog();

#ifdef RGB_BUILTIN
  initRGB();
#endif

#ifdef WIND_SENSOR
  setupWindPin();
#endif

  // 1. Mount LittleFS partition and read config parameters
  gSdOk = LittleFS.begin(true);
  if (gSdOk) {
    Serial.println("LittleFS initialized");
  }

  // 2. Setup standard WiFiManager parameters
  initWiFiManager();

  if (!wm.autoConnect("Weather-Setup")) {
      setRGB(RGB_RED); // Solid Red on catastrophic boot failure
      delay(2000);
      ESP.restart();
  }

  if (!syncTimeWithNTP()) {
    delay(500);
    reportError("Can't sync time with NTP server!");
    ESP.restart();
  }

#ifdef RGB_BUILTIN
  setRGB(RGB_GREEN);
#endif
  Serial.println("Start DNS server");
  if (MDNS.begin("weather")) {
      Serial.println("mDNS started: http://weather.local");
  }
  uint32_t msNow = millis();
  Serial.println("Initializing sensors");
#if defined(BME280_SENSOR) || defined(BMP280_SENSOR)
  gEnvOk = initEnvSensors(msNow);
  if (!gEnvOk) {
    reportError("Error init Environmental Sensors");
  }
#endif

#ifdef AIR_QUAL_SENSOR
  initAirQuality(msNow);
#endif

  // Synch with files off LittleFS Storage
  time_t nowE = epochNow();
  gTodayMidnightEpoch = localMidnight(nowE);
  deleteOldDailyFileIfNeeded(gTodayMidnightEpoch);

  // Remove stale cache file if it exists
  if (gSdOk && LittleFS.exists("/data/day_summaries_cache.csv")) {
    LittleFS.remove("/data/day_summaries_cache.csv");
  }

  // Always load fresh from SD to avoid stale cache issues
  loadDaySummariesFromSD(nowE);
  loadRecentBucketsFromSD(nowE);

  time_t aligned = floorToBucketBoundaryLocal(nowE);
  startBucketAt(aligned);

  if (initWebServer()) {
    reportMessage(RGB_BLUE, "Web Server started!");
  }

#ifdef SUPPORT_OTA
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    (void)error;
  });
  ArduinoOTA.begin();
#endif

#ifdef WIND_SENSOR
  initWindSensor();   // Do this last to prevent counting spurious Pin interrupts
#endif
}

void loop() {
  server.handleClient();
#ifdef SUPPORT_OTA
  ArduinoOTA.handle();
#endif

  uint32_t msNow = millis();

#ifdef WIND_SENSOR
  updateWindPPS(msNow);
#endif
#if defined(BME280_SENSOR) || defined(BMP280_SENSOR)
  pollEnvIfNeeded(msNow);
#endif
#ifdef AIR_QUAL_SENSOR
  pollPMSIfNeeded(msNow);
#endif

  time_t nowE = epochNow();
  maybeRolloverDay(nowE);
  processBucketCatchup(nowE);
}

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
#include <ArduinoOTA.h>
#include <pgmspace.h>

#include <vector>
#include <algorithm>
#include "config.h"
#include "env_sensors.h"
#ifdef AIR_QUAL_SENSOR
  #include "air_quality.h"
#endif
#ifdef WIND_SENSOR
  #include "wind_sensor.h"
#endif
#include "upload_page.h"
#include "api_help_page.h"

// ------------------- FORWARD DECLARATIONS -------------------

struct BucketSample;
struct DaySummary;
void saveDaySummariesCache(const DaySummary* curDay, bool hasCurDay);
bool loadDaySummariesCache();
void pushBucketSample(const BucketSample& b);
void rebuildTodayAggregates();

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

void reportError(const char* msg) {
  Serial.println(msg);
#ifdef RGB_BUILTIN
  setRGB(RGB_RED);
#endif
}

// Data Logging
namespace LogConfig {
  static constexpr int BUCKET_SECONDS = 60;             // Log interval (1 minute)
  static constexpr int BUCKETS_24H = 24 * 60 * 60 / BUCKET_SECONDS;
  static constexpr int RETENTION_DAYS = 0;           // 0 = never delete
  static constexpr int DAYS_HISTORY = 30;               // RAM history
  static_assert((86400 % BUCKET_SECONDS) == 0, "BUCKET_SECONDS must divide evenly into 24h");
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

// Web UI
namespace UIConfig {
  static constexpr int FILES_PER_PAGE = 30;
  static constexpr int MAX_PLOT_POINTS = 500;
}

// ==================== PLOT CONFIGURATION ====================

struct PlotSeries {
  const char* field;       // Data field name (e.g., "avgWind", "maxWind")
  const char* color;       // SVG color
  const char* label;       // Display label for tooltip
  bool visible;            // Show by default
};

struct PlotConfig {
  const char* id;          // Unique ID (e.g., "wind", "temp")
  const char* title;       // Display title
  const char* unit;        // Unit string (e.g., "km/h", "°C")
  float conversionFactor;  // Multiply data by this (e.g., 3.6 for m/s to km/h)
  PlotSeries series[3];    // Max 3 series per plot
  int seriesCount;
};

// Plot definitions
static const PlotConfig PLOTS[] = {
#ifdef WIND_SENSOR
  {
    "wind",
    "Wind Speed",
    "km/h",
    3.6f,  // m/s to km/h
    {
      {"avgWind", "black", "Wind", true},
      {"maxWind", "#f0ad4e", "Gust", true}
    },
    2
  },
#endif
#ifdef TEMP_SENSOR
  {
    "temp",
    "Temperature",
    "°C",
    1.0f,
    {
      {"avgTempC", "#d9534f", "Temp", true},
      {nullptr, nullptr, nullptr, false}
    },
    1
  },
#endif
#ifdef HUMID_SENSOR
  {
    "hum",
    "Humidity",
    "%",
    1.0f,
    {
      {"avgHumRH", "#0275d8", "Humidity", true},
      {nullptr, nullptr, nullptr, false}
    },
    1
  },
#endif
#ifdef PRESS_SENSOR
  {
    "press",
    "Pressure (MSLP)",
    "hPa",
    1.0f,
    {
      {"avgPressHpa", "#5cb85c", "Pressure", true},
      {nullptr, nullptr, nullptr, false}
    },
    1
  },
#endif
#ifdef AIR_QUAL_SENSOR
  {
    "pm",
    "Air Quality (PM)",
    "μg/m³",
    1.0f,
    {
      {"avgPM25", "#ff6600", "PM2.5", true},
      {"avgPM10", "#996633", "PM10", true},
      {"avgPM1", "#9966cc", "PM1.0", true}
    },
    3
  }
#endif
};

#define COUNT_OF(a)    (sizeof(a)/sizeof(a[0]))

// ==================== TABLE CONFIGURATION ====================

struct TableColumn {
  const char* field;       // Data field name
  const char* label;       // Column sub-header (avg, max, min)
  const char* unit;        // Unit to display
  float conversionFactor;  // Multiply by this
  int decimals;            // Decimal places
  const char* bgColor;     // Background color (hex or empty for default)
  const char* group;       // Group header (Wind Speed, Temperature, etc.)
};

static const TableColumn TABLE_COLUMNS[] = {
  {"day", "Day", "", 1.0f, 0, "", ""},
#ifdef WIND_SENSOR
  {"avgWind", "avg", "km/h", 3.6f, 1, "#f8f8f8", "Wind Speed"},
  {"maxWind", "max", "km/h", 3.6f, 1, "#f8f8f8", "Wind Speed"},
#endif
#ifdef TEMP_SENSOR
  {"avgTemp", "avg", "°C", 1.0f, 1, "", "Temperature"},
  {"minTemp", "min", "°C", 1.0f, 1, "", "Temperature"},
  {"maxTemp", "max", "°C", 1.0f, 1, "", "Temperature"},
#endif
#ifdef HUMID_SENSOR
  {"avgHum", "avg", "%", 1.0f, 1, "#f8f8f8", "Humidity"},
  {"minHum", "min", "%", 1.0f, 1, "#f8f8f8", "Humidity"},
  {"maxHum", "max", "%", 1.0f, 1, "#f8f8f8", "Humidity"},
#endif
#ifdef PRESS_SENSOR
  {"avgPress", "avg", "hPa", 1.0f, 1, "", "Pressure"},
  {"minPress", "min", "hPa", 1.0f, 1, "", "Pressure"},
  {"maxPress", "max", "hPa", 1.0f, 1, "", "Pressure"},
#endif
#ifdef AIR_QUAL_SENSOR
  {"avgPM1", "PM1 avg", "μg/m³", 1.0f, 1, "#f8f8f8", "Particulate Matter"},
  {"maxPM1", "PM1 max", "μg/m³", 1.0f, 1, "#f8f8f8", "Particulate Matter"},
  {"avgPM25", "PM2.5 avg", "μg/m³", 1.0f, 1, "#f8f8f8", "Particulate Matter"},
  {"maxPM25", "PM2.5 max", "μg/m³", 1.0f, 1, "#f8f8f8", "Particulate Matter"},
  {"avgPM10", "PM10 avg", "μg/m³", 1.0f, 1, "#f8f8f8", "Particulate Matter"},
  {"maxPM10", "PM10 max", "μg/m³", 1.0f, 1, "#f8f8f8", "Particulate Matter"}
#endif
};

// ------------------- DATA -------------------

struct BucketSample {
  time_t startEpoch;
#ifdef WIND_SENSOR
  float avgWind;         // m/s (NAN = invalid)
  float maxWind;         // m/s (NAN = invalid)
  uint32_t samples;
#endif
#ifdef TEMP_SENSOR
  float avgTempC;        // °C (NAN = invalid)
#endif
#ifdef HUMID_SENSOR
  float avgHumRH;        // % (NAN = invalid)
#endif
#ifdef PRESS_SENSOR
  float avgPressHpa;     // hPa (NAN = invalid)
#endif
#ifdef AIR_QUAL_SENSOR
  float avgPM1;          // μg/m³ (NAN = invalid)
  float avgPM25;         // μg/m³ (NAN = invalid)
  float avgPM10;         // μg/m³ (NAN = invalid)
#endif
};

struct DaySummary {
  time_t dayStartEpoch; // local midnight
#ifdef WIND_SENSOR
  float avgWind;
  float maxWind;
#endif
#ifdef TEMP_SENSOR
  float avgTemp;
  float minTemp;
  float maxTemp;
#endif
#ifdef HUMID_SENSOR
  float avgHum;
  float minHum;
  float maxHum;
#endif
#ifdef PRESS_SENSOR
  float avgPress;
  float minPress;
  float maxPress;
#endif
#ifdef AIR_QUAL_SENSOR
  float avgPM1;
  float maxPM1;
  float avgPM25;
  float maxPM25;
  float avgPM10;
  float maxPM10;
#endif
};

static BucketSample gBuckets[LogConfig::BUCKETS_24H];
static int gBucketWrite = 0;

static DaySummary gDays[LogConfig::DAYS_HISTORY];
static int gDayWrite = 0;
static uint32_t gDaysCount = 0;

// timing / bucket accumulators
static time_t   gCurrentBucketStart = 0;

// daily rollup
static time_t   gTodayMidnightEpoch = 0;

// Auth / rate limit for password-protected endpoints
static int      gPwAttempts = 0;
static time_t   gPwWindowStart = 0;

// ------------------- HELPERS -------------------

static void clearTodayAggregates() {
#ifdef WIND_SENSOR
  clearWindToday();
#endif
  clearEnvToday();
#ifdef AIR_QUAL_SENSOR
  clearAirQualToday();
#endif
}

static bool gEnvOk = false;

// SD status
static bool gSdOk = false;
static constexpr uint32_t FILES_CACHE_TTL_MS = 30000; // 30s reuse to avoid slow SD scans on refresh
static String gFilesCacheDataJson;
static uint32_t gFilesCacheDataMs = 0;

// Web
static WebServer server(80);

// ------------------- TIME HELPERS -------------------
static inline bool timeIsValid(time_t t) { return t > 1577836800; } // > 2020-01-01
static inline time_t epochNow() { return time(nullptr); }

time_t localMidnight(time_t nowEpoch) {
  struct tm tmLocal;
  localtime_r(&nowEpoch, &tmLocal);
  tmLocal.tm_hour = 0; tmLocal.tm_min = 0; tmLocal.tm_sec = 0; tmLocal.tm_isdst = -1;
  return mktime(&tmLocal);
}

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

time_t floorToBucketBoundaryLocal(time_t nowEpoch) {
  struct tm tmLocal;
  localtime_r(&nowEpoch, &tmLocal);
  tmLocal.tm_sec = 0;
  int bucketMinutes = LogConfig::BUCKET_SECONDS / 60;
  int bucketSeconds = LogConfig::BUCKET_SECONDS % 60;
  if (bucketMinutes > 0) {
    tmLocal.tm_min = (tmLocal.tm_min / bucketMinutes) * bucketMinutes;
    tmLocal.tm_sec = (bucketSeconds > 0) ? (tmLocal.tm_sec / bucketSeconds) * bucketSeconds : 0;
  } else {
    tmLocal.tm_sec = (tmLocal.tm_sec / LogConfig::BUCKET_SECONDS) * LogConfig::BUCKET_SECONDS;
  }
  return mktime(&tmLocal);
}

String fmtLocal(time_t t) {
  struct tm tmLocal;
  localtime_r(&t, &tmLocal);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmLocal);
  return String(buf);
}

String ymdString(time_t t) {
  struct tm tmLocal;
  localtime_r(&t, &tmLocal);
  char buf[16];
  strftime(buf, sizeof(buf), "%Y%m%d", &tmLocal);
  return String(buf);
}

time_t subtractDaysLocalMidnight(time_t tMidnightLocal, int days) {
  struct tm tmLocal;
  localtime_r(&tMidnightLocal, &tmLocal);
  tmLocal.tm_mday -= days;
  tmLocal.tm_hour = 0; tmLocal.tm_min = 0; tmLocal.tm_sec = 0; tmLocal.tm_isdst = -1;
  return mktime(&tmLocal);
}

// ------------------- SD HELPERS -------------------

bool ensureDir(const char* path) {
  if (!gSdOk) return false;
  if (LittleFS.exists(path)) return true;
  return LittleFS.mkdir(path);
}

bool appendLine(const String& path, const String& line, const String& headerIfNew = "") {
  if (!gSdOk) return false;

  bool exists = LittleFS.exists(path.c_str());
  File f = LittleFS.open(path.c_str(), FILE_APPEND);
  if (!f) return false;

  if (!exists && headerIfNew.length()) {
    f.print(headerIfNew);
    if (headerIfNew[headerIfNew.length()-1] != '\n') f.print("\n");
  }
  f.print(line);
  if (line.length() && line[line.length()-1] != '\n') f.print("\n");
  f.close();
  return true;
}

void deleteOldDailyFileIfNeeded(time_t todayMidnightLocal) {
  // If LogConfig::RETENTION_DAYS is 0, never delete files
  if (LogConfig::RETENTION_DAYS == 0) return;

  time_t oldMidnight = subtractDaysLocalMidnight(todayMidnightLocal, LogConfig::RETENTION_DAYS + 1);
  String ymd = ymdString(oldMidnight);
  String dataPath = String("/data/") + ymd + ".csv";
  if (gSdOk && LittleFS.exists(dataPath.c_str())) LittleFS.remove(dataPath.c_str());
}

void logBucketToSD(const BucketSample& b) {
  if (!gSdOk) return;

  ensureDir("/data");
  ensureDir("/backup");

  time_t bucketMidnightLocal = localMidnight(b.startEpoch);
  String ymd = ymdString(bucketMidnightLocal);
  String dailyFile = String("/data/") + ymd + ".csv";
  String backupFile = String("/backup/") + ymd + ".csv";

  String header =
    "datetime,epoch"
#ifdef WIND_SENSOR
    ",wind_avg_ms,wind_max_ms"
#endif
#ifdef TEMP_SENSOR
    ",temp_c"
#endif
#ifdef HUMID_SENSOR
    ",hum_rh"
#endif
#ifdef PRESS_SENSOR
    ",press_hpa"
#endif
#ifdef AIR_QUAL_SENSOR
    ",pm1,pm25,pm10,samples"
#endif
  ;

  String line =
    fmtLocal(b.startEpoch) + "," +
    String((uint32_t)b.startEpoch) + "," +
#ifdef WIND_SENSOR
    String(b.avgWind, 3) + "," +
    String(b.maxWind, 3) + "," +
#endif
#ifdef TEMP_SENSOR
    String(b.avgTempC, 2) + "," +
#endif
#ifdef HUMID_SENSOR
    String(b.avgHumRH, 2) + "," +
#endif
#ifdef PRESS_SENSOR
    String(b.avgPressHpa, 2) + "," +
#endif
#ifdef AIR_QUAL_SENSOR
    String(isfinite(b.avgPM1) ? String(b.avgPM1, 1) : String("")) + "," +
    String(isfinite(b.avgPM25) ? String(b.avgPM25, 1) : String("")) + "," +
    String(isfinite(b.avgPM10) ? String(b.avgPM10, 1) : String("")) + "," +
    String(b.samples);
#endif

  appendLine(dailyFile, line, header);
  appendLine(backupFile, line, header);
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

static float parseFloatOrNan(const String& s) {
  if (!s.length()) return NAN;
  String trimmed = s;
  trimmed.trim();
  if (!trimmed.length()) return NAN;
  // Check for common invalid markers
  if (trimmed == "nan" || trimmed == "NaN" || trimmed == "NAN") return NAN;
  if (trimmed == "null" || trimmed == "NULL") return NAN;
  if (trimmed == "-" || trimmed == "--") return NAN;
  // toFloat() returns 0 for invalid input, so check if it's actually a valid number
  // by verifying the string starts with a digit, minus, or decimal point
  char first = trimmed.charAt(0);
  if (first != '-' && first != '.' && !isdigit(first)) return NAN;
  return trimmed.toFloat();
}

static bool splitCSVLine(const String& line, String parts[], int expectedParts) {
  int start = 0;
  int idx = 0;
  while (idx < expectedParts - 1) {
    int comma = line.indexOf(',', start);
    if (comma < 0) break;
    parts[idx++] = line.substring(start, comma);
    start = comma + 1;
  }
  parts[idx++] = line.substring(start);
  return idx == expectedParts;
}

static void invalidateFilesCache() {
  gFilesCacheDataMs = 0;
  gFilesCacheDataJson = "";
}

void loadRecentBucketsFromSD(time_t nowEpoch) {
  if (!gSdOk || !timeIsValid(nowEpoch)) return;

  time_t cutoff = nowEpoch - 86400;
  std::vector<BucketSample> loaded;
  loaded.reserve(LogConfig::BUCKETS_24H);

  auto loadFile = [&](time_t dayMidnightLocal) {
    String path = String("/data/") + ymdString(dayMidnightLocal) + ".csv";
    if (!LittleFS.exists(path.c_str())) return;
    File f = LittleFS.open(path.c_str(), FILE_READ);
    if (!f) return;
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (!line.length()) continue;
      if (line.startsWith("datetime")) continue;

      // Count actual columns
      int commaCount = 0;
      for (int i = 0; i < line.length(); i++) {
        if (line[i] == ',') commaCount++;
      }
      int numCols = commaCount + 1;

      String parts[11];
      splitCSVLine(line, parts, numCols < 11 ? numCols : 11);

      if (numCols < 8) continue;  // Need at least 8 fields
      BucketSample b{};
      b.startEpoch = (time_t)parts[1].toInt();
      if (!timeIsValid(b.startEpoch) || b.startEpoch < cutoff) continue;
      // Skip buckets that don't align with current LogConfig::BUCKET_SECONDS setting
      if (b.startEpoch % LogConfig::BUCKET_SECONDS != 0) continue;
    
      // Load floats directly from CSV
      int col = 2;
#ifdef WIND_SENSOR
      b.avgWind = parseFloatOrNan(parts[col++]);
      b.maxWind = parseFloatOrNan(parts[col++]);
#endif
#ifdef TEMP_SENSOR
      b.avgTempC = parseFloatOrNan(parts[col++]);
#endif
#ifdef HUMID_SENSOR
      b.avgHumRH = parseFloatOrNan(parts[col++]);
#endif
#ifdef PRESS_SENSOR
      b.avgPressHpa = parseFloatOrNan(parts[col++]);
#endif
#ifdef AIR_QUAL_SENSOR
      // PM fields (optional for backward compatibility)
      b.avgPM1 = (numCols > 7) ? parseFloatOrNan(parts[col++]) : NAN;
      b.avgPM25 = (numCols > 8) ? parseFloatOrNan(parts[col++]) : NAN;
      b.avgPM10 = (numCols > 9) ? parseFloatOrNan(parts[col++]) : NAN;
      b.samples = (uint32_t)std::max<long>(0, parts[numCols > 10 ? 10 : 7].toInt());
#endif
      loaded.push_back(b);
    }
    f.close();
  };

  time_t todayMidnight = localMidnight(nowEpoch);
  loadFile(todayMidnight);
  loadFile(subtractDaysLocalMidnight(todayMidnight, 1)); // bring in late-night buckets from previous day

  if (loaded.empty()) return;

  std::sort(loaded.begin(), loaded.end(), [](const BucketSample& a, const BucketSample& b) {
    return a.startEpoch < b.startEpoch;
  });

  if (loaded.size() > (size_t)LogConfig::BUCKETS_24H) {
    loaded.erase(loaded.begin(), loaded.begin() + (loaded.size() - LogConfig::BUCKETS_24H));
  }

  memset(gBuckets, 0, sizeof(gBuckets));
  gBucketWrite = 0;
  for (const auto& b : loaded) {
    pushBucketSample(b);
  }

  rebuildTodayAggregates();
}

// ------------------- BUCKETS / DAILY -------------------

void startBucketAt(time_t bucketStart) {
  gCurrentBucketStart = bucketStart;
#ifdef WIND_SENSOR
  gBucketWindSum = 0.0f;
  gBucketWindMax1 = 0.0f;
  gBucketWindMax2 = 0.0f;
  gBucketSamples = 0;
  gBucketPulseCount = 0;
  gBucketPulseElapsedMs = 0;
#endif
#ifdef TEMP_SENSOR
  gBucketTempSum = 0.0f;
#endif
#ifdef HUMID_SENSOR
  gBucketHumSum = 0.0f;
#endif
#ifdef PRESS_SENSOR
  gBucketPressSumPa = 0.0f;
  gBucketEnvSamples = 0;
#endif
#ifdef AIR_QUAL_SENSOR
  gBucketPM1Sum = 0.0f;
  gBucketPM25Sum = 0.0f;
  gBucketPM10Sum = 0.0f;
  gBucketPmSamples = 0;
#endif
}

void pushBucketSample(const BucketSample& b) {
  gBuckets[gBucketWrite] = b;
  gBucketWrite = (gBucketWrite + 1) % LogConfig::BUCKETS_24H;
}

static void accumulateTodayFromBucket(const BucketSample& b) {
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

void pushDaySummary(time_t dayStart
#ifdef WIND_SENSOR
                   , float avgWind, float maxWind
#endif
#ifdef TEMP_SENSOR
                   , float avgTemp, float minTemp, float maxTemp
#endif
#ifdef HUMID_SENSOR
                   , float avgHum, float minHum, float maxHum
#endif
#ifdef PRESS_SENSOR
                   , float avgPress = NAN, float minPress = NAN, float maxPress = NAN
#endif
#ifdef AIR_QUAL_SENSOR
                   , float avgPM1 = NAN, float maxPM1 = NAN, float avgPM25 = NAN, float maxPM25 = NAN, float avgPM10 = NAN, float maxPM10 = NAN
#endif
                  ) {
  if (!timeIsValid(dayStart)) return;
  DaySummary d{};
  d.dayStartEpoch = dayStart;
#ifdef WIND_SENSOR
  d.avgWind = avgWind;
  d.maxWind = maxWind;
#endif
#ifdef TEMP_SENSOR
  d.avgTemp = avgTemp;
  d.minTemp = minTemp;
  d.maxTemp = maxTemp;
#endif
#ifdef HUMID_SENSOR
  d.avgHum = avgHum;
  d.minHum = minHum;
  d.maxHum = maxHum;
#endif
#ifdef PRESS_SENSOR
  d.avgPress = avgPress;
  d.minPress = minPress;
  d.maxPress = maxPress;
#endif
#ifdef AIR_QUAL_SENSOR
  d.avgPM1 = avgPM1;
  d.maxPM1 = maxPM1;
  d.avgPM25 = avgPM25;
  d.maxPM25 = maxPM25;
  d.avgPM10 = avgPM10;
  d.maxPM10 = maxPM10;
#endif
  gDays[gDayWrite] = d;
  gDayWrite = (gDayWrite + 1) % LogConfig::DAYS_HISTORY;
  if (gDaysCount < (uint32_t)LogConfig::DAYS_HISTORY) gDaysCount++;
}

void maybeRolloverDay(time_t nowEpoch) {
  time_t midnight = localMidnight(nowEpoch);

  if (gTodayMidnightEpoch == 0) {
    gTodayMidnightEpoch = midnight;
    return;
  }

  if (midnight != gTodayMidnightEpoch) {
#ifdef WIND_SENSOR
      float avgWind = gTodayBucketCount ? (gTodaySumOfBucketAvgs / (float)gTodayBucketCount) : NAN;
#endif
#ifdef TEMP_SENSOR
      float avgTemp = (gTodayTempCount > 0) ? (gTodayTempSum / (float)gTodayTempCount) : NAN;
#endif
#ifdef HUMID_SENSOR
      float avgHum  = (gTodayHumCount > 0) ? (gTodayHumSum / (float)gTodayHumCount) : NAN;
#endif
#ifdef PRESS_SENSOR
      float avgPress = (gTodayPressCount > 0) ? (gTodayPressSum / (float)gTodayPressCount) : NAN;
#endif
#ifdef AIR_QUAL_SENSOR
      float avgPM1 = (gTodayPM1Count > 0) ? (gTodayPM1Sum / (float)gTodayPM1Count) : NAN;
      float avgPM25 = (gTodayPM25Count > 0) ? (gTodayPM25Sum / (float)gTodayPM25Count) : NAN;
      float avgPM10 = (gTodayPM10Count > 0) ? (gTodayPM10Sum / (float)gTodayPM10Count) : NAN;
#endif
      pushDaySummary(gTodayMidnightEpoch
#ifdef WIND_SENSOR
                    , avgWind, gTodayMax
#endif
#ifdef TEMP_SENSOR
                    , avgTemp, gTodayTempMin, gTodayTempMax
#endif
#ifdef HUMID_SENSOR
                    , avgHum, gTodayHumMin, gTodayHumMax
#endif
#ifdef PRESS_SENSOR
                    , avgPress, gTodayPressMin, gTodayPressMax
#endif
#ifdef AIR_QUAL_SENSOR
                    , avgPM1, gTodayPM1Max, avgPM25, gTodayPM25Max, avgPM10, gTodayPM10Max
#endif
                  );

    gTodayMidnightEpoch = midnight;
    clearTodayAggregates();
    deleteOldDailyFileIfNeeded(midnight);
  }
}

struct DayAgg {
  float sumWind = 0.0f;
  uint32_t countWind = 0;
  float maxWind = NAN;
  float tempSum = 0.0f;
  uint32_t tempCount = 0;
  float tempMin = NAN;
  float tempMax = NAN;
  float humSum = 0.0f;
  uint32_t humCount = 0;
  float humMin = NAN;
  float humMax = NAN;
  float pressSum = 0.0f;
  uint32_t pressCount = 0;
  float pressMin = NAN;
  float pressMax = NAN;
  float pm1Sum = 0.0f;
  uint32_t pm1Count = 0;
  float pm1Max = NAN;
  float pm25Sum = 0.0f;
  uint32_t pm25Count = 0;
  float pm25Max = NAN;
  float pm10Sum = 0.0f;
  uint32_t pm10Count = 0;
  float pm10Max = NAN;
};

static void computeBucketSample(BucketSample& b, time_t bucketStart) {
  b.startEpoch = bucketStart;
#ifdef WIND_SENSOR
  b.samples = gBucketSamples;
#endif
#ifdef TEMP_SENSOR
    b.avgTempC = (gBucketEnvSamples > 0) ? gBucketTempSum / (float)gBucketEnvSamples : NAN;
#endif
#ifdef HUMID_SENSOR
    b.avgHumRH =  (gBucketEnvSamples > 0) ? gBucketHumSum / (float)gBucketEnvSamples : NAN;
#endif
#ifdef PRESS_SENSOR
    b.avgPressHpa =  (gBucketEnvSamples > 0) ? (gBucketPressSumPa / (float)gBucketEnvSamples) / 100.0f : NAN;
#endif
#ifdef WIND_SENSOR
  // Wind
  float avgWindMs = computeWind();
  b.avgWind = avgWindMs;
  b.maxWind = computeMaxWind(avgWindMs);
#endif
#ifdef AIR_QUAL_SENSOR
  // PM averages
  if (gBucketPmSamples > 0) {
    b.avgPM1 = gBucketPM1Sum / (float)gBucketPmSamples;
    b.avgPM25 = gBucketPM25Sum / (float)gBucketPmSamples;
    b.avgPM10 = gBucketPM10Sum / (float)gBucketPmSamples;
  } else {
    b.avgPM1 = NAN;
    b.avgPM25 = NAN;
    b.avgPM10 = NAN;
  }
#endif
}

void loadDaySummariesFromSD(time_t nowEpoch) {
  if (!gSdOk) return;
  // Reset
  memset(gDays, 0, sizeof(gDays));
  gDayWrite = 0;
  gDaysCount = 0;

  time_t todayMid = localMidnight(nowEpoch);
  String todayYmd = ymdString(todayMid);

  File dir = LittleFS.open("/data");
  if (!dir) return;

  // Collect per-day aggregates
  struct Entry { time_t day; DayAgg agg; };
  Entry entries[30];  // Reduced from 64 to avoid stack overflow
  int entryCount = 0;

  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    if (f.isDirectory()) { f.close(); continue; }
    String name = f.name();
    if (!name.endsWith(".csv")) { f.close(); continue; }
    time_t dayMid = 0;
    if (!parseYmdFromPath(name, dayMid) || !timeIsValid(dayMid)) {
      f.close(); continue;
    }
    // Skip current in-progress day by name as well as by computed midnight to avoid DST skew duplicates.
    if (dayMid == todayMid || name.indexOf(todayYmd) >= 0) { f.close(); continue; }

    // find or add entry
    DayAgg* agg = nullptr;
    for (int i=0;i<entryCount;i++){
      if (entries[i].day == dayMid) { agg = &entries[i].agg; break; }
    }
    if (!agg && entryCount < (int)(sizeof(entries)/sizeof(entries[0]))) {
      entries[entryCount].day = dayMid;
      agg = &entries[entryCount].agg;
      entryCount++;
    }
    if (!agg) { f.close(); continue; }

    // parse file
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (!line.length()) continue;
      if (line.startsWith("datetime")) continue;

      // Count actual columns
      int commaCount = 0;
      for (int i = 0; i < line.length(); i++) {
        if (line[i] == ',') commaCount++;
      }
      int numCols = commaCount + 1;

      String parts[11];
      splitCSVLine(line, parts, numCols < 11 ? numCols : 11);

      // Parse available columns (old format has 8 cols, new format has 11)
      if (numCols < 7) continue;  // Need at least 7 columns for basic data
      float windAvg = parseFloatOrNan(parts[2]);
      float windMax = parseFloatOrNan(parts[3]);
      float temp    = parseFloatOrNan(parts[4]);
      float hum     = parseFloatOrNan(parts[5]);
      float press   = parseFloatOrNan(parts[6]);
      float pm1     = (numCols > 7) ? parseFloatOrNan(parts[7]) : NAN;
      float pm25    = (numCols > 8) ? parseFloatOrNan(parts[8]) : NAN;
      float pm10    = (numCols > 9) ? parseFloatOrNan(parts[9]) : NAN;

      if (isfinite(windAvg)) { agg->sumWind += windAvg; agg->countWind++; }
      if (isfinite(windMax)) { if (!isfinite(agg->maxWind) || windMax > agg->maxWind) agg->maxWind = windMax; }
      if (isfinite(temp)) {
        if (!isfinite(agg->tempMin) || temp < agg->tempMin) agg->tempMin = temp;
        if (!isfinite(agg->tempMax) || temp > agg->tempMax) agg->tempMax = temp;
        agg->tempSum += temp; agg->tempCount++;
      }
      if (isfinite(hum)) {
        if (!isfinite(agg->humMin) || hum < agg->humMin) agg->humMin = hum;
        if (!isfinite(agg->humMax) || hum > agg->humMax) agg->humMax = hum;
        agg->humSum += hum; agg->humCount++;
      }
      if (isfinite(press)) {
        if (!isfinite(agg->pressMin) || press < agg->pressMin) agg->pressMin = press;
        if (!isfinite(agg->pressMax) || press > agg->pressMax) agg->pressMax = press;
        agg->pressSum += press; agg->pressCount++;
      }
      if (isfinite(pm1)) {
        if (!isfinite(agg->pm1Max) || pm1 > agg->pm1Max) agg->pm1Max = pm1;
        agg->pm1Sum += pm1; agg->pm1Count++;
      }
      if (isfinite(pm25)) {
        if (!isfinite(agg->pm25Max) || pm25 > agg->pm25Max) agg->pm25Max = pm25;
        agg->pm25Sum += pm25; agg->pm25Count++;
      }
      if (isfinite(pm10)) {
        if (!isfinite(agg->pm10Max) || pm10 > agg->pm10Max) agg->pm10Max = pm10;
        agg->pm10Sum += pm10; agg->pm10Count++;
      }
    }
    f.close();
  }
  dir.close();

  // sort by day ascending
  for (int i=0;i<entryCount;i++){
    for (int j=i+1;j<entryCount;j++){
      if (entries[j].day < entries[i].day){
        Entry tmp = entries[i]; entries[i]=entries[j]; entries[j]=tmp;
      }
    }
  }
  // push into ring (keep last LogConfig::DAYS_HISTORY)
  int startIdx = entryCount > LogConfig::DAYS_HISTORY ? entryCount - LogConfig::DAYS_HISTORY : 0;
  for (int i=startIdx; i<entryCount; i++){
    DayAgg& a = entries[i].agg;
    if (!timeIsValid(entries[i].day)) continue;
    float avgWind = (a.countWind > 0) ? (a.sumWind / (float)a.countWind) : NAN;
    float maxWind = a.maxWind;
    float avgTemp = (a.tempCount > 0) ? (a.tempSum / (float)a.tempCount) : NAN;
    float avgPress = (a.pressCount > 0) ? (a.pressSum / (float)a.pressCount) : NAN;
    float avgPM1 = (a.pm1Count > 0) ? (a.pm1Sum / (float)a.pm1Count) : NAN;
    float avgPM25 = (a.pm25Count > 0) ? (a.pm25Sum / (float)a.pm25Count) : NAN;
    float avgPM10 = (a.pm10Count > 0) ? (a.pm10Sum / (float)a.pm10Count) : NAN;
    pushDaySummary(entries[i].day
#ifdef WIND_SENSOR
                  , avgWind, maxWind
#endif
#ifdef TEMP_SENSOR
                  , avgTemp, a.tempMin, a.tempMax
#endif
#ifdef HUMID_SENSOR
                  , (a.humCount > 0) ? (a.humSum / (float)a.humCount) : NAN, a.humMin, a.humMax
#endif
#ifdef PRESS_SENSOR
                  , avgPress, a.pressMin, a.pressMax
#endif
#ifdef AIR_QUAL_SENSOR
                  , avgPM1, a.pm1Max, avgPM25, a.pm25Max, avgPM10, a.pm10Max
#endif
                );
  }
}


void finalizeCurrentBucket(time_t bucketStart) {
  BucketSample b{};
  computeBucketSample(b, bucketStart);
  pushBucketSample(b);
  accumulateTodayFromBucket(b);
  logBucketToSD(b);
}

void rebuildTodayAggregates() {
  clearTodayAggregates();
  if (!timeIsValid(gTodayMidnightEpoch)) return;
  for (int i = 0; i < LogConfig::BUCKETS_24H; i++) {
    const BucketSample& b = gBuckets[i];
    if (!timeIsValid(b.startEpoch)) continue;
    if (b.startEpoch < gTodayMidnightEpoch) continue;
    accumulateTodayFromBucket(b);
  }
}

bool buildCurrentDaySummary(DaySummary& out) {
  if (!timeIsValid(gTodayMidnightEpoch)) return false;
#ifdef WIND_SENSOR
  float sumWind = gTodaySumOfBucketAvgs;
  uint32_t windCount = gTodayBucketCount;
  float maxWind = gTodayMax;
#endif
#ifdef TEMP_SENSOR
  float tempSum = gTodayTempSum;
  uint32_t tempCount = gTodayTempCount;
  float tempMin = gTodayTempMin;
  float tempMax = gTodayTempMax;
#endif
#ifdef HUMID_SENSOR
  float humSum = gTodayHumSum;
  uint32_t humCount = gTodayHumCount;
  float humMin = gTodayHumMin;
  float humMax = gTodayHumMax;
#endif
#ifdef PRESS_SENSOR
  float pressSum = gTodayPressSum;
  uint32_t pressCount = gTodayPressCount;
  float pressMin = gTodayPressMin;
  float pressMax = gTodayPressMax;
#endif
  // Do not incluide the in-progress bucket's data in this step.
  bool hasData = 
#ifdef WIND_SENSOR
        (windCount > 0)
#else
        false
#endif
#ifdef TEMP_SENSOR
         || (tempCount > 0)
#endif
#ifdef HUMID_SENSOR
         || (humCount > 0)
#endif
#ifdef PRESS_SENSOR
         || (pressCount > 0)
#endif
  ;
  if (!hasData) return false;
  out.dayStartEpoch = gTodayMidnightEpoch;
#ifdef WIND_SENSOR
  out.avgWind = (windCount > 0) ? (sumWind / (float)windCount) : NAN;
  out.maxWind = (windCount > 0) ? maxWind : NAN;
#endif
#ifdef TEMP_SENSOR
  out.avgTemp = (tempCount > 0) ? (tempSum / (float)tempCount) : NAN;
  out.minTemp = tempMin;
  out.maxTemp = tempMax;
#endif
#ifdef HUMID_SENSOR
  out.avgHum = (humCount > 0) ? (humSum / (float)humCount) : NAN;
  out.minHum = humMin;
  out.maxHum = humMax;
#endif
#ifdef PRESS_SENSOR
  out.avgPress = (pressCount > 0) ? (pressSum / (float)pressCount) : NAN;
  out.minPress = pressMin;
  out.maxPress = pressMax;
#endif
#ifdef AIR_QUAL_SENSOR
  // PM data
  out.avgPM1 = (gTodayPM1Count > 0) ? (gTodayPM1Sum / (float)gTodayPM1Count) : NAN;
  out.maxPM1 = gTodayPM1Max;
  out.avgPM25 = (gTodayPM25Count > 0) ? (gTodayPM25Sum / (float)gTodayPM25Count) : NAN;
  out.maxPM25 = gTodayPM25Max;
  out.avgPM10 = (gTodayPM10Count > 0) ? (gTodayPM10Sum / (float)gTodayPM10Count) : NAN;
  out.maxPM10 = gTodayPM10Max;
#endif

  return true;
}

BucketSample currentBucketSnapshot() {
  BucketSample b{};
  if (!timeIsValid(gCurrentBucketStart)) return b;
  computeBucketSample(b, gCurrentBucketStart);
  return b;
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

static bool parseYmdFromPath(const String& path, time_t& outMidnightLocal) {
  int idx = path.lastIndexOf('/');
  String name = (idx >= 0) ? path.substring(idx + 1) : path;
  if (name.length() < 8) return false;

  // Parse YYYYMMDD.csv format
  String ymd = name.substring(0, 8);
  if (ymd.length() != 8) return false;
  int y = ymd.substring(0,4).toInt();
  int m = ymd.substring(4,6).toInt();
  int d = ymd.substring(6,8).toInt();
  if (y <= 1970 || m < 1 || m > 12 || d < 1 || d > 31) return false;
  struct tm tmLocal = {};
  tmLocal.tm_year = y - 1900;
  tmLocal.tm_mon = m - 1;
  tmLocal.tm_mday = d;
  tmLocal.tm_hour = 0; tmLocal.tm_min = 0; tmLocal.tm_sec = 0; tmLocal.tm_isdst = -1;
  time_t t = mktime(&tmLocal);
  if (!timeIsValid(t)) return false;
  outMidnightLocal = t;
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

// ------------------- WEB UI + API -------------------

void handleRoot() {
  if (gSdOk && LittleFS.exists("/web/index.html")) {
    File f = LittleFS.open("/web/index.html", FILE_READ);
    if (f) {
      server.streamFile(f, "text/html");
      f.close();
      return;
    }
  }

  const char* fallbackHtml = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Weather Station - Setup Required</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; max-width: 600px; }
    h1 { color: #333; }
    a { color: #0275d8; text-decoration: none; }
    a:hover { text-decoration: underline; }
    ul { line-height: 1.8; }
  </style>
</head>
<body>
  <h1>Weather Station - Setup Required</h1>
  <p>Web interface files not found on SD card.</p>
  <ul>
    <li><a href="/upload">Upload UI files (index.html, app.js)</a></li>
    <li><a href="/api/now">View current sensor data (JSON)</a></li>
    <li><a href="/api/buckets">View bucket data (JSON)</a></li>
    <li><a href="/api/days">View daily summaries (JSON)</a></li>
    <li><a href="/api/config">View configuration (JSON)</a></li>
    <li><a href="/api_help">API documentation</a></li>
  </ul>
</body>
</html>
)HTML";

  server.send(200, "text/html", fallbackHtml);
}

void handleApiHelp() {
  server.send_P(200, "text/html", API_HELP_HTML);
}

void handleRootJs() {
  if (gSdOk && LittleFS.exists("/web/app.js")) {
    File f = LittleFS.open("/web/app.js", FILE_READ);
    if (f) {
      server.streamFile(f, "application/javascript");
      f.close();
      return;
    }
  }

  server.send(200, "application/javascript", "console.log('app.js not found on SD card');");
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

void handleApiNow() {
#ifdef WIND_SENSOR
  float pps = calcWindPps();
#endif
  time_t nowE = epochNow();
#ifdef PRESS_SENSOR
  float press_hpa = isfinite(gPressurePa) ? (gPressurePa / 100.0f) : NAN;
#endif

  String out = "{";
  out += "\"epoch\":" + String((uint32_t)nowE) + ",";
  out += "\"local_time\":\"" + fmtLocal(nowE) + "\",";
#ifdef WIND_SENSOR
  out += "\"wind_pps\":" + (isfinite(pps) ? String(pps, 3) : String("null")) + ",";
  out += "\"wind_ms\":" + (isfinite(gNowWindMS) ? String(gNowWindMS, 3) : String("null")) + ",";
#endif
#if defined(BME280_SENSOR) || defined(BMP280_SENSOR)
  out += "\"bme280_ok\":" + String(gEnvOk ? "true" : "false") + ",";
#endif
#ifdef TEMP_SENSOR
  out += "\"temp_c\":" + (isfinite(gTempC) ? String(gTempC, 2) : String("null")) + ",";
#endif
#ifdef HUMID_SENSOR
  out += "\"hum_rh\":" + (isfinite(gHumRH) ? String(gHumRH, 2) : String("null")) + ",";
#endif
#ifdef PRESS_SENSOR
  out += "\"press_hpa\":" + (isfinite(press_hpa) ? String(press_hpa, 2) : String("null")) + ",";
#endif
#ifdef AIR_QUAL_SENSOR
  out += "\"pms5003_ok\":" + String(gPmsOk ? "true" : "false") + ",";
  out += "\"pm1\":" + (isfinite(gPM1) ? String(gPM1, 1) : String("null")) + ",";
  out += "\"pm25\":" + (isfinite(gPM25) ? String(gPM25, 1) : String("null")) + ",";
  out += "\"pm10\":" + (isfinite(gPM10) ? String(gPM10, 1) : String("null")) + ",";

  // Calculate AQI values
  int aqiPM25 = calculateAQI_PM25(gPM25);
  int aqiPM10 = calculateAQI_PM10(gPM10);
  out += "\"aqi_pm25\":" + (aqiPM25 >= 0 ? String(aqiPM25) : String("null")) + ",";
  out += "\"aqi_pm25_category\":\"" + String(getAQICategory(aqiPM25)) + "\",";
  out += "\"aqi_pm10\":" + (aqiPM10 >= 0 ? String(aqiPM10) : String("null")) + ",";
  out += "\"aqi_pm10_category\":\"" + String(getAQICategory(aqiPM10)) + "\",";
#endif
  out += "\"sd_ok\":" + String(gSdOk ? "true" : "false") + ",";
  out += "\"cpu_temp_c\":" + String(temperatureRead(), 1) + ",";
  out += "\"uptime_s\":" + String((uint32_t)(millis() / 1000)) + ",";
  out += "\"retention_days\":" + String(LogConfig::RETENTION_DAYS) + ",";
  out += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  out += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  out += "\"heap_size\":" + String(ESP.getHeapSize());
  out += "}";
  server.send(200, "application/json", out);
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

  // Skip buckets with no valid sensor data
  if (
#ifdef WIND_SENSOR
      !isfinite(avg_wind) && !isfinite(max_wind)
#else
      true
#endif
#ifdef TEMP_SENSOR
      && !isfinite(temp_c)
#endif
#ifdef HUMID_SENSOR
      && !isfinite(hum_rh)
#endif
#ifdef PRESS_SENSOR
      && !isfinite(press_hpa)
#endif
    ) {
    return "";
  }

  String out = "{\"timestamp\":";
  out += String((uint32_t)b.startEpoch);
#ifdef WINO_SENSOR
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
  // Skip buckets with no valid sensor data
  if (true
#ifdef WIND_SENSOR
      && !isfinite(avg_wind) && !isfinite(max_wind)
#endif
#ifdef TEMP_SENSOR
      && !isfinite(temp_c)
#endif
#ifdef HUMID_SENSOR
      && !isfinite(hum_rh)
#endif
#ifdef PRESS_SENSOR
      && !isfinite(press_hpa)
#endif
    ) {
    return "";
  }

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

void handleApiDays() {
  String out;
  out.reserve(8 * 1024);
  out += "{";
  out += "\"days\":[";
  DaySummary curDay{};
  bool hasCurDay = buildCurrentDaySummary(curDay);
  bool firstOut = true;
  if (hasCurDay) {
    out += "{";
    out += "\"dayStartEpoch\":" + String((uint32_t)curDay.dayStartEpoch) + ",";
    out += "\"dayStartLocal\":\"" + fmtLocal(curDay.dayStartEpoch) + "\",";
#ifdef WIND_SENSOR
    out += "\"avgWind\":" + (isfinite(curDay.avgWind) ? String(curDay.avgWind, 3) : String("null")) + ",";
    out += "\"maxWind\":" + (isfinite(curDay.maxWind) ? String(curDay.maxWind, 3) : String("null")) + ",";
#endif
#ifdef TEMP_SENSOR
    out += "\"avgTemp\":" + (isfinite(curDay.avgTemp) ? String(curDay.avgTemp, 2) : String("null")) + ",";
    out += "\"minTemp\":" + (isfinite(curDay.minTemp) ? String(curDay.minTemp, 2) : String("null")) + ",";
    out += "\"maxTemp\":" + (isfinite(curDay.maxTemp) ? String(curDay.maxTemp, 2) : String("null")) + ",";
#endif
#ifdef HUMID_SENSOR
    out += "\"avgHum\":" + (isfinite(curDay.avgHum) ? String(curDay.avgHum, 2) : String("null")) + ",";
    out += "\"minHum\":" + (isfinite(curDay.minHum) ? String(curDay.minHum, 2) : String("null")) + ",";
    out += "\"maxHum\":" + (isfinite(curDay.maxHum) ? String(curDay.maxHum, 2) : String("null")) + ",";
#endif
#ifdef PRESS_SENSOR
    out += "\"avgPress\":" + (isfinite(curDay.avgPress) ? String(curDay.avgPress, 2) : String("null")) + ",";
    out += "\"minPress\":" + (isfinite(curDay.minPress) ? String(curDay.minPress, 2) : String("null")) + ",";
    out += "\"maxPress\":" + (isfinite(curDay.maxPress) ? String(curDay.maxPress, 2) : String("null")) + ",";
#endif
#ifdef AIR_QUAL_SENSOR
    out += "\"avgPM1\":" + (isfinite(curDay.avgPM1) ? String(curDay.avgPM1, 1) : String("null")) + ",";
    out += "\"maxPM1\":" + (isfinite(curDay.maxPM1) ? String(curDay.maxPM1, 1) : String("null")) + ",";
    out += "\"avgPM25\":" + (isfinite(curDay.avgPM25) ? String(curDay.avgPM25, 1) : String("null")) + ",";
    out += "\"maxPM25\":" + (isfinite(curDay.maxPM25) ? String(curDay.maxPM25, 1) : String("null")) + ",";
    out += "\"avgPM10\":" + (isfinite(curDay.avgPM10) ? String(curDay.avgPM10, 1) : String("null")) + ",";
    out += "\"maxPM10\":" + (isfinite(curDay.maxPM10) ? String(curDay.maxPM10, 1) : String("null"));
#endif
    out += "}";
    firstOut = false;
  }
  uint32_t count = gDaysCount;
  // Loop backwards from most recent to oldest
  for (int i = (int)count - 1; i >= 0; i--) {
    int startIdx = (gDayWrite - (int)gDaysCount + LogConfig::DAYS_HISTORY) % LogConfig::DAYS_HISTORY;
    int idx = (startIdx + i) % LogConfig::DAYS_HISTORY;
    const DaySummary& d = gDays[idx];
    if (!timeIsValid(d.dayStartEpoch)) continue;
    if (!firstOut) out += ",";
    firstOut = false;
    out += "{";
    out += "\"dayStartEpoch\":" + String((uint32_t)d.dayStartEpoch) + ",";
    out += "\"dayStartLocal\":\"" + fmtLocal(d.dayStartEpoch) + "\",";
#ifdef WIND_SENSOR
    out += "\"avgWind\":" + (isfinite(d.avgWind) ? String(d.avgWind, 3) : String("null")) + ",";
    out += "\"maxWind\":" + (isfinite(d.maxWind) ? String(d.maxWind, 3) : String("null")) + ",";
#endif
#ifdef TEMP_SENSOR
    out += "\"avgTemp\":" + (isfinite(d.avgTemp) ? String(d.avgTemp, 2) : String("null")) + ",";
    out += "\"minTemp\":" + (isfinite(d.minTemp) ? String(d.minTemp, 2) : String("null")) + ",";
    out += "\"maxTemp\":" + (isfinite(d.maxTemp) ? String(d.maxTemp, 2) : String("null")) + ",";
#endif
#ifdef HUMID_SENSOR
    out += "\"avgHum\":" + (isfinite(d.avgHum) ? String(d.avgHum, 2) : String("null")) + ",";
    out += "\"minHum\":" + (isfinite(d.minHum) ? String(d.minHum, 2) : String("null")) + ",";
    out += "\"maxHum\":" + (isfinite(d.maxHum) ? String(d.maxHum, 2) : String("null")) + ",";
#endif
#ifdef PRESS_SENSOR
    out += "\"avgPress\":" + (isfinite(d.avgPress) ? String(d.avgPress, 2) : String("null")) + ",";
    out += "\"minPress\":" + (isfinite(d.minPress) ? String(d.minPress, 2) : String("null")) + ",";
    out += "\"maxPress\":" + (isfinite(d.maxPress) ? String(d.maxPress, 2) : String("null")) + ",";
#endif
#ifdef AIR_QUAL_SENSOR
    out += "\"avgPM1\":" + (isfinite(d.avgPM1) ? String(d.avgPM1, 1) : String("null")) + ",";
    out += "\"maxPM1\":" + (isfinite(d.maxPM1) ? String(d.maxPM1, 1) : String("null")) + ",";
    out += "\"avgPM25\":" + (isfinite(d.avgPM25) ? String(d.avgPM25, 1) : String("null")) + ",";
    out += "\"maxPM25\":" + (isfinite(d.maxPM25) ? String(d.maxPM25, 1) : String("null")) + ",";
    out += "\"avgPM10\":" + (isfinite(d.avgPM10) ? String(d.avgPM10, 1) : String("null")) + ",";
    out += "\"maxPM10\":" + (isfinite(d.maxPM10) ? String(d.maxPM10, 1) : String("null"));
#endif
    out += "}";
  }
  out += "]}";
  server.send(200, "application/json", out);
}

void handleApiConfig() {
  String out;
  out.reserve(4096);
  out = "{";

  // Plots configuration
  out += "\"plots\":[";
  for (int i = 0; i < COUNT_OF(PLOTS); i++) {
    if (i > 0) out += ",";
    const PlotConfig& p = PLOTS[i];
    out += "{";
    out += "\"id\":\"" + String(p.id) + "\",";
    out += "\"title\":\"" + String(p.title) + "\",";
    out += "\"unit\":\"" + String(p.unit) + "\",";
    out += "\"conversionFactor\":" + String(p.conversionFactor, 3) + ",";
    out += "\"series\":[";
    for (int j = 0; j < p.seriesCount; j++) {
      if (j > 0) out += ",";
      const PlotSeries& s = p.series[j];
      if (s.field) {
        out += "{";
        out += "\"field\":\"" + String(s.field) + "\",";
        out += "\"color\":\"" + String(s.color) + "\",";
        out += "\"label\":\"" + String(s.label) + "\"";
        out += "}";
      }
    }
    out += "]";
    out += "}";
  }
  out += "],";

  // Table configuration
  out += "\"tableColumns\":[";
  for (int i = 0; i < COUNT_OF(TABLE_COLUMNS); i++) {
    if (i > 0) out += ",";
    const TableColumn& c = TABLE_COLUMNS[i];
    out += "{";
    out += "\"field\":\"" + String(c.field) + "\",";
    out += "\"label\":\"" + String(c.label) + "\",";
    out += "\"unit\":\"" + String(c.unit) + "\",";
    out += "\"conversionFactor\":" + String(c.conversionFactor, 3) + ",";
    out += "\"decimals\":" + String(c.decimals) + ",";
    out += "\"bgColor\":\"" + String(c.bgColor) + "\",";
    out += "\"group\":\"" + String(c.group) + "\"";
    out += "}";
  }
  out += "],";

  // UI parameters
  out += "\"filesPerPage\":" + String(UIConfig::FILES_PER_PAGE) + ",";
  out += "\"maxPlotPoints\":" + String(UIConfig::MAX_PLOT_POINTS);

  out += "}";
  server.send(200, "application/json", out);
}

void handleApiUiFiles() {
  if (!gSdOk) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"sd_not_available\"}");
    return;
  }

  String out = "{\"ok\":true,\"files\":[";
  bool first = true;

  const char* webFiles[] = {
    "/web/index.html",
    "/web/app.js",
    "/web/app_base.js",
    "/web/app_base_clean.js"
  };
  for (int i = 0; i < COUNT_OF(webFiles); i++) {
    const char* path = webFiles[i];
    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, FILE_READ);
      if (f) {
        if (!first) out += ",";
        first = false;
        out += "{";
        out += "\"path\":\"" + String(path) + "\",";
        out += "\"size\":" + String((uint32_t)f.size()) + ",";
        out += "\"lastModified\":" + String((uint32_t)f.getLastWrite());
        out += "}";
        f.close();
      }
    }
  }

  out += "]}";
  server.send(200, "application/json", out);
}

void handleUploadPage() {
  server.send_P(200, "text/html", UPLOAD_HTML);
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

void saveDaySummariesCache(const DaySummary* curDay, bool hasCurDay) {
  if (!gSdOk) return;
  ensureDir("/data");
  File f = LittleFS.open("/data/day_summaries_cache.csv", FILE_WRITE);
  if (!f) return;
  f.print("dayStartEpoch,avgWind,maxWind,avgTemp,minTemp,maxTemp,avgHum,minHum,maxHum,avgPress,minPress,maxPress,avgPM1,maxPM1,avgPM25,maxPM25,avgPM10,maxPM10\n");
  auto writeRow = [&](const DaySummary& d){
    if (!timeIsValid(d.dayStartEpoch)) return;
    f.print((uint32_t)d.dayStartEpoch); f.print(",");
#ifdef WIND_SENSOR
    f.print(isfinite(d.avgWind) ? String(d.avgWind,3) : String("")); f.print(",");
    f.print(isfinite(d.maxWind) ? String(d.maxWind,3) : String("")); f.print(",");
#endif
#ifdef TEMP_SENSOR
    f.print(isfinite(d.avgTemp) ? String(d.avgTemp,2) : String("")); f.print(",");
    f.print(isfinite(d.minTemp) ? String(d.minTemp,2) : String("")); f.print(",");
    f.print(isfinite(d.maxTemp) ? String(d.maxTemp,2) : String("")); f.print(",");
#endif
#ifdef HUMID_SENSOR
    f.print(isfinite(d.avgHum) ? String(d.avgHum,2) : String("")); f.print(",");
    f.print(isfinite(d.minHum) ? String(d.minHum,2) : String("")); f.print(",");
    f.print(isfinite(d.maxHum) ? String(d.maxHum,2) : String("")); f.print(",");
#endif
#ifdef PRESS_SENSOR
    f.print(isfinite(d.avgPress) ? String(d.avgPress,2) : String("")); f.print(",");
    f.print(isfinite(d.minPress) ? String(d.minPress,2) : String("")); f.print(",");
    f.print(isfinite(d.maxPress) ? String(d.maxPress,2) : String("")); f.print(",");
#endif
#ifdef AIR_QUAL_SENSOR
    f.print(isfinite(d.avgPM1) ? String(d.avgPM1,1) : String("")); f.print(",");
    f.print(isfinite(d.maxPM1) ? String(d.maxPM1,1) : String("")); f.print(",");
    f.print(isfinite(d.avgPM25) ? String(d.avgPM25,1) : String("")); f.print(",");
    f.print(isfinite(d.maxPM25) ? String(d.maxPM25,1) : String("")); f.print(",");
    f.print(isfinite(d.avgPM10) ? String(d.avgPM10,1) : String("")); f.print(",");
    f.print(isfinite(d.maxPM10) ? String(d.maxPM10,1) : String(""));
#endif
    f.print("\n");
  };
  if (hasCurDay && curDay) writeRow(*curDay);
  uint32_t count2 = gDaysCount;
  int startIdx = (gDayWrite - (int)gDaysCount + LogConfig::DAYS_HISTORY) % LogConfig::DAYS_HISTORY;
  for (uint32_t i = 0; i < count2; i++) {
    int idx = (startIdx + (int)i) % LogConfig::DAYS_HISTORY;
    writeRow(gDays[idx]);
  }
  f.close();
}

bool loadDaySummariesCache() {
  if (!gSdOk || !LittleFS.exists("/data/day_summaries_cache.csv")) return false;
  File f = LittleFS.open("/data/day_summaries_cache.csv", FILE_READ);
  if (!f) return false;
  memset(gDays, 0, sizeof(gDays));
  gDayWrite = 0;
  gDaysCount = 0;
  bool first = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    if (first) { first = false; if (line.startsWith("dayStartEpoch")) continue; }
    String parts[18];
    int numParts = splitCSVLine(line, parts, 18) ? 18 : 0;
    if (numParts < 9) continue;  // Need at least 9 fields for backward compatibility
    DaySummary d{};
    d.dayStartEpoch = (time_t)parts[0].toInt();
    if (!timeIsValid(d.dayStartEpoch)) continue;
    int col = 1;
#ifdef WIND_SENSOR
    d.avgWind = parts[col++].length() ? parts[1].toFloat() : NAN;
    d.maxWind = parts[col++].length() ? parts[2].toFloat() : NAN;
#endif
#ifdef TEMP_SENSOR
    d.avgTemp = parts[col++].length() ? parts[3].toFloat() : NAN;
    d.minTemp = parts[col++].length() ? parts[4].toFloat() : NAN;
    d.maxTemp = parts[col++].length() ? parts[5].toFloat() : NAN;
#endif
#ifdef HUMID_SENSOR
    d.avgHum  = parts[col++].length() ? parts[6].toFloat() : NAN;
    d.minHum  = parts[col++].length() ? parts[7].toFloat() : NAN;
    d.maxHum  = parts[col++].length() ? parts[8].toFloat() : NAN;
#endif
#ifdef PRESS_SENSOR
    // Pressure fields (optional for backward compatibility)
    d.avgPress = (numParts > col && parts[col++].length()) ? parts[9].toFloat() : NAN;
    d.minPress = (numParts > col && parts[col++].length()) ? parts[10].toFloat() : NAN;
    d.maxPress = (numParts > col && parts[col++].length()) ? parts[11].toFloat() : NAN;
#endif
#ifdef AIR_QUAL_SENSOR
    // PM fields (optional for backward compatibility)
    d.avgPM1 = (numParts > col && parts[col++].length()) ? parts[12].toFloat() : NAN;
    d.maxPM1 = (numParts > col && parts[col++].length()) ? parts[13].toFloat() : NAN;
    d.avgPM25 = (numParts > col && parts[col++].length()) ? parts[14].toFloat() : NAN;
    d.maxPM25 = (numParts > col && parts[col++].length()) ? parts[15].toFloat() : NAN;
    d.avgPM10 = (numParts > col && parts[col++].length()) ? parts[16].toFloat() : NAN;
    d.maxPM10 = (numParts > col && parts[col++].length()) ? parts[17].toFloat() : NAN;
#endif
    gDays[gDayWrite] = d;
    gDayWrite = (gDayWrite + 1) % LogConfig::DAYS_HISTORY;
    if (gDaysCount < (uint32_t)LogConfig::DAYS_HISTORY) gDaysCount++;
  }
  f.close();
  return gDaysCount > 0;
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

// ------------------- LOOP HELPERS -------------------

static void processBucketCatchup(time_t nowE) {
  time_t aligned = floorToBucketBoundaryLocal(nowE);
  if (!timeIsValid(gCurrentBucketStart)) startBucketAt(aligned);

  // Limit catch-up work so we keep serving HTTP/OTA while filling missed buckets
  const int MAX_BUCKETS_PER_LOOP = 6;
  int bucketsProcessed = 0;
  while (gCurrentBucketStart < aligned && bucketsProcessed < MAX_BUCKETS_PER_LOOP) {
    finalizeCurrentBucket(gCurrentBucketStart);

    gCurrentBucketStart += LogConfig::BUCKET_SECONDS;
#ifdef WIND_SENSOR
    gBucketWindSum = 0.0f;
    gBucketWindMax1 = 0.0f;
    gBucketWindMax2 = 0.0f;
    gBucketSamples = 0;
    gBucketPulseCount = 0;
    gBucketPulseElapsedMs = 0;
#endif
    resetEnvSamples();
#ifdef AIR_QUAL_SENSOR
    resetAirQuality();
#endif
    bucketsProcessed++;
    if ((bucketsProcessed % 2) == 0) {
      server.handleClient();
      ArduinoOTA.handle();
      delay(0);
    }
  }
}

// ------------------- SETUP / LOOP -------------------
WiFiManager wm;

void initWiFiManager() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  setRGB(RGB_YELLOW);

  wm.setConfigPortalTimeout(120);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  memset(gBuckets, 0, sizeof(gBuckets));
  memset(gDays, 0, sizeof(gDays));
  gBucketWrite = 0;
  gDayWrite = 0;
  gDaysCount = 0;

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

  uint32_t msNow = millis();
  Serial.println("Initializing sensors");
#if defined(BME280_SENSOR) || defined(BMP280_SENSOR)
  gEnvOk = initEnvSensors(msNow);
  if (!gEnvOk) {
    reportError("Error initializing Environmental Sensors");
  }
#endif

#ifdef AIR_QUAL_SENSOR
  initAirQuality(msNow);
#endif

  Serial.println("Start DNS server");
  if (MDNS.begin("weather")) {
      Serial.println("mDNS started: http://weather.local");
  }

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

  // routes
  server.on("/", handleRoot);
  server.on("/root.js", handleRootJs);
  server.on("/api/now", handleApiNow);
  server.on("/api/buckets", handleApiBuckets);
  server.on("/api/buckets_compact", handleApiBucketsCompact);  // Compact format for internal UI
  server.on("/api/days", handleApiDays);
  server.on("/api/config", handleApiConfig);
  server.on("/api/ui_files", handleApiUiFiles);
  server.on("/api_help", handleApiHelp);
  server.on("/api/clear_data", HTTP_POST, handleApiClearData);
  server.on("/api/delete", HTTP_POST, handleApiDelete);
  server.on("/api/reboot", HTTP_POST, handleApiReboot);

  server.on("/api/files", handleApiFiles);
  server.on("/download", handleDownload);
  server.on("/download_zip", handleDownloadZip);
  server.on("/upload", HTTP_GET, handleUploadPage);
  server.on("/upload", HTTP_POST, handleUploadComplete, handleUploadData);

  server.begin();
  Serial.println("Starting web server");
#ifdef RGB_BUILTIN
  setRGB(RGB_BLUE);
#endif

  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    (void)error;
  });
  ArduinoOTA.begin();

#ifdef WIND_SENSOR
  initWindSensor();   // Do this last to prevent counting spurious Pin interrupts
#endif
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();

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

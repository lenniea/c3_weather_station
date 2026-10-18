#include <Arduino.h>
#include <LittleFS.h>
#include <float.h>
#include <WiFi.h>
#include <ESP.h>
#include "config.h"
#include "data_log.h"
#include "wind_sensor.h"
#include "env_sensors.h"

// Web UI
namespace UIConfig {
  static constexpr int FILES_PER_PAGE = 30;
  static constexpr int MAX_PLOT_POINTS = 500;
}

//
//    = = = = =   G L O B A L   V A R I A B L E S   E X P O R T E D   = = = = =
//
bool gEnvOk = false;
bool gSdOk = false;

#ifdef WIND_SENSOR
float    gBucketWindSum = 0.0f;
float    gBucketWindMax1 = 0.0f; // highest observed in bucket
float    gBucketWindMax2 = 0.0f; // second-highest observed in bucket
uint32_t gBucketSamples = 0;
uint32_t gBucketPulseCount = 0;
uint32_t gBucketPulseElapsedMs = 0;

uint32_t gTodayBucketCount = 0;
float    gTodayMax = 0.0f;
uint32_t gLastPpsMillis = 0;
float    gTodaySumOfBucketAvgs = 0.0f;

#endif

#ifdef TEMP_SENSOR
float gTempC = NAN;
float gBucketTempSum = 0.0f;
float gTodayTempSum = 0.0f;
uint32_t gTodayTempCount = 0;
float gTodayTempMin = NAN;
float gTodayTempMax = NAN;
#endif
#ifdef HUMID_SENSOR
float gHumRH = NAN;
float gBucketHumSum = 0.0f;
float gTodayHumSum = 0.0f;
uint32_t gTodayHumCount = 0;
float gTodayHumMin = NAN;
float gTodayHumMax = NAN;
#endif
#ifdef PRESS_SENSOR
float gPressurePa = NAN;
float gBucketPressSumPa = 0.0f;
float gTodayPressSum = 0.0f;
uint32_t gTodayPressCount = 0;
float gTodayPressMin = NAN;
float gTodayPressMax = NAN;
#endif
uint32_t gBucketEnvSamples = 0;

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
BucketSample gBuckets[LogConfig::BUCKETS_24H];
int gBucketWrite = 0;

static DaySummary gDays[LogConfig::DAYS_HISTORY];
static int gDayWrite = 0;
static uint32_t gDaysCount = 0;

// timing / bucket accumulators
time_t   gCurrentBucketStart = 0;

// daily rollup
time_t   gTodayMidnightEpoch = 0;


// ------------------- TIME HELPERS -------------------

time_t localMidnight(time_t nowEpoch) {
  struct tm tmLocal;
  localtime_r(&nowEpoch, &tmLocal);
  tmLocal.tm_hour = 0; tmLocal.tm_min = 0; tmLocal.tm_sec = 0; tmLocal.tm_isdst = -1;
  return mktime(&tmLocal);
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

String buildApiNow(void) {
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
  return out;
}

String buildApiConfig() {
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
  return out;
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

void pushBucketSample(const BucketSample& b) {
  gBuckets[gBucketWrite] = b;
  gBucketWrite = (gBucketWrite + 1) % LogConfig::BUCKETS_24H;
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

void clearTodayAggregates(void) {
#ifdef WIND_SENSOR
  gTodaySumOfBucketAvgs = 0.0f;
  gTodayBucketCount = 0;
  gTodayMax = 0.0f;
#endif
#ifdef TEMP_SENSOR
  gTodayTempSum = 0.0f;
  gTodayTempCount = 0;
  gTodayTempMin = NAN;
  gTodayTempMax = NAN;
#endif
#ifdef HUMID_SENSOR
  gTodayHumSum = 0.0f;
  gTodayHumCount = 0;
  gTodayHumMin = NAN;
  gTodayHumMax = NAN;
#endif
#ifdef PRESS_SENSOR
  gTodayPressSum = 0.0f;
  gTodayPressCount = 0;
  gTodayPressMin = NAN;
  gTodayPressMax = NAN;
#endif
#ifdef AIR_QUAL_SENSOR
  clearAirQualToday();
#endif
}

void deleteOldDailyFileIfNeeded(time_t todayMidnightLocal) {
  // If LogConfig::RETENTION_DAYS is 0, never delete files
  if (LogConfig::RETENTION_DAYS == 0) return;

  time_t oldMidnight = subtractDaysLocalMidnight(todayMidnightLocal, LogConfig::RETENTION_DAYS + 1);
  String ymd = ymdString(oldMidnight);
  String dataPath = String("/data/") + ymd + ".csv";
  if (gSdOk && LittleFS.exists(dataPath.c_str())) LittleFS.remove(dataPath.c_str());
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

String buildApiDays(void) {
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
  return out;
}

BucketSample currentBucketSnapshot(void) {
  BucketSample b{};
  if (!timeIsValid(gCurrentBucketStart)) return b;
  computeBucketSample(b, gCurrentBucketStart);
  return b;
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

bool loadDaySummariesCache(void) {
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

// ------------------- LOOP HELPERS -------------------
void finalizeCurrentBucket(time_t bucketStart) {
  BucketSample b{};
  computeBucketSample(b, bucketStart);
  pushBucketSample(b);
  accumulateTodayFromBucket(b);
  logBucketToSD(b);
}

void startBucketAt(time_t bucketStart) {
  gCurrentBucketStart = bucketStart;
#ifdef WIND_SENSOR
  clearWindSamples();
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

void initDataLog(void) {
  memset(gBuckets, 0, sizeof(gBuckets));
  memset(gDays, 0, sizeof(gDays));
  gBucketWrite = 0;
  gDayWrite = 0;
  gDaysCount = 0;
}

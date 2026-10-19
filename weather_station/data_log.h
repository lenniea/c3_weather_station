#ifndef __DATA_LOG_H__
#define __DATA_LOG_H__

#include <time.h>
#include <WString.h>
#include "config.h"

extern bool gSdOk;

// ==================== DATA LOGGING CONFIGURATION ====================

// Data Logging
namespace LogConfig {
  static constexpr int BUCKET_SECONDS = 60;             // Log interval (1 minute)
  static constexpr int BUCKETS_24H = 24 * 60 * 60 / BUCKET_SECONDS;
  static constexpr int RETENTION_DAYS = 0;           // 0 = never delete
  static constexpr int DAYS_HISTORY = 30;               // RAM history
  static_assert((86400 % BUCKET_SECONDS) == 0, "BUCKET_SECONDS must divide evenly into 24h");
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

//
// Exported global variables
//
extern bool gEnvOk;
extern bool gSdOk;

extern uint32_t gBucketEnvSamples;

extern BucketSample gBuckets[];
extern int gBucketWrite;

extern time_t   gCurrentBucketStart;
extern time_t   gTodayMidnightEpoch;

String ymdString(time_t t);

inline bool timeIsValid(time_t t) { return t > 1577836800; } // > 2020-01-01
inline time_t epochNow() { return time(nullptr); }

void accumulateTodayFromBucket(const BucketSample& b);
void deleteOldDailyFileIfNeeded(time_t todayMidnightLocal);
void loadRecentBucketsFromSD(time_t nowEpoch);
void maybeRolloverDay(time_t nowEpoch);
void saveDaySummariesCache(const DaySummary* curDay, bool hasCurDay);
void loadDaySummariesFromSD(time_t nowEpoch);
bool loadDaySummariesCache(void);
void rebuildTodayAggregates(void);
void processBucketCatchup(time_t nowE);

String buildApiNow(void);
String buildApiDays(void);
BucketSample currentBucketSnapshot(void);
String buildApiConfig(void);

time_t localMidnight(time_t nowEpoch);
time_t floorToBucketBoundaryLocal(time_t nowEpoch);
time_t subtractDaysLocalMidnight(time_t tMidnightLocal, int days);

void finalizeCurrentBucket(time_t bucketStart);
void startBucketAt(time_t bucketStart);

void initDataLog(void);


#endif // __DATA_LOG_H__

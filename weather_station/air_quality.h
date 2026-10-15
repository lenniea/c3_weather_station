#ifndef __AIR_QUALITY_H__
#define __AIR_QUALITY_H__
//
//  Constants for this module
//
const int PMS_FRAME_SIZE = 32;
const uint8_t PMS_HEADER_1 = 0x42;
const uint8_t PMS_HEADER_2 = 0x4d;
//
//  Global Variables exported by this module
//
extern float    gBucketPM1Sum;
extern float    gBucketPM25Sum;
extern float    gBucketPM10Sum;
extern uint32_t gBucketPmSamples;
extern float    gTodayPM1Sum;
extern uint32_t gTodayPM1Count;
extern float    gTodayPM1Max;
extern float    gTodayPM25Sum;
extern uint32_t gTodayPM25Count;
extern float    gTodayPM25Max;
extern float    gTodayPM10Sum;
extern uint32_t gTodayPM10Count;
extern float    gTodayPM10Max;

extern uint8_t gPmsFrameBuffer[32];
extern int gPmsFrameIndex;
extern bool gPmsOk;
extern float gPM1;
extern float gPM25;
extern float gPM10;

void clearAirQualToday(void);
void updateAirQuality(float avgPM1, float avgPM25, float avgPM10);

int calculateAQI_PM25(float pm25);
int calculateAQI_PM10(float pm10);
const char* getAQICategory(int aqi);
int calculateAQI_PM25(float pm25);
const char* getAQICategory(int aqi);

void resetAirQualify(void);
void pollAirQualityIfNeeded(uint32_t msNow);

#endif // __AIR_QUALITY_H__

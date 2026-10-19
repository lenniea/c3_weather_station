#ifndef __ENV_SENSOR_H__
#define __ENV_SENSOR_H__

#include <stdint.h>
#include <WString.h>
#include "config.h"

#ifdef TEMP_SENSOR
extern float gTempC;
extern float gBucketTempSum;
extern float gTodayTempSum;
extern uint32_t gTodayTempCount;
extern float gTodayTempMin;
extern float gTodayTempMax;
String getEnvTemp(int units);
#endif

#ifdef HUMID_SENSOR
extern float gHumRH;
extern float    gBucketHumSum;

extern float    gTodayHumSum;
extern uint32_t gTodayHumCount;
extern float    gTodayHumMin;
extern float    gTodayHumMax;
String getEnvHumidity(int places);
#endif

#ifdef PRESS_SENSOR
extern float gPressurePa;
extern float gBucketPressSumPa;
extern float    gTodayPressSum;
extern uint32_t gTodayPressCount;
extern float    gTodayPressMin;
extern float    gTodayPressMax;
String getEnvPressure(int places);
#endif

extern uint32_t gBucketEnvSamples;

void clearEnvToday(void);
void updateEnvMinMax(float avgTempC, float avgHumRH, float avgPressHpa);

float convertToMSLP(float stationPressurePa, float tempC, float altitudeM);

void clearEnvSamples(void);
void pollEnv(uint32_t msNow);
bool pollEnvIfNeeded(uint32_t msNow);
bool initEnvSensors(uint32_t msNow);

#endif

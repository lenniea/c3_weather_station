#ifndef __ENV_SENSOR_H__
#define __ENV_SENSOR_H__

#include <stdint.h>

// Exported global variables
#ifdef TEMP_SENSOR
extern float gTempC;
extern float gBucketTempSum;
extern float gTodayTempSum;
extern uint32_t gTodayTempCount;
extern float gTodayTempMin;
extern float gTodayTempMax;

#endif
#ifdef HUMID_SENSOR
extern float gHumRH;
extern float    gBucketHumSum;

extern float    gTodayHumSum;
extern uint32_t gTodayHumCount;
extern float    gTodayHumMin;
extern float    gTodayHumMax;

#endif
#ifdef PRESS_SENSOR
extern float gPressurePa;
extern float gBucketPressSumPa;
extern float    gTodayPressSum;
extern uint32_t gTodayPressCount;
extern float    gTodayPressMin;
extern float    gTodayPressMax;
#endif
extern uint32_t gBucketEnvSamples;

void clearEnvToday(void);
void updateEnvMinMax(float avgTempC, float avgHumRH, float avgPressHpa);

float convertToMSLP(float stationPressurePa, float tempC, float altitudeM);

void resetEnvSamples(void);
void pollEnv(uint32_t msNow);
void pollEnvIfNeeded(uint32_t msNow);
bool initEnvSensors(uint32_t msNow);

#endif

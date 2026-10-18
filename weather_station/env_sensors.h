#ifndef __ENV_SENSOR_H__
#define __ENV_SENSOR_H__

#include <stdint.h>


void clearEnvToday(void);
void updateEnvMinMax(float avgTempC, float avgHumRH, float avgPressHpa);

float convertToMSLP(float stationPressurePa, float tempC, float altitudeM);

void clearEnvSamples(void);
void pollEnv(uint32_t msNow);
void pollEnvIfNeeded(uint32_t msNow);
bool initEnvSensors(uint32_t msNow);

#endif

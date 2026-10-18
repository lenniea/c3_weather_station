#ifndef __WIND_SENSOR_H__
#define __WIND_SENSOR_H__

#include <stdint.h>

// wind pulses
extern volatile uint32_t gPulseCount;
extern volatile uint32_t gLastPulseMicros;
extern float gNowWindMS;

void clearWindToday(void);
void clearWindSamples(void);
void updateWind(float avgWind, float maxWind);
float calcWindPps(void);
float computeWind(void);
float computeMaxWind(float avgWindMs);
void updateWindPPS(uint32_t msNow);
void setupWindPin(void);
void initWindSensor(void);

#endif

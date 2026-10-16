#ifndef __WIND_SENSOR_H__
#define __WIND_SENSOR_H__

#include <stdint.h>

extern float    gBucketWindSum;
extern float    gBucketWindMax1; // highest observed in bucket
extern float    gBucketWindMax2; // second-highest observed in bucket
extern uint32_t gBucketSamples;
extern uint32_t gBucketPulseCount;
extern uint32_t gBucketPulseElapsedMs;

extern uint32_t gTodayBucketCount;
extern float    gTodayMax;

extern uint32_t gLastPpsMillis;
extern float    gTodaySumOfBucketAvgs;

// wind pulses
extern volatile uint32_t gPulseCount;
extern volatile uint32_t gLastPulseMicros;
extern float gNowWindMS;

void clearWindToday(void);
void updateWind(float avgWind, float maxWind);
float calcWindPps(void);
float computeWind(void);
float computeMaxWind(float avgWindMs);
void updateWindPPS(uint32_t msNow);
void setupWindPin(void);
void initWindSensor(void);

#endif

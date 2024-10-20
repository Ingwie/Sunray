// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATS_H
#define STATS_H

#include <inttypes.h>

extern uint32_t statIdleDuration; // seconds
extern uint32_t statChargeDuration; // seconds
extern uint32_t statMowDuration ; // seconds
extern uint32_t statMowDurationInvalid ; // seconds
extern uint32_t statMowDurationFloat ; // seconds
extern uint32_t statMowDurationFix ; // seconds
extern uint32_t statMowDurationMotorRecovery ; // seconds
extern uint32_t statMowFloatToFixRecoveries ; // counter
extern uint32_t statMowInvalidRecoveries ; // counter
extern uint32_t statImuRecoveries ; // counter
extern uint32_t statMowObstacles ; // counter
extern uint32_t statGPSJumps ; // counter
extern uint32_t statMowGPSMotionTimeoutCounter;
extern uint32_t statMowBumperCounter;
extern uint32_t statMowSonarCounter;
extern uint32_t statMowLiftCounter;
extern float statMowMaxDgpsAge ; // seconds
extern float statMowDistanceTraveled ; // meter
extern float statTempMin;
extern float statTempMax;

void calcStats();

#endif



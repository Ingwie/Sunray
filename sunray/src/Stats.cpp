// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "Stats.h"
#include "robot.h"
#include "motor.h"


uint32_t statIdleDuration = 0; // seconds
uint32_t statChargeDuration = 0; // seconds
uint32_t statMowDurationMotorRecovery = 0; // seconds
uint32_t statMowDurationInvalid = 0; // seconds
uint32_t statMowDuration = 0; // seconds
uint32_t statMowDurationFloat = 0; // seconds
uint32_t statMowDurationFix = 0; // seconds
uint32_t statMowFloatToFixRecoveries = 0; // counter
uint32_t statMowInvalidRecoveries = 0; // counter
uint32_t statImuRecoveries = 0; // counter
uint32_t statMowObstacles = 0 ; // counter
uint32_t statMowBumperCounter = 0;
uint32_t statMowSonarCounter = 0;
uint32_t statMowLiftCounter = 0;
uint32_t statMowGPSMotionTimeoutCounter = 0;
uint32_t statGPSJumps = 0; // counter
float statTempMin = 9999;
float statTempMax = -9999;
float statMowMaxDgpsAge = 0; // seconds
float statMowDistanceTraveled = 0; // meter


uint32_t nextStatTime = 0;
SolType lastSolution = SOL_INVALID;



// calculate statistics
void calcStats()
{
  if (millis() >= nextStatTime)
    {
      nextStatTime = millis() + 1000;
      switch (stateOp)
        {
        case OP_IDLE:
          statIdleDuration++;
          break;
        case OP_MOW:
          statMowDuration++;
          if (motor.motorRecoveryState) statMowDurationMotorRecovery++;
          if (gps.solution == SOL_FIXED) statMowDurationFix++;
          else if (gps.solution == SOL_FLOAT) statMowDurationFloat++;
          else if (gps.solution == SOL_INVALID) statMowDurationInvalid++;
          if (gps.solution != lastSolution)
            {
              if ((lastSolution == SOL_FLOAT) && (gps.solution == SOL_FIXED)) statMowFloatToFixRecoveries++;
              if (lastSolution == SOL_INVALID) statMowInvalidRecoveries++;
              lastSolution = gps.solution;
            }
          statMowMaxDgpsAge = MAX(statMowMaxDgpsAge, (millis() - gps.dgpsAge)/1000.0);
          break;
        case OP_CHARGE:
          statChargeDuration++;
          break;
        default:
          break;

        }
    }
}



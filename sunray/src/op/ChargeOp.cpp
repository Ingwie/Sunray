// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include "../robot.h"
#include "../StateEstimator.h"
#include "../map.h"


String ChargeOp::name()
{
  return "Charge";
}


void ChargeOp::begin()
{
  nextConsoleDetailsTime = 0;
  retryTouchDock = false;
  betterTouchDock = false;
  wxLogMessage("OP_CHARGE");
  wxLogMessage(" dockOp.initiatedByOperator= %b dockReasonRainTriggered=%b", dockOp.initiatedByOperator, dockOp.dockReasonRainTriggered);

  //motor.stopImmediately(true); // do not use PID to get to stop
  motor.setLinearAngularSpeed(0,0, false);
  motor.setMowState(false);
  //motor.enableTractionMotors(false); // keep traction motors off (motor drivers tend to generate some incorrect encoder values when stopped while not turning)
}


void ChargeOp::end()
{
}

void ChargeOp::run()
{

  if ((retryTouchDock) || (betterTouchDock))
    {
      if (millis() > retryTouchDockSpeedTime)
        {
          retryTouchDockSpeedTime = millis() + 1000;
          motor.enableTractionMotors(true); // allow traction motors to operate
          motor.setLinearAngularSpeed(0.05, 0);
        }
      if (retryTouchDock)
        {
          if (millis() > retryTouchDockStopTime)
            {
              motor.setLinearAngularSpeed(0, 0);
              retryTouchDock = false;
              wxLogMessage("ChargeOp: retryTouchDock failed");
              motor.enableTractionMotors(true); // allow traction motors to operate
              maps.setIsDocked(false);
              changeOp(idleOp);
            }
        }
      else if (betterTouchDock)
        {
          if (millis() > betterTouchDockStopTime)
            {
              wxLogMessage("ChargeOp: betterTouchDock completed");
              motor.setLinearAngularSpeed(0, 0);
              betterTouchDock = false;
            }
        }
    }

  battery.resetIdle();
  if (battery.chargerConnected())
    {
      //wxLogMessage("Op::onChargerConnected");
      maps.setIsDocked(true);
      // get robot position and yaw from docking pos
      // sensing charging contacts means we are in docking station - we use docking point coordinates to get rid of false fix positions in
      // docking station
      maps.getDockingPos(stateX, stateY, stateDelta);
      // get robot yaw orientation from map
      //float tempX;
      //float tempY;
      //maps.setRobotStatePosToDockingPos(tempX, tempY, stateDelta);
      if (battery.chargingHasCompleted())
        {
          if (millis() > nextConsoleDetailsTime)
            {
              nextConsoleDetailsTime = millis() + 30000;
              wxLogMessage("ChargeOp: charging completed (DOCKING_STATION=%b, battery.isDocked=%b, \
                dockOp.initiatedByOperator=%b, maps.mowPointsIdx=%i, DOCK_AUTO_START=%b, dockOp.dockReasonRainTriggered=%b, \
                dockOp.dockReasonRainAutoStartTime(min remain)=%i, timetable.mowingCompletedInCurrentTimeFrame=%b, \
                timetable.mowingAllowed=%b, finishAndRestart=%b )",DOCKING_STATION, battery.isDocked(), dockOp.initiatedByOperator, \
                           maps.mowPointsIdx, DOCK_AUTO_START, dockOp.dockReasonRainTriggered, ((int16_t)(dockOp.dockReasonRainAutoStartTime - millis())) / 60000, \
                           timetable.mowingCompletedInCurrentTimeFrame, timetable.mowingAllowed(), finishAndRestart);
            }
          if (timetable.shouldAutostartNow())
            {
              wxLogMessage("DOCK_AUTO_START: will automatically continue mowing now");
              changeOp(mowOp); // continue mowing
            }
        }
    }
}

void ChargeOp::onTimetableStopMowing()
{
}

void ChargeOp::onTimetableStartMowing()
{
}

void ChargeOp::onChargerDisconnected()
{
  if ((DOCKING_STATION) && (DOCK_RETRY_TOUCH))
    {
      wxLogMessage("ChargeOp::onChargerDisconnected - retryTouchDock");
      retryTouchDock = true;
      retryTouchDockStopTime = millis() + 5000;
      retryTouchDockSpeedTime = millis();
    }
  else
    {
      motor.enableTractionMotors(true); // allow traction motors to operate
      maps.setIsDocked(false);
      changeOp(idleOp);
    }
}

void ChargeOp::onBadChargingContactDetected()
{
  if ((DOCKING_STATION) && (DOCK_RETRY_TOUCH))
    {
      wxLogMessage("ChargeOp::onBadChargingContactDetected - betterTouchDock");
      betterTouchDock = true;
      betterTouchDockStopTime = millis() + 5000;
      retryTouchDockSpeedTime = millis();
    }
}

void ChargeOp::onChargerConnected()
{
  if (retryTouchDock)
    {
      wxLogMessage("ChargeOp: retryTouchDock succeeded");
      motor.setLinearAngularSpeed(0, 0);
      retryTouchDock = false;
    }
}

void ChargeOp::onBatteryUndervoltage()
{
  stateSensor = SENS_BAT_UNDERVOLTAGE;
}

void ChargeOp::onRainTriggered()
{
  if (DOCKING_STATION)
    {
      dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 60; // ensure rain sensor is dry for one hour
      //wxLogMessage("RAIN TRIGGERED dockOp.dockReasonRainAutoStartTime=");
      //wxLogMessage(dockOp.dockReasonRainAutoStartTime);
    }
}

// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "Storage.h"
#include "StateEstimator.h"
#include "robot.h"
#include "map.h"
#include "config.h"
#include "reset.h"


wxFile stateFile;
double stateCRC = 0;


double calcStateCRC()
{
  return (stateOp *10 + maps.mowPointsIdx + maps.dockPointsIdx + maps.freePointsIdx + ((int8_t)maps.wayMode)
          + sonar.enabled + fixTimeout + setSpeed + ((int8_t)sonar.enabled)
          + ((int8_t)absolutePosSource) + absolutePosSourceLon + absolutePosSourceLat + motor.pwmMaxMow
          + ((int8_t)finishAndRestart) + ((int8_t)motor.motorMowForwardSet) + ((int8_t)battery.docked)
          + timetable.crc() );
}


void dumpState()
{
  wxLogMessage("dumpState: X=%f Y=%f delta=%f mapCRC=%i mowPointsIdx=%i dockPointsIdx=%i freePointsIdx=%i", \
               stateX, stateY, stateDelta, maps.mapCRC, maps.mowPointsIdx, maps.dockPointsIdx, maps.freePointsIdx);
  wxLogMessage("wayMode=%i op=%i sensor=%i sonar.enabled=%b fixTimeout=%i absolutePosSource=%b", \
               (int8_t)maps.wayMode, (int8_t)stateOp, (int8_t)stateSensor, sonar.enabled, fixTimeout, absolutePosSource);
  wxLogMessage("lon= %lf  lat= %lf pwmMaxMow=%i finishAndRestart=%b motorMowForwardSet=%b", \
               absolutePosSourceLon, absolutePosSourceLat, motor.pwmMaxMow, finishAndRestart, motor.motorMowForwardSet);
}

void updateStateOpText()
{
  switch (stateOp)
    {
    case OP_IDLE:
      stateOpText = "idle";
      break;
    case OP_MOW:
      stateOpText = "mow";
      break;
    case OP_CHARGE:
      stateOpText = "charge";
      break;
    case OP_ERROR:
      stateOpText = "error (";
      switch (stateSensor)
        {
        case SENS_NONE:
          stateOpText += "none)";
          break;
        case SENS_BAT_UNDERVOLTAGE:
          stateOpText += "unvervoltage)";
          break;
        case SENS_OBSTACLE:
          stateOpText += "obstacle)";
          break;
        case SENS_GPS_FIX_TIMEOUT:
          stateOpText += "fix timeout)";
          break;
        case SENS_IMU_TIMEOUT:
          stateOpText += "imu timeout)";
          break;
        case SENS_IMU_TILT:
          stateOpText += "imu tilt)";
          break;
        case SENS_KIDNAPPED:
          stateOpText += "kidnapped)";
          break;
        case SENS_OVERLOAD:
          stateOpText += "overload)";
          break;
        case SENS_MOTOR_ERROR:
          stateOpText += "motor error)";
          break;
        case SENS_GPS_INVALID:
          stateOpText += "gps invalid)";
          break;
        case SENS_ODOMETRY_ERROR:
          stateOpText += "odo error)";
          break;
        case SENS_MAP_NO_ROUTE:
          stateOpText += "no map route)";
          break;
        case SENS_MEM_OVERFLOW:
          stateOpText += "mem overflow)";
          break;
        case SENS_BUMPER:
          stateOpText += "bumper)";
          break;
        case SENS_SONAR:
          stateOpText += "sonar)";
          break;
        case SENS_LIFT:
          stateOpText += "lift)";
          break;
        case SENS_RAIN:
          stateOpText += "rain)";
          break;
        case SENS_STOP_BUTTON:
          stateOpText += "stop button)";
          break;
        default:
          stateOpText += "unknown)";
          break;
        }
      break;
    case OP_DOCK:
      stateOpText = "dock";
      break;
    default:
      stateOpText = "unknown";
      break;
    }
  switch (gps.solution)
    {
    case SOL_INVALID:
      gpsSolText = "invalid";
      break;
    case SOL_FLOAT:
      gpsSolText = "float";
      break;
    case SOL_FIXED:
      gpsSolText ="fixed";
      break;
    default:
      gpsSolText = "unknown";
    }
}


bool loadState()
{
#if defined(ENABLE_SD_RESUME)
  bool res = true;
  wxLogMessage("resuming is activated");
  wxLogMessage("state load... ");
  wxString statefilename = AppPath + "/state.bin";
  if (!(wxFile::Exists(statefilename)))
    {
      wxLogMessage("no state file!");
      return false;
    }
  if (!stateFile.Create(statefilename, true, wxS_DEFAULT))
    {
      wxLogMessage("ERROR opening file for reading");
      return false;
    }
  uint32_t marker = 0;
  WXFILEREAD(stateFile, marker, res);
  if ((marker != 0x10001007) || (!res))
    {
      wxLogMessage("ERROR: invalid marker: %#x", marker);
      return false;
    }
  int32_t crc = 0;
  WXFILEREAD(stateFile, crc, res);
  if ((crc != maps.mapCRC) || (!res))
    {
      wxLogMessage("ERROR: non-matching map CRC: %#x Expected %#x", crc, maps.mapCRC);
      return false;
    }
  OperationType savedOp;
  WXFILEREAD(stateFile, stateX, res);
  WXFILEREAD(stateFile, stateY, res);
  WXFILEREAD(stateFile, stateDelta, res);
  WXFILEREAD(stateFile, maps.mowPointsIdx, res);
  WXFILEREAD(stateFile, maps.dockPointsIdx, res);
  WXFILEREAD(stateFile, maps.freePointsIdx, res);
  WXFILEREAD(stateFile, maps.wayMode, res);
  WXFILEREAD(stateFile, savedOp, res);
  WXFILEREAD(stateFile, stateSensor, res);
  WXFILEREAD(stateFile, sonar.enabled, res);
  WXFILEREAD(stateFile, fixTimeout, res);
  WXFILEREAD(stateFile, setSpeed, res);
  WXFILEREAD(stateFile, absolutePosSource, res);
  WXFILEREAD(stateFile, absolutePosSourceLon, res);
  WXFILEREAD(stateFile, absolutePosSourceLat, res);
  WXFILEREAD(stateFile, motor.pwmMaxMow, res);
  WXFILEREAD(stateFile, finishAndRestart, res);
  WXFILEREAD(stateFile, motor.motorMowForwardSet, res);
  WXFILEREAD(stateFile, timetable.timetable, res);
  WXFILEREAD(stateFile, battery.docked, res);
  stateFile.Close();
  wxLogMessage("ok");
  stateCRC = calcStateCRC();
  dumpState();
  timetable.dump();
  if (getResetCause() == RST_WATCHDOG)
    {
      wxLogMessage("resuming operation due to watchdog trigger");
      stateOp = savedOp;
      setOperation(stateOp, true);
    }
#endif
  return true;
}


bool saveState()
{
  bool res = true;
#if defined(ENABLE_SD_RESUME)
  double crc = calcStateCRC();
  //wxLogMessage("stateCRC=");
  //wxLogMessage(stateCRC);
  //wxLogMessage(" crc=");
  //wxLogMessage(crc);
  if (crc == stateCRC) return true;
  stateCRC = crc;
  dumpState();
  wxLogMessage("save state... ");
  wxString statefilename = AppPath + "/state.bin";
  if (!stateFile.Create(statefilename, true, wxS_DEFAULT))
    {
      wxLogMessage("ERROR opening file for writing");
      return false;
    }
  uint32_t marker = 0x10001007;
  WXFILEWRITE(stateFile, marker, res);
  WXFILEWRITE(stateFile, maps.mapCRC, res);
  WXFILEWRITE(stateFile, stateX, res);
  WXFILEWRITE(stateFile, stateY, res);
  WXFILEWRITE(stateFile, stateDelta, res);
  WXFILEWRITE(stateFile, maps.mowPointsIdx, res);
  WXFILEWRITE(stateFile, maps.dockPointsIdx, res);
  WXFILEWRITE(stateFile, maps.freePointsIdx, res);
  WXFILEWRITE(stateFile, maps.wayMode, res);
  WXFILEWRITE(stateFile, stateOp, res);
  WXFILEWRITE(stateFile, stateSensor, res);
  WXFILEWRITE(stateFile, sonar.enabled, res);
  WXFILEWRITE(stateFile, fixTimeout, res);
  WXFILEWRITE(stateFile, setSpeed, res);
  WXFILEWRITE(stateFile, absolutePosSource, res);
  WXFILEWRITE(stateFile, absolutePosSourceLon, res);
  WXFILEWRITE(stateFile, absolutePosSourceLat, res);
  WXFILEWRITE(stateFile, motor.pwmMaxMow, res);
  WXFILEWRITE(stateFile, finishAndRestart, res);
  WXFILEWRITE(stateFile, motor.motorMowForwardSet, res);
  WXFILEWRITE(stateFile, timetable.timetable, res);
  WXFILEWRITE(stateFile, battery.docked, res);
  if (res)
    {
      wxLogMessage("ok");
    }
  else
    {
      wxLogMessage("ERROR saving state");
    }
  stateFile.Flush();
  stateFile.Close();
#endif
  return res;
}




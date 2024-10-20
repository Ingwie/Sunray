#include "comm.h"
#include "config.h"
#include "robot.h"
#include "StateEstimator.h"
#include "LineTracker.h"
#include "Stats.h"
#include "op/op.h"
#include "reset.h"
#include "mqtt.h"
#include "httpserver.h"
#include "pgmspace.h"
#include "BridgeClient.h"

#include "WiFi.h"
#include "sys/resource.h"
#include "timetable.h"
#include "Storage.h"


//#define VERBOSE 1

uint32_t nextInfoTime = 0;
bool triggerWatchdog = false;

int16_t encryptMode = 0; // 0=off, 1=encrypt
uint32_t encryptPass = PASS;
int16_t encryptChallenge = 0;
int16_t encryptKey = 0;

bool simFaultyConn = false; // simulate a faulty connection?
int16_t simFaultConnCounter = 0;

String cmd;
String cmdResponse;

float statControlCycleTime = 0;
float statMaxControlCycleTime = 0;


// answer Bluetooth with CRC
void cmdAnswer(String s)
{
  uint8_t crc = 0;
  for (int16_t i=0; i < s.length(); i++) crc += s[i];
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);
  s += F("\r\n");
  //CONSOLE.print(s);
  cmdResponse = s;
}

// request tune param
void cmdTuneParam()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t paramIdx = -1;
  int16_t lastCommaIdx = 0;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              paramIdx = floatValue;
            }
          else if (counter == 2)
            {
              wxLogMessage("tuneParam %i = %f", paramIdx, floatValue);
              switch (paramIdx)
                {
                case 0:
                  stanleyTrackingNormalP = floatValue;
                  break;
                case 1:
                  stanleyTrackingNormalK = floatValue;
                  break;
                case 2:
                  stanleyTrackingSlowP = floatValue;
                  break;
                case 3:
                  stanleyTrackingSlowK = floatValue;
                  break;
                }
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  String s = F("CT");
  cmdAnswer(s);
}

// request operation
void cmdControl()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  int16_t op = -1;
  bool restartRobot = false;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          int16_t intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
          float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              if (intValue >= 0)
                {
                  motor.enableMowMotor = (intValue == 1);
                  motor.setMowState( (intValue == 1) );
                }
            }
          else if (counter == 2)
            {
              if (intValue >= 0) op = intValue;
            }
          else if (counter == 3)
            {
              if (floatValue >= 0) setSpeed = floatValue;
            }
          else if (counter == 4)
            {
              if (intValue >= 0) fixTimeout = intValue;
            }
          else if (counter == 5)
            {
              if (intValue >= 0) finishAndRestart = (intValue == 1);
            }
          else if (counter == 6)
            {
              if (floatValue >= 0)
                {
                  maps.setMowingPointPercent(floatValue);
                  restartRobot = true;
                }
            }
          else if (counter == 7)
            {
              if (intValue > 0)
                {
                  maps.skipNextMowingPoint();
                  restartRobot = true;
                }
            }
          else if (counter == 8)
            {
              if (intValue >= 0) sonar.enabled = (intValue == 1);
            }
          else if (counter == 9)
            {
              if (intValue >= 0) motor.setMowMaxPwm(intValue);
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  /*CONSOLE.print("linear=");
  CONSOLE.print(linear);
  CONSOLE.print(" angular=");
  wxLogMessage(angular);*/
  OperationType oldStateOp = stateOp;
  if (restartRobot)
    {
      // certain operations may require a start from IDLE state (https://github.com/Ardumower/Sunray/issues/66)
      setOperation(OP_IDLE);
    }
  if (op >= 0) setOperation((OperationType)op, false); // new operation by operator
  else if (restartRobot)      // no operation given by operator, continue current operation from IDLE state
    {
      setOperation(oldStateOp);
    }
  String s = F("C");
  cmdAnswer(s);
}

// request motor
void cmdMotor()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  float linear=0;
  float angular=0;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          float value = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              linear = value;
            }
          else if (counter == 2)
            {
              angular = value;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  /*CONSOLE.print("linear=");
  CONSOLE.print(linear);
  CONSOLE.print(" angular=");
  wxLogMessage(angular);*/
  motor.setLinearAngularSpeed(linear, angular, false);
  String s = F("M");
  cmdAnswer(s);
}

void cmdMotorTest()
{
  String s = F("E");
  cmdAnswer(s);
  motor.test();
}

void cmdMotorPlot()
{
  String s = F("Q");
  cmdAnswer(s);
  motor.plot();
}

void cmdSensorTest()
{
  String s = F("F");
  cmdAnswer(s);
  sensorTest();
}

// request timetable (mowing allowed day masks for 24 UTC hours)
// TT,enable,daymask,daymask,daymask,daymask,daymask,...
// TT,1,0,0,0,0,0,0,0,0,0,0,127,127,127,127,127,127,127,127,127,0,0,0,0,0
// NOTE: protocol for this command will change in near future (please do not assume this a final implementation)
void cmdTimetable()
{
  if (cmd.length()<6) return;
  //wxLogMessage(cmd);
  int16_t lastCommaIdx = 0;
  bool success = true;
  timetable.clear();
  int16_t counter = 0;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          int16_t intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
          //float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              timetable.setEnabled(intValue == 1);
            }
          else if (counter > 1)
            {
              daymask_t daymask = intValue;
              if (!timetable.setDayMask(counter-2, daymask))
                {
                  success = false;
                  break;
                }
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  timetable.dump();
  String s = F("TT");
  cmdAnswer(s);

  if (!success)
    {
      stateSensor = SENS_MEM_OVERFLOW;
      setOperation(OP_ERROR);
    }
  else
    {
      saveState();
    }
}

// request waypoint (perim,excl,dock,mow,free)
// W,startidx,x,y,x,y,x,y,x,y,...
void cmdWaypoint()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  int16_t widx=0;
  float x=0;
  float y=0;
  bool success = true;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          int16_t intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
          float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              widx = intValue;
            }
          else if (counter == 2)
            {
              x = floatValue;
            }
          else if (counter == 3)
            {
              y = floatValue;
              if (!maps.setPoint(widx, x, y))
                {
                  success = false;
                  break;
                }
              widx++;
              counter = 1;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  /*CONSOLE.print("waypoint (");
  CONSOLE.print(widx);
  CONSOLE.print("/");
  CONSOLE.print(count);
  CONSOLE.print(") ");
  CONSOLE.print(x);
  CONSOLE.print(",");
  wxLogMessage(y);*/

  String s = F("W,");
  s += widx;
  cmdAnswer(s);

  if (!success)
    {
      stateSensor = SENS_MEM_OVERFLOW;
      setOperation(OP_ERROR);
    }
}


// request waypoints count
// N,#peri,#excl,#dock,#mow,#free
void cmdWayCount()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          int16_t intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
          //float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              if (!maps.setWayCount(WAY_PERIMETER, intValue)) return;
            }
          else if (counter == 2)
            {
              if (!maps.setWayCount(WAY_EXCLUSION, intValue)) return;
            }
          else if (counter == 3)
            {
              if (!maps.setWayCount(WAY_DOCK, intValue)) return;
            }
          else if (counter == 4)
            {
              if (!maps.setWayCount(WAY_MOW, intValue)) return;
            }
          else if (counter == 5)
            {
              if (!maps.setWayCount(WAY_FREE, intValue)) return;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  String s = F("N");
  cmdAnswer(s);
  //maps.dump();
}


// request exclusion count
// X,startidx,cnt,cnt,cnt,cnt,...
void cmdExclusionCount()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  int16_t widx=0;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          int16_t intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
          //float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
          if (counter == 1)
            {
              widx = intValue;
            }
          else if (counter == 2)
            {
              if (!maps.setExclusionLength(widx, intValue)) return;
              widx++;
              counter = 1;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  String s = F("X,");
  s += widx;
  cmdAnswer(s);
}


// request position mode
void cmdPosMode()
{
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          int16_t intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
          double doubleValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toDouble();
          if (counter == 1)
            {
              absolutePosSource = bool(intValue);
            }
          else if (counter == 2)
            {
              absolutePosSourceLon = doubleValue;
            }
          else if (counter == 3)
            {
              absolutePosSourceLat = doubleValue;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  wxLogMessage("absolutePosSource=%b lon=%d lat=%d", absolutePosSource, absolutePosSourceLon, absolutePosSourceLat);
  String s = F("P");
  cmdAnswer(s);
}

// request version
void cmdVersion()
{
#if (ENABLE_MQTT == true) && defined(__linux__) // No ESP needed
  mqttRequestTopic |= MQTT_REQUEST_PROPS; // Poll datas -> processWifiMqttClient
  return;
#endif // ENABLE_MQTT
#ifdef ENABLE_PASS
  if (encryptMode == 0)
    {
      encryptMode = 1;
      randomSeed(millis());
      while (true)
        {
          encryptChallenge = random(0, 200); // random number between 0..199
          encryptKey = encryptPass % encryptChallenge;
          if ( (encryptKey >= 1) && (encryptKey <= 94) ) break;  // random key between 1..94
        }
    }
#endif
  String s = F("V,");
  s += F(VER);
  s += F(",");
  s += encryptMode;
  s += F(",");
  s += encryptChallenge;
  s += F(",");
  s += BOARD;
  s += F(",");
#ifdef DRV_SERIAL_ROBOT
  s += "SR";
#elif DRV_ARDUMOWER
  s += "AM";
#else
  s += "XX";
#endif
  String id = "";
  String mcuFwName = "";
  String mcuFwVer = "";
  robotDriver.getRobotID(id);
  robotDriver.getMcuFirmwareVersion(mcuFwName, mcuFwVer);
  s += F(",");
  s += mcuFwName;
  s += F(",");
  s += mcuFwVer;
  s += F(",");
  s += id;
  wxLogMessage("sending encryptMode=%i encryptChallenge=%i", encryptMode, encryptChallenge);
  cmdAnswer(s);
}

// request add obstacle
void cmdObstacle()
{
  String s = F("O");
  cmdAnswer(s);
  triggerObstacle();
}

// request rain
void cmdRain()
{
  String s = F("O2");
  cmdAnswer(s);
  activeOp->onRainTriggered();
}

// request battery low
void cmdBatteryLow()
{
  String s = F("O3");
  cmdAnswer(s);
  activeOp->onBatteryLowShouldDock();
}

// perform pathfinder stress test
void cmdStressTest()
{
  String s = F("Z");
  cmdAnswer(s);
  maps.stressTest();
}

// perform hang test (watchdog should trigger and restart robot)
void cmdTriggerWatchdog()
{
  String s = F("Y");
  cmdAnswer(s);
  setOperation(OP_IDLE);
  wxExecute("reboot", wxEXEC_BLOCK);
}

// perform hang test (watchdog should trigger)
void cmdGNSSReboot()
{
  String s = F("Y2");
  cmdAnswer(s);
  wxLogMessage("GNNS reboot");
  gps.reboot();
}

// switch-off robot
void cmdSwitchOffRobot()
{
  String s = F("Y3");
  cmdAnswer(s);
  setOperation(OP_IDLE);
  battery.switchOff();
}

// kidnap test (kidnap detection should trigger)
void cmdKidnap()
{
  String s = F("K");
  cmdAnswer(s);
  wxLogMessage("kidnapping robot - kidnap detection should trigger");
  stateX = 0;
  stateY = 0;
}

// toggle GPS solution (invalid,float,fix) for testing
void cmdToggleGPSSolution()
{
  String s = F("G");
  cmdAnswer(s);
  wxLogMessage("toggle GPS solution");
  switch (gps.solution)
    {
    case SOL_INVALID:
      gps.solutionAvail = true;
      gps.solution = SOL_FLOAT;
      gps.relPosN = stateY - 2.0;  // simulate pos. solution jump
      gps.relPosE = stateX - 2.0;
      lastFixTime = millis();
      stateGroundSpeed = 0.1;
      break;
    case SOL_FLOAT:
      gps.solutionAvail = true;
      gps.solution = SOL_FIXED;
      stateGroundSpeed = 0.1;
      gps.relPosN = stateY + 2.0;  // simulate undo pos. solution jump
      gps.relPosE = stateX + 2.0;
      break;
    case SOL_FIXED:
      gps.solutionAvail = true;
      gps.solution = SOL_INVALID;
      break;
    }
}


// request obstacles
void cmdObstacles()
{
  String s = F("S2,");
  s += maps.obstacles.numPolygons;
  for (int16_t idx=0; idx < maps.obstacles.numPolygons; idx++)
    {
      s += ",0.5,0.5,1,"; // red,green,blue (0-1)
      s += maps.obstacles.polygons[idx].numPoints;
      for (int16_t idx2=0 ; idx2 < maps.obstacles.polygons[idx].numPoints; idx2++)
        {
          s += ",";
          s += maps.obstacles.polygons[idx].points[idx2].x();
          s += ",";
          s += maps.obstacles.polygons[idx].points[idx2].y();
        }
    }

  cmdAnswer(s);
}

// request summary
void cmdSummary()
{
#if (ENABLE_MQTT == true) && defined(__linux__) // No ESP needed
  mqttRequestTopic |= MQTT_REQUEST_STATE; // Poll datas -> processWifiMqttClient
  return;
#endif // ENABLE_MQTT
  String s = F("S,");
  s += battery.batteryVoltage;
  s += ",";
  s += stateX;
  s += ",";
  s += stateY;
  s += ",";
  s += stateDelta;
  s += ",";
  s += gps.solution;
  s += ",";
  s += stateOp;
  s += ",";
  s += maps.mowPointsIdx;
  s += ",";
  s += (millis() - gps.dgpsAge)/1000.0;
  s += ",";
  s += stateSensor;
  s += ",";
  s += maps.targetPoint.x();
  s += ",";
  s += maps.targetPoint.y();
  s += ",";
  s += gps.accuracy;
  s += ",";
  s += gps.numSV;
  s += ",";
  if (stateOp == OP_CHARGE)
    {
      s += "-";
      s += battery.chargingCurrent;
    }
  else
    {
      s += motor.motorsSenseLP;
    }
  s += ",";
  s += gps.numSVdgps;
  s += ",";
  s += maps.mapCRC;
  s += ",";
  s += lateralError;
  s += ",";
  if (stateOp == OP_MOW)
    {
      s += timetable.autostopTime.dayOfWeek;
      s += ",";
      s += timetable.autostopTime.hour;
    }
  else if (stateOp == OP_CHARGE)
    {
      s += timetable.autostartTime.dayOfWeek;
      s += ",";
      s += timetable.autostartTime.hour;
    }
  else
    {
      s += "-1,0";
    }
  cmdAnswer(s);
}

// request statistics
void cmdStats()
{
#if (ENABLE_MQTT == true) && defined(__linux__) // No ESP needed
  mqttRequestTopic |= MQTT_REQUEST_STATS; // Poll datas -> processWifiMqttClient
  return;
#endif // ENABLE_MQTT
  String s = F("T,");
  s += statIdleDuration;
  s += ",";
  s += statChargeDuration;
  s += ",";
  s += statMowDuration;
  s += ",";
  s += statMowDurationFloat;
  s += ",";
  s += statMowDurationFix;
  s += ",";
  s += statMowFloatToFixRecoveries;
  s += ",";
  s += statMowDistanceTraveled;
  s += ",";
  s += statMowMaxDgpsAge;
  s += ",";
  s += statImuRecoveries;
  s += ",";
  s += statTempMin;
  s += ",";
  s += statTempMax;
  s += ",";
  s += gps.chksumErrorCounter;
  s += ",";
  //s += ((float)gps.dgpsChecksumErrorCounter) / ((float)(gps.dgpsPacketCounter));
  s += gps.dgpsChecksumErrorCounter;
  s += ",";
  s += statMaxControlCycleTime;
  s += ",";
  s += SERIAL_BUFFER_SIZE;
  s += ",";
  s += statMowDurationInvalid;
  s += ",";
  s += statMowInvalidRecoveries;
  s += ",";
  s += statMowObstacles;
  s += ",";
  s += freeMemory();
  s += ",";
  s += getResetCause();
  s += ",";
  s += statGPSJumps;
  s += ",";
  s += statMowSonarCounter;
  s += ",";
  s += statMowBumperCounter;
  s += ",";
  s += statMowGPSMotionTimeoutCounter;
  s += ",";
  s += statMowDurationMotorRecovery;
  cmdAnswer(s);
}

// clear statistics
void cmdClearStats()
{
  String s = F("L");
  statMowDurationMotorRecovery = 0;
  statIdleDuration = 0;
  statChargeDuration = 0;
  statMowDuration = 0;
  statMowDurationInvalid = 0;
  statMowDurationFloat = 0;
  statMowDurationFix = 0;
  statMowFloatToFixRecoveries = 0;
  statMowInvalidRecoveries = 0;
  statMowDistanceTraveled = 0;
  statMowMaxDgpsAge = 0;
  statImuRecoveries = 0;
  statTempMin = 9999;
  statTempMax = -9999;
  gps.chksumErrorCounter = 0;
  gps.dgpsChecksumErrorCounter = 0;
  statMaxControlCycleTime = 0;
  statMowObstacles = 0;
  statMowBumperCounter = 0;
  statMowSonarCounter = 0;
  statMowLiftCounter = 0;
  statMowGPSMotionTimeoutCounter = 0;
  statGPSJumps = 0;
  cmdAnswer(s);
}

// scan WiFi networks
void cmdWiFiScan()
{
  wxLogMessage("cmdWiFiScan");
  String s = F("B1,");
#ifdef __linux__
  int16_t numNetworks = WiFi.scanNetworks();
  wxLogMessage("numNetworks=%i", numNetworks);
  for (int16_t i=0; i < numNetworks; i++)
    {
      wxLogMessage("%s",WiFi.SSID(i));
      s += WiFi.SSID(i);
      if (i < numNetworks-1) s += ",";
    }
#endif
  cmdAnswer(s);
}

// setup WiFi
void cmdWiFiSetup()
{
  wxLogMessage("cmdWiFiSetup");
#ifdef __linux__
  if (cmd.length()<6) return;
  int16_t counter = 0;
  int16_t lastCommaIdx = 0;
  String ssid = "";
  String pass = "";
  for (int16_t idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          String str = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1);
          if (counter == 1)
            {
              ssid = str;
            }
          else if (counter == 2)
            {
              pass = str;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  /*CONSOLE.print("ssid=");
  CONSOLE.print(ssid);
  CONSOLE.print(" pass=");
  wxLogMessage(pass);*/
  WiFi.begin((char*)ssid.c_str(), (char*)pass.c_str());
#endif
  String s = F("B2");
  cmdAnswer(s);
}

// request WiFi status
void cmdWiFiStatus()
{
  String s = F("B3,");
#ifdef __linux__
  IPAddress addr = WiFi.localIP();
  s += addr[0];
  s += ".";
  s += addr[1];
  s += ".";
  s += addr[2];
  s += ".";
  s += addr[3];
#endif
  cmdAnswer(s);
}

// process request
void processCmd(bool checkCrc, bool decrypt)
{
  cmdResponse = "";
  if (cmd.length() < 4) return;
#if (ENABLE_MQTT == false) && defined(__linux__) // MQTT without crc
#ifdef ENABLE_PASS
  if (decrypt)
    {
      String s = cmd.substring(0,4);
      if ( s != "AT+V")
        {
          if (encryptMode == 1)
            {
              // decrypt
              for (int16_t i=0; i < cmd.length(); i++)
                {
                  if ( (uint8_t(cmd[i]) >= 32) && (uint8_t(cmd[i]) <= 126) )   // ASCII between 32..126
                    {
                      int16_t code = uint8_t(cmd[i]);
                      code -= encryptKey;
                      if (code <= 31) code = 126 + (code-31);
                      cmd[i] = char(code);
                    }
                }
#ifdef VERBOSE
              wxLogMessage"decrypt:%s", cmd);
#endif
            }
        }
    }
#endif
  uint8_t expectedCrc = 0;
  int16_t idx = cmd.lastIndexOf(',');
  if (idx < 1)
    {
      if (checkCrc)
        {
          wxLogMessage("COMM CRC ERROR: %s", cmd.c_str());
          return;
        }
    }
  else
    {
      for (int16_t i=0; i < idx; i++) expectedCrc += cmd[i];
      String s = cmd.substring(idx+1, idx+5);
      int16_t crc = strtol(s.c_str(), NULL, 16);
      bool crcErr = false;
      simFaultConnCounter++;
      if ((simFaultyConn) && (simFaultConnCounter % 10 == 0)) crcErr = true;
      if ((expectedCrc != crc) && (checkCrc)) crcErr = true;
      if (crcErr)
        {
          wxLogMessage("CRC ERROR %#02 , %#02", crc, expectedCrc);
          return;
        }
      else
        {
          // remove CRC
          cmd = cmd.substring(0, idx);
          //wxLogMessage(cmd);
        }
    }
#endif // MQTT
  if (cmd[0] != 'A') return;
  if (cmd[1] != 'T') return;
  if (cmd[2] != '+') return;
  if (cmd[3] == 'S')
    {
      if (cmd.length() <= 4)
        {
          cmdSummary();
        }
      else
        {
          if (cmd[4] == '2') cmdObstacles();
        }
    }
  if (cmd[3] == 'M') cmdMotor();
  if (cmd[3] == 'C')
    {
      if ((cmd.length() > 4) && (cmd[4] == 'T')) cmdTuneParam();
      else cmdControl();
    }
  if (cmd[3] == 'W') cmdWaypoint();
  if (cmd[3] == 'N') cmdWayCount();
  if (cmd[3] == 'X') cmdExclusionCount();
  if (cmd[3] == 'V') cmdVersion();
  if (cmd[3] == 'P') cmdPosMode();
  if (cmd[3] == 'T')
    {
      if ((cmd.length() > 4) && (cmd[4] == 'T')) cmdTimetable();
      else cmdStats();
    }
  if (cmd[3] == 'L') cmdClearStats();
  if (cmd[3] == 'E') cmdMotorTest();
  if (cmd[3] == 'Q') cmdMotorPlot();
  if (cmd[3] == 'O')
    {
      if (cmd.length() <= 4)
        {
          cmdObstacle();   // for developers
        }
      else
        {
          if (cmd[4] == '2') cmdRain();   // for developers
          if (cmd[4] == '3') cmdBatteryLow();   // for developers
        }
    }
  if (cmd[3] == 'F') cmdSensorTest();
  if (cmd[3] == 'B')
    {
      if (cmd[4] == '1') cmdWiFiScan();
      if (cmd[4] == '2') cmdWiFiSetup();
      if (cmd[4] == '3') cmdWiFiStatus();
    }
  if (cmd[3] == 'G') cmdToggleGPSSolution();   // for developers
  if (cmd[3] == 'K') cmdKidnap();   // for developers
  if (cmd[3] == 'Z') cmdStressTest();   // for developers
  if (cmd[3] == 'Y')
    {
      if (cmd.length() <= 4)
        {
          cmdTriggerWatchdog();   // for developers
        }
      else
        {
          if (cmd[4] == '2') cmdGNSSReboot();   // for developers
          if (cmd[4] == '3') cmdSwitchOffRobot();   // for developers
        }
    }
}

// process console input
void processConsole()
{
  char ch;
  /* if (CONSOLE.available()){
     battery.resetIdle();
     while ( CONSOLE.available() ){
       ch = CONSOLE.read();
       if ((ch == '\r') || (ch == '\n')) {
         wxLogMessage("CON:%s", cmd);
         processCmd(false, false);
         wxLogMessage("%s", cmdResponse);
         cmd = "";
       } else if (cmd.length() < 500){
         cmd += ch;
       }
     }
   }*/
}




void processComm()
{
  processConsole();
  processWifiAppServer();
  processWifiRelayClient();
  processWifiMqttClient();

  if (triggerWatchdog)
    {
      wxLogMessage("hang test - watchdog should trigger and perform a reset");
      while (true)
        {
          wxLogMessage("triggerWatchdog");
          // do nothing, just hang
        }
    }
}


// output summary on console
void outputConsole()
{
  //return;
  if (millis() > nextInfoTime)
    {
      bool started = (nextInfoTime == 0);
      nextInfoTime = millis() + 5000;
      uint32_t totalsecs = millis()/1000;
      uint32_t totalmins = totalsecs/60;
      uint32_t hour = totalmins/60;
      uint32_t min = totalmins % 60;
      uint32_t sec = totalsecs % 60;/* TODO
    CONSOLE.print (hour);
    CONSOLE.print (":");
    CONSOLE.print (min);
    CONSOLE.print (":");
    CONSOLE.print (sec);
    CONSOLE.print (" ctlDur=");
    //if (!imuIsCalibrating){
    if (!started){
      if (controlLoops > 0){
        statControlCycleTime = 1.0 / (((float)controlLoops)/5.0);
      } else statControlCycleTime = 5;
      statMaxControlCycleTime = MAX(statMaxControlCycleTime, statControlCycleTime);
    }
    controlLoops=0;
    CONSOLE.print (statControlCycleTime);
    CONSOLE.print (" op=");
    CONSOLE.print(activeOp->OpChain);
    //CONSOLE.print (stateOp);
    #ifdef __linux__
      CONSOLE.print (" mem=");
      struct rusage r_usage;
      getrusage(RUSAGE_SELF,&r_usage);
      CONSOLE.print((uint32_t)r_usage.ru_maxrss);
      #ifdef __arm__
        CONSOLE.print(" sp=");
        uint64_t spReg;
        asm( "mov %0, %%sp" : "=rm" ( spReg ));
        CONSOLE.print ( ((uint32_t)spReg), HEX);
      #endif
    #else
      CONSOLE.print (" freem=");
      CONSOLE.print (freeMemory());
      uint32_t *spReg = (uint32_t*)__get_MSP();   // stack pointer
      CONSOLE.print (" sp=");
      CONSOLE.print (*spReg, HEX);
    #endif
    CONSOLE.print(" bat=");
    CONSOLE.print(battery.batteryVoltage);
    CONSOLE.print(",");
    CONSOLE.print(battery.batteryVoltageSlope, 3);
    CONSOLE.print("(");
    CONSOLE.print(motor.motorsSenseLP);
    CONSOLE.print(") chg=");
    CONSOLE.print(battery.chargingVoltage);
    CONSOLE.print(",");
    CONSOLE.print(int16_t(battery.chargingHasCompleted()));
    CONSOLE.print("(");
    CONSOLE.print(battery.chargingCurrent);
    CONSOLE.print(") diff=");
    CONSOLE.print(battery.chargingVoltBatteryVoltDiff, 3);
    CONSOLE.print(" tg=");
    CONSOLE.print(maps.targetPoint.x());
    CONSOLE.print(",");
    CONSOLE.print(maps.targetPoint.y());
    CONSOLE.print(" x=");
    CONSOLE.print(stateX);
    CONSOLE.print(" y=");
    CONSOLE.print(stateY);
    CONSOLE.print(" delta=");
    CONSOLE.print(stateDelta);
    CONSOLE.print(" tow=");
    CONSOLE.print(gps.iTOW);
    CONSOLE.print(" lon=");
    CONSOLE.print(gps.lon,8);
    CONSOLE.print(" lat=");
    CONSOLE.print(gps.lat,8);
    CONSOLE.print(" h=");
    CONSOLE.print(gps.height,1);
    CONSOLE.print(" n=");
    CONSOLE.print(gps.relPosN);
    CONSOLE.print(" e=");
    CONSOLE.print(gps.relPosE);
    CONSOLE.print(" d=");
    CONSOLE.print(gps.relPosD);
    CONSOLE.print(" sol=");
    CONSOLE.print(gps.solution);
    CONSOLE.print(" age=");
    CONSOLE.print((millis()-gps.dgpsAge)/1000.0);
    wxLogMessage();
    //logCPUHealth();*/wxLogMessage("TODO outputConsole");
    }
}

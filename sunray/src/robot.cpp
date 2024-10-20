// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "WiFi.h"
#include "robot.h"
#include "StateEstimator.h"
#include "Storage.h"
#include "Stats.h"
#include "LineTracker.h"
#include "comm.h"
#include "op/op.h"
#include "BridgeClient.h"
#include "PubSubClient.h"
#include "RunningMedian.h"
#include "motor.h"
#include "driver/MeuhRobotDriver.h"
#include "battery.h"
#include "gps.h"
#include "ublox/ublox.h"
#include "helper.h"
#include "buzzer.h"
#include "rcmodel.h"
#include "map.h"
#include "config.h"
#include "reset.h"
//#include "test/test.h"
#include "bumper.h"
#include "mqtt.h"

// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))


LinuxSerial SerialWIFI(SERIAL_WIFI_PATH); // WIFI
LinuxSerial SerialGPS(SERIAL_GPS_PATH); // GPS



const signed char orientationMatrix[9] =
{
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

#ifdef DRV_SIM_ROBOT
SimImuDriver imuDriver(robotDriver);
#elif defined(DRV_MEUH_ROBOT)
MeuhImuDriver imuDriver(robotDriver);
#endif
#if defined(DRV_MEUH_ROBOT)
MeuhRobotDriver robotDriver;
MeuhMotorDriver motorDriver(robotDriver);
MeuhBatteryDriver batteryDriver(robotDriver);
MeuhBumperDriver bumperDriver(robotDriver);
MeuhStopButtonDriver stopButton(robotDriver);
MeuhRainSensorDriver rainDriver(robotDriver);
MeuhLiftSensorDriver liftDriver(robotDriver);
MeuhBuzzerDriver buzzerDriver(robotDriver);
#elif defined(DRV_SIM_ROBOT)
SimRobotDriver robotDriver;
SimMotorDriver motorDriver(robotDriver);
SimBatteryDriver batteryDriver(robotDriver);
SimBumperDriver bumperDriver(robotDriver);
SimStopButtonDriver stopButton(robotDriver);
SimRainSensorDriver rainDriver(robotDriver);
SimLiftSensorDriver liftDriver(robotDriver);
SimBuzzerDriver buzzerDriver(robotDriver);
#endif
Motor motor;
Battery battery;
//#ifdef DRV_SIM_ROBOT
//SimGpsDriver gps(robotDriver);
#ifdef TESTNOROOT
SimGpsDriver gps(robotDriver);
#else
//UBLOX gps;
#endif
Buzzer buzzer;
Bumper bumper;
Sonar sonar;
Map maps;
RCModel rcmodel;
TimeTable timetable;

int16_t stateButton = 0;
int16_t stateButtonTemp = 0;
uint32_t stateButtonTimeout = 0;

OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor

uint32_t controlLoops = 0;
String stateOpText = "";  // current operation as text
String gpsSolText = ""; // current gps solution as text
float stateTemp = 20; // degreeC
//float stateHumidity = 0; // percent
uint32_t stateInMotionLastTime = 0;
bool stateChargerConnected = false;
bool stateInMotionLP = false; // robot is in angular or linear motion? (with motion low-pass filtering)

float linearSpeedSetLast = 0;
bool linearSpeedSetDeadTimeIsSet = false;
uint32_t linearSpeedSetDeadTime = 0;

uint32_t lastFixTime = 0;
int16_t fixTimeout = 0;
bool absolutePosSource = false;
double absolutePosSourceLon = 0;
double absolutePosSourceLat = 0;
float lastGPSMotionX = 0;
float lastGPSMotionY = 0;
uint32_t nextGPSMotionCheckTime = 0;

bool finishAndRestart = false;

uint32_t nextBadChargingContactCheck = 0;
uint32_t nextToFTime = 0;
uint32_t linearMotionStartTime = 0;
uint32_t angularMotionStartTime = 0;
uint32_t overallMotionTimeout = 0;
uint32_t nextControlTime = 0;
uint32_t lastComputeTime = 0;

uint32_t nextLedTime = 0;
uint32_t nextImuTime = 0;
uint32_t nextTempTime = 0;
uint32_t imuDataTimeout = 0;
uint32_t nextSaveTime = 0;
uint32_t nextTimetableTime = 0;

//##################################################################################
uint32_t loopTime = millis();
uint32_t loopTimeNow = 0;
uint32_t loopTimeMax = 0;
float loopTimeMean = 0;
uint32_t loopTimeMin = 99999;
uint32_t loopTimeTimer = 0;
uint32_t wdResetTimer = millis();
//##################################################################################

bool wifiFound = false;
char ssid[] = WIFI_SSID;      // your network SSID (name)
char pass[] = WIFI_PASS;        // your network password
WiFiEspServer server(80);
bool hasClient = false;
WiFiEspClient client;
WiFiEspClient espClient;
PubSubClient mqttClient(espClient);
//int16_t status = WL_IDLE_STATUS;     // the Wifi radio's status
#ifdef ENABLE_NTRIP
NTRIPClient ntrip;  // NTRIP tcp client (optional)
#endif
#ifdef GPS_USE_TCP
WiFiClient gpsClient; // GPS tcp client (optional)
#endif

int16_t motorErrorCounter = 0;


RunningMedian<uint16_t,3> tofMeasurements;


// must be defined to override default behavior
void watchdogSetup (void) {}


// reset linear motion measurement
void resetLinearMotionMeasurement()
{
  linearMotionStartTime = millis();
  //stateGroundSpeed = 1.0;
}

// reset angular motion measurement
void resetAngularMotionMeasurement()
{
  angularMotionStartTime = millis();
}

// reset overall motion timeout
void resetOverallMotionTimeout()
{
  overallMotionTimeout = millis() + 10000;
}

void updateGPSMotionCheckTime()
{
  nextGPSMotionCheckTime = millis() + GPS_MOTION_DETECTION_TIMEOUT * 1000;
}



void sensorTest()
{
  wxLogMessage("testing sensors for 60 seconds...");
  uint32_t stopTime = millis() + 60000;
  uint32_t nextMeasureTime = 0;
  while (millis() < stopTime)
    {
      sonar.run();
      bumper.run();
      liftDriver.run();
      if (millis() > nextMeasureTime)
        {
          nextMeasureTime = millis() + 1000;
          if (SONAR_ENABLE)
            {
              wxLogMessage("sonar (enabled,left,center,right,triggered): %b %i %i %i %i", sonar.enabled, sonar.distanceLeft, sonar.distanceCenter, sonar.distanceRight, (int16_t)sonar.obstacle());
            }
          if (BUMPER_ENABLE)
            {
              wxLogMessage("bumper (left,right,triggered): %i %i %i", (int16_t)bumper.testLeft(), (int16_t)bumper.testRight(), (int16_t)bumper.obstacle());
            }
#ifdef ENABLE_LIFT_DETECTION
          wxLogMessage("lift sensor (triggered): %i", (int16_t)liftDriver.triggered());
#endif

          //watchdogReset();
          robotDriver.run();
        }
    }
  wxLogMessage("end of sensor test - please ignore any IMU/GPS errors");
}


void startWIFI()
{
  WiFi.begin();
  wifiFound = true;
#if defined(ENABLE_UDP)
  udpSerial.beginUDP();
#endif
  if (ENABLE_SERVER)
    {
      //server.listenOnLocalhost();
      server.begin();
    }
  if (ENABLE_MQTT)
    {
      wxLogMessage("MQTT: enabled");
      mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
      mqttClient.setCallback(mqttCallback);
    }
}

void outputConfig()
{
#ifdef ENABLE_PASS
  wxLogMessage("ENABLE_PASS");
#endif
#ifdef ENABLE_TILT_DETECTION
  wxLogMessage("ENABLE_TILT_DETECTION");
#endif
  wxLogMessage("FREEWHEEL_IS_AT_BACKSIDE: %b", FREEWHEEL_IS_AT_BACKSIDE);
  wxLogMessage("WHEEL_BASE_CM: %f", (float)WHEEL_BASE_CM);
  wxLogMessage("WHEEL_DIAMETER: %f", (float)WHEEL_DIAMETER);
#ifdef ENABLE_LIFT_DETECTION
  wxLogMessage("ENABLE_LIFT_DETECTION");
#ifdef LIFT_OBSTACLE_AVOIDANCE
  wxLogMessage("LIFT_OBSTACLE_AVOIDANCE");
#endif
#endif
  wxLogMessage("ENABLE_ODOMETRY_ERROR_DETECTION: %b", ENABLE_ODOMETRY_ERROR_DETECTION);
  wxLogMessage("TICKS_PER_REVOLUTION: %i", (int)TICKS_PER_REVOLUTION);
#ifdef MOTOR_DRIVER_BRUSHLESS
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS");
#endif

#ifdef MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_MOW_A4931
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_MOW_A4931");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_MOW_JYQD
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_MOW_JYQD");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_MOW_OWL
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_MOW_OWL");
#endif

#ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_A4931
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_GEARS_A4931");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD");
#endif
#ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_OWL
  wxLogMessage("MOTOR_DRIVER_BRUSHLESS_GEARS_OWL");
#endif

  wxLogMessage("MOTOR_FAULT_CURRENT: %.2f", (float)MOTOR_FAULT_CURRENT);
  wxLogMessage("MOTOR_OVERLOAD_CURRENT: %.2f", (float)MOTOR_OVERLOAD_CURRENT);
  wxLogMessage("USE_LINEAR_SPEED_RAMP: %b", USE_LINEAR_SPEED_RAMP);
  wxLogMessage("MOTOR_PID_KP: %.3f", (float)MOTOR_PID_KP);
  wxLogMessage("MOTOR_PID_KI: %.3f", (float)MOTOR_PID_KI);
  wxLogMessage("MOTOR_PID_KD: %.3f", (float)MOTOR_PID_KD);
#ifdef MOTOR_LEFT_SWAP_DIRECTION
  wxLogMessage("MOTOR_LEFT_SWAP_DIRECTION");
#endif
#ifdef MOTOR_RIGHT_SWAP_DIRECTION
  wxLogMessage("MOTOR_RIGHT_SWAP_DIRECTION");
#endif
#ifdef MAX_MOW_PWM
  wxLogMessage("MAX_MOW_PWM: %i", MAX_MOW_PWM);
#endif
  wxLogMessage("MOW_FAULT_CURRENT: %.2f", (float)MOW_FAULT_CURRENT);
  wxLogMessage("MOW_OVERLOAD_CURRENT: %.2f", (float)MOW_OVERLOAD_CURRENT);
  wxLogMessage("ENABLE_OVERLOAD_DETECTION: %b", ENABLE_OVERLOAD_DETECTION);
  wxLogMessage("ENABLE_FAULT_DETECTION: %b", ENABLE_FAULT_DETECTION);
  wxLogMessage("ENABLE_FAULT_OBSTACLE_AVOIDANCE: %b", ENABLE_FAULT_OBSTACLE_AVOIDANCE);
  wxLogMessage("ENABLE_RPM_FAULT_DETECTION: %b", ENABLE_RPM_FAULT_DETECTION);
#ifdef SONAR_INSTALLED
  wxLogMessage("SONAR_INSTALLED");
  wxLogMessage("SONAR_ENABLE: %b", SONAR_ENABLE);
  wxLogMessage("SONAR_TRIGGER_OBSTACLES: %b", SONAR_TRIGGER_OBSTACLES);
#endif
  wxLogMessage("RAIN_ENABLE: %b", RAIN_ENABLE);
  wxLogMessage("BUMPER_ENABLE: %b", BUMPER_ENABLE);
  wxLogMessage("BUMPER_DEADTIME: %i", BUMPER_DEADTIME);
  wxLogMessage("BUMPER_TRIGGER_DELAY: %i", BUMPER_TRIGGER_DELAY);
  wxLogMessage("BUMPER_MAX_TRIGGER_TIME: %i", BUMPER_MAX_TRIGGER_TIME);
  wxLogMessage("CURRENT_FACTOR: %f", (float)CURRENT_FACTOR);
  wxLogMessage("GO_HOME_VOLTAGE: %f", (float)GO_HOME_VOLTAGE);
  wxLogMessage("BAT_FULL_VOLTAGE: %.2f", (float)BAT_FULL_VOLTAGE);
  wxLogMessage("BAT_FULL_CURRENT: %.2f", (float)BAT_FULL_CURRENT);
  wxLogMessage("BAT_SWITCH_OFF_IDLE: %b", BAT_SWITCH_OFF_IDLE);
  wxLogMessage("BAT_SWITCH_OFF_UNDERVOLTAGE: %b", BAT_SWITCH_OFF_UNDERVOLTAGE);
#ifdef GPS_USE_TCP
  wxLogMessage("GPS_USE_TCP");
#endif
#ifdef GPS_SKYTRAQ
  wxLogMessage("GPS_USE_SKYTRAQ");
#endif
  wxLogMessage("REQUIRE_VALID_GPS: %b", REQUIRE_VALID_GPS);
  wxLogMessage("GPS_SPEED_DETECTION: %b", GPS_SPEED_DETECTION);
  wxLogMessage("GPS_MOTION_DETECTION: %b", GPS_MOTION_DETECTION);
  wxLogMessage("GPS_REBOOT_RECOVERY: %b", GPS_REBOOT_RECOVERY);
  wxLogMessage("GPS_CONFIG: %b", GPS_CONFIG);
  wxLogMessage("GPS_CONFIG_FILTER: %b", GPS_CONFIG_FILTER);
  wxLogMessage("CPG_CONFIG_FILTER_MINELEV: %i", CPG_CONFIG_FILTER_MINELEV);
  wxLogMessage("CPG_CONFIG_FILTER_NCNOTHRS: %i", CPG_CONFIG_FILTER_NCNOTHRS);
  wxLogMessage("CPG_CONFIG_FILTER_CNOTHRS: %i", CPG_CONFIG_FILTER_CNOTHRS);
  wxLogMessage("ALLOW_ROUTE_OUTSIDE_PERI_METER: %f", ALLOW_ROUTE_OUTSIDE_PERI_METER);
  wxLogMessage("OBSTACLE_DETECTION_ROTATION: %b", OBSTACLE_DETECTION_ROTATION);
  wxLogMessage("KIDNAP_DETECT: %b", KIDNAP_DETECT);
  wxLogMessage("KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE: %f", KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE);
  wxLogMessage("DOCKING_STATION: %b", DOCKING_STATION);
  wxLogMessage("DOCK_IGNORE_GPS: %b", DOCK_IGNORE_GPS);
  wxLogMessage("DOCK_AUTO_START: %b", DOCK_AUTO_START);
  wxLogMessage("TARGET_REACHED_TOLERANCE: %.2f", (float)TARGET_REACHED_TOLERANCE);
  wxLogMessage("STANLEY_CONTROL_P_NORMAL: %.3f", (float)STANLEY_CONTROL_P_NORMAL);
  wxLogMessage("STANLEY_CONTROL_K_NORMAL: %.3f", (float)STANLEY_CONTROL_K_NORMAL);
  wxLogMessage("STANLEY_CONTROL_P_SLOW: %.3f", (float)STANLEY_CONTROL_P_SLOW);
  wxLogMessage("STANLEY_CONTROL_K_SLOW: %.3f", (float)STANLEY_CONTROL_K_SLOW);
  wxLogMessage("BUTTON_CONTROL: %b", BUTTON_CONTROL);
  wxLogMessage("USE_TEMP_SENSOR: %b", USE_TEMP_SENSOR);
#ifdef BUZZER_ENABLE
  wxLogMessage("BUZZER_ENABLE");
#endif
}


// robot start routine
void start()
{
  StartIsRunning = true;

  logResetCause();

  wxLogMessage(VER);
  wxLogMessage("compiled for: %s", BOARD);

  robotDriver.begin();
  String rid = "";
  robotDriver.getRobotID(rid);
  wxLogMessage("robot id: %s", rid.c_str());
  motorDriver.begin();
  rainDriver.begin();
  liftDriver.begin();
  battery.begin();
  stopButton.begin();
  // keep battery switched ON
  batteryDriver.begin();
  //CONSOLE.begin(CONSOLE_BAUDRATE);
  buzzerDriver.begin();
  buzzer.begin();

  rcmodel.begin();
  motor.begin();
  sonar.begin();
  bumper.begin();

  outputConfig();

  wxLogMessage("SERIAL_BUFFER_SIZE=%i", SERIAL_BUFFER_SIZE);
  wxLogMessage(" (increase if you experience GPS checksum errors)");
  //wxLogMessage("-----------------------------------------------------");
  //wxLogMessage("NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:");
  //wxLogMessage("1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom");
  //wxLogMessage("2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h");
  //wxLogMessage("   for Grand Central M4 'packages/adafruit/hardware/samd/xxxxx/cores/arduino/RingBuffer.h");
  //wxLogMessage("change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024");
  wxLogMessage("-----------------------------------------------------");

#ifdef GPS_USE_TCP
  gps.begin(gpsClient, GPS_HOST, GPS_PORT);
#else
  gps.begin(GPS, GPS_BAUDRATE);
#endif

  maps.begin();
  //maps.clipperTest();

  // initialize ESP module
  startWIFI();
#ifdef ENABLE_NTRIP
  ntrip.begin();
#endif


  buzzer.sound(SND_READY);
  battery.resetIdle();
  loadState();

#ifdef DRV_SIM_ROBOT
  robotDriver.setSimRobotPosState(stateX, stateY, stateDelta);
  tester.begin();
#endif
  StartIsRunning = false;
  StartRunDone = true;
}



// should robot move?
bool robotShouldMove()
{
  /*wxLogMessage(motor.linearSpeedSet);
  wxLogMessage(",");
  wxLogMessage(motor.angularSpeedSet / PI * 180.0);  */
  if (motor.linearSpeedSet * linearSpeedSetLast < 0)
    {
      // direction change
      linearSpeedSetDeadTime = millis() + 10000;
      linearSpeedSetDeadTimeIsSet = true;
    }
  linearSpeedSetLast = motor.linearSpeedSet;
  if (linearSpeedSetDeadTimeIsSet)
    {
      if (millis() < linearSpeedSetDeadTime)
        {
          return false;  // wait dead-time (due to direction change)
        }
      linearSpeedSetDeadTimeIsSet = false;
    }
  return ( fabs(motor.linearSpeedSet) > 0.001 );
}


bool robotShouldMoveForward()
{
  return ( motor.linearSpeedSet > 0.001 );
}

// should robot rotate?
bool robotShouldRotate()
{
  return ( (fabs(motor.linearSpeedSet) < 0.001) &&  (fabs(motor.angularSpeedSet) > 0.001) );
}

// should robot be in motion? NOTE: function ignores very short motion pauses (with motion low-pass filtering)
bool robotShouldBeInMotion()
{
  if (robotShouldMove() || (robotShouldRotate()))
    {
      stateInMotionLastTime = millis();
      stateInMotionLP = true;
    }
  if (millis() > stateInMotionLastTime + 2000)
    {
      stateInMotionLP = false;
    }
  return stateInMotionLP;
}


// drive reverse if robot cannot move forward
void triggerObstacle()
{
  activeOp->onObstacle();
}


// detect sensor malfunction
void detectSensorMalfunction()
{
  if (ENABLE_ODOMETRY_ERROR_DETECTION)
    {
      if (motor.odometryError)
        {
          wxLogMessage("odometry error!");
          activeOp->onOdometryError();
          return;
        }
    }
  if (ENABLE_OVERLOAD_DETECTION)
    {
      if (motor.motorOverloadDuration > 20000)
        {
          // one motor is taking too much current over a int32_t time (too high gras etc.) and we should stop mowing
          wxLogMessage("overload!");
          activeOp->onMotorOverload();
          return;
        }
    }
  if (ENABLE_FAULT_OBSTACLE_AVOIDANCE)
    {
      // there is a motor error (either unrecoverable fault signal or a malfunction) and we should try an obstacle avoidance
      if (motor.motorError)
        {
          wxLogMessage("motor error!");
          activeOp->onMotorError();
          return;
        }
    }
}

// detect lift
// returns true, if lift detected, otherwise false
bool detectLift()
{
#ifdef ENABLE_LIFT_DETECTION
  if (liftDriver.triggered())
    {
      wxLogMessage("LIFT triggered");
      return true;
    }
#endif
  return false;
}

// detect obstacle (bumper, sonar, ToF)
// returns true, if obstacle detected, otherwise false
bool detectObstacle()
{
  if (! ((robotShouldMoveForward()) || (robotShouldRotate())) ) return false;
#ifdef ENABLE_LIFT_DETECTION
#ifdef LIFT_OBSTACLE_AVOIDANCE
  if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (liftDriver.triggered()) )
    {
      wxLogMessage("lift sensor obstacle!");
      statMowBumperCounter++;
      triggerObstacle();
      return true;
    }
#endif
#endif

  if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (bumper.obstacle()) )
    {
      wxLogMessage("bumper obstacle!");
      statMowBumperCounter++;
      triggerObstacle();
      return true;
    }

  if (sonar.obstacle() && (maps.wayMode != WAY_DOCK))
    {
      wxLogMessage("sonar obstacle!");
      statMowSonarCounter++;
      if (SONAR_TRIGGER_OBSTACLES)
        {
          triggerObstacle();
          return true;
        }
    }
  // check if GPS motion (obstacle detection)
  if ((millis() > nextGPSMotionCheckTime) || (millis() > overallMotionTimeout))
    {
      updateGPSMotionCheckTime();
      resetOverallMotionTimeout(); // this resets overall motion timeout (overall motion timeout happens if e.g.
      // motion between anuglar-only and linar-only toggles quickly, and their specific timeouts cannot apply due to the quick toggling)
      float dX = lastGPSMotionX - stateX;
      float dY = lastGPSMotionY - stateY;
      float delta = sqrt( SQ(dX) + SQ(dY) );
      if (delta < 0.05)
        {
          if (GPS_MOTION_DETECTION)
            {
              wxLogMessage("gps no motion => obstacle!");
              statMowGPSMotionTimeoutCounter++;
              triggerObstacle();
              return true;
            }
        }
      lastGPSMotionX = stateX;
      lastGPSMotionY = stateY;
    }
  return false;
}

// stuck rotate avoidance (drive forward if robot cannot rotate)
void triggerObstacleRotation()
{
  activeOp->onObstacleRotation();
}

// stuck rotate detection (e.g. robot cannot due to an obstacle outside of robot rotation point)
// returns true, if stuck detected, otherwise false
bool detectObstacleRotation()
{
  if (!robotShouldRotate())
    {
      return false;
    }
  if (!OBSTACLE_DETECTION_ROTATION) return false;
  if (millis() > angularMotionStartTime + 15000)   // too int32_t rotation time (timeout), e.g. due to obstacle
    {
      wxLogMessage("too int32_t rotation time (timeout) for requested rotation => assuming obstacle");
      triggerObstacleRotation();
      return true;
    }
  /*if (BUMPER_ENABLE){
    if (millis() > angularMotionStartTime + 500) { // FIXME: do we actually need a deadtime here for the freewheel sensor?
      if (bumper.obstacle()){
        wxLogMessage("bumper obstacle!");
        statMowBumperCounter++;
        triggerObstacleRotation();
        return true;
      }
    }
  }*/
  if (imuDriver.imuFound)
    {
      if (millis() > angularMotionStartTime + 3000)
        {
          if (fabs(stateDeltaSpeedLP) < 3.0/180.0 * PI)  // less than 3 degree/s yaw speed, e.g. due to obstacle
            {
              wxLogMessage("no IMU rotation speed detected for requested rotation => assuming obstacle");
              triggerObstacleRotation();
              return true;
            }
        }
      if (diffIMUWheelYawSpeedLP > 10.0/180.0 * PI)    // yaw speed difference between wheels and IMU more than 8 degree/s, e.g. due to obstacle
        {
          wxLogMessage("yaw difference between wheels and IMU for requested rotation => assuming obstacle");
          triggerObstacleRotation();
          return true;
        }
    }
  return false;
}




// robot main loop
void loop()
{
  LoopIsRunning = true;
#ifdef ENABLE_NTRIP
  ntrip.run();
#endif
#ifdef DRV_SIM_ROBOT
  tester.run();
#endif
  robotDriver.run();
  buzzer.run();
  buzzerDriver.run();
  stopButton.run();
  battery.run();
  batteryDriver.run();
  motorDriver.run();
  rainDriver.run();
  liftDriver.run();
  motor.run();
  sonar.run();
  maps.run();
  rcmodel.run();
  bumper.run();

  // state saving
  if (millis() >= nextSaveTime)
    {
      nextSaveTime = millis() + 5000;
      saveState();
    }

  // temp
  if (millis() > nextTempTime)
    {
      nextTempTime = millis() + 60000;
      float batTemp = batteryDriver.getBatteryTemperature();
      float cpuTemp = robotDriver.getCpuTemperature();
      wxLogMessage("batTemp=%f cpuTemp=%f", batTemp, cpuTemp);
      //logCPUHealth();
      if (batTemp < -999)
        {
          stateTemp = cpuTemp;
        }
      else
        {
          stateTemp = batTemp;
        }
      statTempMin = MIN(statTempMin, stateTemp);
      statTempMax = MAX(statTempMax, stateTemp);
    }

  // IMU
  if (millis() > nextImuTime)
    {
      nextImuTime = millis() + IMU_LOOP_TIME;
      //imu.resetFifo();
      if (imuIsCalibrating)
        {
          activeOp->onImuCalibration();
        }
      else
        {
          readIMU();
        }
    }

  // LED states
  if (millis() > nextLedTime)
    {
      nextLedTime = millis() + 1000;
      robotDriver.ledStateGpsFloat = (gps.solution == SOL_FLOAT);
      robotDriver.ledStateGpsFix = (gps.solution == SOL_FIXED);
      robotDriver.ledStateError = (stateOp == OP_ERROR);
    }

  gps.run();

  if (millis() > nextTimetableTime)
    {
      nextTimetableTime = millis() + 30000;
      gps.decodeTOW();
      timetable.setCurrentTime(gps.hour, gps.mins, gps.dayOfWeek);
      timetable.run();
    }

  calcStats();


  if (millis() >= nextControlTime)
    {
      nextControlTime = millis() + 20;
      controlLoops++;

      computeRobotState();
      if (!robotShouldMove())
        {
          resetLinearMotionMeasurement();
          updateGPSMotionCheckTime();
        }
      if (!robotShouldRotate())
        {
          resetAngularMotionMeasurement();
        }
      if (!robotShouldBeInMotion())
        {
          resetOverallMotionTimeout();
          lastGPSMotionX = 0;
          lastGPSMotionY = 0;
        }

      /*if (gpsJump) {
        // gps jump: restart current operation from new position (restart path planning)
        wxLogMessage("restarting operation (gps jump)");
        gpsJump = false;
        motor.stopImmediately(true);
        setOperation(stateOp, true);    // restart current operation
      }*/

      if (battery.chargerConnected() != stateChargerConnected)
        {
          stateChargerConnected = battery.chargerConnected();
          if (stateChargerConnected)
            {
              // charger connected event
              activeOp->onChargerConnected();
            }
          else
            {
              activeOp->onChargerDisconnected();
            }
        }
      if (millis() > nextBadChargingContactCheck)
        {
          if (battery.badChargerContact())
            {
              nextBadChargingContactCheck = millis() + 60000; // 1 min.
              activeOp->onBadChargingContactDetected();
            }
        }

      if (battery.underVoltage())
        {
          activeOp->onBatteryUndervoltage();
        }
      else
        {
          if (USE_TEMP_SENSOR)
            {
              if (stateTemp > DOCK_OVERHEAT_TEMP)
                {
                  activeOp->onTempOutOfRangeTriggered();
                }
              else if (stateTemp < DOCK_TOO_COLD_TEMP)
                {
                  activeOp->onTempOutOfRangeTriggered();
                }
            }
          if (RAIN_ENABLE)
            {
              // rain sensor should trigger serveral times to robustly detect rain (robust rain detection)
              // it should not trigger if one rain drop or wet tree leaves touches the sensor
              if (rainDriver.triggered())
                {
                  //wxLogMessage("RAIN TRIGGERED ");
                  activeOp->onRainTriggered();
                }
            }
          if (battery.shouldGoHome())
            {
              if (DOCKING_STATION)
                {
                  activeOp->onBatteryLowShouldDock();
                }
            }

          if (battery.chargerConnected())
            {
              if (battery.chargingHasCompleted())
                {
                  activeOp->onChargingCompleted();
                }
            }
        }

      //wxLogMessage("active:");
      //wxLogMessage(activeOp->name());
      activeOp->checkStop();
      activeOp->run();

      // process button state
      if (stateButton == 5)
        {
          stateButton = 0; // reset button state
          stateSensor = SENS_STOP_BUTTON;
          setOperation(OP_DOCK, false);
        }
      else if (stateButton == 6)
        {
          stateButton = 0; // reset button state
          stateSensor = SENS_STOP_BUTTON;
          setOperation(OP_MOW, false);
        }
      //else if (stateButton > 0){  // stateButton 1 (or unknown button state)
      else if (stateButton == 1)   // stateButton 1
        {
          stateButton = 0;  // reset button state
          stateSensor = SENS_STOP_BUTTON;
          setOperation(OP_IDLE, false);
        }
      else if (stateButton == 9)
        {
          stateButton = 0;  // reset button state
          stateSensor = SENS_STOP_BUTTON;
          cmdSwitchOffRobot();
        }
      else if (stateButton == 12)
        {
          stateButton = 0; // reset button state
          stateSensor = SENS_STOP_BUTTON;
          WiFi.startWifiProtectedSetup();
        }

      // update operation type
      stateOp = activeOp->getGoalOperationType();

    }   // if (millis() >= nextControlTime)

  // ----- read serial input (BT/console) -------------
  processComm();
  outputConsole();

  //##############################################################################

  if(millis() > wdResetTimer + 1000)
    {
      //watchdogReset();
    }

  loopTimeNow = millis() - loopTime;
  loopTimeMin = MIN(loopTimeNow, loopTimeMin);
  loopTimeMax = MAX(loopTimeNow, loopTimeMax);
  loopTimeMean = 0.99 * loopTimeMean + 0.01 * loopTimeNow;
  loopTime = millis();

  if(millis() > loopTimeTimer + 10000)
    {
      if(loopTimeMax > 500)
        {
          wxLogMessage("WARNING - LoopTime: %i - %i - %f - %i mS", loopTimeNow, loopTimeMin, loopTimeMean, loopTimeMax);
        }
      else
        {
          wxLogMessage("Info - LoopTime: %i - %i - %f - %i mS", loopTimeNow, loopTimeMin, loopTimeMean, loopTimeMax);
        }
      loopTimeMin = 99999;
      loopTimeMax = 0;
      loopTimeTimer = millis();
    }
  //##############################################################################

  // compute button state (stateButton)
  if (BUTTON_CONTROL)
    {
      if (stopButton.triggered())
        {
          if (millis() > stateButtonTimeout)
            {
              stateButtonTimeout = millis() + 1000;
              stateButtonTemp++; // next state
              buzzer.sound(SND_READY, true);
              wxLogMessage("BUTTON %i S", stateButtonTemp);
            }

        }
      else
        {
          if (stateButtonTemp > 0)
            {
              // button released => set stateButton
              stateButtonTimeout = 0;
              stateButton = stateButtonTemp;
              stateButtonTemp = 0;
              wxLogMessage("stateButton %i", stateButton);
            }
        }
    }
  LoopIsRunning = false;
}


// set new robot operation
void setOperation(OperationType op, bool allowRepeat)
{
  if ((stateOp == op) && (!allowRepeat)) return;
  wxLogMessage("setOperation op= %i", (int)op);
  stateOp = op;
  activeOp->changeOperationTypeByOperator(stateOp);
  saveState();
}

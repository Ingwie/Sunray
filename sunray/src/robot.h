// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#ifndef ROBOT_H
#define ROBOT_H

#include "NutsBolts.h"
#include "motor.h"
#include "config.h"
#include "driver/MeuhRobotDriver.h"
#include "battery.h"
#include "bumper.h"
#include "buzzer.h"
#include "sonar.h"
#include "map.h"
#include "ublox/ublox.h"
#include "BridgeClient.h"
#include "BridgeServer.h"
#include "PubSubClient.h"
#include "timetable.h"
#include "LinuxSerial.h"

#include "../Sunray_RobotMeuhMain.h"

#include <math.h>


#define VER "Sunray,1.0.318"

extern LinuxSerial SerialWIFI;
extern LinuxSerial SerialGPS;


// operation types
enum OperationType
{
  OP_IDLE,      // idle
  OP_MOW,       // mowing
  OP_CHARGE,    // charging
  OP_ERROR,     // serious error
  OP_DOCK,      // go to docking
};

// sensor errors
enum Sensor
{
  SENS_NONE,              // no error
  SENS_BAT_UNDERVOLTAGE,  // battery undervoltage
  SENS_OBSTACLE,          // obstacle triggered
  SENS_GPS_FIX_TIMEOUT,   // gps fix timeout
  SENS_IMU_TIMEOUT,       // imu timeout
  SENS_IMU_TILT,          // imut tilt
  SENS_KIDNAPPED,         // robot has been kidnapped (is no longer on planned track)
  SENS_OVERLOAD,          // motor overload
  SENS_MOTOR_ERROR,       // motor error
  SENS_GPS_INVALID,       // gps is invalid or not working
  SENS_ODOMETRY_ERROR,    // motor odometry error
  SENS_MAP_NO_ROUTE,      // robot cannot find a route to next planned point
  SENS_MEM_OVERFLOW,      // cpu memory overflow
  SENS_BUMPER,            // bumper triggered
  SENS_SONAR,             // ultrasonic triggered
  SENS_LIFT,              // lift triggered
  SENS_RAIN,              // rain sensor triggered
  SENS_STOP_BUTTON,       // emergency/stop button triggered
  SENS_TEMP_OUT_OF_RANGE, // temperature out-of-range triggered
};

#ifndef __linux__
#define FILE_CREATE  (O_WRITE | O_CREAT)
#endif

extern OperationType stateOp; // operation
extern Sensor stateSensor; // last triggered sensor
extern String stateOpText;  // current operation as text
extern String gpsSolText; // current gps solution as text
extern int16_t stateButton;  // button state
extern float stateTemp;  // current temperature

extern float setSpeed; // linear speed (m/s)
extern int16_t fixTimeout;
extern bool finishAndRestart; // auto-restart when mowing finished?
extern bool absolutePosSource;
extern double absolutePosSourceLon;
extern double absolutePosSourceLat;

extern uint32_t linearMotionStartTime;
extern uint32_t angularMotionStartTime;
extern bool stateInMotionLP; // robot is in angular or linear motion? (with motion low-pass filtering)

extern uint32_t lastFixTime;

extern WiFiEspClient client;
extern WiFiEspServer server;
extern PubSubClient mqttClient;
extern bool hasClient;

extern uint32_t controlLoops;
extern bool wifiFound;
extern int16_t motorErrorCounter;


#ifdef DRV_SERIAL_ROBOT
extern SerialRobotDriver robotDriver;
extern SerialMotorDriver motorDriver;
extern SerialBatteryDriver batteryDriver;
extern SerialBumperDriver bumperDriver;
extern SerialStopButtonDriver stopButton;
extern SerialRainSensorDriver rainDriver;
extern SerialLiftSensorDriver liftDriver;
extern SerialBuzzerDriver buzzerDriver;
#elif DRV_MEUH_ROBOT
extern MeuhRobotDriver robotDriver;
extern MeuhMotorDriver motorDriver;
extern MeuhBatteryDriver batteryDriver;
extern MeuhBumperDriver bumperDriver;
extern MeuhStopButtonDriver stopButton;
extern MeuhRainSensorDriver rainDriver;
extern MeuhLiftSensorDriver liftDriver;
extern MeuhBuzzerDriver buzzerDriver;
#elif DRV_SIM_ROBOT
extern SimRobotDriver robotDriver;
extern SimMotorDriver motorDriver;
extern SimBatteryDriver batteryDriver;
extern SimBumperDriver bumperDriver;
extern SimStopButtonDriver stopButton;
extern SimRainSensorDriver rainDriver;
extern SimLiftSensorDriver liftDriver;
extern SimBuzzerDriver buzzerDriver;
#else
extern AmRobotDriver robotDriver;
extern AmMotorDriver motorDriver;
extern AmBatteryDriver batteryDriver;
extern AmBumperDriver bumperDriver;
extern AmStopButtonDriver stopButton;
extern AmRainSensorDriver rainDriver;
extern AmLiftSensorDriver liftDriver;
extern AmBuzzerDriver buzzerDriver;
#endif

#ifdef DRV_SIM_ROBOT
extern SimImuDriver imuDriver;
#elif defined(DRV_MEUH_ROBOT)
extern MeuhImuDriver imuDriver;
#endif

extern Motor motor;
extern Battery battery;
//extern BLEConfig bleConfig;
extern Bumper bumper;
extern Buzzer buzzer;
extern Sonar sonar;
//extern PinManager pinMan;
extern Map maps;
extern TimeTable timetable;
#ifdef TESTNOROOT
extern SimGpsDriver gps;
#else
extern UBLOX gps;
#endif

int32_t freeMemory();
void start();
void loop();
void setOperation(OperationType op, bool allowRepeat = false);
void triggerObstacle();
void sensorTest();
void updateStateOpText();
void detectSensorMalfunction();
bool detectLift();
bool detectObstacle();
bool detectObstacleRotation();



#endif

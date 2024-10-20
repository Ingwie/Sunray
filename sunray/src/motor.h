// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;


class Motor
{
public:
  float robotPitch;  // robot pitch (rad)
  float wheelBaseCm;  // wheel-to-wheel diameter
  int16_t wheelDiameter;   // wheel diameter (mm)
  uint32_t ticksPerRevolution; // ticks per revolution
  float ticksPerCm;  // ticks per cm
  bool activateLinearSpeedRamp;  // activate ramp to accelerate/slow down linear speed?
  bool toggleMowDir; // toggle mowing motor direction each mow motor start?
  bool motorLeftSwapDir;
  bool motorRightSwapDir;
  bool motorError;
  bool motorLeftOverload;
  bool motorRightOverload;
  bool motorMowOverload;
  bool tractionMotorsEnabled;
  bool enableMowMotor;
  bool motorMowForwardSet;
  bool odometryError;
  uint32_t motorOverloadDuration; // accumulated duration (ms)
  int16_t  pwmMax;
  int16_t  pwmMaxMow;
  float  pwmSpeedOffset;
  float mowMotorCurrentAverage;
  float currentFactor;
  bool pwmSpeedCurveDetection;
  uint32_t motorLeftTicks;
  uint32_t motorRightTicks;
  uint32_t motorMowTicks;
  float linearSpeedSet; // m/s
  float angularSpeedSet; // rad/s
  float motorLeftSense; // left motor current (amps)
  float motorRightSense; // right  motor current (amps)
  float motorMowSense;  // mower motor current (amps)
  float motorLeftSenseLP; // left motor current (amps, low-pass)
  float motorRightSenseLP; // right  motor current (amps, low-pass)
  float motorMowSenseLP;  // mower motor current (amps, low-pass)
  float motorsSenseLP; // all motors current (amps, low-pass)
  float motorLeftSenseLPNorm;
  float motorRightSenseLPNorm;
  uint32_t motorMowSpinUpTime;
  bool motorRecoveryState;
  void begin();
  void run();
  void test();
  void plot();
  void enableTractionMotors(bool enable);
  void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
  void setMowState(bool switchOn);
  void setMowMaxPwm( int16_t val );
  void stopImmediately(bool includeMowerMotor);
protected:
  float motorLeftRpmSet; // set speed
  float motorRightRpmSet;
  float motorLeftRpmCurr;
  float motorRightRpmCurr;
  float motorMowRpmCurr;
  float motorLeftRpmCurrLP;
  float motorRightRpmCurrLP;
  float motorMowRpmCurrLP;
  float motorLeftRpmLast;
  float motorRightRpmLast;
  float motorMowPWMSet;
  float motorMowPWMCurr;
  int16_t motorLeftPWMCurr;
  int16_t motorRightPWMCurr;
  float motorMowPWMCurrLP;
  float motorLeftPWMCurrLP;
  float motorRightPWMCurrLP;
  uint32_t lastControlTime;
  uint32_t nextSenseTime;
  bool recoverMotorFault;
  int16_t recoverMotorFaultCounter;
  uint32_t nextRecoverMotorFaultTime;
  int16_t motorLeftTicksZero;
  int16_t motorRightTicksZero;
  PID motorLeftPID;
  PID motorRightPID;
  bool setLinearAngularSpeedTimeoutActive;
  uint32_t setLinearAngularSpeedTimeout;
  void speedPWM ( int16_t pwmLeft, int16_t pwmRight, int16_t pwmMow );
  void control();
  bool checkFault();
  void checkOverload();
  bool checkOdometryError();
  bool checkMowRpmFault();
  bool checkCurrentTooHighError();
  bool checkCurrentTooLowError();
  void sense();
  void dumpOdoTicks(int16_t seconds);
};


#endif

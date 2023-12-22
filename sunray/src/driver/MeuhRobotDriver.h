/*
.-------.        ,-----.     _______       ,-----.  ,---------. ,---.    ,---.    .-''-.    ___    _ .---.  .---.
|  _ _   \     .'  .-,  '.  \  ____  \   .'  .-,  '.\          \|    \  /    |  .'_ _   \ .'   |  | ||   |  |_ _|
| ( ' )  |    / ,-.|  \ _ \ | |    \ |  / ,-.|  \ _ \`--.  ,---'|  ,  \/  ,  | / ( ` )   '|   .'  | ||   |  ( ' )
|(_ o _) /   ;  \  '_ /  | :| |____/ / ;  \  '_ /  | :  |   \   |  |\_   /|  |. (_ o _)  |.'  '_  | ||   '-(_{;}_)
| (_,_).' __ |  _`,/ \ _/  ||   _ _ '. |  _`,/ \ _/  |  :_ _:   |  _( )_/ |  ||  (_,_)___|'   ( \.-.||      (_,_)
|  |\ \  |  |: (  '\_/ \   ;|  ( ' )  \: (  '\_/ \   ;  (_I_)   | (_ o _) |  |'  \   .---.' (`. _` /|| _ _--.   |
|  | \ `'   / \ `"/  \  ) / | (_{;}_) | \ `"/  \  ) /  (_(=)_)  |  (_,_)  |  | \  `-'    /| (_ (_) _)|( ' ) |   |
|  |  \    /   '. \_/``".'  |  (_,_)  /  '. \_/``".'    (_I_)   |  |      |  |  \       /  \ /  . \ /(_{;}_)|   |
''-'   `'-'      '-----'    /_______.'     '-----'      '---'   '--'      '--'   `'-..-'    ``-'`-'' '(_,_) '---'
*/
/*          Copyright 2023 by Ingwie (Bracame)         */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*    Ardumower Alfred mod to drive my autoclip 325    */

#ifndef MEUH_ROBOT_DRIVER_H
#define MEUH_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#ifdef __linux__
#include <Process.h>
#endif

#include "../meuh/pwm_linux.h"
#include "../meuh/ADS1115/ADS1115_WE.h"
//#include "../meuh/PCF8575/PCF8575.h"
#include "../meuh/TMCStepper-0.7.3/TMCStepper.h"

//-----> Mecanics informations
// Wheel diameter 270 mm
// pulley/belt reduction 12/80
// 1572 steps/meter

//-----> BPI-M4 GPIO Pin
#define pin_i2c_sda       17 // I2C used
#define pin_i2c_scl       18 // I2C used
#define pin_pwm_jyqd      21 // was pin_pwm1
#define pin_oe_txs108e    9  // was pin_ur1_tx
#define pin_enable_jyqd   8  // was pin_ur1_tx
#define pin_pulses_jyqd   42 // was pin_sdio_d0
#define pin_cw_ccw_jyqd   3  // was aio_bck
#define pin_power_relay   43 // was pin_sdio_d1
#define pin_charge_relay  44 // was pin_sdio_d2
#define pin_rain_sensor   11 // was pin_ur1_rts
#define pin_ur1_cts       10
#define pin_spi_mosi      31 // SPI used
#define pin_spi_miso      18 // SPI used
#define pin_enable_tmc    47 // was pin_gpio47
#define pin_spi_sck       19 // SPI used
#define pin_cs_r_tmc      20 // was pin_spi_cs .. Can be used for other task ??
#define pin_cs_l_tmc      22 // was pin_pwm2
#define pin_buzzer        45 // was pin_sdio_d3
#define pin_pwm3          23
#define pin_sdio_clk      41
#define pin_sdio_cmd      40
#define pin_spdif         50
#define pin_aio_ck        4
#define pin_aio_lrsk      2
#define pin_gpio53        53
#define pin_gpio34        34
#define pin_aisd          5
#define pin_aosd          6
//----- BPI-M4 hardware alias
#define PWM1              1 // use pin_pwm_jyqd

//-----> ADS1115 Macro resolution 2048 -> 0.0625mV/bit
#define ASD_BAT_CHANNEL         ADS1115_COMP_0_GND
#define ASD_CHARGE_CHANNEL      ADS1115_COMP_1_GND
#define ASD_ACS_CHANNEL         ADS1115_COMP_2_GND
#define POT_FACTOR(RMeas, RAds) RAds/(RAds + RMeas)
#define BAT_POT_FACTOR          POT_FACTOR(22000.0f, 300000.0f) // todo measure real values
#define CHARGE_POT_FACTOR       POT_FACTOR(22000.0f, 300000.0f)
#define ACS_POT_FACTOR          POT_FACTOR(4700.0f, 6800.0f)
// ACS712 30A Sensitivity (66mV/A)
#define ACS_MID_VOLTAGE         2.5f
#define ACS_AMPS_TO_VOLTS(x)    (((x/ACS_POT_FACTOR) - ACS_MID_VOLTAGE) / 0.066f)

//-----> TMC settings and helper
#define TMC_RsensE           0.22f // ohms
#define TMC_RMS_CURRENT_MA   600 // mA
#define TMC_SPEED_MULT       1 // pwm to tmc mult value
// tmc start sequence macro
#define START_TMC_SEQUENCE(x) \
x.begin(); \
x.XACTUAL(0);       /* Resetet position */ \
x.XTARGET(0);       /* Reset target mode position */ \
x.rms_current(TMC_RMS_CURRENT_MA); /* Set motor RMS current (mA) */ \
x.microsteps(32);   /* Set microsteps */ \
x.VMAX(0);          /* 44739 . Max speed (5rev/S) @ fck 12Mhz */ \
x.AMAX(489);        /* Acceleration (velocity mode) 1Sec . 0 to VMAX */ \
x.hstrt(7);         /* Chopconf param from excel computation */ \
x.hend(0);          /* Chopconf param from excel computation */ \
x.semin(8);         /* CoolStep low limit (activate) */ \
x.semax(8);         /* CoolStep hight limit (desactivate)*/ \
x.seup(2);          /* CoolStep increment */ \
x.sedn(1);          /* CoolStep current down step speed */ \
x.sgt(0);           /* StallGuard2 sensitivity */ \
x.sfilt(1);         /* StallGuard2 filter */ \
x.TCOOLTHRS(10000); /* CoolStep lower velocity to active StallGuard2 stall flag */ \
// tmc error check macro
#define CHECK_AND_COMPUTE_TMC_ERROR(status, stepper, spiStatus, errorBool, current) \
 status.sr = stepper.DRV_STATUS(); /* load satus */ \
 errorBool = (spiStatus & 0x3); /* error or reset occured*/ \
 current = (TMC_RMS_CURRENT_MA / 1000) * (status.cs_actual + 1) / 32; /* compute current (mA) */ \
 /*errorBool |= status.stallGuard; /* check stall */ \
 errorBool |= status.ot; /* check over temperature */ \
 errorBool |= status.olb; /* check open load b */ \
 errorBool |= status.ola; /* check open load a */ \
 errorBool |= status.s2gb; /* check short to ground b */ \
 errorBool |= status.s2ga; /* check short to ground a */ \
 errorBool |= status.stst; /* stabdstill in each operation */ \

//-----> PWM macros used to drive the JYQD
#define JYQD_PWM_PERIOD      1000000 // 1mS

#define PWM1_INIT() \
pwmUnexport(PWM1); \
delay(5); \
pwmExport(PWM1); \
pwmSetEnable(PWM1, 0); \
pwmSetPolarity(PWM1, 0); \
pwmSetPeriod(PWM1, JYQD_PWM_PERIOD); \
pwmSetDutyCycle(PWM1, 0); \
pwmSetEnable(PWM1, 1)

#define SETPWM1DUTYCYCLE(x) \
uint32_t pwmVal = map(x, 0, 255, 0, JYQD_PWM_PERIOD); \
pwmSetDutyCycle(PWM1, pwmVal)

//-----> level converter module TXS108E macro (security)
#define TXS108E_OUTPUT_ENABLE()  digitalWrite(pin_oe_txs108e, 1)
#define TXS108E_OUTPUT_DISABLE() digitalWrite(pin_oe_txs108e, 0)

//-----> relay module HW383 macro
enum relayStateEnum
{
  ALL_STOP = 0,
  POWER_ON,
  CHARGE_ON
};

#define RELAY_STOP_ALL() \
digitalWrite(pin_power_relay, 0); \
digitalWrite(pin_charge_relay, 0)

#define RELAY_POWER_ON() \
digitalWrite(pin_charge_relay, 0); \
delay(100); \
digitalWrite(pin_power_relay, 1)

#define RELAY_CHARGE_ON() \
digitalWrite(pin_power_relay, 0); \
delay(100); \
digitalWrite(pin_charge_relay, 1)

struct TMC5160_DRV_STATUS_t
{
  union
  {
    uint32_t sr;
    struct
    {
      uint16_t sg_result : 10;
      uint8_t            : 5;
      bool fsactive : 1;
      uint8_t cs_actual : 5,
              : 3;
      bool  stallGuard : 1,
            ot : 1,
            otpw : 1,
            s2ga : 1,
            s2gb : 1,
            ola : 1,
            olb : 1,
            stst : 1;
    };
  };
};

class MeuhRobotDriver: public RobotDriver
{
public:
  String robotID;
  String mcuFirmwareName;
  String mcuFirmwareVersion;
  int lastLeftPwm;
  int lastRightPwm;
  int lastMowPwm;
  //unsigned long encoderTicksLeft;
  //unsigned long encoderTicksRight;
  //bool mcuCommunicationLost;
  float batteryVoltage;
  float chargeVoltage;
  float chargeCurrent;
  float idleCurrent;
  float stepperCurrent;
  float mowCurr;
  float motorLeftCurr;
  float motorRightCurr;
  //bool resetMotorTicks;
  float cpuTemp;
  ADS1115_WE adc;
  //PCF8575 pcf8575;
  bool triggeredLeftBumper;
  bool triggeredRightBumper;
  bool triggeredLift;
  //bool triggeredRain;
  bool triggeredStopButton;
  void begin() override;
  void run() override;
  bool getRobotID(String &id) override;
  bool getMcuFirmwareVersion(String &name, String &ver) override;
  float getCpuTemperature() override;
  //void requestMotorPwm(int leftPwm, int rightPwm, int mowPwm);
  void updateCpuTemperature();
  void updateWifiConnectionState();
  bool setFanPowerState(bool state);
  float readAdcChannel(ADS1115_MUX channel);
  //bool setImuPowerState(bool state);
protected:
#ifdef __linux__
  Process cpuTempProcess;
  Process wifiStatusProcess;
#endif
  String cmd;
  String cmdResponse;
  unsigned long nextMotorTime;
  unsigned long nextSummaryTime;
  unsigned long nextConsoleTime;
  unsigned long nextTempTime;
  unsigned long nextWifiTime;
  //int cmdMotorCounter;
  //int cmdSummaryCounter;
  //int cmdMotorResponseCounter;
  //int cmdSummaryResponseCounter;
  //void processComm();
  //void processResponse(bool checkCrc);
  //void motorResponse();
  //void summaryResponse();
  void versionResponse();
};

// struct DriverChip defines logic levels how a motor driver works:
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   nPWM               H     Reverse
// 1) if pwm pin is normal (PWM) or inverted (nPWM) for forward
// 2) if pwm pin is normal (PWM) or inverted (nPWM) for reverse
// 3) if direction pin is LOW (or HIGH) for forward
// 4) if direction pin is LOW (or HIGH) for reverse
// 5) if fault signal is active high (or low)
// 6) if enable signal is active high (or low)
// 7) if there is a minimum PWM speed to ensure (or zero)
// 8) the PWM frequency it can work with

/*struct DriverChip {
    char const *driverName;       // name of driver (MC33926 etc.)
    bool forwardPwmInvert;  // forward pin uses inverted pwm?
    bool forwardDirLevel;   // forward pin level
    bool reversePwmInvert;  // reverse pin uses inverted pwm?
    bool reverseDirLevel;   // reverse pin level
    bool usePwmRamp;        // use a ramp to get to PWM value?
    bool faultActive;       // level for fault active (LOW/HIGH)
    bool resetFaultByToggleEnable; // reset a fault by toggling enable?
    bool enableActive;      // level for enable active (LOW/HIGH)
    bool disableAtPwmZeroSpeed; // disable driver at PWM zero speed? (brake function)
    bool keepPwmZeroSpeed;  // keep zero PWM value (disregard minPwmSpeed at zero speed)?
    int minPwmSpeed;        // minimum PWM speed to ensure
    int maxPwmSpeed;        // maximum PWM speed to ensure
    byte pwmFreq;           // PWM frequency (PWM_FREQ_3900 or PWM_FREQ_29300)
    float adcVoltToAmpOfs;  // ADC voltage to amps (offset)     // current (amps)= ((ADCvoltage + ofs)^pow) * scale
    float adcVoltToAmpScale; // ADC voltage to amps (scale)
    float adcVoltToAmpPow;   // ADC voltage to amps (power of number)
    //bool drivesMowingMotor; // drives mowing motor?
};*/

class MeuhMotorDriver: public MotorDriver
{
public:
  unsigned long lastEncoderTicksLeft;
  unsigned long lastEncoderTicksRight;
  unsigned long lastEncoderTicksMow;
  MeuhRobotDriver &meuhRobot;
  MeuhMotorDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) override;
  void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
  void resetMotorFaults()  override;
  void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
  void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
protected:
  uint8_t L_SpiStatus;
  uint8_t R_SpiStatus;
  bool L_MotorFault;
  bool R_MotorFault;
  bool M_MotorFault;
  TMC5160_DRV_STATUS_t L_DrvStatus;
  TMC5160_DRV_STATUS_t R_DrvStatus;
};

class MeuhBatteryDriver : public BatteryDriver
{
public:
  float batteryTemp;
  unsigned long nextTempTime;
  unsigned long linuxShutdownTime;
#ifdef __linux__
  Process batteryTempProcess;
#endif
  MeuhRobotDriver &meuhRobot;
  MeuhBatteryDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  float getBatteryVoltage() override;
  float getChargeVoltage() override;
  float getChargeCurrent() override;
  float getBatteryTemperature() override;
  virtual void enableCharging(bool flag) override;
  virtual void keepPowerOn(bool flag) override;
  void updateBatteryTemperature();
};

class MeuhBumperDriver: public BumperDriver
{
public:
  MeuhRobotDriver &meuhRobot;
  MeuhBumperDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  bool obstacle() override;
  bool getLeftBumper() override;
  bool getRightBumper() override;
  void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;
};

class MeuhStopButtonDriver: public StopButtonDriver
{
public:
  MeuhRobotDriver &meuhRobot;
  MeuhStopButtonDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  bool triggered() override;
};

class MeuhRainSensorDriver: public RainSensorDriver
{
public:
  MeuhRobotDriver &meuhRobot;
  MeuhRainSensorDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  bool triggered() override;
protected:
  unsigned long nextControlTime;
  bool isRaining;
};

class MeuhLiftSensorDriver: public LiftSensorDriver
{
public:
  MeuhRobotDriver &meuhRobot;
  MeuhLiftSensorDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  bool triggered() override;
};

class MeuhBuzzerDriver: public BuzzerDriver
{
public:
  MeuhRobotDriver &meuhRobot;
  MeuhBuzzerDriver(MeuhRobotDriver &sr);
  void begin() override;
  void run() override;
  void noTone() override;
  void tone(int freq) override;
};


#endif

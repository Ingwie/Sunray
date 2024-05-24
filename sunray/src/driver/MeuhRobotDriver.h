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
#include <Process.h>

#include "../meuh/CPeripheryInterface.h"
#include "../meuh/ADS1115/ADS1115_WE.h"
//#include "../meuh/PCF8575/PCF8575.h"
#include "../meuh/TMCStepper-0.7.3/TMCStepper.h"
//-----> Mecanics informations
// Wheel diameter 270 mm
// pulley/belt reduction 12/80
// 1572 steps/meter

//-----> BPI-M4 GPIO Pin
#define pin_i2c_sda_Number        17 // I2C used
#define pin_i2c_scl_Number        18 // I2C used
#define pin_pwm_jyqd_Number       21 // 5V 74HCT541 output was pin_pwm1
#define pin_oe_74HCT541_Number    9  // 5V 74HCT541 output was pin_ur1_tx
#define pin_enable_jyqd_Number    8  // 5V 74HCT541 output was pin_ur1_rx
#define pin_cw_ccw_jyqd_Number    42 // 5V 74HCT541 output was pin_sdio_d0
#define pin_power_relay_Number    3  // 5V 74HCT541 output was aio_bck
#define pin_charge_relay_Number   43 // 5V 74HCT541 output was pin_sdio_d1
#define pin_pin_sdio_d2_Number    44 // 5V 74HCT541 output free
#define pin_ur1_rts_Number        11 // 5V 74HCT541 output
#define pin_ur1_cts_Number        10
#define pin_spi_mosi_Number       31 // SPI used
#define pin_spi_miso_Number       18 // SPI used
#define pin_tmc_3V3_Number        47 // was pin_gpio47
#define pin_spi_sck_Number        19 // SPI used
#define pin_cs_l_tmc_Number       20 // was pin_spi_cs .. Can be used for other task ??
#define pin_cs_r_tmc_Number       22 // was pin_pwm2
#define pin_buzzer_Number         45 // was pin sdio_d3
#define pin_pwm_fan_Number        23 // was pin pwm3
#define pin_sdio_clk_Number       41
#define pin_sdio_cmd_Number       40
#define pin_spdif_Number          50
#define pin_aio_ck_Number         4
#define pin_aio_lrsk_Number       2
#define pin_gpio53_Number         53
#define pin_gpio34_Number         34
#define pin_rain_sensor_Number    5 // 5V -> 3.3V input was pin_aisdz
#define pin_pulses_jyqd_Number    6 // 5V -> 3.3V input was pin_aosd

//-----> ADS1115 Macro resolution 2048 -> 0.0625mV/bit
#define ASD_BAT_CHANNEL         ADS1115_COMP_0_GND
#define ASD_CHARGE_CHANNEL      ADS1115_COMP_1_GND
#define ASD_ACS_CHANNEL         ADS1115_COMP_2_GND
#define ASD_VCC_CHANNEL         ADS1115_COMP_3_GND
#define POT_FACTOR(RMeas, RAds) (RAds + RMeas)/RMeas
#define BAT_POT_FACTOR          15.4711862309 // POT_FACTOR(22000.0f, 300000.0f) // todo measure real values
#define CHARGE_POT_FACTOR       15.8162508059 // POT_FACTOR(22000.0f, 300000.0f)
#define ACS_POT_FACTOR          2.656293181 // POT_FACTOR(4700.0f, 6800.0f)
#define VCC_POT_FACTOR          5.300404035
// ACS712 30A Sensitivity (66mV/A)
#define ACS_MID_VOLTAGE         (vccVoltage/2)
#define ACS_VOLTS_TO_AMPS(x)    (((x * ACS_POT_FACTOR)-ACS_MID_VOLTAGE) / 0.066f)

//-----> TMC settings and helper

struct TMC5160_DRV_STATUS_t
{
  union
  {
    uint32_t sr;
    struct
    {
      uint16_t sg_result : 10;
      uint8_t            : 2;
      bool s2vsa : 1;
      bool s2vsb : 1;
      bool stealth : 1;
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

#define TMC_RsensE           0.22f // ohms
#define TMC_RMS_CURRENT_MA   600.0 // mA
#define TMC_SPEED_MULT       59652.0f/255.0f // pwm to tmc mult value -> 1 M/S Max

//-----> PWM macros used to drive the JYQD
#define JYQD_PWM_PERIOD      10e3 // 0.1mS-10KHz

class MeuhRobotDriver: public RobotDriver
{
public:
  String robotID;
  String mcuFirmwareName = "RobotMeuh";
  String mcuFirmwareVersion = "0.00";
  int lastLeftPwm;
  int lastRightPwm;
  int lastMowPwm;
  uint64_t encoderTicksMow;
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
  bool setFanPowerTune(float temp);
  float readAdcChannel(ADS1115_MUX channel);
  void exitApp(int error);
  void tmcLogicOff();
  void tmcLogicOn();
  void relayStopAll();
  void relayPowerOn();
  void relayChargeOn();
  void set74HCTOutputEnable();
  void set74HCTOutputDisable();
  void getTicksMow();
  //bool setImuPowerState(bool state);

  // counter File
  int ticksMowFD;

protected:
  Process cpuTempProcess;
  Process wifiStatusProcess;
  String cmd;
  //String cmdResponse;
  //unsigned long nextMotorTime;
  //unsigned long nextSummaryTime;
  //unsigned long nextConsoleTime;
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

class MeuhMotorDriver: public MotorDriver
{
public:
  uint32_t lastEncoderTicksLeft;
  uint32_t lastEncoderTicksRight;
  uint64_t lastEncoderTicksMow;
  MeuhRobotDriver &meuhRobot;
  MeuhMotorDriver(MeuhRobotDriver &sr);
  void initTmc5160(TMC5160Stepper &stepper);
  void begin() override;
  void run() override;
  void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) override;
  void printTmcError(TMC5160_DRV_STATUS_t status);
  void checkTmcState(TMC5160Stepper &stepper, TMC5160_DRV_STATUS_t &status, bool &errorBool, float &current, int &lastPwm);
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

class MeuhImuDriver: public ImuDriver
{
public:
  MeuhRobotDriver &meuhRobot;
  MeuhImuDriver(MeuhRobotDriver &sr);
  void detect() override;
  bool begin() override;
  void run() override;
  bool isDataAvail() override;
  void resetData() override;
protected:
  unsigned long nextUpdateTime;
};

#endif

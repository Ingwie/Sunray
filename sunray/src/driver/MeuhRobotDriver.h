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

#include "../meuh/ADS1115/ADS1115_WE.h"
#include "../meuh/PCF8575/PCF8575.h"
#include "../meuh/TMCStepper-0.7.3/TMCStepper.h"

//-----> BPI-M4 GPIO Pin
#define pin_i2c_sda       17 // I2C used
#define pin_i2c_scl       18 // I2C used
#define pin_pwm_jyqd      21 // was pin_pwm1
#define pin_enable_jyqd   9  // was pin_ur1_tx
#define pin_cw_ccw_jyqd   8  // was pin_ur1_tx
#define pin_pulses_jyqd   42 // was pin_sdio_d0
#define pin_aio_bck       3
#define pin_sdio_d1       43
#define pin_sdio_d2       44
#define pin_cs_r_tmc      11 // was pin_ur1_rts
#define pin_cs_l_tmc      10 // was pin_ur1_cts
#define pin_spi_mosi      31 // SPI used
#define pin_spi_miso      18 // SPI used
#define pin_enable_tmc    47 // was pin_gpio47
#define pin_spi_sck       19 // SPI used
#define pin_spi_cs        20 // Can be used for other task ??
#define pin_pwm2          22
#define pin_sdio_d3       45
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
//-----
#define PWM1              1 // use pin_pwm_jyqd
//-----

class MeuhRobotDriver: public RobotDriver {
  public:
    String robotID;
    String mcuFirmwareName;
    String mcuFirmwareVersion;
    int requestLeftPwm;
    int requestRightPwm;
    int requestMowPwm;
    unsigned long encoderTicksLeft;
    unsigned long encoderTicksRight;
    unsigned long encoderTicksMow;
    //bool mcuCommunicationLost;
    bool motorFault;
    float batteryVoltage;
    float chargeVoltage;
    float chargeCurrent;
    float mowCurr;
    float motorLeftCurr;
    float motorRightCurr;
    bool resetMotorTicks;
    float batteryTemp;
    float cpuTemp;
    ADS1115_WE adc;
    PCF8575 pcf8575;
    TMC5160Stepper *R_Stepper;
    TMC5160Stepper *L_Stepper;
    bool triggeredLeftBumper;
    bool triggeredRightBumper;
    bool triggeredLift;
    bool triggeredRain;
    bool triggeredStopButton;
    void begin() override;
    void run() override;
    bool getRobotID(String &id) override;
    bool getMcuFirmwareVersion(String &name, String &ver) override;
    float getCpuTemperature() override;
    void requestMotorPwm(int leftPwm, int rightPwm, int mowPwm);
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

class MeuhMotorDriver: public MotorDriver {
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
  //protected:
    //DriverChip JYQD;
};

class MeuhBatteryDriver : public BatteryDriver {
  public:
    float batteryTemp;
    bool mcuBoardPoweredOn;
    unsigned long nextTempTime;
    unsigned long nextADCTime;
    bool adcTriggered;
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

class MeuhBumperDriver: public BumperDriver {
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

class MeuhStopButtonDriver: public StopButtonDriver {
  public:
    MeuhRobotDriver &meuhRobot;
    MeuhStopButtonDriver(MeuhRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;
};

class MeuhRainSensorDriver: public RainSensorDriver {
  public:
    MeuhRobotDriver &meuhRobot;
    MeuhRainSensorDriver(MeuhRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;
};

class MeuhLiftSensorDriver: public LiftSensorDriver {
  public:
    MeuhRobotDriver &meuhRobot;
    MeuhLiftSensorDriver(MeuhRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;
};

class MeuhBuzzerDriver: public BuzzerDriver {
  public:
    MeuhRobotDriver &meuhRobot;
    MeuhBuzzerDriver(MeuhRobotDriver &sr);
    void begin() override;
    void run() override;
    void noTone() override;
    void tone(int freq) override;
};


#endif

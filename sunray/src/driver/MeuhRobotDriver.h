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
    bool mcuCommunicationLost;
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
    void requestSummary();
    void requestVersion();
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
    int cmdMotorCounter;
    int cmdSummaryCounter;
    int cmdMotorResponseCounter;
    int cmdSummaryResponseCounter;
    void sendRequest(String s);
    void processComm();
    void processResponse(bool checkCrc);
    void motorResponse();
    void summaryResponse();
    void versionResponse();
};

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

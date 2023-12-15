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


#include "MeuhRobotDriver.h"
#include "../../config.h"
//#include "../../ioboard.h"
#include "../meuh/Imu.h"
#include "../meuh/FusionImu.h"


#define COMM  ROBOT

//#define DEBUG_SERIAL_ROBOT 1

// JYQDpusles ISR
volatile unsigned long encoderTicksMow = 0;
void pulsesMowISR()
{
  ++encoderTicksMow;
}

void MeuhRobotDriver::begin()
{
  CONSOLE.println("using robot driver: MeuhRobotDriver");
  COMM.begin(ROBOT_BAUDRATE);

// init GPIO

  pinMode(pin_oe_txs108e, OUTPUT);
  TXS108E_OUTPUT_DISABLE();
  pinMode(pin_power_relay, OUTPUT);
  pinMode(pin_charge_relay, OUTPUT);
  RELAY_STOP_ALL();
// Mow driver (JYQD)
  //pinMode(pin_pwm_jyqd, OUTPUT); // needed by linux pwm driver? no
  pinMode(pin_enable_jyqd, OUTPUT);
  pinMode(pin_cw_ccw_jyqd, OUTPUT);
  pinMode(pin_pulses_jyqd, INPUT);
  attachInterrupt(pin_pulses_jyqd, pulsesMowISR, CHANGE);
  digitalWrite(pin_enable_jyqd, 0); // disable JYQD


// TMC5160 stepper drivers (wheels)
  //pinMode(pin_spi_mosi, OUTPUT);
  //pinMode(pin_spi_miso, OUTPUT);
  //pinMode(pin_spi_sck, OUTPUT);
  //pinMode(pin_cs_r_tmc, OUTPUT);
  //pinMode(pin_cs_l_tmc, OUTPUT);
  pinMode(pin_enable_tmc, OUTPUT); // todo Do i need it ?


  //encoderTicksLeft = 0;
  //encoderTicksRight = 0;
  //encoderTicksMow = 0;
  chargeVoltage = 0;
  chargeCurrent = 0;
  batteryVoltage = 28;
  cpuTemp = 30;
  mowCurr = 0;
  motorLeftCurr = 0;
  motorRightCurr = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
  triggeredStopButton = false;
  triggeredLift = false;
  //mcuCommunicationLost = true;
  nextSummaryTime = 0;
  nextConsoleTime = 0;
  nextMotorTime = 0;
  nextTempTime = 0;
  nextWifiTime = 0;
  //cmdMotorResponseCounter = 0;
  //cmdSummaryResponseCounter = 0;
  //cmdMotorCounter = 0;
  //cmdSummaryCounter = 0;
  lastLeftPwm = lastRightPwm = lastMowPwm = 0;
  robotID = "XX";

  CONSOLE.println("reading robot ID...");
  Process p;
  p.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
  robotID = p.readString();
  robotID.trim();

  // start IMU
  CONSOLE.println("starting IMU");
  // switch-on and configure IMU GY85
  initImusGY85();
  // init fusion computation
  initFusionImu();
  // and read first values to test I2C communications
  computeFusionImu();

  // start IO Expander
  CONSOLE.println("starting IO Expander");
  // init PCF8575
  pcf8575 = PCF8575(0x38);
  if (!pcf8575.isConnected()) CONSOLE.println("IO Expander error");
  // set all pin to input TODO check I/O pin needed and write macros
  pcf8575.write16(0xFFFF);

  // switch-on fan
  //setFanPowerState(true);

  // buzzer test
  if (false)
    {
      CONSOLE.println("buzzer test");
      ///ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
      delay(500);
      ///ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);
    }
  // start ADC
  CONSOLE.println("starting ADC");
  adc = ADS1115_WE(0x48);
  // test communication
  if (!adc.init()) CONSOLE.println("ADC error");
  // set range
  adc.setVoltageRange_mV(ADS1115_RANGE_2048); // 0.0625mV/bit
  // set ref to gnd
  //adc.setCompareChannels(ADS1115_COMP_0_GND | ADS1115_COMP_1_GND | ADS1115_COMP_2_GND | ADS1115_COMP_3_GND);
  // set rate
  adc.setConvRate(ADS1115_250_SPS);
  // set mode
  adc.setMeasureMode(ADS1115_CONTINOUS);
  // ADC test
  if (true)
    {
      delay(5);
      CONSOLE.print("ADC S0 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_0_GND));
      CONSOLE.print("ADC S1 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_1_GND));
      CONSOLE.print("ADC S2 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_2_GND));
      CONSOLE.print("ADC S3 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_3_GND));
    }

  delay(10);
  idleCurrent = ACS_AMPS_TO_VOLTS(readAdcChannel(ASD_ACS_CHANNEL));
  CONSOLE.print("IDLE CURRENT = ");
  CONSOLE.println(idleCurrent);

}

float MeuhRobotDriver::readAdcChannel(ADS1115_MUX channel)   // 8mS @ ADS1115_250_SPS to verify ... and optimise
{
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

bool MeuhRobotDriver::setFanPowerState(bool state)
{
  CONSOLE.print("FAN POWER STATE ");
  CONSOLE.println(state);
  return 1;///ioExpanderOut(EX1_I2C_ADDR, EX1_FAN_POWER_PORT, EX1_FAN_POWER_PIN, state);
}

/*bool MeuhRobotDriver::setImuPowerState(bool state){
  CONSOLE.print("IMU POWER STATE ");
  CONSOLE.println(state);
  return 1; ///ioExpanderOut(EX1_I2C_ADDR, EX1_IMU_POWER_PORT, EX1_IMU_POWER_PIN, state);
}*/

bool MeuhRobotDriver::getRobotID(String &id)
{
  id = robotID;
  return true;
}

bool MeuhRobotDriver::getMcuFirmwareVersion(String &name, String &ver)
{
  name = mcuFirmwareName;
  ver = mcuFirmwareVersion;
  return true;
}

float MeuhRobotDriver::getCpuTemperature()
{
  return cpuTemp;
}

void MeuhRobotDriver::updateCpuTemperature()
{
  //unsigned long startTime = millis();
  String s;
  while (cpuTempProcess.available()) s+= (char)cpuTempProcess.read();
  if (s.length() > 0)
    {
      cpuTemp = s.toFloat() / 1000.0;
      //CONSOLE.print("updateCpuTemperature cpuTemp=");
      //CONSOLE.println(cpuTemp);
    }
  cpuTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone0/temp");
  //unsigned long duration = millis() - startTime;
  //CONSOLE.print("updateCpuTemperature duration: ");
  //CONSOLE.println(duration);
}

void MeuhRobotDriver::updateWifiConnectionState()
{
  //unsigned long startTime = millis();
  String s;
  while (wifiStatusProcess.available()) s+= (char)wifiStatusProcess.read();
  if (s.length() > 0)
    {
      s.trim();
      //CONSOLE.print("updateWifiConnectionState state=");
      //CONSOLE.println(s);
      // DISCONNECTED, SCANNING, INACTIVE, COMPLETED
      //CONSOLE.println(s);
      ledStateWifiConnected = (s == "COMPLETED");
      ledStateWifiInactive = (s == "INACTIVE");
    }
  wifiStatusProcess.runShellCommand("wpa_cli -i wlan0 status | grep wpa_state | cut -d '=' -f2");
  //unsigned long duration = millis() - startTime;
  //CONSOLE.print("updateWifiConnectionState duration: ");
  //CONSOLE.println(duration);
}

// request MCU motor PWM
//void MeuhRobotDriver::requestMotorPwm(int leftPwm, int rightPwm, int mowPwm){}


void MeuhRobotDriver::versionResponse()
{
  if (cmd.length()<6) return;
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++)
    {
      char ch = cmd[idx];
      //Serial.print("ch=");
      //Serial.println(ch);
      if ((ch == ',') || (idx == cmd.length()-1))
        {
          String s = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1);
          if (counter == 1)
            {
              mcuFirmwareName = s;
            }
          else if (counter == 2)
            {
              mcuFirmwareVersion = s;
            }
          counter++;
          lastCommaIdx = idx;
        }
    }
  CONSOLE.print("MCU FIRMWARE: ");
  CONSOLE.print(mcuFirmwareName);
  CONSOLE.print(",");
  CONSOLE.println(mcuFirmwareVersion);
}


void MeuhRobotDriver::run()
{
  //processComm();
  if (millis() > nextMotorTime)
    {
      nextMotorTime = millis() + 20; // 50 hz
      //requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
    }
  if (millis() > nextSummaryTime)
    {
      nextSummaryTime = millis() + 500; // 2 hz
      //requestSummary();
    }
  if (millis() > nextConsoleTime)
    {
      nextConsoleTime = millis() + 1000;  // 1 hz

    }
  if (millis() > nextTempTime)
    {
      nextTempTime = millis() + 59000; // 59 sec
      updateCpuTemperature();
      if (cpuTemp < 60)
        {
          setFanPowerState(false);
        }
      else if (cpuTemp > 65)
        {
          setFanPowerState(true);
        }
    }
  if (millis() > nextWifiTime)
    {
      nextWifiTime = millis() + 7000; // 7 sec
      updateWifiConnectionState();
    }
}


// ------------------------------------------------------------------------------------

MeuhMotorDriver::MeuhMotorDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
}

void MeuhMotorDriver::begin()
{
  lastEncoderTicksLeft = lastEncoderTicksRight = lastEncoderTicksMow = 0;
  L_MotorFault = R_MotorFault = M_MotorFault = false;
  R_DrvStatus.sr = L_DrvStatus.sr = 0;
  L_SpiStatus, R_SpiStatus = 0;

  PWM1_INIT(); // Jyqd pwm (maw)

  SPI.begin();

  // start TMC5160 stepper drivers (wheels)
  CONSOLE.println("starting TMC5160");
  TMC5160Stepper R_Stepper = TMC5160Stepper(pin_cs_r_tmc, TMC_RsensE);
  TMC5160Stepper L_Stepper = TMC5160Stepper(pin_cs_l_tmc, TMC_RsensE);
  START_TMC_SEQUENCE(R_Stepper); // A lot of todo to switch to speed mode, collstep, stallguard .... and test
  START_TMC_SEQUENCE(L_Stepper);

  // power on motors
  TXS108E_OUTPUT_ENABLE();
  RELAY_POWER_ON();
  digitalWrite(pin_enable_tmc, 1); // check if needed

  // Check current
  meuhRobot.stepperCurrent = ACS_AMPS_TO_VOLTS(meuhRobot.readAdcChannel(ASD_ACS_CHANNEL)); // measure actual current steppers actives
  meuhRobot.stepperCurrent -= meuhRobot.idleCurrent; // remove offset  todo check excess
  CONSOLE.print("IDLE STEPPERS CURRENT = ");
  CONSOLE.println(meuhRobot.stepperCurrent);


  // Check spi_status
  R_SpiStatus = R_Stepper.status_response;
  L_SpiStatus = L_Stepper.status_response;

}

void MeuhMotorDriver::run()
{
}

void MeuhMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm)
{
  // JYQD
  if (mowPwm == 0)  // stop
    {
      digitalWrite(pin_enable_jyqd, 0); // todo test active brake
      SETPWM1DUTYCYCLE(0);
    }
  else
    {
      if (mowPwm > 0)
        {
          digitalWrite(pin_enable_jyqd, 0);
          SETPWM1DUTYCYCLE(mowPwm);
        }
      else
        {
          digitalWrite(pin_enable_jyqd, 1);
          SETPWM1DUTYCYCLE(abs(mowPwm));
        }
    }

  // TMC 5160
  if (leftPwm == 0)  // stop
    {
      L_Stepper->VMAX(0);
    }
  else
    {
      if (leftPwm > 0)
        {
          L_Stepper->RAMPMODE(1); // Velocity switch to positive
          L_Stepper->VMAX(leftPwm * TMC_SPEED_MULT);
        }
      else
        {
          L_Stepper->RAMPMODE(2); // Velocity switch to negative
          L_Stepper->VMAX(leftPwm * TMC_SPEED_MULT);
        }
    }
// Check spi_status
  L_SpiStatus = L_Stepper->status_response;

  if (rightPwm == 0)  // stop
    {
      R_Stepper->VMAX(0);
    }
  else
    {
      if (rightPwm > 0)
        {
          L_Stepper->RAMPMODE(1); // Velocity switch to positive
          R_Stepper->VMAX(rightPwm * TMC_SPEED_MULT);
        }
      else
        {
          R_Stepper->RAMPMODE(2); // Velocity switch to negative
          R_Stepper->VMAX(rightPwm * TMC_SPEED_MULT);
        }
    }
// Check spi_status
  R_SpiStatus = R_Stepper-> status_response;

  meuhRobot.lastLeftPwm = leftPwm;
  meuhRobot.lastRightPwm = rightPwm;
  meuhRobot.lastMowPwm = mowPwm;

}

void MeuhMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault)
{

  bool M_Error = false;
  bool L_Error = false;
  bool R_Error = false;

  CHECK_AND_COMPUTE_TMC_ERROR(L_DrvStatus, L_Stepper, L_SpiStatus, L_Error, meuhRobot.motorLeftCurr);
  CHECK_AND_COMPUTE_TMC_ERROR(R_DrvStatus, R_Stepper, R_SpiStatus, R_Error, meuhRobot.motorRightCurr);

  if (encoderTicksMow == 0) M_Error = true;

  if (L_Error)
    {
      CONSOLE.print("motorFault (lefCurr=");
      CONSOLE.print(meuhRobot.motorLeftCurr);
    }
  if (R_Error)
    {
      CONSOLE.print("motorFault (rightCurr=");
      CONSOLE.print(meuhRobot.motorRightCurr);
    }
  if (M_Error)
    {
      CONSOLE.print(" mowCurr=");
      CONSOLE.println(meuhRobot.mowCurr);
    }

  // send states
  leftFault = L_Error;
  rightFault = R_Error;
  mowFault = M_Error;;
}

void MeuhMotorDriver::resetMotorFaults()
{
  CONSOLE.println("meuhRobot: resetting motor fault");
  digitalWrite(pin_enable_jyqd, 0); // disable JYQD
  delay(500);
  digitalWrite(pin_enable_jyqd, 10); // enable JYQD
}

void MeuhMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent)
{
  leftCurrent = meuhRobot.motorLeftCurr;
  rightCurrent = meuhRobot.motorRightCurr;
  meuhRobot.mowCurr = ACS_AMPS_TO_VOLTS(meuhRobot.readAdcChannel(ASD_ACS_CHANNEL)); // measure actual current
  meuhRobot.mowCurr -= (meuhRobot.stepperCurrent + meuhRobot.idleCurrent); //remove offset
  mowCurrent = meuhRobot.mowCurr;
}

void MeuhMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks)
{

  int32_t actualTicksLeft = L_Stepper->XACTUAL();
  int32_t actualTicksRight = R_Stepper->XACTUAL();

  leftTicks = abs(actualTicksLeft - lastEncoderTicksLeft);
  rightTicks = abs(actualTicksRight - lastEncoderTicksRight);
  mowTicks = encoderTicksMow;

  lastEncoderTicksLeft = actualTicksLeft;
  lastEncoderTicksRight = actualTicksRight;
  lastEncoderTicksMow = encoderTicksMow;
  encoderTicksMow = 0;
}


// ------------------------------------------------------------------------------------

MeuhBatteryDriver::MeuhBatteryDriver(MeuhRobotDriver &sr) : meuhRobot(sr)
{
  nextTempTime = 0;
  batteryTemp = 0;
  linuxShutdownTime = 0;
}

void MeuhBatteryDriver::begin()
{
}

void MeuhBatteryDriver::run()
{
  if (millis() > nextTempTime)
    {
      nextTempTime = millis() + 57000; // 57 sec
      updateBatteryTemperature();
    }
}

void MeuhBatteryDriver::updateBatteryTemperature()
{
  //unsigned long startTime = millis();
  String s;
  while (batteryTempProcess.available()) s+= (char)batteryTempProcess.read();
  if (s.length() > 0)
    {
      batteryTemp = s.toFloat() / 1000.0;
      //CONSOLE.print("updateBatteryTemperature batteryTemp=");
      //CONSOLE.println(batteryTemp);
    }
  batteryTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone1/temp");
  //unsigned long duration = millis() - startTime;
  //CONSOLE.print("updateBatteryTemperature duration: ");
  //CONSOLE.println(duration);
}


float MeuhBatteryDriver::getBatteryTemperature()
{
  return -9999; //batteryTemp; // linux reported bat temp not useful as seem to be constant 31 degree
}

float MeuhBatteryDriver::getBatteryVoltage()
{
  meuhRobot.batteryVoltage = (meuhRobot.readAdcChannel(ASD_BAT_CHANNEL) * BAT_POT_FACTOR);
  return meuhRobot.batteryVoltage;
}

float MeuhBatteryDriver::getChargeVoltage()
{
  meuhRobot.chargeVoltage = (meuhRobot.readAdcChannel(ASD_CHARGE_CHANNEL) * CHARGE_POT_FACTOR);
  return meuhRobot.chargeVoltage;
}

float MeuhBatteryDriver::getChargeCurrent()
{
  meuhRobot.chargeCurrent = ACS_AMPS_TO_VOLTS(meuhRobot.readAdcChannel(ASD_ACS_CHANNEL));
  return meuhRobot.chargeCurrent;
}

void MeuhBatteryDriver::enableCharging(bool flag)
{
  if (flag)
    {
      RELAY_CHARGE_ON();
    }
  else
    {
      RELAY_STOP_ALL();
    }
}


void MeuhBatteryDriver::keepPowerOn(bool flag)
{
  if (flag)
    {
      // keep power on
      linuxShutdownTime = 0;
      meuhRobot.ledStateShutdown = false;
    }
  else
    {
      // shutdown linux - request could be for two reasons:
      // 1. battery voltage sent by MUC-PCB seem to be too low
      // 2. MCU-PCB is powered-off
      if (millis() > linuxShutdownTime)
        {
          linuxShutdownTime = millis() + 10000; // re-trigger linux command after 10 secs
          CONSOLE.println("LINUX will SHUTDOWN!");
          // switch-off fan via port-expander PCA9555
          meuhRobot.setFanPowerState(false);
          Process p;
          p.runShellCommand("shutdown now");
        }
    }
}


// ------------------------------------------------------------------------------------

MeuhBumperDriver::MeuhBumperDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
}

void MeuhBumperDriver::begin()
{
}

void MeuhBumperDriver::run()
{

}

bool MeuhBumperDriver::obstacle()
{
  return (meuhRobot.triggeredLeftBumper || meuhRobot.triggeredRightBumper);
}

bool MeuhBumperDriver::getLeftBumper()
{
  return (meuhRobot.triggeredLeftBumper);
}

bool MeuhBumperDriver::getRightBumper()
{
  return (meuhRobot.triggeredRightBumper);
}

void MeuhBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper)
{
  leftBumper = meuhRobot.triggeredLeftBumper;
  rightBumper = meuhRobot.triggeredRightBumper;
}


// ------------------------------------------------------------------------------------


MeuhStopButtonDriver::MeuhStopButtonDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
}

void MeuhStopButtonDriver::begin()
{
}

void MeuhStopButtonDriver::run()
{

}

bool MeuhStopButtonDriver::triggered()
{
  return (meuhRobot.triggeredStopButton);
}

// ------------------------------------------------------------------------------------


MeuhRainSensorDriver::MeuhRainSensorDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
}

void MeuhRainSensorDriver::begin()
{
  nextControlTime = 0;
  isRaining = false;
  pinMode(pin_rain_sensor, INPUT);
}

void MeuhRainSensorDriver::run()
{
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 10000;    // save CPU resources by running at 0.1 Hz
  isRaining = (digitalRead(pin_rain_sensor) == LOW);
}

bool MeuhRainSensorDriver::triggered()
{
  return (isRaining);
}

// ------------------------------------------------------------------------------------

MeuhLiftSensorDriver::MeuhLiftSensorDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
}

void MeuhLiftSensorDriver::begin()
{
}

void MeuhLiftSensorDriver::run()
{
}

bool MeuhLiftSensorDriver::triggered()
{
  return (meuhRobot.triggeredLift);
}


// ------------------------------------------------------------------------------------

MeuhBuzzerDriver::MeuhBuzzerDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
}

void MeuhBuzzerDriver::begin()
{
  pinMode(pin_buzzer, OUTPUT);
  digitalWrite(pin_buzzer, LOW);
}

void MeuhBuzzerDriver::run()
{
}

void MeuhBuzzerDriver::noTone()
{
  digitalWrite(pin_buzzer, LOW);
}

void MeuhBuzzerDriver::tone(int freq)
{
  digitalWrite(pin_buzzer, HIGH);
}




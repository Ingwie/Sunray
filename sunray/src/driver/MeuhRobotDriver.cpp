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
#include "../meuh/Imu.h"
#include "../meuh/FusionImu.h"
#include <fcntl.h>

#define COMM  ROBOT

// gpio pin pointer
  gpio_t * pin_i2c_sda;
  gpio_t * pin_i2c_scl;
  gpio_t * pin_pwm_jyqd;
  gpio_t * pin_oe_74HCT541;
  gpio_t * pin_enable_jyqd;
  gpio_t * pin_cw_ccw_jyqd;
  gpio_t * pin_power_relay;
  gpio_t * pin_charge_relay;
  gpio_t * pin_sdio_d2;
  gpio_t * pin_ur1_rts;
  gpio_t * pin_ur1_cts;
  gpio_t * pin_spi_mosi;
  gpio_t * pin_spi_miso;
  gpio_t * pin_tmc_3V3;
  gpio_t * pin_spi_sck;
  gpio_t * pin_cs_r_tmc;
  gpio_t * pin_cs_l_tmc;
  gpio_t * pin_buzzer;
  gpio_t * pin_pwm_fan;
  gpio_t * pin_sdio_clk;
  gpio_t * pin_sdio_cmd;
  gpio_t * pin_spdif;
  gpio_t * pin_aio_ck;
  gpio_t * pin_aio_lrsk;
  gpio_t * pin_gpio53;
  gpio_t * pin_gpio34;
  gpio_t * pin_pulses_jyqd;
  gpio_t * pin_rain_sensor;

// pwm pointer
  pwm_t * pwmFan;

TMC5160Stepper R_Stepper(pin_cs_r_tmc, TMC_RsensE);
TMC5160Stepper L_Stepper(pin_cs_l_tmc, TMC_RsensE);
I2CC I2C;
bool relayCharge;
bool relayPower;
bool tmc3V3Powered;


void MeuhRobotDriver::begin()
{
  CONSOLE.println("using robot driver: MeuhRobotDriver");
  COMM.begin(ROBOT_BAUDRATE);

// init GPIO

  SetGpioPin(pin_cs_r_tmc, GPIO_DIR_OUT_HIGH);
  SetGpioPin(pin_cs_l_tmc, GPIO_DIR_OUT_HIGH);
  SetGpioPin(pin_tmc_3V3, GPIO_DIR_OUT_HIGH);
  SetGpioPin(pin_oe_74HCT541, GPIO_DIR_OUT_HIGH);
  SetGpioPin(pin_power_relay, GPIO_DIR_OUT_LOW);
  SetGpioPin(pin_charge_relay, GPIO_DIR_OUT_LOW);
  tmcLogicOff();
  set74HCTOutputDisable();
  relayStopAll();
  SetGpioPin(pin_buzzer, GPIO_DIR_OUT_LOW);
  //SetGpioPin(pin_pwm_fan, GPIO_DIR_OUT); // needed by linux pwm driver? no
  SetNewPwm(pwmFan, 3); // Jyqd pwm (maw)
  PwmSetFrequency(pwmFan, 10e3);
  SetGpioPin(pin_rain_sensor, GPIO_DIR_IN);

// Mow driver (JYQD)
  //SetGpioPin(pin_pwm_jyqd, GPIO_DIR_OUT); // needed by linux pwm driver? no
  SetGpioPin(pin_enable_jyqd, GPIO_DIR_OUT_LOW);
  SetGpioPin(pin_cw_ccw_jyqd, GPIO_DIR_OUT_LOW);
  SetGpioPin(pin_pulses_jyqd, GPIO_DIR_IN);
  //GpioPinWrite(pin_enable_jyqd, 0); // disable JYQD


// TMC5160 stepper drivers (wheels)
  //SetGpioPin(pin_spi_mosi, GPIO_DIR_OUT);
  //SetGpioPin(pin_spi_miso, GPIO_DIR_IN);
  //SetGpioPin(pin_spi_sck, GPIO_DIR_OUT);

  //encoderTicksLeft = 0;
  //encoderTicksRight = 0;
  encoderTicksMow = 0;
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

  // start IO Expander
  //CONSOLE.println("starting IO Expander");
  // init PCF8575
  //pcf8575 = PCF8575(0x38);
  //if (!pcf8575.isConnected()) CONSOLE.println("IO Expander error");
  // set all pin to input TODO check I/O pin needed and write macros
  //pcf8575.write16(0xFFFF);

  // switch-on fan
  setFanPowerTune(60);

  // buzzer test
  if (false)
    {
      CONSOLE.println("buzzer test");
      GpioPinWrite(pin_buzzer, 1);
      delay(500);
      GpioPinWrite(pin_buzzer, 0);
    }

  CONSOLE.println("starting I2C bus");
  I2C.begin();
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
      unsigned long startTime = millis();
      CONSOLE.print("ADC S0 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_0_GND));
      CONSOLE.print("ADC S1 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_1_GND));
      CONSOLE.print("ADC S2 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_2_GND));
      CONSOLE.print("ADC S3 = ");
      CONSOLE.println(readAdcChannel(ADS1115_COMP_3_GND));
      CONSOLE.print("Four ADC conversion duration: ");
      CONSOLE.println((uint32_t) millis() - startTime);
    }

  delay(10);
  idleCurrent = ACS_AMPS_TO_VOLTS(readAdcChannel(ASD_ACS_CHANNEL));
  CONSOLE.print("IDLE CURRENT = ");
  CONSOLE.println(idleCurrent);

  if ((ticksMowFD = open("/sys/class/gpio-counter/GPIO-5/value", O_WRONLY | O_SYNC)) < 0)
  {
    CONSOLE.println("Can't open MowCouter value");
    exitApp();
  }
}

void MeuhRobotDriver::getTicksMow()
{
  // TODO call kernel module counter !!
}

void MeuhRobotDriver::tmcLogicOn()
{
  GpioPinWrite(pin_tmc_3V3, 0);
  tmc3V3Powered = true;
}

void MeuhRobotDriver::tmcLogicOff()
{
  /* 24V must be off before turning logic off */
  GpioPinWrite(pin_power_relay, 0);
  relayPower = false;
  delay(100);
  GpioPinWrite(pin_tmc_3V3, 1);
  tmc3V3Powered = false;
}

void MeuhRobotDriver::relayStopAll()
{
  GpioPinWrite(pin_power_relay, 0);
  relayPower = false;
  GpioPinWrite(pin_charge_relay, 0);
  relayCharge = false;
}

void MeuhRobotDriver::relayPowerOn()
{
  GpioPinWrite(pin_charge_relay, 0);
  relayCharge = false;
  delay(100);
  if (tmc3V3Powered == true)
    {
      GpioPinWrite(pin_power_relay, 1);
      relayPower = true;
    }
  else CONSOLE.println("TMC logic supply is missing !");
}

void MeuhRobotDriver::relayChargeOn()
{
  GpioPinWrite(pin_power_relay, 0);
  relayPower = false;
  delay(100);
  GpioPinWrite(pin_charge_relay, 1);
  relayCharge = true;
}

void MeuhRobotDriver::exitApp() // Close sunray
{
  ///relayStopAll(); // turn OFF power boards before quit
  ///tmcLogicOff();
  ///set74HCTOutputDisable();
  ///exit(1);
}

// level converter ship 74HCT541 functions (security)
void MeuhRobotDriver::set74HCTOutputEnable()
{
  GpioPinWrite(pin_oe_74HCT541, 0);
}

void MeuhRobotDriver::set74HCTOutputDisable()
{
  GpioPinWrite(pin_oe_74HCT541, 1);
}

float MeuhRobotDriver::readAdcChannel(ADS1115_MUX channel)   // 8mS @ ADS1115_250_SPS to verify ... and optimise
{
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

bool MeuhRobotDriver::setFanPowerTune(float temp)
{
  CONSOLE.print("FAN POWER STATE ");
  CONSOLE.println(temp);
  PwmSetDutyCycle(pwmFan, map(temp, 40, 80, 0, 1));
  return 1;
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
      setFanPowerTune(cpuTemp);
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

  CONSOLE.println("starting JYQD PWM");
  SetNewPwm(pwmJYQD, 1); // Jyqd pwm (maw)
  //PwmSetPolarity(pwmJYQD, PWM_POLARITY_INVERSED);
  PwmSetFrequency(pwmJYQD, JYQD_PWM_PERIOD);

  CONSOLE.println("starting SPI bus");
  SPI.begin();
  SPISettings dummy;
  SPI.beginTransaction(dummy); // SPISetting are hard coded in CPeripheryInterface.cpp
  SPI.transfer(0xFF); // initial dummy transfert


  // start TMC5160 stepper drivers (wheels)
  CONSOLE.println("starting TMC5160");
  meuhRobot.tmcLogicOn();
  delay(5);
  ///meuhRobot.set74HCTOutputEnable();
  ///meuhRobot.relayPowerOn();
  R_Stepper.begin();
  uint8_t rVers = R_Stepper.version();
  L_Stepper.begin();
  uint8_t lVers = L_Stepper.version();
  CONSOLE.print("TMC versions:");
  CONSOLE.print(rVers);
  CONSOLE.print(" - ");
  CONSOLE.println(lVers);
  if ((rVers == 0xFF) || (lVers == 0xFF))
    {
      CONSOLE.println("No TMC communication - Sunray close ");
      meuhRobot.exitApp();
    }
  START_TMC_SEQUENCE(R_Stepper); // A lot of todo to switch to speed mode, collstep, stallguard .... and test
  START_TMC_SEQUENCE(L_Stepper);

  // Check spi_status
  R_SpiStatus = R_Stepper.status_response;
  L_SpiStatus = L_Stepper.status_response;

}

void MeuhMotorDriver::run()
{
}

void MeuhMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm)
{
  if (!relayPower)
    {
      // power on motors
      meuhRobot.set74HCTOutputEnable();
      meuhRobot.relayPowerOn();
      // Check current
      meuhRobot.stepperCurrent = ACS_AMPS_TO_VOLTS(meuhRobot.readAdcChannel(ASD_ACS_CHANNEL)); // measure actual current steppers actives
      meuhRobot.stepperCurrent -= meuhRobot.idleCurrent; // remove offset  todo check excess
      CONSOLE.print("IDLE STEPPERS CURRENT = ");
      CONSOLE.println(meuhRobot.stepperCurrent);
    }

  // JYQD
  if (mowPwm == 0)  // stop
    {
      PwmSetDutyCycle(pwmJYQD, 0);
      // delay(300); // todo test active brake
      GpioPinWrite(pin_enable_jyqd, 0);
    }
  else
    {
      GpioPinWrite(pin_enable_jyqd, 1);
      if (mowPwm > 0)
        {
          GpioPinWrite(pin_cw_ccw_jyqd, 0);
          PwmSetDutyCycle(pwmJYQD, map(mowPwm, 0, 255, 0, 1));
        }
      else
        {
          GpioPinWrite(pin_cw_ccw_jyqd, 1);
          PwmSetDutyCycle(pwmJYQD, map(abs(mowPwm), 0, 255, 0, 1));
        }
    }

  // TMC 5160
  if (leftPwm == 0)  // stop
    {
      L_Stepper.VMAX(0);
    }
  else
    {
      if (leftPwm > 0)
        {
          L_Stepper.RAMPMODE(1); // Velocity switch to positive
          L_Stepper.VMAX(leftPwm * TMC_SPEED_MULT);
        }
      else
        {
          L_Stepper.RAMPMODE(2); // Velocity switch to negative
          L_Stepper.VMAX(leftPwm * TMC_SPEED_MULT);
        }
    }
// Check spi_status
  L_SpiStatus = L_Stepper.status_response;

  if (rightPwm == 0)  // stop
    {
      R_Stepper.VMAX(0);
    }
  else
    {
      if (rightPwm > 0)
        {
          L_Stepper.RAMPMODE(1); // Velocity switch to positive
          R_Stepper.VMAX(rightPwm * TMC_SPEED_MULT);
        }
      else
        {
          R_Stepper.RAMPMODE(2); // Velocity switch to negative
          R_Stepper.VMAX(rightPwm * TMC_SPEED_MULT);
        }
    }
// Check spi_status
  R_SpiStatus = R_Stepper.status_response;

  meuhRobot.lastLeftPwm = leftPwm;
  meuhRobot.lastRightPwm = rightPwm;
  meuhRobot.lastMowPwm = mowPwm;

}

void MeuhMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault)
{

  CHECK_AND_COMPUTE_TMC_ERROR(L_DrvStatus, L_Stepper, L_SpiStatus, L_MotorFault, meuhRobot.motorLeftCurr);
  CHECK_AND_COMPUTE_TMC_ERROR(R_DrvStatus, R_Stepper, R_SpiStatus, R_MotorFault, meuhRobot.motorRightCurr);

  if ((meuhRobot.lastMowPwm != 0) && (lastEncoderTicksMow == 0))
    {
      M_MotorFault = true;
    }

  if (L_MotorFault)
    {
      CONSOLE.print("motorFault (lefCurr=");
      CONSOLE.print(meuhRobot.motorLeftCurr);
    }
  if (R_MotorFault)
    {
      CONSOLE.print("motorFault (rightCurr=");
      CONSOLE.print(meuhRobot.motorRightCurr);
    }
  if (M_MotorFault)
    {
      CONSOLE.print(" mowCurr=");
      CONSOLE.println(meuhRobot.mowCurr);
    }

  // send states
  leftFault = L_MotorFault;
  rightFault = R_MotorFault;
  mowFault = M_MotorFault;;
}

void MeuhMotorDriver::resetMotorFaults()
{
  CONSOLE.println("meuhRobot: resetting motor fault");
  if (M_MotorFault)
    {
      PwmSetDutyCycle(pwmJYQD, 0);
      GpioPinWrite(pin_enable_jyqd, 0); // disable JYQD
      delay(500);
      GpioPinWrite(pin_enable_jyqd, 1); // enable JYQD
    }

  if (L_MotorFault)
    {
      if (L_DrvStatus.stallGuard) L_MotorFault = false;
      else meuhRobot.exitApp(); // prefer stop all
    }

  if (R_MotorFault)
    {
      if (R_DrvStatus.stallGuard) R_MotorFault = false;
      else meuhRobot.exitApp();
    }
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
  int32_t actualTicksLeft = L_Stepper.XACTUAL();
  int32_t actualTicksRight = R_Stepper.XACTUAL();

  leftTicks = abs(actualTicksLeft - lastEncoderTicksLeft);
  rightTicks = abs(actualTicksRight - lastEncoderTicksRight);
  mowTicks = meuhRobot.encoderTicksMow;

  lastEncoderTicksLeft = actualTicksLeft;
  lastEncoderTicksRight = actualTicksRight;
  lastEncoderTicksMow = meuhRobot.encoderTicksMow;
  meuhRobot.encoderTicksMow = 0;
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
  return batteryTemp; // linux reported bat temp not useful as seem to be constant 31 degree
}

float MeuhBatteryDriver::getBatteryVoltage()
{
  meuhRobot.batteryVoltage = 20.5;///(meuhRobot.readAdcChannel(ASD_BAT_CHANNEL) * BAT_POT_FACTOR);
  return meuhRobot.batteryVoltage;
}

float MeuhBatteryDriver::getChargeVoltage()
{
  meuhRobot.chargeVoltage = 28.4;///(meuhRobot.readAdcChannel(ASD_CHARGE_CHANNEL) * CHARGE_POT_FACTOR);
  return meuhRobot.chargeVoltage;
}

float MeuhBatteryDriver::getChargeCurrent()
{
  meuhRobot.chargeCurrent = ACS_AMPS_TO_VOLTS(meuhRobot.readAdcChannel(ASD_ACS_CHANNEL)) + meuhRobot.idleCurrent;
  return meuhRobot.chargeCurrent;
}

void MeuhBatteryDriver::enableCharging(bool flag)
{
  if (flag)
    {
      meuhRobot.relayChargeOn();
    }
  else
    {
      meuhRobot.relayStopAll();
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
      // 1. battery voltage seem to be too low
      // 2. MCU-PCB is powered-off
      if (millis() > linuxShutdownTime)
        {
          linuxShutdownTime = millis() + 10000; // re-trigger linux command after 10 secs
          CONSOLE.println("LINUX will SHUTDOWN!");
          // switch-off fan via port-expander PCA9555
          meuhRobot.setFanPowerTune(false);
          meuhRobot.relayStopAll();
          meuhRobot.tmcLogicOff();
          meuhRobot.set74HCTOutputDisable();

          Process p;
          ///p.runShellCommand("shutdown now");
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
  return (meuhRobot.triggeredLeftBumper || meuhRobot.triggeredRightBumper); // todo use stallguard
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
}

void MeuhRainSensorDriver::run()
{
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 10000;    // save CPU resources by running at 0.1 Hz
  bool val;
  GpioPinRead(pin_rain_sensor, val);
  isRaining = (!val);
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
  GpioPinWrite(pin_buzzer, LOW);
}

void MeuhBuzzerDriver::run()
{
}

void MeuhBuzzerDriver::noTone()
{
  GpioPinWrite(pin_buzzer, LOW);
}

void MeuhBuzzerDriver::tone(int freq)
{
  GpioPinWrite(pin_buzzer, HIGH);
}

// ------------------------------------------------------------------------------------

MeuhImuDriver::MeuhImuDriver(MeuhRobotDriver &sr): meuhRobot(sr)
{
  nextUpdateTime = 0;
}

void MeuhImuDriver::detect()
{
  imuFound = true;
}

bool MeuhImuDriver::begin()
{
  CONSOLE.println("using imu driver: GY85 + FUSION code");
  // switch-on and configure IMU GY85
  initImusGY85();
  // init fusion computation
  initFusionImu();
  // and read first values to test I2C communications
  unsigned long startTime = millis();
  bool ret = computeFusionImu();
  CONSOLE.print("Fusion imu read and computation duration: ");
  CONSOLE.println((uint32_t) millis() - startTime);
  CONSOLE.print("ROLL: ");
  CONSOLE.println(eulerAngles.angle.roll);
  CONSOLE.print("PITCH: ");
  CONSOLE.println(eulerAngles.angle.pitch);
  CONSOLE.print("YAW: ");
  CONSOLE.println(eulerAngles.angle.yaw);
  CONSOLE.print("Heading: ");
  CONSOLE.println(fusionHeading);
  return ret;
}

void MeuhImuDriver::run()
{
}

bool MeuhImuDriver::isDataAvail()
{
  if (millis() < nextUpdateTime) return false;
  nextUpdateTime = millis() + 200; // 5 Hz update FUSIONPERIOD if changed
  bool ret = computeFusionImu();
  //quatW = ?; not used
  //quatX = ?;
  //quatY = ?;
  //quatZ = ?;
  roll = eulerAngles.angle.roll / 180.0 * PI;
  pitch = eulerAngles.angle.pitch / 180.0 * PI;
  yaw = eulerAngles.angle.yaw / 180.0 * PI;
  //heading = fusionHeading;
  return ret;
}

void MeuhImuDriver::resetData()
{
}



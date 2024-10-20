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
#include "../NutsBolts.h"
#include "../config.h"
#include "../meuh/Imu.h"
#include "../meuh/FusionImu.h"
#include <fcntl.h>

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
gpio_t * pin_cur_pol;
gpio_t * pin_aio_lrsk;
gpio_t * pin_gpio53;
gpio_t * pin_gpio34;
gpio_t * pin_rain_sensor;
gpio_t * pin_pulses_jyqd;

// pwm pointer
pwm_t * pwmJYQD;
pwm_t * pwmFan;

// stepper drivers
TMC5160Stepper R_Stepper(pin_cs_r_tmc_Number, pin_cs_r_tmc, TMC_RsensE);
TMC5160Stepper L_Stepper(pin_cs_l_tmc_Number, pin_cs_l_tmc, TMC_RsensE);

// I2C driver
I2CC I2C;

bool relayCharge;
bool relayPower;
bool tmc3V3Powered;

float vccVoltage = 0.0;

void MeuhRobotDriver::begin()
{
  wxLogMessage("using robot driver: MeuhRobotDriver");

// init GPIO

  SetGpioPin(pin_tmc_3V3, GPIO_DIR_OUT_HIGH);
  SetGpioPin(pin_oe_74HCT541, GPIO_DIR_OUT_HIGH);
  SetGpioPin(pin_power_relay, GPIO_DIR_OUT_LOW);
  SetGpioPin(pin_charge_relay, GPIO_DIR_OUT_LOW);
  tmcLogicOff();
  set74HCTOutputDisable();
  SetGpioPin(pin_buzzer, GPIO_DIR_OUT_LOW);
  //SetGpioPin(pin_pwm_fan, GPIO_DIR_OUT); // needed by linux pwm driver? no
  SetNewPwm(pwmFan, 3); // Jyqd pwm (mow)
  PwmSetFrequency(pwmFan, 10e3);
  PwmSetDutyCycle(pwmFan, 0.0);
  SetGpioPin(pin_rain_sensor, GPIO_DIR_IN);

// Mow driver (JYQD)
  //SetGpioPin(pin_pwm_jyqd, GPIO_DIR_OUT); // needed by linux pwm driver? no
  SetGpioPin(pin_enable_jyqd, GPIO_DIR_OUT_LOW);
  SetGpioPin(pin_cw_ccw_jyqd, GPIO_DIR_OUT_LOW);
  //SetGpioPin(pin_pulses_jyqd, GPIO_DIR_IN); // GPIO counter in kernel use it ;-)

// MAX471
  SetGpioPin(pin_cur_pol, GPIO_DIR_IN);

// TMC5160 stepper drivers (wheels)
  //SetGpioPin(pin_cs_r_tmc, GPIO_DIR_OUT_HIGH); // init done by TMC driver
  //SetGpioPin(pin_cs_l_tmc, GPIO_DIR_OUT_HIGH);
  //SetGpioPin(pin_spi_mosi, GPIO_DIR_OUT);
  //SetGpioPin(pin_spi_miso, GPIO_DIR_IN);
  //SetGpioPin(pin_spi_sck, GPIO_DIR_OUT);

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
  //nextSummaryTime = 0;
  //nextConsoleTime = 0;
  //nextMotorTime = 0;
  nextTempTime = 0;
  nextWifiTime = 0;
  //cmdMotorResponseCounter = 0;
  //cmdSummaryResponseCounter = 0;
  //cmdMotorCounter = 0;
  //cmdSummaryCounter = 0;
  lastLeftPwm = lastRightPwm = lastMowPwm = 0;
  robotID = "XX";

  wxArrayString output;
  if (!wxExecute("hostname", output, wxEXEC_BLOCK))
    robotID = output[0];
  wxLogMessage("reading robot ID...   %s", output[0].c_str());

  /*
  Process p;
  p.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
  robotID = p.readString();
  robotID.trim();
  */

  // start IO Expander
  //wxLogMessage("starting IO Expander");
  // init PCF8575
  //pcf8575 = PCF8575(0x38);
  //if (!pcf8575.isConnected()) wxLogMessage("IO Expander error");
  // set all pin to input TODO check I/O pin needed and write macros
  //pcf8575.write16(0xFFFF);

  // switch-on fan
  setFanPowerTune(60.0);

  // buzzer test
  if (true)
    {
      wxLogMessage("buzzer test");
      GpioPinWrite(pin_buzzer, 1);
      delay(200);
      GpioPinWrite(pin_buzzer, 0);
    }

  wxLogMessage("starting I2C bus");
  I2C.begin();
  // start ADC
  wxLogMessage("starting ADC");
  adc = ADS1115_WE(0x48);
  // test communication
  if (!adc.init()) wxLogMessage("ADC error");
  // set range
  adc.setVoltageRange_mV(ADS1115_RANGE_2048); // 0.0625mV/bit
  // set ref to gnd
  //adc.setCompareChannels(ADS1115_COMP_0_GND | ADS1115_COMP_1_GND | ADS1115_COMP_2_GND | ADS1115_COMP_3_GND);
  // set rate
  adc.setConvRate(ADS1115_128_SPS);
  // set mode
  adc.setMeasureMode(ADS1115_CONTINUOUS);
  // ADC test
  do
    {
      delay(5);
      uint32_t startTime = millis();
      wxLogMessage("ADC S0 = %.2f V", readAdcChannel(ADS1115_COMP_0_GND));
      wxLogMessage("ADC S1 = %.2f V", readAdcChannel(ADS1115_COMP_1_GND));
      wxLogMessage("ADC S2 = %.2f V", readAdcChannel(ADS1115_COMP_2_GND));
      wxLogMessage("ADC S3 = %.2f V", readAdcChannel(ADS1115_COMP_3_GND));
      wxLogMessage("Four ADC conversion duration: %i mS", (uint32_t) millis() - startTime);
      delay(200);
    }
  while(false);   // true to test adc

  do
    {
      //set74HCTOutputEnable();
      //GpioPinWrite(pin_charge_relay, 1);

      float raw = readAdcChannel(ASD_BAT_CHANNEL) * BAT_POT_FACTOR;
      ADC_FILTER(batteryVoltage, raw);
      raw = readAdcChannel(ASD_CHARGE_CHANNEL) * CHARGE_POT_FACTOR;
      ADC_FILTER(chargeVoltage, raw);
      vccVoltage = readAdcChannel(ASD_VCC_CHANNEL) * VCC_POT_FACTOR;
      idleCurrent = readBatteryCurrent();
      wxLogMessage("Batterie voltage = %.2f V Charge voltage = %.2f V VCC voltage = %.2f V Current = %.2f A", batteryVoltage, chargeVoltage, vccVoltage, idleCurrent);
      delay(200);
    }
  while(false);   // true to test adc conv


  for (uint8_t i = 0; i < 2; i++)
    {
      float raw = readAdcChannel(ASD_BAT_CHANNEL) * BAT_POT_FACTOR;
      ADC_FILTER(batteryVoltage, raw);
      raw = readAdcChannel(ASD_CHARGE_CHANNEL) * CHARGE_POT_FACTOR;
      ADC_FILTER(chargeVoltage, raw);
      vccVoltage = readAdcChannel(ASD_VCC_CHANNEL) * VCC_POT_FACTOR;
      idleCurrent = readBatteryCurrent();
    }

  wxLogMessage("VCC (5V) = %.2f V IDLE CURRENT = %.2f A", vccVoltage, idleCurrent);

  if ((ticksMowFD = open("/sys/class/gpio-counter/gpio_counter_6/count", O_RDONLY | O_SYNC)) < 0)
    {
      wxLogMessage("Can't open kernel MowCouter value");
      exitApp(1);
    }
  else
    {
      wxLogMessage("Kernel MowCouter found !");
    }
  getTicksMow();
//???????????????????????????????????////??????????????????
/// test test
  // switch-on and configure IMU GY85
  initImusGY85();
  // init fusion computation
  initFusionImu();
  // and read first values to test I2C communications


  do
    {
      //wxLogMessage("?\e[1;1H\e[2J?"); // clear console
      uint32_t startTime = millis();
      computeFusionImu();

      wxLogMessage("RAW GYRO X: %i", imuGyro.x);
      wxLogMessage("RAW GYRO Y: %i", imuGyro.y);
      wxLogMessage("RAW GYRO Z: %i", imuGyro.z);

      wxLogMessage("RAW ACC X: %i", imuAcc.x);
      wxLogMessage("RAW ACC Y: %i", imuAcc.y);
      wxLogMessage("RAW ACC Z: %i", imuAcc.z);

      wxLogMessage("RAW MAG X: %i", imuMag.x);
      wxLogMessage("RAW MAG Y: %i", imuMag.y);
      wxLogMessage("RAW MAG Z: %i", imuMag.z);

      wxLogMessage("ROLL: %.2f", eulerAngles.angle.roll);
      wxLogMessage("PITCH: %.2f", eulerAngles.angle.pitch);
      wxLogMessage("YAW: %.2f", eulerAngles.angle.yaw);

      wxLogMessage("GPS OFFSET X: %.2f", gpsOffset.axis.x);
      wxLogMessage("GPS OFFSET Y: %.2f", gpsOffset.axis.y);
      wxLogMessage("GPS OFFSET Z: %.2f", gpsOffset.axis.z);

      wxLogMessage("GYROTEMP: %.2f", (float)gyroTemp/10);

      wxLogMessage("Temps imu: %i mS", (uint32_t) millis() - startTime);
      delay(200);
    }
  while(false);   // true to test imu

}

float MeuhRobotDriver::readBatteryCurrent()
{
  float raw = MAX471_VOLTS_TO_AMPS(readAdcChannel(ASD_CUR_CHANNEL));
  bool pol = GpioPinRead(pin_cur_pol);
  raw *= (pol==true)? 1 : -1;
  return raw;
}

void MeuhRobotDriver::getTicksMow()
{
  // read kernel module counter value
#define TICKMOWNUMCHAR 11
  char buf[TICKMOWNUMCHAR];
  int16_t c = pread(ticksMowFD, buf, TICKMOWNUMCHAR, 0);
  if (c > -1)
    encoderTicksMow = atoi(buf);
  else
    encoderTicksMow = 0;
}

void MeuhRobotDriver::tmcLogicOn()
{
  L_Stepper.CS_OFF(); // send 1 signal to deconnect
  L_Stepper.CS_OFF();
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
  L_Stepper.CS_ON(); //send zero volt to the controler
  L_Stepper.CS_ON();
  tmc3V3Powered = false;
}

void MeuhRobotDriver::relayStopAll()
{
  PwmSetDutyCycle(pwmJYQD, 0.0); // stop blade if running
  L_Stepper.toff(0); // turn OFF stepper
  R_Stepper.toff(0); // turn OFF stepper
  delay(500);
  GpioPinWrite(pin_power_relay, 0);
  relayPower = false;
  GpioPinWrite(pin_charge_relay, 0);
  relayCharge = false;
}

void MeuhRobotDriver::relayPowerOn()
{
  GpioPinWrite(pin_charge_relay, 0);
  relayCharge = false;
  PwmSetDutyCycle(pwmJYQD, 0.0);
  GpioPinWrite(pin_cw_ccw_jyqd, 1);
  GpioPinWrite(pin_enable_jyqd, JYQD_ON);
  delay(100);
  if (tmc3V3Powered == true)
    {
      GpioPinWrite(pin_power_relay, 1); // needed to switch JY01 to hall sensor mode at power On
      relayPower = true;
    }
  else wxLogMessage("TMC logic supply is missing !");
}

void MeuhRobotDriver::relayChargeOn()
{
  GpioPinWrite(pin_power_relay, 0);
  relayPower = false;
  delay(100);
  GpioPinWrite(pin_charge_relay, 1);
  relayCharge = true;
}

void MeuhRobotDriver::exitApp(int16_t error) // Close sunray
{
#ifndef TESTNOROOT
  PwmSetDutyCycle(pwmJYQD, 0.0); // stop blade if running
  delay(1000);
  L_Stepper.toff(0x0); // turn OFF stepper
  R_Stepper.toff(0x0); // turn OFF stepper
  relayStopAll(); // turn OFF power boards before quit
  tmcLogicOff();
  set74HCTOutputDisable();
  exit(error);
#endif // TESTNOROOT
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
  adc.setCompareChannels(channel);
  return adc.getResult_V(); // alternative: getResult_mV for Millivolt
}

bool MeuhRobotDriver::setFanPowerTune(float temp)
{
  double dt = MAX(0.0, map(temp, FAN_MIN_TEMP, 80.0, FAN_MIN_PWM, 1.0));
  if (temp < FAN_MIN_TEMP) dt = 0.0;
  wxLogMessage("FAN POWER STATE %.2f C - %.2f %%", temp, dt*100);
  PwmSetDutyCycle(pwmFan, dt);
  return 1;
}

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
  //uint32_t startTime = millis();
  /*String s;
  while (cpuTempProcess.available()) s+= (char)cpuTempProcess.read();
  if (s.length() > 0)
    {
      cpuTemp = s.toFloat() / 1000.0;
      //wxLogMessage("updateCpuTemperature cpuTemp=");
      //wxLogMessage(cpuTemp);
    }
  cpuTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone0/temp");*/
  wxArrayString output;
  if (!wxExecute("cat /sys/class/thermal/thermal_zone0/temp", output, wxEXEC_BLOCK))
    {
      int32_t tmp;
      output[0].ToInt(&tmp);
      cpuTemp = (float)tmp/1000;
    }
  //wxLogMessage(output[0]);

  //uint32_t duration = millis() - startTime;
  //wxLogMessage("updateCpuTemperature duration: ");
  //wxLogMessage(duration);
}

void MeuhRobotDriver::updateWifiConnectionState()
{
  //uint32_t startTime = millis();
  /*String s;
  while (wifiStatusProcess.available()) s+= (char)wifiStatusProcess.read();
  if (s.length() > 0)
    {
      s.trim();
      //wxLogMessage("updateWifiConnectionState state=");
      //wxLogMessage(s);
      // DISCONNECTED, SCANNING, INACTIVE, COMPLETED
      //wxLogMessage(s);
      ledStateWifiConnected = (s == "COMPLETED");
      ledStateWifiInactive = (s == "INACTIVE");
    }
  wifiStatusProcess.runShellCommand("wpa_cli -i wlan0 status | grep wpa_state | cut -d '=' -f2");*/
  int todo = 22;
  wxArrayString output;
  if (!wxExecute("nmcli connection show --active", output, wxEXEC_BLOCK))
    {
      wxLogStatus(output[0]);
      wxLogStatus(output[1]);
    }

  //uint32_t duration = millis() - startTime;
  //wxLogMessage("updateWifiConnectionState duration: ");
  //wxLogMessage(duration);
}


void MeuhRobotDriver::run()
{
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

void MeuhMotorDriver::initTmc5160(TMC5160Stepper &stepper)
{
  stepper.begin();
  stepper.XACTUAL(0);       /* Resetet position */
  stepper.XTARGET(0);       /* Reset target mode position */
  stepper.rms_current(TMC_RMS_CURRENT_MA); /* Set motor RMS current (mA) */
  stepper.RAMPMODE(0x1);
  stepper.microsteps(32);   /* Set microsteps */
  stepper.vSTART(0);        /* 0 */
  stepper.VMAX(0);          /* 59652 . Max speed (6.667rev/S -> 1M/S ) @ fck 12Mhz TSTEP = 35 */
  stepper.AMAX(1500);        /* Acceleration (velocity mode) 3Sec . 0 to VMAX */
  stepper.hstrt(6);         /* Chopconf param from excel computation */
  stepper.hend(3);          /* Chopconf param from excel computation */
  stepper.semin(8);         /* CoolStep low limit (activate) */
  stepper.semax(8);         /* CoolStep hight limit (desactivate)*/
  stepper.seup(2);          /* CoolStep increment */
  stepper.sedn(1);          /* CoolStep current down step speed */
  stepper.sgt(5);          /* StallGuard2 sensitivity to tune (obstacle detection) */
  stepper.sfilt(1);         /* StallGuard2 filter */
  stepper.THIGH(30);        /* Stay in CoolStep mode */
  stepper.TCOOLTHRS(140);   /* CoolStep lower velocity to active StallGuard2 stall flag */
  stepper.sg_stop(1);       /* Stop by StallGuard2 setting */
  stepper.toff(4);          /* Enable driver */
}

void MeuhMotorDriver::begin()
{
  lastEncoderTicksMow = 0;
  L_MotorFault = R_MotorFault = M_MotorFault = false;
  R_DrvStatus.sr = L_DrvStatus.sr = 0;
  L_SpiStatus = R_SpiStatus = 0;

  wxLogMessage("starting JYQD PWM");
  GpioPinWrite(pin_enable_jyqd, JYQD_OFF);
  SetNewPwm(pwmJYQD, 1); // Jyqd pwm (mow)
  //PwmSetPolarity(pwmJYQD, PWM_POLARITY_INVERSED);
  PwmSetFrequency(pwmJYQD, JYQD_PWM_PERIOD);
  PwmSetDutyCycle(pwmJYQD, 0.0);

  wxLogMessage("starting SPI bus");
  SPI.begin();
  SPISettings dummy;
  SPI.beginTransaction(dummy); // SPISetting are hard coded in CPeripheryInterface.cpp
  SPI.transfer(0xFF); // initial dummy transfert


  // start TMC5160 stepper drivers (wheels)
  wxLogMessage("starting TMC5160");
  meuhRobot.tmcLogicOn();
  meuhRobot.set74HCTOutputEnable();
  meuhRobot.relayPowerOn();
  delay(5);
  initTmc5160(L_Stepper);
  uint8_t rVers = R_Stepper.version();
  initTmc5160(R_Stepper);
  uint8_t lVers = L_Stepper.version();
  wxLogMessage("TMC versions: %u - %u", rVers, lVers);
  if ((rVers == 0xFF) || (lVers == 0xFF) || (rVers == 0x0) || (lVers == 0x0))
    {
      wxLogMessage("No TMC communication - Sunray close ");
      meuhRobot.exitApp(1);
    }

  // Check spi_status
  R_SpiStatus = R_Stepper.status_response;
  L_SpiStatus = L_Stepper.status_response;


  meuhRobot.getTicksMow();
  meuhRobot.encoderTicksMow = 0;
  meuhRobot.relayStopAll();
}

void MeuhMotorDriver::run()
{
}

void MeuhMotorDriver::setMotorPwm(int16_t leftPwm, int16_t rightPwm, int16_t mowPwm)
{
  // left/right pwm sign : front +,+..rear -,-..left -,+..right +,-

  if ((!relayPower) && ((leftPwm != 0) || (rightPwm != 0) || (mowPwm != 0))) // Power needed ?
    {
      // power on motors controlers
      meuhRobot.set74HCTOutputEnable();
      meuhRobot.tmcLogicOn();
      meuhRobot.relayPowerOn();
      //GpioPinWrite(pin_enable_jyqd, JYQD_ON);
      PwmSetDutyCycle(pwmJYQD, 0.0);
      delay(20);
      initTmc5160(L_Stepper);
      initTmc5160(R_Stepper);
      delay(1);
      // Check current
      meuhRobot.stepperCurrent = meuhRobot.readBatteryCurrent(); // measure actual current steppers actives
      meuhRobot.stepperCurrent -= meuhRobot.idleCurrent; // remove offset  todo check excess
      wxLogMessage("IDLE STEPPERS CURRENT FROMB AT= %.2f A", meuhRobot.stepperCurrent);
      checkTmcState(L_Stepper, L_DrvStatus, L_MotorFault, meuhRobot.motorLeftCurr, meuhRobot.lastLeftPwm);
      wxLogMessage("Real lefCurr= %.2f A", meuhRobot.motorLeftCurr);
      checkTmcState(R_Stepper, R_DrvStatus, R_MotorFault, meuhRobot.motorRightCurr, meuhRobot.lastRightPwm);
      wxLogMessage("rightCurr= %.2f A", meuhRobot.motorRightCurr);
    }

  // JYQD
  if (mowPwm == 0)  // stop
    {
      PwmSetDutyCycle(pwmJYQD, 0.0);
      // delay(300); // todo test active brake
      GpioPinWrite(pin_enable_jyqd, JYQD_OFF);
    }
  else
    {
      if (ABS(mowPwm) < 64) (mowPwm < 0) ? mowPwm = -150 : mowPwm = 150;
      const double dt = (double)ABS(mowPwm)/255.0f;

      GpioPinWrite(pin_enable_jyqd, JYQD_ON);

      if (mowPwm > 0)
        {
          GpioPinWrite(pin_cw_ccw_jyqd, 0);
          PwmSetDutyCycle(pwmJYQD, dt);
        }
      else
        {
          GpioPinWrite(pin_cw_ccw_jyqd, 1);
          PwmSetDutyCycle(pwmJYQD, dt);
        }
    }

  // TMC 5160
#define STEPPER_DIRECTIONF 0x1
#define STEPPER_DIRECTIONB 0x0
  if (leftPwm == 0)  // stop
    {
      L_Stepper.VMAX(0);
    }
  else
    {
      if (leftPwm > 0)
        {
          L_Stepper.shaft(STEPPER_DIRECTIONB); // change direction
          L_Stepper.VMAX(ABS(leftPwm) * TMC_SPEED_MULT);
        }
      else
        {
          L_Stepper.shaft(STEPPER_DIRECTIONF);
          L_Stepper.VMAX(ABS(leftPwm) * TMC_SPEED_MULT);
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
          R_Stepper.shaft(STEPPER_DIRECTIONF);
          R_Stepper.VMAX(ABS(rightPwm) * TMC_SPEED_MULT);
        }
      else
        {
          R_Stepper.shaft(STEPPER_DIRECTIONB);
          R_Stepper.VMAX(ABS(rightPwm) * TMC_SPEED_MULT);
        }
    }
#undef STEPPER_DIRECTIONF
#undef STEPPER_DIRECTIONB

// Check spi_status
  R_SpiStatus = R_Stepper.status_response;

  meuhRobot.lastLeftPwm = leftPwm;
  meuhRobot.lastRightPwm = rightPwm;
  meuhRobot.lastMowPwm = mowPwm;

}

void MeuhMotorDriver::printTmcError(TMC5160_DRV_STATUS_t status)
{
  if (status.stallGuard) wxLogMessage("Stallguard");
  if (status.stst)       wxLogMessage("Standstill");
  if (status.s2vsa)      wxLogMessage("Short to supply phase A");
  if (status.s2vsb)      wxLogMessage("Short to supply phase B");
  if (status.s2ga)       wxLogMessage("Short to ground phase A");
  if (status.s2gb)       wxLogMessage("Short to ground phase B");
  if (status.ola)        wxLogMessage("Open load phase A");
  if (status.olb)        wxLogMessage("Open load phase B");
  if (status.ot)         wxLogMessage("Overtemperature");
  if (status.otpw)       wxLogMessage("Overtemperature warning");
}

void MeuhMotorDriver::checkTmcState(TMC5160Stepper &stepper, TMC5160_DRV_STATUS_t &status, bool &errorBool, float &current, int16_t &lastPwm)
{
  errorBool = 0;
  status.sr = stepper.DRV_STATUS(); /* load satus */
  current = (TMC_RMS_CURRENT_MA / 1000) * (status.cs_actual + 1) / 32; /* compute current (mA) */
  errorBool |= status.ot; /* check over temperature */
  errorBool |= status.olb; /* check open load b */
  errorBool |= status.ola; /* check open load a */
  errorBool |= status.s2gb; /* check short to ground b */
  errorBool |= status.s2ga; /* check short to ground a */
  //errorBool |= status.stallGuard; /* check stall */
  //errorBool |= status.stst; /* stabdstill */
  errorBool |= (status.stallGuard && status.stst); /* check stallgard with stabdstill */
  if (lastPwm != 0) errorBool = 0; /* don't care status value if stepper is in stop condition */
}

void MeuhMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault)
{
  if (relayPower) // check only if drirers are powered
    {
      ///if ((meuhRobot.lastMowPwm != 0) && (meuhRobot.encoderTicksMow == lastEncoderTicksMow))
      {
        ///M_MotorFault = true;
        ///wxLogMessage(" mowCurr=");
        ///wxLogMessage(meuhRobot.mowCurr);
      }

      checkTmcState(L_Stepper, L_DrvStatus, L_MotorFault, meuhRobot.motorLeftCurr, meuhRobot.lastLeftPwm);
      if (L_MotorFault)
        {
          wxLogMessage("motorFault lefCurr= %.2f A", meuhRobot.motorLeftCurr);
          printTmcError(L_DrvStatus);
        }

      checkTmcState(R_Stepper, R_DrvStatus, R_MotorFault, meuhRobot.motorRightCurr, meuhRobot.lastRightPwm);
      if (R_MotorFault)
        {
          wxLogMessage("motorFault rightCurr= %.2f A", meuhRobot.motorRightCurr);
          printTmcError(R_DrvStatus);
        }
    }
  else
    {
      L_MotorFault = R_MotorFault = M_MotorFault = false;
    }
  // send states
  leftFault = L_MotorFault;
  rightFault = R_MotorFault;
  mowFault = M_MotorFault;;
}

void MeuhMotorDriver::resetMotorFaults()
{
  wxLogMessage("meuhRobot: resetting motor fault");
  if (M_MotorFault)
    {
      wxLogMessage("Fault : Re-init JYQD");
      PwmSetDutyCycle(pwmJYQD, 0.0);
      GpioPinWrite(pin_enable_jyqd, JYQD_OFF); // disable JYQD
      delay(50); // wait ....
    }

  if (L_MotorFault)
    {
      if ((L_DrvStatus.stallGuard) && (L_DrvStatus.stst)) // overload detection ?
        {
          L_MotorFault = false;
          L_Stepper.sg_stop(0); // reset
          L_Stepper.sg_stop(1); // re activate
          meuhRobot.triggeredLeftBumper = 1; // simulate L bumper
        }
      else
        {
          wxLogMessage("Fault : Re-init TMC Left");
          initTmc5160(L_Stepper);
        }
    }

  if (R_MotorFault)
    {
      if ((R_DrvStatus.stallGuard) && (R_DrvStatus.stst)) // overload detection ?
        {
          R_MotorFault = false;
          R_Stepper.sg_stop(0); // reset
          R_Stepper.sg_stop(1); // re activate
          meuhRobot.triggeredRightBumper = 1; // simulate R bumper
        }
      else
        {
          wxLogMessage("Fault : Re-init TMC Right");
          initTmc5160(R_Stepper);
        }
    }
}

void MeuhMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent)
{
  leftCurrent = meuhRobot.motorLeftCurr;
  rightCurrent = meuhRobot.motorRightCurr;
  meuhRobot.mowCurr = meuhRobot.readBatteryCurrent(); // measure actual current
  meuhRobot.mowCurr -= (meuhRobot.stepperCurrent + meuhRobot.idleCurrent); //remove offset
  mowCurrent = meuhRobot.mowCurr;
}

void MeuhMotorDriver::getMotorEncoderTicks(int16_t &leftTicks, int16_t &rightTicks, int16_t &mowTicks)
{
  int32_t actualTicksLeft = 0;
  int32_t actualTicksRight = 0;

  if (relayPower) // XACTUAL is valid if TMCs are powered
    {
      actualTicksLeft = L_Stepper.XACTUAL();
      L_Stepper.XACTUAL(0); // reset value to avoid overflow
      actualTicksRight = R_Stepper.XACTUAL();
      R_Stepper.XACTUAL(0); // reset value to avoid overflow
      meuhRobot.getTicksMow();
    }

  leftTicks = (int16_t)ABS(actualTicksLeft);
  rightTicks = (int16_t)ABS(actualTicksRight);
  mowTicks = meuhRobot.encoderTicksMow - lastEncoderTicksMow;

  lastEncoderTicksMow = meuhRobot.encoderTicksMow;
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
      vccVoltage = meuhRobot.readAdcChannel(ASD_VCC_CHANNEL) * VCC_POT_FACTOR;
      if (vccVoltage > 5.25)
        {
          wxLogMessage("Vcc is %.2f V -> Shutdown now !", vccVoltage);
          keepPowerOn(0);
        }
    }
}

void MeuhBatteryDriver::updateBatteryTemperature()
{
  //uint32_t startTime = millis();
  /*String s;
  while (batteryTempProcess.available()) s+= (char)batteryTempProcess.read();
  if (s.length() > 0)
    {
      batteryTemp = s.toFloat() / 1000.0;
      //wxLogMessage("updateBatteryTemperature batteryTemp=");
      //wxLogMessage(batteryTemp);
    }
  batteryTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone1/temp");*/

  wxArrayString output;
  if (!wxExecute("cat /sys/class/thermal/thermal_zone1/temp", output, wxEXEC_BLOCK))
    {
      int32_t tmp;
      output[0].ToInt(&tmp);
      batteryTemp = (float)tmp/1000;
    }

  //uint32_t duration = millis() - startTime;
  //wxLogMessage("updateBatteryTemperature duration: ");
  //wxLogMessage(duration);
}


float MeuhBatteryDriver::getBatteryTemperature()
{
  return batteryTemp; // linux reported bat temp not useful as seem to be constant 31 degree
}

float MeuhBatteryDriver::getBatteryVoltage()
{
  float raw = meuhRobot.readAdcChannel(ASD_BAT_CHANNEL) * BAT_POT_FACTOR;
  ADC_FILTER(meuhRobot.batteryVoltage, raw);
  return meuhRobot.batteryVoltage;
}

float MeuhBatteryDriver::getChargeVoltage()
{
  float raw = meuhRobot.readAdcChannel(ASD_CHARGE_CHANNEL) * CHARGE_POT_FACTOR;
  ADC_FILTER(meuhRobot.chargeVoltage, raw);
  return meuhRobot.chargeVoltage;
}

float MeuhBatteryDriver::getChargeCurrent()
{
  meuhRobot.chargeCurrent = meuhRobot.readBatteryCurrent() + meuhRobot.idleCurrent;
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
          wxLogMessage("LINUX will SHUTDOWN!");
          meuhRobot.setFanPowerTune(0);
#ifndef TESTNOROOT
          wxExecute("shutdown now", wxEXEC_BLOCK);
          meuhRobot.exitApp(0); // needed ?
#endif // TESTNOROOT
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
}

void MeuhRainSensorDriver::run()
{
  uint32_t t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 10000;    // save CPU resources by running at 0.1 Hz
  bool val = GpioPinRead(pin_rain_sensor);
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

void MeuhBuzzerDriver::tone(int16_t freq)
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
  //imuFound = false;
}

bool MeuhImuDriver::begin()
{
  wxLogMessage("using imu driver: GY85 + FUSION code");
  // switch-on and configure IMU GY85
  initImusGY85();
  // init fusion computation
  initFusionImu();
  // and read first values to test I2C communications
  //uint32_t startTime = millis();
  bool ret = computeFusionImu();
  //wxLogMessage("Fusion imu read and computation duration: ");
  //wxLogMessage((uint32_t) millis() - startTime);
  wxLogMessage("ROLL: %.2f", eulerAngles.angle.roll);
  wxLogMessage("PITCH: %.2f", eulerAngles.angle.pitch);
  wxLogMessage("YAW: %.2f", eulerAngles.angle.yaw);
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
  gpsOffset_X = gpsOffset.axis.x / 100;
  gpsOffset_Y = gpsOffset.axis.y / 100;
  gpsOffset_Z = gpsOffset.axis.z / 100;
  roll = eulerAngles.angle.roll / 180.0 * PI;
  pitch = eulerAngles.angle.pitch / 180.0 * PI;
  yaw = eulerAngles.angle.yaw / 180.0 * PI;
  /*
  wxLogMessage("ROLL: ");
  wxLogMessage(roll);
  wxLogMessage("PITCH: ");
  wxLogMessage(pitch);
  wxLogMessage("YAW: ");
  wxLogMessage(yaw);
  wxLogMessage("Heading: ");
  wxLogMessage(fusionHeading);
  */
  return ret;
}

void MeuhImuDriver::resetData()
{
}




// -------------------------------------------------------------------------------------


SimGpsDriver::SimGpsDriver(MeuhRobotDriver &sr): simRobot(sr)
{
  nextSolutionTime = 0;
  floatX = 0;
  floatY = 0;
  solutionAvail = false;
  simGpsJump = false;
  setSimSolution(SOL_INVALID);
}

void SimGpsDriver::begin(Client &client, char *host, uint16_t port)
{
  resetTime = millis();
}


void SimGpsDriver::begin(HardwareSerial& bus,uint32_t baud)
{
  resetTime = millis();
}


void SimGpsDriver::run()
{
  if (true)
    {
      if (millis() > nextSolutionTime)
        {
          nextSolutionTime = millis() + 200; // 5 hz
          iTOW += 200 * 100;  // sim has 100 times faster gps time
          //relPosE = simRobot.simX;
          //relPosN = simRobot.simY;
          relPosD = 100;
          if (simGpsJump)
            {
              relPosE += 3.0;
              relPosN += 3.0;
            }
          if (solution == SOL_INVALID)
            {
              //wxLogMessage(resetTime);
              //wxLogMessage(",");
              //wxLogMessage(millis());
              if (millis() > resetTime + 2000)
                {
                  solution = SOL_FLOAT;
                }
            }
          // switch to RTK FLOAT from time to time
          if (random(1000) < 5)
            {
              if (solution == SOL_FLOAT) solution = SOL_FIXED;
              else solution = SOL_FLOAT;
            }
          // simulate RTK FLOAT
          if (solution == SOL_FLOAT)
            {
              relPosE += floatX;
              relPosN += floatY;
            }
          if (random(100) < 50) floatX = MIN(1.5, floatX+0.01);
          else floatX = MAX(-1.5, floatX-0.01);
          if (random(100) < 50) floatY = MIN(1.5, floatY+0.01);
          else floatY = MAX(-1.5, floatY-0.01);

          if (solution == SOL_FIXED)
            {
              accuracy = 0.01;
              hAccuracy = accuracy;
              vAccuracy = accuracy;
            }
          else
            {
              accuracy = MAX(floatX, floatY);
              hAccuracy = floatX;
              vAccuracy = floatY;
            }

          lon = relPosE;
          lat =relPosN;
          height = relPosD;
          dgpsAge = millis();
          //groundSpeed = simRobot.linearSpeed;
          solutionAvail = true;
        }
    }
}

bool SimGpsDriver::configure()
{
  return true;
}


void SimGpsDriver::reboot()
{
  wxLogMessage("SimGpsDriver::reboot");
  resetTime = millis();
  solution = SOL_INVALID;
}


void SimGpsDriver::setSimSolution(SolType sol)
{
  solution = sol;
}


void SimGpsDriver::setSimGpsJump(bool flag)
{
  simGpsJump = flag;
}


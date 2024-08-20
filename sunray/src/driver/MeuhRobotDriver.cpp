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
  CONSOLE.println("using robot driver: MeuhRobotDriver");
  COMM.begin(ROBOT_BAUDRATE);

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
  SetGpioPin(pin_rain_sensor, GPIO_DIR_IN);

// Mow driver (JYQD)
  //SetGpioPin(pin_pwm_jyqd, GPIO_DIR_OUT); // needed by linux pwm driver? no
  SetGpioPin(pin_enable_jyqd, GPIO_DIR_OUT_HIGH);
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
      CONSOLE.print(readAdcChannel(ADS1115_COMP_0_GND));
      CONSOLE.println(" V");
      CONSOLE.print("ADC S1 = ");
      CONSOLE.print(readAdcChannel(ADS1115_COMP_1_GND));
      CONSOLE.println(" V");
      CONSOLE.print("ADC S2 = ");
      CONSOLE.print(readAdcChannel(ADS1115_COMP_2_GND));
      CONSOLE.println(" V");
      CONSOLE.print("ADC S3 = ");
      CONSOLE.print(readAdcChannel(ADS1115_COMP_3_GND));
      CONSOLE.println(" V");
      CONSOLE.print("Four ADC conversion duration: ");
      CONSOLE.print((uint32_t) millis() - startTime);
      CONSOLE.println(" mS");
    }

  delay(10);
  vccVoltage = readAdcChannel(ASD_VCC_CHANNEL) * VCC_POT_FACTOR;
  CONSOLE.print("VCC (5V) = ");
  CONSOLE.print(vccVoltage);
  CONSOLE.println(" V");

  idleCurrent = readBatteryCurrent();
  CONSOLE.print("IDLE CURRENT = ");
  CONSOLE.print(idleCurrent);
  CONSOLE.println(" A");

  if ((ticksMowFD = open("/sys/class/gpio-counter/gpio_counter_6/count", O_RDONLY | O_SYNC)) < 0)
    {
      CONSOLE.println("Can't open kernel MowCouter value");
      exitApp(1);
    }
  else
    {
      CONSOLE.println("Kernel MowCouter found !");
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
    //CONSOLE.println("?\e[1;1H\e[2J?"); // clear console
  unsigned long startTime = millis();
  computeFusionImu();

  CONSOLE.print("RAW GYRO X: ");
  CONSOLE.println(imuGyro.x);
  CONSOLE.print("RAW GYRO Y: ");
  CONSOLE.println(imuGyro.y);
  CONSOLE.print("RAW GYRO Z: ");
  CONSOLE.println(imuGyro.z);

  CONSOLE.print("RAW ACC X: ");
  CONSOLE.println(imuAcc.x);
  CONSOLE.print("RAW ACC Y: ");
  CONSOLE.println(imuAcc.y);
  CONSOLE.print("RAW ACC Z: ");
  CONSOLE.println(imuAcc.z);

  CONSOLE.print("RAW MAG X: ");
  CONSOLE.println(imuMag.x);
  CONSOLE.print("RAW MAG Y: ");
  CONSOLE.println(imuMag.y);
  CONSOLE.print("RAW MAG Z: ");
  CONSOLE.println(imuMag.z);

  CONSOLE.print("ROLL: ");
  CONSOLE.println(eulerAngles.angle.roll);
  CONSOLE.print("PITCH: ");
  CONSOLE.println(eulerAngles.angle.pitch);
  CONSOLE.print("YAW: ");
  CONSOLE.println(eulerAngles.angle.yaw);

  CONSOLE.print("GPS OFFSET X: ");
  CONSOLE.println(gpsOffset.axis.x);
  CONSOLE.print("GPS OFFSET Y: ");
  CONSOLE.println(gpsOffset.axis.y);
  CONSOLE.print("GPS OFFSET Z: ");
  CONSOLE.println(gpsOffset.axis.z);

  CONSOLE.print("GYROTEMP: ");
  CONSOLE.println((float)gyroTemp/10);

  CONSOLE.print("Temps imu: ");
  CONSOLE.print((uint32_t) millis() - startTime);
  CONSOLE.println(" mS");
  delay(200);
  } while(true); // true to test imu

}

float MeuhRobotDriver::readBatteryCurrent()
{
  float curr = MAX471_VOLTS_TO_AMPS(readAdcChannel(ASD_CUR_CHANNEL));
  bool pol = 0;
  GpioPinRead(pin_cur_pol, &pol);
  curr *= pol? 1 : -1;
  return curr;
}

void MeuhRobotDriver::getTicksMow()
{
  // read kernel module counter value
#define TICKMOWNUMCHAR 11
  char buf[TICKMOWNUMCHAR];
  int c = pread(ticksMowFD, buf, TICKMOWNUMCHAR, 0);
  if (c > -1)
    encoderTicksMow = atoi(buf);
  else
    encoderTicksMow = 0;
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
  PwmSetDutyCycle(pwmJYQD, 0); // stop blade if running
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
  PwmSetDutyCycle(pwmJYQD, 0);
  delay(100);
  if (tmc3V3Powered == true)
    {
      PwmSetDutyCycle(pwmJYQD, 0);
      GpioPinWrite(pin_power_relay, 1); // needed to switch JY01 to hall sensor mode at power On
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

void MeuhRobotDriver::exitApp(int error) // Close sunray
{
  PwmSetDutyCycle(pwmJYQD, 0); // stop blade if running
  delay(1000);
  L_Stepper.toff(0x0); // turn OFF stepper
  R_Stepper.toff(0x0); // turn OFF stepper
  relayStopAll(); // turn OFF power boards before quit
  tmcLogicOff();
  set74HCTOutputDisable();
  ///exit(error);
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
  CONSOLE.print("FAN POWER STATE ");
  CONSOLE.print(temp);
  CONSOLE.print(" C° - ");
  double dt = max(0, map(temp, 30, 80, 0, 1));
  CONSOLE.print(dt/100);
  CONSOLE.println(" %");
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
  stepper.AMAX(1500);       /* Acceleration (velocity mode) 1Sec . 0 to VMAX */
  stepper.hstrt(6);         /* Chopconf param from excel computation */
  stepper.hend(3);          /* Chopconf param from excel computation */
  stepper.semin(8);         /* CoolStep low limit (activate) */
  stepper.semax(8);         /* CoolStep hight limit (desactivate)*/
  stepper.seup(2);          /* CoolStep increment */
  stepper.sedn(1);          /* CoolStep current down step speed */
  stepper.sgt(1);          /* StallGuard2 sensitivity to tune (obstacle detection) */
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
  L_SpiStatus, R_SpiStatus = 0;

  CONSOLE.println("starting JYQD PWM");
  GpioPinWrite(pin_enable_jyqd, 0);
  SetNewPwm(pwmJYQD, 1); // Jyqd pwm (mow)
  //PwmSetPolarity(pwmJYQD, PWM_POLARITY_INVERSED);
  PwmSetFrequency(pwmJYQD, JYQD_PWM_PERIOD);
  PwmSetDutyCycle(pwmJYQD, 0);

  CONSOLE.println("starting SPI bus");
  SPI.begin();
  SPISettings dummy;
  SPI.beginTransaction(dummy); // SPISetting are hard coded in CPeripheryInterface.cpp
  SPI.transfer(0xFF); // initial dummy transfert


  // start TMC5160 stepper drivers (wheels)
  CONSOLE.println("starting TMC5160");
  meuhRobot.tmcLogicOn();
  meuhRobot.set74HCTOutputEnable();
  meuhRobot.relayPowerOn();
  delay(5);
  initTmc5160(L_Stepper);
  uint8_t rVers = R_Stepper.version();
  initTmc5160(R_Stepper);
  uint8_t lVers = L_Stepper.version();
  CONSOLE.print("TMC versions: ");
  CONSOLE.print(rVers);
  CONSOLE.print(" - ");
  CONSOLE.println(lVers);
  if ((rVers == 0xFF) || (lVers == 0xFF) || (rVers == 0x0) || (lVers == 0x0))
    {
      CONSOLE.println("No TMC communication - Sunray close ");
      meuhRobot.exitApp(1);
    }

  // Check spi_status
  R_SpiStatus = R_Stepper.status_response;
  L_SpiStatus = L_Stepper.status_response;

  if (0) // test
    {
      GpioPinWrite(pin_enable_jyqd, 0);
      meuhRobot.getTicksMow();
      CONSOLE.println(meuhRobot.encoderTicksMow);
      PwmSetDutyCycle(pwmJYQD, 0.15);
      GpioPinWrite(pin_enable_jyqd, 0);
      GpioPinWrite(pin_enable_jyqd, 1);
      delay(1000);
      meuhRobot.getTicksMow();
      CONSOLE.println(meuhRobot.encoderTicksMow);
      delay(5000);
      meuhRobot.getTicksMow();
      CONSOLE.println(meuhRobot.encoderTicksMow);
      GpioPinWrite(pin_enable_jyqd, 0);
      PwmSetDutyCycle(pwmJYQD, 0);
      L_Stepper.shaft(1);
      L_Stepper.VMAX(59652);
      R_Stepper.VMAX(59652);
      delay(2500);
      L_Stepper.VMAX(0);
      R_Stepper.VMAX(0);
      meuhRobot.encoderTicksMow = 0;
      meuhRobot.exitApp(1);
    }


  meuhRobot.getTicksMow();
  meuhRobot.encoderTicksMow = 0;
  meuhRobot.relayStopAll();
}

void MeuhMotorDriver::run()
{
}

void MeuhMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm)
{
    // left/right pwm sign : front +,+..rear -,-..left -,+..right +,-

 if ((!relayPower) && ((leftPwm != 0) || (rightPwm != 0) || (mowPwm != 0))) // Power needed ?
    {
      // power on motors controlers
      meuhRobot.set74HCTOutputEnable();
      meuhRobot.tmcLogicOn();
      meuhRobot.relayPowerOn();
      //GpioPinWrite(pin_cw_ccw_jyqd, 0);
      PwmSetDutyCycle(pwmJYQD, 0.5);
      delay(20);
      initTmc5160(L_Stepper);
      initTmc5160(R_Stepper);
      delay(1);
      // Check current
      meuhRobot.stepperCurrent = meuhRobot.readBatteryCurrent(); // measure actual current steppers actives
      meuhRobot.stepperCurrent -= meuhRobot.idleCurrent; // remove offset  todo check excess
      CONSOLE.print("IDLE STEPPERS CURRENT FROMB AT= ");
      CONSOLE.print(meuhRobot.stepperCurrent);
      CONSOLE.println(" A");
      checkTmcState(L_Stepper, L_DrvStatus, L_MotorFault, meuhRobot.motorLeftCurr, meuhRobot.lastLeftPwm);
      CONSOLE.print("Real lefCurr=");
      CONSOLE.print(meuhRobot.motorLeftCurr);
      CONSOLE.print(" A - ");
      checkTmcState(R_Stepper, R_DrvStatus, R_MotorFault, meuhRobot.motorRightCurr, meuhRobot.lastRightPwm);
      CONSOLE.print("rightCurr=");
      CONSOLE.print(meuhRobot.motorRightCurr);
      CONSOLE.println(" A");
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
      if (abs(mowPwm) < 64) (mowPwm < 0) ? mowPwm = -128 : mowPwm = 128;
      if (mowPwm > 0)
        {
          GpioPinWrite(pin_cw_ccw_jyqd, 0);
          PwmSetDutyCycle(pwmJYQD, 0.5);//map(mowPwm, 0, 255, 0, 1.0f));
        }
      else
        {
          GpioPinWrite(pin_cw_ccw_jyqd, 1);
          PwmSetDutyCycle(pwmJYQD, 0.5);//map(abs(mowPwm), 0, 255, 0, 1.0f));
        }
      GpioPinWrite(pin_enable_jyqd, 1);
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
          L_Stepper.VMAX(abs(leftPwm) * TMC_SPEED_MULT);
        }
      else
        {
          L_Stepper.shaft(STEPPER_DIRECTIONF);
          L_Stepper.VMAX(abs(leftPwm) * TMC_SPEED_MULT);
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
          R_Stepper.VMAX(abs(rightPwm) * TMC_SPEED_MULT);
        }
      else
        {
          R_Stepper.shaft(STEPPER_DIRECTIONB);
          R_Stepper.VMAX(abs(rightPwm) * TMC_SPEED_MULT);
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
  if (status.stallGuard) CONSOLE.println("Stallguard");
  if (status.stst)       CONSOLE.println("Standstill");
  if (status.s2vsa)      CONSOLE.println("Short to supply phase A");
  if (status.s2vsb)      CONSOLE.println("Short to supply phase B");
  if (status.s2ga)       CONSOLE.println("Short to ground phase A");
  if (status.s2gb)       CONSOLE.println("Short to ground phase B");
  if (status.ola)        CONSOLE.println("Open load phase A");
  if (status.olb)        CONSOLE.println("Open load phase B");
  if (status.ot)         CONSOLE.println("Overtemperature");
  if (status.otpw)       CONSOLE.println("Overtemperature warning");
}

void MeuhMotorDriver::checkTmcState(TMC5160Stepper &stepper, TMC5160_DRV_STATUS_t &status, bool &errorBool, float &current, int &lastPwm)
{
  status.sr = stepper.DRV_STATUS(); /* load satus */
  current = (TMC_RMS_CURRENT_MA / 1000) * (status.cs_actual + 1) / 32; /* compute current (mA) */
  errorBool |= status.ot; /* check over temperature */
  errorBool |= status.olb; /* check open load b */
  errorBool |= status.ola; /* check open load a */
  errorBool |= status.s2gb; /* check short to ground b */
  errorBool |= status.s2ga; /* check short to ground a */
  errorBool |= status.stallGuard; /* check stall */
  errorBool |= status.stst; /* stabdstill in each operation */
  if (lastPwm != 0) errorBool = 0; /* don't care status value if stepper is in stop condition */
}

void MeuhMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault)
{
  if ((meuhRobot.lastMowPwm != 0) && (meuhRobot.encoderTicksMow == lastEncoderTicksMow))
    {
      M_MotorFault = true;
      CONSOLE.print(" mowCurr=");
      CONSOLE.println(meuhRobot.mowCurr);
    }

  checkTmcState(L_Stepper, L_DrvStatus, L_MotorFault, meuhRobot.motorLeftCurr, meuhRobot.lastLeftPwm);
  if (L_MotorFault)
    {
      CONSOLE.print("motorFault lefCurr=");
      CONSOLE.print(meuhRobot.motorLeftCurr);
      CONSOLE.println(" A");
      printTmcError(L_DrvStatus);
    }

  checkTmcState(R_Stepper, R_DrvStatus, R_MotorFault, meuhRobot.motorRightCurr, meuhRobot.lastRightPwm);
  if (R_MotorFault)
    {
      CONSOLE.print("motorFault rightCurr=");
      CONSOLE.print(meuhRobot.motorRightCurr);
      CONSOLE.println(" A");
      printTmcError(R_DrvStatus);
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
      delay(500); // wait ....
    }

  if (L_MotorFault)
    {
      if ((L_DrvStatus.stallGuard) || (L_DrvStatus.stst))
        {
          L_MotorFault = false;
          L_Stepper.sg_stop(0); // reset
          L_Stepper.sg_stop(1); // re activate
          meuhRobot.triggeredLeftBumper = 1; // simulate bumper
        }
        else
        {
          CONSOLE.println("Fault : Re-init TMC Left");
          initTmc5160(L_Stepper);
        }
    }

  if (R_MotorFault)
    {
      if ((R_DrvStatus.stallGuard) || (R_DrvStatus.stst))
        {
          R_MotorFault = false;
          R_Stepper.sg_stop(0); // reset
          R_Stepper.sg_stop(1); // re activate
          meuhRobot.triggeredRightBumper = 1; // simulate bumper
        }
        else
        {
          CONSOLE.println("Fault : Re-init TMC Right");
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

void MeuhMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks)
{
  int32_t actualTicksLeft = L_Stepper.XACTUAL();
  L_Stepper.XACTUAL(0); // reset value to avoid overflow
  int32_t actualTicksRight = R_Stepper.XACTUAL();
  R_Stepper.XACTUAL(0); // reset value to avoid overflow
  meuhRobot.getTicksMow();

  leftTicks = (int)abs(actualTicksLeft);
  rightTicks = (int)abs(actualTicksRight);
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
  float raw = meuhRobot.readAdcChannel(ASD_BAT_CHANNEL);
  meuhRobot.batteryVoltage = raw * BAT_POT_FACTOR;
  return meuhRobot.batteryVoltage;
}

float MeuhBatteryDriver::getChargeVoltage()
{
  meuhRobot.chargeVoltage = (meuhRobot.readAdcChannel(ASD_CHARGE_CHANNEL) * CHARGE_POT_FACTOR);
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
          CONSOLE.println("LINUX will SHUTDOWN!");

          meuhRobot.setFanPowerTune(false);
          Process p;
          p.runShellCommand("shutdown now");
          meuhRobot.exitApp(0); // needed ?
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
  GpioPinRead(pin_rain_sensor, &val);
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
  //imuFound = false;
}

bool MeuhImuDriver::begin()
{
  CONSOLE.println("using imu driver: GY85 + FUSION code");
  // switch-on and configure IMU GY85
  initImusGY85();
  // init fusion computation
  initFusionImu();
  // and read first values to test I2C communications
  //unsigned long startTime = millis();
  bool ret = computeFusionImu();
  //CONSOLE.print("Fusion imu read and computation duration: ");
  //CONSOLE.println((uint32_t) millis() - startTime);
  CONSOLE.print("ROLL: ");
  CONSOLE.println(eulerAngles.angle.roll);
  CONSOLE.print("PITCH: ");
  CONSOLE.println(eulerAngles.angle.pitch);
  CONSOLE.print("YAW: ");
  CONSOLE.println(eulerAngles.angle.yaw);
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
  CONSOLE.print("ROLL: ");
  CONSOLE.println(roll);
  CONSOLE.print("PITCH: ");
  CONSOLE.println(pitch);
  CONSOLE.print("YAW: ");
  CONSOLE.println(yaw);
  CONSOLE.print("Heading: ");
  CONSOLE.println(fusionHeading);
  */
  return ret;
}

void MeuhImuDriver::resetData()
{
}



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

void MeuhRobotDriver::begin(){
  CONSOLE.println("using robot driver: MeuhRobotDriver");
  COMM.begin(ROBOT_BAUDRATE);
  encoderTicksLeft = 0;
  encoderTicksRight = 0;
  encoderTicksMow = 0;
  chargeVoltage = 0;
  chargeCurrent = 0;
  batteryVoltage = 28;
  cpuTemp = 30;
  mowCurr = 0;
  motorLeftCurr = 0;
  motorRightCurr = 0;
  resetMotorTicks = true;
  batteryTemp = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
  triggeredRain = false;
  triggeredStopButton = false;
  triggeredLift = false;
  motorFault = false;
  mcuCommunicationLost = true;
  nextSummaryTime = 0;
  nextConsoleTime = 0;
  nextMotorTime = 0;
  nextTempTime = 0;
  nextWifiTime = 0;
  cmdMotorResponseCounter = 0;
  cmdSummaryResponseCounter = 0;
  cmdMotorCounter = 0;
  cmdSummaryCounter = 0;
  requestLeftPwm = requestRightPwm = requestMowPwm = 0;
  robotID = "XX";

  #ifdef __linux__
    CONSOLE.println("reading robot ID...");
    Process p;
    p.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
	  robotID = p.readString();
    robotID.trim();


    // IMU/fan power-on code (Alfred-PCB-specific)

    // start IMU
    CONSOLE.println("starting IMU");
    // switch-on and configure IMU GY85
    initImusGY85();
    // init fusion computation
    initFusionImu();
    // and read first values to test I2C communications
    computeFusionImu();

    //setImuPowerState(true);

    // switch-on fan
    //setFanPowerState(true);

    // buzzer test
    if (false){
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
    adc.setVoltageRange_mV(ADS1115_RANGE_2048);
    // set ref to gnd
    //adc.setCompareChannels(ADS1115_COMP_0_GND | ADS1115_COMP_1_GND | ADS1115_COMP_2_GND | ADS1115_COMP_3_GND);
    // set rate
    adc.setConvRate(ADS1115_128_SPS);
    // set mode
    adc.setMeasureMode(ADS1115_CONTINOUS);

    // ADC test
    if (true){
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

    // EEPROM test
    if (false){
      CONSOLE.println("EEPROM test");
      ///ioEepromWriteByte( EEPROM_I2C_ADDR, 0, 42);
      delay(50);
      int v = ///ioEepromReadByte( EEPROM_I2C_ADDR, 0);
      CONSOLE.print("EEPROM=");
      CONSOLE.println(v);
    }

  #endif
}

float MeuhRobotDriver::readAdcChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

bool MeuhRobotDriver::setFanPowerState(bool state){
  CONSOLE.print("FAN POWER STATE ");
  CONSOLE.println(state);
  return 1;///ioExpanderOut(EX1_I2C_ADDR, EX1_FAN_POWER_PORT, EX1_FAN_POWER_PIN, state);
}

/*bool MeuhRobotDriver::setImuPowerState(bool state){
  CONSOLE.print("IMU POWER STATE ");
  CONSOLE.println(state);
  return 1; ///ioExpanderOut(EX1_I2C_ADDR, EX1_IMU_POWER_PORT, EX1_IMU_POWER_PIN, state);
}*/

bool MeuhRobotDriver::getRobotID(String &id){
  id = robotID;
  return true;
}

bool MeuhRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = mcuFirmwareName;
  ver = mcuFirmwareVersion;
  return true;
}

float MeuhRobotDriver::getCpuTemperature(){
  #ifdef __linux__
    return cpuTemp;
  #else
    return -9999;
  #endif
}

void MeuhRobotDriver::updateCpuTemperature(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;
    while (cpuTempProcess.available()) s+= (char)cpuTempProcess.read();
    if (s.length() > 0) {
      cpuTemp = s.toFloat() / 1000.0;
      //CONSOLE.print("updateCpuTemperature cpuTemp=");
      //CONSOLE.println(cpuTemp);
    }
    cpuTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone0/temp");
    //unsigned long duration = millis() - startTime;
    //CONSOLE.print("updateCpuTemperature duration: ");
    //CONSOLE.println(duration);
  #endif
}

void MeuhRobotDriver::updateWifiConnectionState(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;
    while (wifiStatusProcess.available()) s+= (char)wifiStatusProcess.read();
    if (s.length() > 0){
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
  #endif
}

// send serial request to MCU
void MeuhRobotDriver::sendRequest(String s){
  byte crc = 0;
  for (int i=0; i < s.length(); i++) crc += s[i];
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);
  s += F("\r\n");
  #ifdef DEBUG_SERIAL_ROBOT
    CONSOLE.print("MeuhRobot request: ");
    CONSOLE.println(s);
  #endif
  //cmdResponse = s;
  COMM.print(s);
}


// request MCU SW version
void MeuhRobotDriver::requestVersion(){
  String req;
  req += "AT+V";
  sendRequest(req);
}


// request MCU summary
void MeuhRobotDriver::requestSummary(){
  String req;
  req += "AT+S";
  sendRequest(req);
  cmdSummaryCounter++;
}


// request MCU motor PWM
void MeuhRobotDriver::requestMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  String req;
  req += "AT+M,";
  req += rightPwm;
  req += ",";
  req += leftPwm;
  req += ",";
  req += mowPwm;
  //if (abs(mowPwm) > 0)
  //  req += "1";
  //else
  //  req += "0";
  sendRequest(req);
  cmdMotorCounter++;
}

void MeuhRobotDriver::motorResponse(){
  if (cmd.length()<6) return;
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
      if (counter == 1){
        encoderTicksRight = intValue;  // ag
      } else if (counter == 2){
        encoderTicksLeft = intValue;   // ag
      } else if (counter == 3){
        encoderTicksMow = intValue;
      } else if (counter == 4){
        chargeVoltage = floatValue;
      } else if (counter == 5){
        triggeredLeftBumper = (intValue != 0);
      } else if (counter == 6){
        triggeredLift = (intValue != 0);
      } else if (counter == 7){
        triggeredStopButton = (intValue != 0);
      }
      counter++;
      lastCommaIdx = idx;
    }
  }
  if (triggeredStopButton){
    //CONSOLE.println("STOPBUTTON");
  }
  //CONSOLE.println(encoderTicksMow);
  cmdMotorResponseCounter++;
  mcuCommunicationLost=false;
}


void MeuhRobotDriver::versionResponse(){
  if (cmd.length()<6) return;
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      String s = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1);
      if (counter == 1){
        mcuFirmwareName = s;
      } else if (counter == 2){
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


void MeuhRobotDriver::summaryResponse(){
  if (cmd.length()<6) return;
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();
      if (counter == 1){
        batteryVoltage = floatValue;
      } else if (counter == 2){
        chargeVoltage = floatValue;
      } else if (counter == 3){
        chargeCurrent = floatValue;
      } else if (counter == 4){
        triggeredLift = (intValue != 0);
      } else if (counter == 5){
        triggeredLeftBumper = (intValue != 0);
      } else if (counter == 6){
        triggeredRain = (intValue != 0);
      } else if (counter == 7){
        motorFault = (intValue != 0);
      } else if (counter == 8){
        //CONSOLE.println(cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1));
        mowCurr = floatValue;
      } else if (counter == 9){
        motorLeftCurr = floatValue;
      } else if (counter == 10){
        motorRightCurr = floatValue;
      } else if (counter == 11){
        batteryTemp = floatValue;
      }
      counter++;
      lastCommaIdx = idx;
    }
  }
  cmdSummaryResponseCounter++;
  /*CONSOLE.print("motor currents=");
  CONSOLE.print(mowCurr);
  CONSOLE.print(",");
  CONSOLE.print(motorLeftCurr);
  CONSOLE.print(",");
  CONSOLE.println(motorRightCurr);*/
  //CONSOLE.print("batteryTemp=");
  //CONSOLE.println(batteryTemp);
}

// process response
void MeuhRobotDriver::processResponse(bool checkCrc){
  cmdResponse = "";
  if (cmd.length() < 4) return;
  byte expectedCrc = 0;
  int idx = cmd.lastIndexOf(',');
  if (idx < 1){
    if (checkCrc){
      CONSOLE.println("MeuhRobot: CRC ERROR");
      return;
    }
  } else {
    for (int i=0; i < idx; i++) expectedCrc += cmd[i];
    String s = cmd.substring(idx+1, idx+5);
    int crc = strtol(s.c_str(), NULL, 16);
    if (expectedCrc != crc){
      if (checkCrc){
        CONSOLE.print("MeuhRobot: CRC ERROR");
        CONSOLE.print(crc,HEX);
        CONSOLE.print(",");
        CONSOLE.print(expectedCrc,HEX);
        CONSOLE.println();
        return;
      }
    } else {
      #ifdef DEBUG_SERIAL_ROBOT
        CONSOLE.print("MeuhRobot resp:");
        CONSOLE.println(cmd);
      #endif
      // remove CRC
      cmd = cmd.substring(0, idx);
    }
  }
  if (cmd[0] == 'M') motorResponse();
  if (cmd[0] == 'S') summaryResponse();
  if (cmd[0] == 'V') versionResponse();
}


// process console input
void MeuhRobotDriver::processComm(){
  char ch;
  if (COMM.available()){
    //battery.resetIdle();
    while ( COMM.available() ){
      ch = COMM.read();
      if ((ch == '\r') || (ch == '\n')) {
        //CONSOLE.println(cmd);
        processResponse(true);
        //CONSOLE.print(cmdResponse);
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }
}

void MeuhRobotDriver::run(){
  processComm();
  if (millis() > nextMotorTime){
    nextMotorTime = millis() + 20; // 50 hz
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
  }
  if (millis() > nextSummaryTime){
    nextSummaryTime = millis() + 500; // 2 hz
    requestSummary();
  }
  if (millis() > nextConsoleTime){
    nextConsoleTime = millis() + 1000;  // 1 hz
    if (!mcuCommunicationLost){
      if (mcuFirmwareName == ""){
        requestVersion();
      }
    }
    if ((cmdMotorCounter > 0) && (cmdMotorResponseCounter == 0)){
      CONSOLE.println("WARN: resetting motor ticks");
      resetMotorTicks = true;
      mcuCommunicationLost = true;
    }
    if ( (cmdMotorResponseCounter < 30) ) { // || (cmdSummaryResponseCounter == 0) ){
      CONSOLE.print("WARN: MeuhRobot unmet communication frequency: motorFreq=");
      CONSOLE.print(cmdMotorCounter);
      CONSOLE.print("/");
      CONSOLE.print(cmdMotorResponseCounter);
      CONSOLE.print("  summaryFreq=");
      CONSOLE.print(cmdSummaryCounter);
      CONSOLE.print("/");
      CONSOLE.println(cmdSummaryResponseCounter);
      if (cmdMotorResponseCounter == 0){
        // FIXME: maybe reset motor PID controls here?
      }
    }
    cmdMotorCounter=cmdMotorResponseCounter=cmdSummaryCounter=cmdSummaryResponseCounter=0;
  }
  if (millis() > nextTempTime){
    nextTempTime = millis() + 59000; // 59 sec
    updateCpuTemperature();
    if (cpuTemp < 60){
      setFanPowerState(false);
    } else if (cpuTemp > 65){
      setFanPowerState(true);
    }
  }
  if (millis() > nextWifiTime){
    nextWifiTime = millis() + 7000; // 7 sec
    updateWifiConnectionState();
  }
}


// ------------------------------------------------------------------------------------

MeuhMotorDriver::MeuhMotorDriver(MeuhRobotDriver &sr): meuhRobot(sr){
}

void MeuhMotorDriver::begin(){
  lastEncoderTicksLeft=0;
  lastEncoderTicksRight=0;
  lastEncoderTicksMow=0;
}

void MeuhMotorDriver::run(){
}

void MeuhMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  //meuhRobot.requestMotorPwm(leftPwm, rightPwm, mowPwm);
  meuhRobot.requestLeftPwm = leftPwm;
  meuhRobot.requestRightPwm = rightPwm;
  // Alfred mowing motor driver seem to start start mowing motor more successfully with full PWM (100%) values...
  if (mowPwm > 0) mowPwm = 255;
    else if (mowPwm < 0) mowPwm = -255;
  meuhRobot.requestMowPwm = mowPwm;
}

void MeuhMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
  leftFault = meuhRobot.motorFault;
  rightFault = meuhRobot.motorFault;
  if (meuhRobot.motorFault){
    CONSOLE.print("meuhRobot: motorFault (lefCurr=");
    CONSOLE.print(meuhRobot.motorLeftCurr);
    CONSOLE.print(" rightCurr=");
    CONSOLE.print(meuhRobot.motorRightCurr);
    CONSOLE.print(" mowCurr=");
    CONSOLE.println(meuhRobot.mowCurr);
  }
  mowFault = false;
}

void MeuhMotorDriver::resetMotorFaults(){
  CONSOLE.println("meuhRobot: resetting motor fault");
  //meuhRobot.requestMotorPwm(1, 1, 0);
  //delay(1);
  //meuhRobot.requestMotorPwm(0, 0, 0);
}

void MeuhMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {
  //leftCurrent = 0.5;
  //rightCurrent = 0.5;
  //mowCurrent = 0.8;
  leftCurrent = meuhRobot.motorLeftCurr;
  rightCurrent = meuhRobot.motorRightCurr;
  mowCurrent = meuhRobot.mowCurr;
}

void MeuhMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  if (meuhRobot.mcuCommunicationLost) {
    //CONSOLE.println("getMotorEncoderTicks: no ticks!");
    leftTicks = rightTicks = 0; mowTicks = 0;
    return;
  }
  if (meuhRobot.resetMotorTicks){
    meuhRobot.resetMotorTicks = false;
    //CONSOLE.println("getMotorEncoderTicks: resetMotorTicks");
    lastEncoderTicksLeft = meuhRobot.encoderTicksLeft;
    lastEncoderTicksRight = meuhRobot.encoderTicksRight;
    lastEncoderTicksMow = meuhRobot.encoderTicksMow;
  }
  leftTicks = meuhRobot.encoderTicksLeft - lastEncoderTicksLeft;
  rightTicks = meuhRobot.encoderTicksRight - lastEncoderTicksRight;
  mowTicks = meuhRobot.encoderTicksMow - lastEncoderTicksMow;
  if (leftTicks > 1000){
    leftTicks = 0;
  }
  if (rightTicks > 1000){
    rightTicks = 0;
  }
  if (mowTicks > 1000){
    mowTicks = 0;
  }
  lastEncoderTicksLeft = meuhRobot.encoderTicksLeft;
  lastEncoderTicksRight = meuhRobot.encoderTicksRight;
  lastEncoderTicksMow = meuhRobot.encoderTicksMow;
}


// ------------------------------------------------------------------------------------

MeuhBatteryDriver::MeuhBatteryDriver(MeuhRobotDriver &sr) : meuhRobot(sr){
  mcuBoardPoweredOn = true;
  nextADCTime = 0;
  nextTempTime = 0;
  batteryTemp = 0;
  adcTriggered = false;
  linuxShutdownTime = 0;
}

void MeuhBatteryDriver::begin(){
}

void MeuhBatteryDriver::run(){
  if (millis() > nextTempTime){
    nextTempTime = millis() + 57000; // 57 sec
    updateBatteryTemperature();
  }
}

void MeuhBatteryDriver::updateBatteryTemperature(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;
    while (batteryTempProcess.available()) s+= (char)batteryTempProcess.read();
    if (s.length() > 0) {
      batteryTemp = s.toFloat() / 1000.0;
      //CONSOLE.print("updateBatteryTemperature batteryTemp=");
      //CONSOLE.println(batteryTemp);
    }
    batteryTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone1/temp");
    //unsigned long duration = millis() - startTime;
    //CONSOLE.print("updateBatteryTemperature duration: ");
    //CONSOLE.println(duration);
  #endif
}


float MeuhBatteryDriver::getBatteryTemperature(){
  #ifdef __linux__
    return -9999; //batteryTemp; // linux reported bat temp not useful as seem to be constant 31 degree
  #else
    return -9999;
  #endif
}

float MeuhBatteryDriver::getBatteryVoltage(){
  #ifdef __linux__
    // detect if MCU PCB is switched-off
    if (millis() > nextADCTime){
      if (!adcTriggered){
        // trigger ADC measurement (mcuAna)
        //////ioAdcMux(ADC_MCU_ANA);
        //////ioAdcTrigger(ADC_I2C_ADDR);
        adcTriggered = true;
        nextADCTime = millis() + 5;
      } else {
        nextADCTime = millis() + 1000;
        adcTriggered = false;
        float v = ///ioAdc(ADC_I2C_ADDR);
        mcuBoardPoweredOn = true;
        if (v < 0){
          CONSOLE.println("ERROR reading ADC channel mcuAna!");
          // reset ADC
          //////ioAdcStart(ADC_I2C_ADDR, false, true);
        } else {
          if ((v >0) && (v < 0.8)){
            // no mcuAna, MCU PCB is probably switched off
            CONSOLE.print("mcuAna=");
            CONSOLE.println(v);
            CONSOLE.println("MCU PCB powered OFF!");
            mcuBoardPoweredOn = false;
          }
        }
      }
    }
    if (meuhRobot.mcuCommunicationLost){
      // return 0 volt if MCU PCB is connected and powered-off (Linux will shutdown)
      //if (!mcuBoardPoweredOn) return 0;
      // return 28 volts if MCU PCB is not connected (so Linux can be tested without MCU PCB
      // and will not shutdown if mower is not connected)
      return 28;
    }
  #endif
  return meuhRobot.batteryVoltage;
}

float MeuhBatteryDriver::getChargeVoltage(){
  return meuhRobot.chargeVoltage;
}

float MeuhBatteryDriver::getChargeCurrent(){
  return meuhRobot.chargeCurrent;
}

void MeuhBatteryDriver::enableCharging(bool flag){
}


void MeuhBatteryDriver::keepPowerOn(bool flag){
  #ifdef __linux__
    if (flag){
      // keep power on
      linuxShutdownTime = 0;
      meuhRobot.ledStateShutdown = false;
    } else {
      // shutdown linux - request could be for two reasons:
      // 1. battery voltage sent by MUC-PCB seem to be too low
      // 2. MCU-PCB is powered-off
      if (millis() > linuxShutdownTime){
        linuxShutdownTime = millis() + 10000; // re-trigger linux command after 10 secs
        CONSOLE.println("LINUX will SHUTDOWN!");
        // switch-off fan via port-expander PCA9555
        meuhRobot.setFanPowerState(false);
        Process p;
        p.runShellCommand("shutdown now");
      }
    }
  #endif
}


// ------------------------------------------------------------------------------------

MeuhBumperDriver::MeuhBumperDriver(MeuhRobotDriver &sr): meuhRobot(sr){
}

void MeuhBumperDriver::begin(){
}

void MeuhBumperDriver::run(){

}

bool MeuhBumperDriver::obstacle(){
  return (meuhRobot.triggeredLeftBumper || meuhRobot.triggeredRightBumper);
}

bool MeuhBumperDriver::getLeftBumper(){
  return (meuhRobot.triggeredLeftBumper);
}

bool MeuhBumperDriver::getRightBumper(){
  return (meuhRobot.triggeredRightBumper);
}

void MeuhBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = meuhRobot.triggeredLeftBumper;
  rightBumper = meuhRobot.triggeredRightBumper;
}


// ------------------------------------------------------------------------------------


MeuhStopButtonDriver::MeuhStopButtonDriver(MeuhRobotDriver &sr): meuhRobot(sr){
}

void MeuhStopButtonDriver::begin(){
}

void MeuhStopButtonDriver::run(){

}

bool MeuhStopButtonDriver::triggered(){
  return (meuhRobot.triggeredStopButton);
}

// ------------------------------------------------------------------------------------


MeuhRainSensorDriver::MeuhRainSensorDriver(MeuhRobotDriver &sr): meuhRobot(sr){
}

void MeuhRainSensorDriver::begin(){
}

void MeuhRainSensorDriver::run(){

}

bool MeuhRainSensorDriver::triggered(){
  return (meuhRobot.triggeredRain);
}

// ------------------------------------------------------------------------------------

MeuhLiftSensorDriver::MeuhLiftSensorDriver(MeuhRobotDriver &sr): meuhRobot(sr){
}

void MeuhLiftSensorDriver::begin(){
}

void MeuhLiftSensorDriver::run(){
}

bool MeuhLiftSensorDriver::triggered(){
  return (meuhRobot.triggeredLift);
}


// ------------------------------------------------------------------------------------

MeuhBuzzerDriver::MeuhBuzzerDriver(MeuhRobotDriver &sr): meuhRobot(sr){
}

void MeuhBuzzerDriver::begin(){
}

void MeuhBuzzerDriver::run(){
}

void MeuhBuzzerDriver::noTone(){
  ///ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);
}

void MeuhBuzzerDriver::tone(int freq){
  ///ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
}




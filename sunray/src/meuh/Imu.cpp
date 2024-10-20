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
/*          Copyright 2023-2024 by Ingwie (Bracame)    */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*    Ardumower Alfred mod to drive my autoclip 325    */



#include "Imu.h"
#include <arpa/inet.h>

//#define X_TRANSFOM -1.0
//#define Y_TRANSFOM 1.0
#define Z_TRANSFOM -1.0

void initImusGY85() // init Gyro, accel and compass
{
  initGyro();
  initAcc();
  initMag();
}

// ITG3205 code --------------------------------------------------

#define GYRO_ADRESS      (0x68)

// Register 0x15 – Sample Rate Divider : SMPLRT_DIV
// F sample = 1Khz / (SMPLRT_DIV+1)
#define SMPLRT_DIV       0x1F // 32mS

// Register 0x16 – DLPF, Full Scale
#define FS_SEL           0x03
#define DLPF_CFG         0x03 // 42Hz low pass filter
#define DLPF             ((FS_SEL << 3) | DLPF_CFG)

// Registers 0x1B to 0x22 – Sensor Registers
#define TEMP_OUT_H       0x1B
#define TEMP_OUT_L       0x1C
#define GYRO_XOUT_H      0x1D
#define GYRO_XOUT_L      0x1E
#define GYRO_YOUT_H      0x1F
#define GYRO_YOUT_L      0x20
#define GYRO_ZOUT_H      0x21
#define GYRO_ZOUT_L      0x22

// Register 0x3E – Power Management
#define CLK_SEL          0x03 // PLL with Z Gyro reference

imu_t imuGyro;
int16_t gyroTemp; // in °C X 10

void initGyro()
{
  i2c_writeRegByte(GYRO_ADRESS, 0x15, SMPLRT_DIV); // Sample Rate Divider
  i2c_writeRegByte(GYRO_ADRESS, 0x16, DLPF); // low pass filter
  i2c_writeRegByte(GYRO_ADRESS, 0x3E, CLK_SEL); // frequency source
}

bool readGyro()
{
  bool tmp = i2c_readReg(GYRO_ADRESS, GYRO_XOUT_H, (uint8_t*)&imuGyro, 6);
// swap bytes
  imuGyro.x = htons(imuGyro.x);
  imuGyro.y = htons(imuGyro.y);
  imuGyro.z = htons(imuGyro.z);
// offset correction (manually)
  imuGyro.x -= 8;
  imuGyro.y -= 10;
  imuGyro.z -= 110;
// Imu is upside down so (TODO use Misalignment matrix in Fusionimu.cpp)
// imuGyro.x *= X_TRANSFOM;
// imuGyro.y *= Y_TRANSFOM;
  imuGyro.z *= Z_TRANSFOM;

  return tmp;
}

void readGyroTemp()
{
  i2c_readReg(GYRO_ADRESS, TEMP_OUT_H, (uint8_t*)&gyroTemp, 2);
  gyroTemp = htons(gyroTemp);
  gyroTemp = (350 + (gyroTemp + 13200) / 28);
}


// ADXL345 code --------------------------------------------------

#define ACC_ADRESS      (0x53)

//Register 0x2C—BW_RATE
#define BW_RATE         0x2C
/*
Output Data
Rate (Hz) Bandwidth (Hz) Rate Code
400         200            1100
200         100            1011
100         50             1010
50          25             1001
25          12.5           1000
12.5        6.25           0111
*/
#define RATE            0x09

//Register 0x2D—POWER_CTL
#define POWER_CTL       0x2D
#define ACCMeasure      (1 << (3))

//Register 0x31—DATA_FORMAT
#define DATA_FORMAT     0x31
#define FULL_RES        (1 << (3))
#define SET16G          0x03

// Data
#define DATAX0          0x32

imu_t imuAcc;

void initAcc()
{
  i2c_writeRegByte(ACC_ADRESS, BW_RATE, RATE);
  i2c_writeRegByte(ACC_ADRESS, POWER_CTL, ACCMeasure);
  i2c_writeRegByte(ACC_ADRESS, DATA_FORMAT, FULL_RES | SET16G);
}

bool readAcc() // return 0 if fail
{
  bool tmp = i2c_readReg(ACC_ADRESS, DATAX0, (uint8_t*)&imuAcc, 6);
// offset correction (manually)
  imuAcc.x += 1;
  imuAcc.y += 12;
  imuAcc.z += 26;
// Imu is upside down so
// imuAcc.x *= X_TRANSFOM;
// imuAcc.y *= Y_TRANSFOM;
  imuAcc.z *= Z_TRANSFOM;

  return tmp;
}


// QMC5883L code --------------------------------------------------

#define MAG_ADRESS      (0x0D)

#define CTRLREG1        0x09
#define CTRLREG1VAL     0b00000101 // 50Hz, High filter, 2 Gauss

#define CTRLREG2        0x0A
#define ROL_PNT         (1 << (6)) // To enable pointer roll-over function

#define SETRESETREG     0x0B
#define SETRESETREGVAL  0x01

imu_t imuMag;

void initMag()
{
  i2c_writeRegByte(MAG_ADRESS, CTRLREG1, CTRLREG1VAL);
  i2c_writeRegByte(MAG_ADRESS, CTRLREG2, ROL_PNT);
  i2c_writeRegByte(MAG_ADRESS, SETRESETREG, SETRESETREGVAL);
}

bool readMag() // return 0 if fail
{
  bool tmp = i2c_readReg(MAG_ADRESS, 0x00, (uint8_t*)&imuMag, 6);
// Imu is upside down so
// imuMag.x *= X_TRANSFOM;
// imuMag.y *= Y_TRANSFOM;
  imuMag.z *= Z_TRANSFOM;
  return tmp;
}

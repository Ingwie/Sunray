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


#ifndef IMU_H_INCLUDED
#define IMU_H_INCLUDED

#include "I2chelper.h"

/*
ITG3205  - 0x68 — Three axis gyroscope
ADXL345 -  0x53 — Three axis acceleration
QMC5883L - 0x0D — Three axis magnetic field
*/

#define GYRO_RATE_XYZ      (-1.0f/14.375f) // 14.375 LSBs per °/sec
#define ACC_RATE_XYZ       (-0.0039f)        // 3.9mG
#define MAG_RATE_XYZ       (-0.1f/12.0f)   //  12 lsb = 1mG . 1mG = 0.1 uT

struct imu_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

extern imu_t imuGyro;
extern imu_t imuAcc;
extern imu_t imuMag;

extern int16_t gyroTemp;

extern void initImusGY85();

extern void initGyro();
extern bool readGyro();
extern void readGyroTemp();

extern void initAcc();
extern bool readAcc();

extern void initMag();
extern bool readMag();

#endif // IMU_H_INCLUDED

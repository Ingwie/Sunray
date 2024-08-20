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

#include "FusionImu.h"

#define SAMPLE_RATE 33 // samples per seconde

// gyroscope
#define  uncalibratedGyroscope  {(float)imuGyro.x, (float)imuGyro.y, (float)imuGyro.z}

// accelerometer
#define uncalibratedAccelerometer {(float)imuAcc.x, (float)imuAcc.y, (float)imuAcc.z}

// magnetometer
#define  uncalibratedMagnetometer  {(float)imuMag.x*MAG_RATE_XYZ, (float)imuMag.y*MAG_RATE_XYZ, (float)imuMag.z*MAG_RATE_XYZ} // measurement in uT

// Euler angles
FusionEuler eulerAngles;

// GPS position
FusionVector gpsOffset;

// Define calibration
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {GYRO_RATE_XYZ, GYRO_RATE_XYZ, GYRO_RATE_XYZ};// sensitivity in degrees per second per lsb
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {ACC_RATE_XYZ, ACC_RATE_XYZ, ACC_RATE_XYZ};// Sensitivity in g per lsb
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

void initFusionImu()
{
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings =
  {
    .convention = FusionConventionEnu,
    .gain = 0.5f,
    .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
    .accelerationRejection = 10.0f,
    .magneticRejection = 10.0f,
    .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);
}

bool computeFusionImu() // return false if fail
{
  static uint32_t lastNow = millis();
  bool ret = true;

// Time
  uint32_t now = millis();
  float lastTime = now - lastNow;
  lastTime /= 1000;
  lastNow = now;

// Measure imu
  if (readGyro())
    {
      ret = false;
    }
  if (readAcc())
    {
      ret = false;
    }
  if (readMag())
    {
      ret = false;
    }

  FusionVector gyroscope = uncalibratedGyroscope;
  FusionVector accelerometer = uncalibratedAccelerometer;
  FusionVector magnetometer = uncalibratedMagnetometer;

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, lastTime);

  // Get result quaternion
  FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

  // Compute algorithm outputs
  eulerAngles = FusionQuaternionToEuler(quaternion);

// Read Gyro temperature
  readGyroTemp();

// GPS offset
 FusionMatrix matrix = FusionQuaternionToMatrix(quaternion);
 FusionVector gpsOrigin = {15, 0 , 10};
 gpsOffset = FusionMatrixMultiplyVector(matrix, gpsOrigin);

  return ret;
}


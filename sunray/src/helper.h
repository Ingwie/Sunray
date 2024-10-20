// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// utilities

#ifndef HELPER_H
#define HELPER_H

#include "NutsBolts.h"
#include "HardwareSerial.h"

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352


typedef union
{
  float floatingPoint;
  uint32_t ulong;
  uint8_t binary[4];
} binaryLongOrFloat;

typedef union
{
  uint16_t uint;
  uint8_t binary[2];
} binaryInt;


float scalePI(float v);
float scale180(float v);
float distancePI(float x, float w);
float distance180(float x, float w);
float distanceLineInfinite(float px, float py, float x1, float y1, float x2, float y2);
float distanceLine(float px, float py, float x1, float y1, float x2, float y2);
float fusionPI(float w, float a, float b);
float scalePIangles(float setAngle, float currAngle);
float distance(float x1, float y1, float x2, float y2);
double deg2rad(double deg);
double rad2deg(double rad);
float pointsAngle(float x1, float y1, float x2, float y2);
double distanceLL(double lat1, double lon1, double lat2, double lon2);
void relativeLL(double lat1, double lon1, double lat2, double lon2, float &n, float &e);

int16_t sign(double x);

void printFloat(float v);
void printLong(uint32_t v);
void printInt(uint16_t v);

uint32_t serialToLong(HardwareSerial *serial);
float serialToFloat(HardwareSerial *serial);

float parseFloatValue(String s, String key);

int16_t freeRam ();

/*
 * Returns random number in normal distribution centering on 0.
 * ~95% of numbers returned should fall between -2 and 2
 */
float gaussRandom();

/*
 * Returns member of set with a given mean and standard deviation
 * mean: mean
 * standard deviation: std_dev
 */
float gauss(float mean, float std_dev);

// calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float gaussian(float mu, float sigma, float x);


// quaternion to euler angles
void toEulerianAngle(float w, float x, float y, float z, float& roll, float& pitch, float& yaw);

#endif

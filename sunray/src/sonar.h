// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// HC-SR04 ultrasonic sensor driver (2cm - 400cm)
// for 3 sensors, optimized for speed: based on hardware interrupts (no polling)
// up to 100 Hz measurements tested

#ifndef SONAR_H
#define SONAR_H

#include <inttypes.h>

class Sonar
{
public:
  bool enabled;
  int16_t triggerLeftBelow;
  int16_t triggerCenterBelow;
  int16_t triggerRightBelow;
  void begin();
  void run();
  bool obstacle();
  bool nearObstacle();
  uint16_t distanceLeft; // cm
  uint16_t distanceRight;
  uint16_t distanceCenter;
  bool verboseOutput;
protected:
  uint16_t convertCm(uint16_t echoTime);
  uint32_t nearObstacleTimeout;
};



#endif


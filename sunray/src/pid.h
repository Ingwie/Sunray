#ifndef PID_H
#define PID_H

#include <stdint.h>


/*
  digital PID controller
*/

class PID
{
public:
  PID();
  PID(float Kp, float Ki, float Kd);
  void reset(void);
  float compute();
  double TaMax; // maximum expected sample time
  double Ta; // sampling time
  float w; // set value
  float x; // current value
  float esum; // error sum
  float eold; // last error
  float y;   // control output
  float y_min; // minimum control output
  float y_max; // maximum control output
  float max_output; // maximum output
  float Kp;   // proportional control
  float Ki;   // integral control
  float Kd;   // differential control
  uint32_t lastControlTime;
  uint32_t consoleWarnTimeout;
};


class VelocityPID
{
public:
  VelocityPID();
  VelocityPID(float Kp, float Ki, float Kd);
  float compute();
  double Ta; // sampling time
  float w; // set value
  float x; // current value
  float eold1; // last error
  float eold2; // error n-2
  int16_t y;   // control output
  int16_t yold;   // last control output
  int16_t y_min; // minimum control output
  int16_t y_max; // maximum control output
  int16_t max_output; // maximum output
  float Kp;   // proportional control
  float Ki;   // integral control
  float Kd;   // differential control
  uint32_t lastControlTime;
};



#endif


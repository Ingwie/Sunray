// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "op/op.h"


//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used
Polygon circle(8);

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

float setSpeed = 0.1; // linear speed (m/s)
Point last_rotation_target;
bool rotateLeft = false;
bool rotateRight = false;
bool angleToTargetFits = false;
bool langleToTargetFits = false;
bool targetReached = false;
float trackerDiffDelta = 0;
bool stateKidnapped = false;
bool printmotoroverload = false;
bool trackerDiffDelta_positive = false;

int16_t get_turn_direction_preference()
{
  Point target = maps.targetPoint;
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  float center_x = stateX;
  float center_y = stateY;
  float r = (MOWER_SIZE / 100);
  float cur_angle = stateDelta;

  if (FREEWHEEL_IS_AT_BACKSIDE)
    {
      cur_angle = scalePI(stateDelta + PI);
      targetDelta = scalePI(targetDelta + PI);
    }

  // create circle / octagon around center angle 0 - "360"
  circle.points[0].setXY(center_x + cos(deg2rad(0)) * r, center_y + sin(deg2rad(0)) * r);
  circle.points[1].setXY(center_x + cos(deg2rad(45)) * r, center_y + sin(deg2rad(45)) * r);
  circle.points[2].setXY(center_x + cos(deg2rad(90)) * r, center_y + sin(deg2rad(90)) * r);
  circle.points[3].setXY(center_x + cos(deg2rad(135)) * r, center_y + sin(deg2rad(135)) * r);
  circle.points[4].setXY(center_x + cos(deg2rad(180)) * r, center_y + sin(deg2rad(180)) * r);
  circle.points[5].setXY(center_x + cos(deg2rad(225)) * r, center_y + sin(deg2rad(225)) * r);
  circle.points[6].setXY(center_x + cos(deg2rad(270)) * r, center_y + sin(deg2rad(270)) * r);
  circle.points[7].setXY(center_x + cos(deg2rad(315)) * r, center_y + sin(deg2rad(315)) * r);

  // wxLogMessage("get_turn_direction_preference: ");
  // wxLogMessage(" pos: ");
  // wxLogMessage(stateX);
  // wxLogMessage("/");
  // wxLogMessage(stateY);
  // wxLogMessage(" stateDelta: ");
  // wxLogMessage(cur_angle);
  // wxLogMessage(" targetDelta: ");
  // wxLogMessage(targetDelta);
  int16_t right = 0;
  int16_t left = 0;
  for(int16_t i = 0; i < circle.numPoints; ++i)
    {
      float angle = pointsAngle(stateX, stateY, circle.points[i].x(), circle.points[i].y());
      // wxLogMessage(angle);
      // wxLogMessage(" ");
      // wxLogMessage(i);
      // wxLogMessage(": ");
      // wxLogMessage(circle.points[i].x());
      // wxLogMessage("/");
      // wxLogMessage(circle.points[i].y());
      if (maps.checkpoint(circle.points[i].x(), circle.points[i].y()))
        {

          // skip points in front of us
          if (fabs(angle-cur_angle) < 0.05)
            {
              continue;
            }

          if (cur_angle < targetDelta)
            {
              if (angle >= cur_angle && angle <= targetDelta)
                {
                  left++;
                }
              else
                {
                  right++;
                }
            }
          else
            {
              if (angle <= cur_angle && angle >= targetDelta)
                {
                  right++;
                }
              else
                {
                  left++;
                }
            }
        }
    }
  // wxLogMessage("left/right: ");
  // wxLogMessage(left);
  // wxLogMessage("/");
  // wxLogMessage(right);

  if (right == left)
    {
      return 0;
    }

  if (right < left)
    {
      return 1;
    }

  return -1;
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl)
{
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  float linear = 1.0;
  bool mow = true;
  if (stateOp == OP_DOCK) mow = false;
  float angular = 0;
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);

  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);
  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);
  else
    targetReached = (targetDist < TARGET_REACHED_TOLERANCE);

  if ( (last_rotation_target.x() != target.x() || last_rotation_target.y() != target.y()) &&
       (rotateLeft || rotateRight ) )
    {
      // wxLogMessage("reset left / right rot (target point changed)");
      rotateLeft = false;
      rotateRight = false;
    }

  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  if ( (targetDist < 0.5) || (lastTargetDist < 0.5) || (fabs(distToPath) > 0.5) ||
       rotateLeft || rotateRight )
    {
      if (SMOOTH_CURVES)
        angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);
      else
        angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);
    }
  else
    {
      // while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);
      //angleToTargetFits = true;
    }

  if (!angleToTargetFits)
    {
      // angular control (if angle to far away, rotate to next waypoint)
      linear = 0;
      angular = 29.0 / 180.0 * PI; //  29 degree/s (0.5 rad/s);
      if ((!rotateLeft) && (!rotateRight))  // decide for one rotation direction (and keep it)
        {
          int16_t r = 0;
          // no idea but don't work in reverse mode...
          if (!maps.trackReverse)
            {
              r = get_turn_direction_preference();
            }
          // store last_rotation_target point
          last_rotation_target.setXY(target.x(), target.y());

          if (r == 1)
            {
              //wxLogMessage("force turn right");
              rotateLeft = false;
              rotateRight = true;
            }
          else if (r == -1)
            {
              //wxLogMessage("force turn left");
              rotateLeft = true;
              rotateRight = false;
            }
          else if (trackerDiffDelta < 0)
            {
              rotateRight = true;
            }
          else
            {
              rotateLeft = true;
            }

          trackerDiffDelta_positive = (trackerDiffDelta >= 0);
        }
      if (trackerDiffDelta_positive != (trackerDiffDelta >= 0))
        {
          wxLogMessage("reset left / right rotation - DiffDelta overflow");
          rotateLeft = false;
          rotateRight = false;
          // reverse rotation (*-1) - slowly rotate back
          angular = 10.0 / 180.0 * PI * -1; //  10 degree/s (0.19 rad/s);
        }
      if (rotateRight) angular *= -1;
    }
  else
    {
      // line control (stanley)
      bool straight = maps.nextPointIsStraight();
      bool trackslow_allowed = true;

      rotateLeft = false;
      rotateRight = false;

      // in case of docking or undocking - check if trackslow is allowed
      if ( maps.isUndocking() || maps.isDocking() )
        {
          float dockX = 0;
          float dockY = 0;
          float dockDelta = 0;
          maps.getDockingPos(dockX, dockY, dockDelta);
          float dist_dock = distance(dockX, dockY, stateX, stateY);
          // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
          if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE)
            {
              trackslow_allowed = false;
            }
        }

      if (maps.trackSlow && trackslow_allowed)
        {
          // planner forces slow tracking (e.g. docking etc)
          linear = 0.1;
        }
      else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < 0.5) && (!straight))     // approaching
                    || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))                      // leaving
              )
        {
          linear = 0.1; // reduce speed when approaching/leaving waypoints
        }
      else
        {
          if (gps.solution == SOL_FLOAT)
            linear = MIN(setSpeed, 0.1); // reduce speed for float solution
          else
            linear = setSpeed;         // desired speed
          if (sonar.nearObstacle()) linear = 0.1; // slow down near obstacles
        }
      // slow down speed in case of overload and overwrite all prior speed
      if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) )
        {
          if (!printmotoroverload)
            {
              wxLogMessage("motor overload detected: reduce linear speed to 0.1");
            }
          printmotoroverload = true;
          linear = 0.1;
        }
      else
        {
          printmotoroverload = false;
        }

      //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors
      float k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
      float p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;
      if (maps.trackSlow && trackslow_allowed)
        {
          k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;
          p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;
        }
      angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(motor.linearSpeedSet)));       // correct for path errors
      /*pidLine.w = 0;
      pidLine.x = lateralError;
      pidLine.max_output = PI;
      pidLine.y_min = -PI;
      pidLine.y_max = PI;
      pidLine.compute();
      angular = -pidLine.y;   */
      //wxLogMessage(lateralError);
      //wxLogMessage(",");
      //wxLogMessage(angular/PI*180.0);
      if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
      // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
      //if (!SMOOTH_CURVES) angular = max(-PI/16, MIN(PI/16, angular));
    }
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0)
    {
      if (millis() > lastFixTime + fixTimeout * 1000.0)
        {
          activeOp->onGpsFixTimeout();
        }
    }

  if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT))
    {
      if (ABS(linear) > 0.06)
        {
          if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03))
            {
              // if in linear motion and not enough ground speed => obstacle
              //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) {
              if (GPS_SPEED_DETECTION)
                {
                  wxLogMessage("gps no speed => obstacle!");
                  triggerObstacle();
                  return;
                }
            }
        }
    }
  else
    {
      // no gps solution
      if (REQUIRE_VALID_GPS)
        {
          wxLogMessage("WARN: no gps solution!");
          activeOp->onGpsNoSignal();
        }
    }

  // gps-jump/false fix check
  if (KIDNAP_DETECT)
    {
      float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;
      if ( maps.isUndocking() || maps.isDocking() )
        {
          float dockX = 0;
          float dockY = 0;
          float dockDelta = 0;
          maps.getDockingPos(dockX, dockY, dockDelta);
          float dist = distance(dockX, dockY, stateX, stateY);
          // check if current distance to docking station is below
          // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
          if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK)
            {
              allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
            }
        }
      if (fabs(distToPath) > allowedPathTolerance)  // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
        {
          if (!stateKidnapped)
            {
              stateKidnapped = true;
              activeOp->onKidnapped(stateKidnapped);
            }
        }
      else
        {
          if (stateKidnapped)
            {
              stateKidnapped = false;
              activeOp->onKidnapped(stateKidnapped);
            }
        }
    }

  // in any case, turn off mower motor if lifted
  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance
  if (detectLift()) mow = false;

  if (mow)
    {
      if (millis() < motor.motorMowSpinUpTime + 10000)
        {
          // wait until mowing motor is running
          if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
          linear = 0;
          angular = 0;
        }
    }

  if (runControl)
    {
      if (angleToTargetFits != langleToTargetFits)
        {
          //wxLogMessage("angleToTargetFits: ");
          //wxLogMessage(angleToTargetFits);
          //wxLogMessage(" trackerDiffDelta: ");
          //wxLogMessage(trackerDiffDelta);
          langleToTargetFits = angleToTargetFits;
        }

      motor.setLinearAngularSpeed(linear, angular);
      motor.setMowState(mow);
    }

  if (targetReached)
    {
      rotateLeft = false;
      rotateRight = false;
      activeOp->onTargetReached();
      //bool straight = maps.nextPointIsStraight();
      if (!maps.nextPoint(false,stateX,stateY))
        {
          // finish
          activeOp->onNoFurtherWaypoints();
        }
      else
        {
          // next waypoint
          //if (!straight) angleToTargetFits = false;
        }
    }
}



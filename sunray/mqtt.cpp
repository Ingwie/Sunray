#include "mqtt.h"
#include "config.h"
#include "robot.h"
#include "StateEstimator.h"
#include "LineTracker.h"
#include "Stats.h"
#include "src/op/op.h"
#include "reset.h"
#include "timetable.h"
#include "comm.h"

// mqtt
#define MSG_BUFFER_SIZE	(50)
char mqttMsg[MSG_BUFFER_SIZE];
unsigned long nextMQTTPublishTime = 0;
unsigned long nextMQTTLoopTime = 0;
uint8_t mqttRequestTopic = MQTT_REQUEST_ONLINE;



void mqttReconnect() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    CONSOLE.println("MQTT: Attempting connection...");
    // Create a random client ID
    String clientId = "sunray-ardumower";
    // Attempt to connect
#ifdef MQTT_USER
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
#else
    if (mqttClient.connect(clientId.c_str())) {
#endif
      CONSOLE.println("MQTT: connected");
      // Once connected, publish an announcement...
      //mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      CONSOLE.println("MQTT: subscribing " MQTT_TOPIC_PREFIX "/command");
      mqttClient.subscribe(MQTT_TOPIC_PREFIX "/command");
    } else {
      CONSOLE.print("MQTT: failed, rc=");
      CONSOLE.print(mqttClient.state());
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  CONSOLE.print("MQTT: Message arrived [");
  CONSOLE.print(topic);
  CONSOLE.print("] ");
  cmd = "";
  for (int i = 0; i < length; i++) {
    cmd += (char)payload[i];
  }
  CONSOLE.println(cmd);

    if (cmd == "start")
      {
        cmd = "AT+C,-1,1,0.2,100,0,-1,-1,1";
      }
    else if (cmd == "stop")
      {
        cmd = "AT+C,-1,0,-1,-1,-1,-1,-1,-1";
      }
    else if (cmd == "dock")
      {
        cmd = "AT+C,-1,4,-1,-1,-1,-1,-1,1";
      }
    else if (cmd == "reboot")
      {
        cmd = "AT+Y";
      }
    else if (cmd == "shutdown")
      {
        cmd = "AT+Y3";
      }

  processCmd(false, false);

}

// define a macro so avoid repetitive code lines for sending single values via MQTT
#define MQTT_PUBLISH(VALUE, FORMAT, TOPIC) \
      snprintf (mqttMsg, MSG_BUFFER_SIZE, FORMAT, VALUE); \
      mqttClient.publish(MQTT_TOPIC_PREFIX TOPIC, mqttMsg)


// process MQTT input/output (subcriber/publisher)
void processWifiMqttClient()
{
  if (!ENABLE_MQTT) return;
  unsigned long milli = millis();
  if (milli >= nextMQTTPublishTime){
    nextMQTTPublishTime = millis() + 800;
    if (mqttClient.connected()) {
      updateStateOpText();

// test to remove
mqttRequestTopic = MQTT_REQUEST_ONLINE|MQTT_REQUEST_STATE;//|MQTT_REQUEST_STATS|MQTT_REQUEST_PROPS;

/*
Props: {"firmware":"Ardumower Sunray","version":"1.0.223"}
States: {"battery_voltage":25.82,
"position":{"x":XX.XX,"y":-X.XX,"delta":-1.36,
"solution":2,"age":1.89,"accuracy":0.02,"visible_satellites":36,"visible_satellites_dgps":35,"mow_point_index":431},
"target":{"x":X.XX,"y":-X.XX},"job":0,"sensor":0,"amps":0.02,"map_crc":XXXXX}
*/

      // online
      if ((mqttRequestTopic & MQTT_REQUEST_ONLINE) == MQTT_REQUEST_ONLINE) {
        MQTT_PUBLISH("true", "%s", "/online/");
        MQTT_PUBLISH(milli, "%lu", "/online");
        mqttRequestTopic &= ~MQTT_REQUEST_ONLINE;
      }

      // props
      if ((mqttRequestTopic & MQTT_REQUEST_PROPS) == MQTT_REQUEST_PROPS) {
        String mcuFwName = "";
        String mcuFwVer = "";
        robotDriver.getMcuFirmwareVersion(mcuFwName, mcuFwVer);
        MQTT_PUBLISH(mcuFwName.c_str(), "%s", "/props/firmware");
        MQTT_PUBLISH(mcuFwVer.c_str(), "%s", "/props/version");
        MQTT_PUBLISH(milli, "%lu", "/props");
        mqttRequestTopic &= ~MQTT_REQUEST_PROPS;
      }


      // state
      if ((mqttRequestTopic & MQTT_REQUEST_STATE) == MQTT_REQUEST_STATE) {
        MQTT_PUBLISH(battery.batteryVoltage, "%.2f", "/state/battery_voltage");
        MQTT_PUBLISH(stateX, "%f" , "/state/position/x");
        MQTT_PUBLISH(stateY, "%f" , "/state/position/y");
        MQTT_PUBLISH(stateDelta, "%f" , "/state/position/delta");
        MQTT_PUBLISH(gps.solution, "%i", "/state/position/solution");
        //MQTT_PUBLISH(stateOpText.c_str(), "%s", "state/job");
        MQTT_PUBLISH(stateOp, "%i", "/state/job");
        MQTT_PUBLISH(maps.mowPointsIdx, "%i", "/state/position/mow_point_index");
        MQTT_PUBLISH((millis() - gps.dgpsAge)/1000.0, "%f" , "/state/position_age");
        MQTT_PUBLISH(stateSensor, "%i", "/state/sensor");
        MQTT_PUBLISH(maps.targetPoint.x() , "%f" , "/state/target/x");
        MQTT_PUBLISH(maps.targetPoint.y() , "%f" , "/state/target/y");
        MQTT_PUBLISH(gps.accuracy, "%f" , "/state/position/accuracy");
        MQTT_PUBLISH(gps.numSV, "%i", "/state/position/visible_satellites");
        MQTT_PUBLISH((stateOp == OP_CHARGE)? -battery.chargingCurrent : motor.motorsSenseLP, "%f" , "/state/amps");
        MQTT_PUBLISH(gps.numSVdgps, "%i", "/state/position/visible_satellites_dgps");
        MQTT_PUBLISH(maps.mapCRC, "%li", "/state/map_crc");
        MQTT_PUBLISH(lateralError, "%f" , "/state/lateral_error");
        //MQTT_PUBLISH(timetable.autostopTime.dayOfWeek,, "/state/timetable_autostartstop_dayofweek");
        //MQTT_PUBLISH(timetable.autostartTime.hour,, "/state/timetabel_autostartstop_hour");
        //MQTT_PUBLISH(??,, "/state/timestamp");
        MQTT_PUBLISH(milli, "%lu", "/state");
        mqttRequestTopic &= ~MQTT_REQUEST_STATE;
      }

      // stats
      if ((mqttRequestTopic & MQTT_REQUEST_STATS) == MQTT_REQUEST_STATS) {
        MQTT_PUBLISH(statIdleDuration, "%lu" , "/stats/duration_idle");
        MQTT_PUBLISH(statChargeDuration, "%lu" , "/stats/duration_charge");
        MQTT_PUBLISH(statMowDuration, "%lu" , "/stats/duration_mow");
        MQTT_PUBLISH(statMowDurationFloat, "%lu" , "/stats/duration_mow_float");
        MQTT_PUBLISH(statMowDurationFix, "%lu" , "/stats/duration_mow_fix");
        MQTT_PUBLISH(statMowFloatToFixRecoveries, "%lu" , "/stats/counter_float_recoveries");
        MQTT_PUBLISH(statMowDistanceTraveled, "%.1f" , "/stats/distance_mow_traveled");
        MQTT_PUBLISH(statMowMaxDgpsAge, "%.2f" , "/stats/time_max_dpgs_age");
        MQTT_PUBLISH(statImuRecoveries, "%lu" , "/stats/counter_imu_triggered");
        MQTT_PUBLISH(statTempMin, "%.1f" , "/stats/temp_min");
        MQTT_PUBLISH(statTempMax, "%.1f" , "/stats/temp_max");
        MQTT_PUBLISH(gps.chksumErrorCounter, "%lu" , "/stats/counter_gps_chk_sum_errors");
        MQTT_PUBLISH(gps.dgpsChecksumErrorCounter, "%lu" , "/stats/counter_dgps_chk_sum_errors");
        static float statMaxControlCycleTime = max(statMaxControlCycleTime, (1.0 / (((float)controlLoops)/5.0)));
        MQTT_PUBLISH(statMaxControlCycleTime, "%.3f" , "/stats/time_max_cycle");
        MQTT_PUBLISH(SERIAL_BUFFER_SIZE, "%u" , "/stats/serial_buffer_size");
        MQTT_PUBLISH(statMowDurationInvalid, "%lu" , "/stats/duration_mow_invalid");
        MQTT_PUBLISH(statMowInvalidRecoveries, "%lu" , "/stats/counter_invalid_recoveries");
        MQTT_PUBLISH(statMowObstacles, "%lu" , "/stats/counter_obstacles");
        MQTT_PUBLISH(freeMemory(), "%i" , "/stats/free_memory");
        MQTT_PUBLISH(getResetCause(), "%u" , "/stats/reset_cause");
        MQTT_PUBLISH(statGPSJumps, "%lu" , "/stats/counter_gps_jumps");
        MQTT_PUBLISH(statMowSonarCounter, "%lu" , "/stats/counter_sonar_triggered");
        MQTT_PUBLISH(statMowBumperCounter, "%lu" , "/stats/counter_bumper_triggered");
        MQTT_PUBLISH(statMowGPSMotionTimeoutCounter, "%lu" , "/stats/counter_gps_motion_timeout");
        MQTT_PUBLISH(statMowDurationMotorRecovery, "%lu" , "/stats/duration_mow_motor_recovery");
        MQTT_PUBLISH(milli, "%lu", "/stats");
        mqttRequestTopic &= ~MQTT_REQUEST_STATS;
      }

        /*      // operational state
      //CONSOLE.println("MQTT: publishing " MQTT_TOPIC_PREFIX "/status");
      MQTT_PUBLISH(stateOpText.c_str(), "%s", "/op")
      MQTT_PUBLISH(maps.percentCompleted, "%d", "/progress")

      // GPS related information
      snprintf (mqttMsg, MSG_BUFFER_SIZE, "%.2f, %.2f", gps.relPosN, gps.relPosE);
      mqttClient.publish(MQTT_TOPIC_PREFIX "/gps/pos", mqttMsg);
      MQTT_PUBLISH(gpsSolText.c_str(), "%s", "/gps/sol")
      MQTT_PUBLISH(gps.iTOW, "%lu", "/gps/tow")

      MQTT_PUBLISH(gps.lon, "%.8f", "/gps/lon")
      MQTT_PUBLISH(gps.lat, "%.8f", "/gps/lat")
      MQTT_PUBLISH(gps.height, "%.1f", "/gps/height")
      MQTT_PUBLISH(gps.relPosN, "%.4f", "/gps/relNorth")
      MQTT_PUBLISH(gps.relPosE, "%.4f", "/gps/relEast")
      MQTT_PUBLISH(gps.relPosD, "%.2f", "/gps/relDist")
      MQTT_PUBLISH((millis()-gps.dgpsAge)/1000.0, "%.2f","/gps/ageDGPS")
      MQTT_PUBLISH(gps.accuracy, "%.2f", "/gps/accuracy")
      MQTT_PUBLISH(gps.groundSpeed, "%.4f", "/gps/groundSpeed")

      // power related information
      MQTT_PUBLISH(battery.batteryVoltage, "%.2f", "/power/battery/voltage")
      MQTT_PUBLISH(motor.motorsSenseLP, "%.2f", "/power/motor/current")
      MQTT_PUBLISH(battery.chargingVoltage, "%.2f", "/power/battery/charging/voltage")
      MQTT_PUBLISH(battery.chargingCurrent, "%.2f", "/power/battery/charging/current")

      // map related information
      MQTT_PUBLISH(maps.targetPoint.x(), "%.2f", "/map/targetPoint/X")
      MQTT_PUBLISH(maps.targetPoint.y(), "%.2f", "/map/targetPoint/Y")
      MQTT_PUBLISH(stateX, "%.2f", "/map/pos/X")
      MQTT_PUBLISH(stateY, "%.2f", "/map/pos/Y")
      MQTT_PUBLISH(stateDelta, "%.2f", "/map/pos/Dir")

      // statistics
      MQTT_PUBLISH((int)statIdleDuration, "%d", "/stats/idleDuration")
      MQTT_PUBLISH((int)statChargeDuration, "%d", "/stats/chargeDuration")
      MQTT_PUBLISH((int)statMowDuration, "%d", "/stats/mow/totalDuration")
      MQTT_PUBLISH((int)statMowDurationInvalid, "%d", "/stats/mow/invalidDuration")
      MQTT_PUBLISH((int)statMowDurationFloat, "%d", "/stats/mow/floatDuration")
      MQTT_PUBLISH((int)statMowDurationFix, "%d", "/stats/mow/fixDuration")
      MQTT_PUBLISH((int)statMowFloatToFixRecoveries, "%d", "/stats/mow/floatToFixRecoveries")
      MQTT_PUBLISH((int)statMowObstacles, "%d", "/stats/mow/obstacles")
      MQTT_PUBLISH((int)statMowGPSMotionTimeoutCounter, "%d", "/stats/mow/gpsMotionTimeouts")
      MQTT_PUBLISH((int)statMowBumperCounter, "%d", "/stats/mow/bumperEvents")
      MQTT_PUBLISH((int)statMowSonarCounter, "%d", "/stats/mow/sonarEvents")
      MQTT_PUBLISH((int)statMowLiftCounter, "%d", "/stats/mow/liftEvents")
      MQTT_PUBLISH(statMowMaxDgpsAge, "%.2f", "/stats/mow/maxDgpsAge")
      MQTT_PUBLISH(statMowDistanceTraveled, "%.1f", "/stats/mow/distanceTraveled")
      MQTT_PUBLISH((int)statMowInvalidRecoveries, "%d", "/stats/mow/invalidRecoveries")
      MQTT_PUBLISH((int)statImuRecoveries, "%d", "/stats/imuRecoveries")
      MQTT_PUBLISH((int)statGPSJumps, "%d", "/stats/gpsJumps")
      MQTT_PUBLISH(statTempMin, "%.1f", "/stats/tempMin")
      MQTT_PUBLISH(statTempMax, "%.1f", "/stats/tempMax")
      MQTT_PUBLISH(stateTemp, "%.1f", "/stats/curTemp")
*/
    } else {
      mqttReconnect();
    }
  }
  if (millis() > nextMQTTLoopTime){
    nextMQTTLoopTime = millis() + 20000;
    mqttClient.loop();
  }
}



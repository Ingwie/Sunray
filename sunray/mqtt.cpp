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
char mqttTxPayload[MQTT_MAX_PACKET_SIZE];
unsigned long nextMQTTPublishTime = 0;
unsigned long nextMQTTLoopTime = 0;
uint8_t mqttRequestTopic = 0;

void mqttReconnect()
{
  // Loop until we're reconnected
  if (!mqttClient.connected())
    {
      CONSOLE.println("MQTT: Attempting connection...");
      // Create a random client ID
      String clientId = "sunray-ardumower";
      // Attempt to connect
#ifdef MQTT_USER
      if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS))
        {
#else
      if (mqttClient.connect(clientId.c_str()))
        {
#endif
          CONSOLE.println("MQTT: connected");
          // Once connected, publish an announcement...
          //mqttClient.publish("outTopic", "hello world");
          // ... and resubscribe
          CONSOLE.println("MQTT: subscribing " MQTT_TOPIC_PREFIX "/command");
          mqttClient.subscribe(MQTT_TOPIC_PREFIX "/command");
        }
      else
        {
          CONSOLE.print("MQTT: failed, rc=");
          CONSOLE.print(mqttClient.state());
        }
    }
}


void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  CONSOLE.print("MQTT: Message arrived [");
  CONSOLE.print(topic);
  CONSOLE.print("] ");
  cmd = "";
  for (int i = 0; i < length; i++)
    {
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

// process MQTT input/output (subcriber/publisher)
void processWifiMqttClient()
{
  static uint8_t metronom = 0;

  if (!ENABLE_MQTT) return;
  unsigned long milli = millis();
  if (milli >= nextMQTTPublishTime)
    {
      nextMQTTPublishTime = millis() + 800;
      if (mqttClient.connected())
        {
          //updateStateOpText();
          ++metronom;

          if (metronom & 1) mqttRequestTopic |= MQTT_REQUEST_STATE; // fastest
          if ((metronom & 7) == 4) mqttRequestTopic |= MQTT_REQUEST_STATS;
          if ((metronom & 15) == 8) mqttRequestTopic |= MQTT_REQUEST_PROPS;
          if (metronom == 16) { mqttRequestTopic |= MQTT_REQUEST_ONLINE; metronom = 0; }// slowest

  //unsigned long startTime = millis();

          // online
          if ((mqttRequestTopic & MQTT_REQUEST_ONLINE) == MQTT_REQUEST_ONLINE)
            {
              mqttClient.publish(MQTT_TOPIC_PREFIX "/online", "{\"true\"}");
              mqttRequestTopic &= ~MQTT_REQUEST_ONLINE;
            }

          // props
          if ((mqttRequestTopic & MQTT_REQUEST_PROPS) == MQTT_REQUEST_PROPS)
            {
              String mcuFwName = "";
              String mcuFwVer = "";
              robotDriver.getMcuFirmwareVersion(mcuFwName, mcuFwVer);
              snprintf (mqttTxPayload, MQTT_MAX_PACKET_SIZE, "{\"firmware\":\"%s\",\"version\":\"%s\"}", mcuFwName.c_str(), mcuFwVer.c_str());

              mqttClient.publish(MQTT_TOPIC_PREFIX "/props", mqttTxPayload);
              mqttRequestTopic &= ~MQTT_REQUEST_PROPS;
            }

          // state
          if ((mqttRequestTopic & MQTT_REQUEST_STATE) == MQTT_REQUEST_STATE)
            {
              float amps = (stateOp == OP_CHARGE)? -battery.chargingCurrent : motor.motorsSenseLP;
              int timetable_autostartstop_dayofweek = (stateOp == OP_MOW)? timetable.autostopTime.dayOfWeek : timetable.autostartTime.dayOfWeek;
              int timetabel_autostartstop_hour = (stateOp == OP_CHARGE)? timetable.autostopTime.hour : timetable.autostartTime.hour;
              if ((stateOp != OP_MOW) && (stateOp != OP_CHARGE)) timetable_autostartstop_dayofweek = timetabel_autostartstop_hour = 0;
              snprintf (mqttTxPayload, MQTT_MAX_PACKET_SIZE, \
"{\"battery_voltage\":%.2f,\"position\":{\"x\":%f,\"y\":%f,\"delta\":%.2f,\"solution\":%i,\"age\":%.2f,\
\"accuracy\":%.2f,\"visible_satellites\":%i,\"visible_satellites_dgps\":%i,\"mow_point_index\":%i},\
\"target\":{\"x\":%f,\"y\":%f},\"job\":%i,\"sensor\":%i,\"amps\":%.2f,\"map_crc\":%li,\"lateral_error\":%.2f,\
\"timetable_autostartstop_dayofweek\":%.i,\"timetabel_autostartstop_hour\":%.i}", \
                        battery.batteryVoltage, stateX, stateY, stateDelta, gps.solution, ((millis() - gps.dgpsAge)/1000.0), \
                        gps.accuracy, gps.numSV, gps.numSVdgps, maps.mowPointsIdx, maps.targetPoint.x(), maps.targetPoint.y(), \
                        stateOp, stateSensor, amps, \
                        maps.mapCRC, lateralError);

              mqttClient.publish(MQTT_TOPIC_PREFIX "/state", mqttTxPayload);
              mqttRequestTopic &= ~MQTT_REQUEST_STATE;
            }

          // stats
          if ((mqttRequestTopic & MQTT_REQUEST_STATS) == MQTT_REQUEST_STATS)
            {
              static float statMaxControlCycleTime = max(statMaxControlCycleTime, (1.0 / (((float)controlLoops)/5.0)));
              snprintf (mqttTxPayload, MQTT_MAX_PACKET_SIZE, \
"{\"duration_idle\":%lu,\"duration_charge\":%lu,\"duration_mow\":%lu,\"duration_mow_float\":%lu,\"duration_mow_fix\":%lu,\
\"counter_float_recoveries\":%lu,\"distance_mow_traveled\":%.1f,\"time_max_dpgs_age\":%.2f,\"counter_imu_triggered\":%lu,\
\"temp_min\":%.1f,\"temp_max\":%.1f,\"counter_gps_chk_sum_errors\":%lu,\"counter_dgps_chk_sum_errors\":%lu,\"time_max_cycle\":%.3f,\
\"serial_buffer_size\":%u,\"duration_mow_invalid\":%lu,\"counter_invalid_recoveries\":%lu,\"counter_obstacles\":%lu,\"free_memory\":%i,\
\"reset_cause\":%u,\"counter_gps_jumps\":%lu,\"counter_sonar_triggered\":%lu,\"counter_bumper_triggered\":%lu,\
\"counter_gps_motion_timeout\":%lu,\"duration_mow_motor_recovery\":%lu}", \
                        statIdleDuration, statChargeDuration, statMowDuration, statMowDurationFloat, statMowDurationFix, statMowFloatToFixRecoveries, \
                        statMowDistanceTraveled, statMowMaxDgpsAge, statImuRecoveries, statTempMin, statTempMax, gps.chksumErrorCounter, \
                        gps.dgpsChecksumErrorCounter, statMaxControlCycleTime, SERIAL_BUFFER_SIZE, \
                        statMowDurationInvalid, statMowInvalidRecoveries, statMowObstacles, freeMemory(), getResetCause(), statGPSJumps, \
                        statMowSonarCounter, statMowBumperCounter, statMowGPSMotionTimeoutCounter, statMowDurationMotorRecovery);

              mqttClient.publish(MQTT_TOPIC_PREFIX "/stats", mqttTxPayload);
              mqttRequestTopic &= ~MQTT_REQUEST_STATS;
            }
  //unsigned long duration = millis() - startTime;
  //CONSOLE.print("MQTT build and send duration: ");
  //CONSOLE.println(duration);

        }
      else
        {
          mqttReconnect();
        }
    }
  if (millis() > nextMQTTLoopTime)
    {
      nextMQTTLoopTime = millis() + 1000;
      mqttClient.loop();
    }
}



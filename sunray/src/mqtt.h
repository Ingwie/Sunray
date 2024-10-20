#ifndef MQTT_H
#define MQTT_H

#include <inttypes.h>

#define BIT(n)  (1 << (n))

#define MQTT_REQUEST_ONLINE BIT(0)
#define MQTT_REQUEST_PROPS  BIT(1)
#define MQTT_REQUEST_STATS  BIT(2)
#define MQTT_REQUEST_STATE  BIT(3)

extern uint8_t mqttRequestTopic;
void mqttReconnect();
void mqttCallback(char* topic, uint8_t* payload, uint16_t length);
void processWifiMqttClient();


#endif

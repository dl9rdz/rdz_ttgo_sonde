#include "../features.h"
#if FEATURE_MQTT

#include <Arduino.h>
#include "conn-mqtt.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ESPmDNS.h>
#include "json.h"

extern const char *version_name;
extern const char *version_id;

/*  configuration paramters are in the config, no need to duplicate :-) 
  {"mqtt.active", 0, &sonde.config.mqtt.active},
  {"mqtt.id", 63, &sonde.config.mqtt.id},
  {"mqtt.host", 63, &sonde.config.mqtt.host},
  {"mqtt.port", 0, &sonde.config.mqtt.port},
  {"mqtt.username", 63, &sonde.config.mqtt.username},
  {"mqtt.password", 63, &sonde.config.mqtt.password},
  {"mqtt.prefix", 63, &sonde.config.mqtt.prefix},
 */

TimerHandle_t mqttReconnectTimer;

/* Global initalization (on TTGO startup) */
void MQTT::init() {
}


// Internal helper function for netsetup
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/* Network initialization (as soon as network becomes available) */
void MQTT::netsetup() {
    if (!sonde.config.mqtt.active)
        return;
    if (strlen(sonde.config.mqtt.host)==0)
        return;

    WiFi.hostByName(sonde.config.mqtt.host, this->ip);

    Serial.println("[MQTT] pubsub client");
    mqttClient.setServer(ip, sonde.config.mqtt.port);
    snprintf(clientID, 20, "%s%04d", sonde.config.mqtt.id, (int)random(0, 1000));
    clientID[20] = 0;
    Serial.print(clientID);
    mqttClient.setClientId(clientID);
    if (strlen(sonde.config.mqtt.password) > 0) {
        mqttClient.setCredentials(sonde.config.mqtt.username, sonde.config.mqtt.password);
    }
}


void MQTT::updateSonde( SondeInfo *si ) {
    if(!sonde.config.mqtt.active)
	return;

    if(1 /*connected*/) {
        Serial.println("Sending sonde info via MQTT");
	// TODO: Check if si is good / fresh
        publishPacket(si);
    }
}

void MQTT::updateStation( PosInfo *pi ) {
    if(!sonde.config.mqtt.active)
	return;

    int now = millis();
    if ( (lastMqttUptime == 0 || (lastMqttUptime + 60000 < now) || (lastMqttUptime > now))) {
        publishUptime();
        lastMqttUptime = now;
    }
}

// Internal (private) functions
//void MQTT::connectToMqtt() {
//  Serial.println("Connecting to MQTT...");
//  mqttClient.connect();
//}

void MQTT::publishUptime()
{
    mqttClient.connect(); // ensure we've got connection

    Serial.println("[MQTT] writing");
    char payload[256];
    // maybe TODO: Use dynamic position if GPS is available?
    // rxlat, rxlon only if not empty
    snprintf(payload, 256, "{\"uptime\": %lu, \"user\": \"%s\", ", millis(), sonde.config.mqtt.username);
    if( !isnan(sonde.config.rxlat) && !isnan(sonde.config.rxlon) ) {
        snprintf(payload, 256, "%s\"rxlat\": %.5f, \"rxlon\": %.5f, ", payload, sonde.config.rxlat, sonde.config.rxlon);
    }
    snprintf(payload, 256, "%s\"SW\": \"%s\", \"VER\": \"%s\"}", payload, version_name, version_id);
    Serial.println(payload);
    char topic[128];
    snprintf(topic, 128, "%s%s", sonde.config.mqtt.prefix, "uptime");
    mqttClient.publish(topic, 1, 1, payload);
}

void MQTT::publishPacket(SondeInfo *si)
{
    SondeData *s = &(si->d);
    mqttClient.connect(); // ensure we've got connection

    char payload[1024];
    payload[0] = '{';
    int n = sonde2json(payload+1, 1023, si);
    if(n<0) {
	// ERROR
        Serial.println("publishPacket: sonde2json failed, string too long");
    }
    strcat(payload, "}");   // terminate payload string

    char topic[128];
    snprintf(topic, 128, "%s%s", sonde.config.mqtt.prefix, "packet");
    Serial.print(payload);
    mqttClient.publish(topic, 1, 1, payload);
}

MQTT connMQTT;
#endif

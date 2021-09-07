#include <Arduino.h>
#include "mqtt.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ESPmDNS.h>

TimerHandle_t mqttReconnectTimer;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void MQTT::init(const char* host, uint16_t port, const char* id, const char *username, const char *password, const char *prefix)
{
    WiFi.hostByName(host, this->ip);
    this->port = port;
    this->username = username;
    this->password = password;
    this->prefix = prefix;
    
    char buffer[20];
    snprintf(buffer, 20, "%s%6ld", id, random(0, 1000));
    this->id = buffer;

    Serial.println("[MQTT] pubsub client");
    mqttClient.setServer(ip, port);
    mqttClient.setClientId(id);
    if (strlen(password) > 0) {
        mqttClient.setCredentials(username, password);
    }
}

void MQTT::connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void MQTT::publishUptime()
{
    mqttClient.connect(); // ensure we've got connection

    Serial.println("[MQTT] writing");
    char payload[12];
    snprintf(payload, 12, "%lu", millis());
    char topic[128];
    snprintf(topic, 128, "%s%s", this->prefix, "uptime");
    mqttClient.publish(topic, 1, 1, payload);
}

void MQTT::publishPacket(SondeInfo *s)
{
    mqttClient.connect(); // ensure we've got connection

    char payload[1024];
    snprintf(payload, 1024, "{"
        "\"active\": %d,"
        "\"freq\": %.2f,"
        "\"id\": \"%s\","
        "\"ser\": \"%s\","
        "\"validId\": %d,"
        "\"launchsite\": \"%s\","
        "\"lat\": %.5f,"
        "\"lon\": %.5f,"
        "\"alt\": %.1f,"
        "\"vs\": %.1f,"
        "\"hs\": %.1f,"
        "\"dir\": %.1f,"
        "\"sats\": %d,"
        "\"validPos\": %d,"
        "\"time\": %d,"
        "\"frame\": %d,"
        "\"validTime\": %d,"
        "\"rssi\": %d,"
        "\"afc\": %d,"
        "\"rxStat\": \"%s\","
        "\"rxStart\": %d,"
        "\"norxStart\": %d,"
        "\"viewStart\": %d,"
        "\"lastState\": %d,"
        "\"launchKT\": %d,"
        "\"burstKT\": %d,"
        "\"countKT\": %d,"
        "\"crefKT\": %d"
        "}",
        (int)s->active,
        s->freq,
        s->id,
        s->ser,
        (int)s->validID,
        s->launchsite,
        s->lat,
        s->lon,
        s->alt,
        s->vs,
        s->hs,
        s->dir,
        s->sats,
        s->validPos,
        s->time,
        s->frame,
        (int)s->validTime,
        s->rssi,
        s->afc,
        s->rxStat,
        s->rxStart,
        s->norxStart,
        s->viewStart,
        s->lastState,
        s->launchKT,
        s->burstKT,
        s->countKT,
        s->crefKT
    );

    char topic[128];
    snprintf(topic, 128, "%s%s", this->prefix, "packet");
    mqttClient.publish(topic, 1, 1, payload);
}

#include <Arduino.h>
#include "mqtt.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ESPmDNS.h>
#include "RS41.h"
#include "json.h"

extern const char *version_name;
extern const char *version_id;

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

static char buffer[21];
void MQTT::init(const char* host, uint16_t port, const char* id, const char *username, const char *password, const char *prefix)
{
    WiFi.hostByName(host, this->ip);
    this->port = port;
    this->username = username;
    this->password = password;
    this->prefix = prefix;
    
    Serial.println("[MQTT] pubsub client");
    mqttClient.setServer(ip, port);
    snprintf(buffer, 20, "%s%04d", id, (int)random(0, 1000));
    buffer[20] = 0;
    Serial.print(buffer);
    mqttClient.setClientId(buffer);
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
    char payload[256];
    // maybe TODO: Use dynamic position if GPS is available?
    // rxlat, rxlon only if not empty
    snprintf(payload, 256, "{\"uptime\": %lu, \"user\": \"%s\", ", millis(), username);
    if( !isnan(sonde.config.rxlat) && !isnan(sonde.config.rxlon) ) {
        snprintf(payload, 256, "%s\"rxlat\": %.5f, \"rxlon\": %.5f, ", payload, sonde.config.rxlat, sonde.config.rxlon);
    }
    snprintf(payload, 256, "%s\"SW\": \"%s\", \"VER\": \"%s\"}", payload, version_name, version_id);
    Serial.println(payload);
    char topic[128];
    snprintf(topic, 128, "%s%s", this->prefix, "uptime");
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

#if 0
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
        "\"time\": %u,"
        "\"frame\": %u,"
        "\"validTime\": %d,"
        "\"rssi\": %d,"
        "\"afc\": %d,"
        "\"rxStat\": \"%s\","
        "\"rxStart\": %u,"
        "\"norxStart\": %u,"
        "\"viewStart\": %u,"
        "\"lastState\": %d,"
        "\"launchKT\": %d,"
        "\"burstKT\": %d,"
        "\"countKT\": %d,"
        "\"crefKT\": %d",
        (int)si->active,
        si->freq,
        s->id,
        s->ser,
        (int)s->validID,
        si->launchsite,
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
        si->rssi,
        si->afc,
        si->rxStat,
        si->rxStart,
        si->norxStart,
        si->viewStart,
        si->lastState,
        s->launchKT,
        s->burstKT,
        s->countKT,
        s->crefKT
    );
    if ( !isnan( s->temperature ) ) {
        snprintf(payload, 1024, "%s%s%.1f", payload, ",\"temp\": ", s->temperature );
    }
    if ( !isnan( s->relativeHumidity ) ) {
        snprintf(payload, 1024, "%s%s%.1f", payload, ",\"humidity\": ", s->relativeHumidity );
    }
    if ( !isnan( s->pressure ) ) {
        snprintf(payload, 1024, "%s%s%.1f", payload, ",\"pressure\": ", s->pressure );
    }
    if ( !isnan( s->batteryVoltage && s->batteryVoltage > 0 ) ) {
        snprintf(payload, 1024, "%s%s%.1f", payload, ",\"batt\": ", s->batteryVoltage );
    }
    char subtype[11];
    if ( RS41::getSubtype( subtype, 11, si) == 0 ) {
        snprintf(payload, 1024, "%s%s%s%s", payload, ",\"subtype\": \"", subtype, "\"" );
    }
    snprintf(payload, 1024, "%s%s", payload, "}" ); // terminate payload string
#endif
    strcat(payload, "}");   // terminate payload string

    char topic[128];
    snprintf(topic, 128, "%s%s", this->prefix, "packet");
    Serial.print(payload);
    mqttClient.publish(topic, 1, 1, payload);
}

#include "../features.h"
#if FEATURE_MQTT

#define TAG "conn-mqtt"
#include "logger.h"

#include "../core.h"

#include <Arduino.h>
#include "conn-mqtt.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ESPmDNS.h>
#include <time.h>
#include "json.h"
#include "pmu.h"
#include "posinfo.h"

extern PMU *pmu;
extern struct StationPos gpsPos;
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
  {"mqtt.report_interval", 0, &sonde.config.mqtt.interval},
 */

TimerHandle_t mqttReconnectTimer;

extern t_wifi_state wifi_state;
char time_str[32];


/* Global initalization (on TTGO startup) */
void MQTT::init() {
}

// Internal helper function for netsetup
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  LOG_D(TAG, "mqttCallback: rx on topic [%s]", topic);
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/* Network initialization (as soon as network becomes available) */
void MQTT::netsetup() {
    if (0 == sonde.config.mqtt.active)
        return;
    if (strlen(sonde.config.mqtt.host)==0)
        return;

    if (sonde.config.mqtt.report_interval <1000)
        sonde.config.mqtt.report_interval = 60000;
    WiFi.hostByName(sonde.config.mqtt.host, this->ip);

    mqttClient.setServer(ip, sonde.config.mqtt.port);
    snprintf(clientID, 20, "%s%04d", sonde.config.mqtt.id, (int)random(0, 1000));
    clientID[20] = 0;
    LOG_I(TAG, "pubsub client %s connecting to %s\n", clientID, sonde.config.mqtt.host);
    mqttClient.setClientId(clientID);
    if (strlen(sonde.config.mqtt.password) > 0) {
        mqttClient.setCredentials(sonde.config.mqtt.username, sonde.config.mqtt.password);
    }
    MQTT::connectToMqtt();
}

void MQTT::netshutdown() {
    mqttClient.disconnect(false);  // nice shutdown....
    delay(200);
    mqttClient.disconnect(true);  // force
}

void MQTT::updateSonde( SondeInfo *si ) {
    if(mqttGate(MQTT_SEND_UPTIME)){
        LOG_D(TAG, "updateSonde publishing sonde info");
	// TODO: Check if si is good / fresh
        publishPacket(si);
    }
}

void MQTT::updateStation( PosInfo *pi ) {
    unsigned long now = millis();
    if ( (lastMqttUptime == 0) || (now - lastMqttUptime >= sonde.config.mqtt.report_interval) ) {
      MQTT::connectToMqtt();
      publishUptime();
      publishPmuInfo();
      publishGps();
      lastMqttUptime = now;
    }
}

// Internal (private) functions
int MQTT::mqttGate(uint flag){
  // Decide whether or not to send (gate) the message based on selected MQTT
  // message types and MQTT connection. If the conditions are not met, then
  // the publisher function will simply return without sending anything.
  return ((sonde.config.mqtt.active & flag) && mqttClient.connected());
}

int MQTT::connectToMqtt() {
  if(mqttClient.connected())
    return 1;
  if(wifi_state != WIFI_CONNECTED)
    return 0;
  if(0 == sonde.config.mqtt.active)
    return 0;
  LOG_D(TAG, "MQTT not connected, connecting....");
  mqttClient.connect();
  return 1;
}

// Get current time and format it into a string.
// FIXME - make this globally accessible so that any function that wants to
// use the current time as a string can do this
void MQTT::timeFormat()
{
    static unsigned long last_updated = 0;
    if ((millis() - last_updated) < 500)
      return;

    last_updated = millis();
    time_t now;
    time(&now);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", gmtime(&now));
}

void MQTT::publishUptime()
{
    // ensure we've got connection
    if(!mqttGate(MQTT_SEND_UPTIME))
        return;

    char payload[256];

    timeFormat();

    // maybe TODO: Use dynamic position if GPS is available?
    // rxlat, rxlon only if not empty
    snprintf(payload, 256,
        "{\"uptime\": %.1f, \"user\": \"%s\", \"time\": %s,",
        millis() / 1000.0, sonde.config.mqtt.username, time_str );

    if (!isnan(sonde.config.rxlat) && !isnan(sonde.config.rxlon)) {
        snprintf(payload, 256,
            "%s \"rxlat\": %.5f, \"rxlon\": %.5f,",
            payload, sonde.config.rxlat, sonde.config.rxlon);
    }
    snprintf(payload, 256,
        "%s \"SW\": \"%s\", \"VER\": \"%s\"}",
        payload, version_name, version_id);
    LOG_D(TAG, "publishUptime: sending %s\n", payload);
    char topic[128];
    snprintf(topic, 128, "%s%s", sonde.config.mqtt.prefix, "uptime");
    mqttClient.publish(topic, 1, 1, payload);
}

void MQTT::publishPmuInfo()
{
    if(!mqttGate(MQTT_SEND_PMU))
        return;

    char payload[256];
    float i_d = pmu->getBattDischargeCurrent();
    float i_c = pmu->getBattChargeCurrent();
    float i_batt = 0;

    if (i_c)
      i_batt = i_c;
    else if (i_d)
      i_batt = -i_d;

    snprintf(payload, sizeof(payload),
        "{\"I_Batt\": %.1f, \"V_Batt\": %.3f, \"I_Vbus\": %.1f, \"V_Vbus\": %.3f, \"T_sys\": %.1f}",
        i_batt, pmu->getBattVoltage() / 1000.,
        pmu->getVbusCurrent(), pmu->getVbusVoltage() / 1000.,
        pmu->getTemperature());
    LOG_D(TAG, "publishPmuInfo: sending %s\n", payload);

    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", sonde.config.mqtt.prefix, "pmu");
    mqttClient.publish(topic, 1, 1, payload);
}


void MQTT::publishGps()
{
    if((sonde.config.gps_rxd==-1) || !mqttGate(MQTT_SEND_GPS))
        return;

    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"valid\": %1d, \"systime\": \"%s\", \"gpstime\": \"%s\", "
             "\"lat\": %f, \"lon\": %f, \"alt\": %d, "
             "\"course\":%d, \"speed\":%.1f, \"sat\": %d}",
             gpsPos.valid, time_str, gpsPos.time,
             gpsPos.lat, gpsPos.lon, gpsPos.alt,
             gpsPos.course, gpsPos.speed, gpsPos.sat
        );
    LOG_D(TAG, "publishGps: sending %s\n", payload);

    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", sonde.config.mqtt.prefix, "gps");
    mqttClient.publish(topic, 1, 1, payload);
}


void MQTT::publishPeak(double pf, int rssi)
{
    if(!mqttGate(MQTT_SEND_RFINFO))
      return;

    timeFormat();
    char payload[256];
    snprintf(payload, 256, "{\"time\": \"%s\", \"peak\": %.3f, \"rssi\": %.1f}",time_str, pf*1e-6, rssi/2.0);
    LOG_D(TAG, "publishPeak: sending %s\n", payload);

    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", sonde.config.mqtt.prefix, "spectrum");
    mqttClient.publish(topic, 1, /* retain */ false, payload);
}

// What's the scanner looking at?
void MQTT::publishQRG(int num, const char* type, char* launchsite, float mhz)
{
    if(!mqttGate(MQTT_SEND_RFINFO))
      return;

    char payload[256];
    snprintf(
        payload, 256,
        "{\"num\": %d, \"type\": \"%s\", \"site\": \"%s\", \"freq\": %.3f}",
        num, type, launchsite, mhz);
    LOG_D(TAG, "publishQRG: sending %s\n", payload);

    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", sonde.config.mqtt.prefix, "qrg");
    mqttClient.publish(topic, 1, /*retain*/ false, payload);
}


// think "syslog over mqtt"
void MQTT::publishDebug(char *debugmsg)
{
    if(!mqttGate(MQTT_SEND_DEBUG))
        return;

    char payload[256];
    snprintf(payload, 256, "{\"msg\": %s}", debugmsg);
    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", sonde.config.mqtt.prefix, "debug");
    mqttClient.publish(topic, 1, /*retain*/ false, payload);
}

void MQTT::publishPacket(SondeInfo *si)
{
    SondeData *s = &(si->d);
    // ensure we've got connection
    if(!mqttGate(MQTT_SEND_UPTIME))
        return;

    char payload[1024];
    payload[0] = '{';
    int n = sonde2json(payload+1, 1023, si);
    if(n<0) {
	// ERROR
        LOG_E(TAG, "publishPacket: sonde2json failed, string too long");
    }
    strcat(payload, "}");   // terminate payload string

    char topic[128];
    snprintf(topic, 128, "%s%s", sonde.config.mqtt.prefix, "packet");
    LOG_D(TAG, "publishPacket: %s\n", payload);
    mqttClient.publish(topic, 1, 1, payload);
}

String MQTT::getStatus() {
    if(!sonde.config.mqtt.active) return String("disabled");
    if(mqttClient.connected()) return String("connected");
    else return String("not connected");
}

String MQTT::getName() {
    return String("MQTT");
}

MQTT connMQTT;
#endif

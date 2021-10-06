#ifndef MQTT_h
#define MQTT_h

#include <WiFi.h>
#include <AsyncMqttClient.h>
#include "Sonde.h"
#include "RS41.h"

class MQTT
{
public:
    WiFiClient mqttWifiClient;
    AsyncMqttClient mqttClient;
    TimerHandle_t mqttReconnectTimer;
    IPAddress ip;
    uint16_t port;
    const char *username;
    const char *password;
    const char *prefix;

    void init(const char *host, uint16_t port, const char *id, const char *username, const char *password, const char *prefix);
    void publishPacket(SondeInfo *s);
    void publishUptime();
private:
    void connectToMqtt();
};

#endif

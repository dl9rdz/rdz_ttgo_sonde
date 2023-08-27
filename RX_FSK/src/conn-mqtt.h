#include "../features.h"
#if FEATURE_MQTT

#ifndef MQTT_h
#define MQTT_h

#include <WiFi.h>
#include <AsyncMqttClient.h>
#include "Sonde.h"
//#include "RS41.h"
#include "conn.h"

class MQTT : public Conn
{
public:
        /* Called once on startup */
        void init();

        /* Called whenever the network becomes available */
        void netsetup();

        /* Called approx 1x / second (maybe only if good data is available) */
        virtual void updateSonde( SondeInfo *si );

        /* Called approx 1x / second* */
        virtual void updateStation( PosInfo *pi );


private:
    WiFiClient mqttWifiClient;
    AsyncMqttClient mqttClient;
    TimerHandle_t mqttReconnectTimer;
    IPAddress ip;
    //uint16_t port;
    //const char *username;
    //const char *password;
    //const char *prefix;
    char clientID[21];

    //void init(const char *host, uint16_t port, const char *id, const char *username, const char *password, const char *prefix);
    void publishPacket(SondeInfo *s);
    void publishUptime();
    //void connectToMqtt();

    unsigned long lastMqttUptime = 0;
    boolean mqttEnabled;
};

extern MQTT connMQTT;
#endif

#endif

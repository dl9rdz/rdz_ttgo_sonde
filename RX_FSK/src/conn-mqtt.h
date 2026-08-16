#include "../features.h"
#if FEATURE_MQTT

#ifndef MQTT_h
#define MQTT_h

#include <WiFi.h>
#include <AsyncMqttClient.h>
#include "Sonde.h"
#include "conn.h"

#define MQTT_SEND_SONDE 0x01
#define MQTT_SEND_UPTIME 0x02
#define MQTT_SEND_PMU 0x04
#define MQTT_SEND_GPS 0x08
#define MQTT_SEND_RFINFO 0x10
#define MQTT_SEND_DEBUG 0x80
#define MQTT_SEND_ANY (MQTT_SEND_UPTIME|MQTT_SEND_SONDE|MQTT_SEND_PMU|MQTT_SEND_GPS|MQTT_SEND_RFINFO|MQTT_SEND_DEBUG)

#define MQTT_QOS_NONE 0
#define MQTT_QOS 1
#define MQTT_RETAIN_TRUE true
#define MQTT_RETAIN_FALSE false

class MQTT : public Conn
{
public:
        /* Called once on startup */
        void init();

        /* Called whenever the network becomes available */
        void netsetup();

        /* Close connections */
        void netshutdown();

        /* Called approx 1x / second (maybe only if good data is available) */
        virtual void updateSonde( SondeInfo *si );

        /* Called approx 1x / second* */
        void updateStation( PosInfo *pi );

        /* Called when frequency changes */
        void updateQRG( int nextIndex );

        /* Say whether MQTT is connected, disconnected, or even enabled */
	String getStatus();

        String getName();

        /* Radio debug - spectrum and scanner*/
        void publishPeak(double pf, int rssi);
        void publishDebug(char* debugmsg);

       private:
        WiFiClient mqttWifiClient;
        AsyncMqttClient mqttClient;
        TimerHandle_t mqttReconnectTimer;
        IPAddress ip;
        // uint16_t port;
        // const char *username;
        // const char *password;
        // const char *prefix;
        char clientID[21];

        // void init(const char *host, uint16_t port, const char *id, const char
        // *username, const char *password, const char *prefix);
        void publishPacket(SondeInfo* s);
        void publishUptime();
        void publishPmuInfo();
        void publishGps();
        void publishLwt(const char *message);
        void timeFormat();
        int mqttGate(uint flag);
        int connectToMqtt();

        unsigned long lastMqttUptime = 0;
        boolean mqttEnabled;
};

extern MQTT connMQTT;
#endif

#endif

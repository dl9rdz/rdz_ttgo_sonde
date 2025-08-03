#include "../features.h"
#if FEATURE_SONDESEEKER

#include "conn-sondeseeker.h"
#include <WiFiUdp.h>
#include "posinfo.h"
#include "json.h"
#define TAG "conn-sondeseeker"
#include "logger.h"
#include "../core.h"

extern const char *sondeTypeStrSH[];
extern WiFiUDP udp;

void ConnSondeseeker::init() {
}

void ConnSondeseeker::netsetup() {
}

void ConnSondeseeker::netshutdown() {
}

void ConnSondeseeker::updateSonde(SondeInfo *si) {

    if (!sonde.config.ss.active)
        return;

    if (wifi_state != WIFI_APMODE && wifi_state != WIFI_CONNECTED)
        return;

    char buf[1024];

    strcpy(buf, "{\"sonde\": {");
    sonde2json(buf + strlen(buf), 1024, si);
    strcat(buf, "}}");

    //Serial.printf("Sending Sondeseeker json: %s\n", buf);
    udp.beginPacket(sonde.config.ss.host, sonde.config.ss.port);
    udp.write((const uint8_t *)buf, strlen(buf));
    udp.endPacket();

}

void ConnSondeseeker::updateStation(PosInfo *pi) {
}

// What's the scanner looking at?
void ConnSondeseeker::updateQRG(int sondeIndex) {
    if (!sonde.config.ss.active)
        return;

    if (wifi_state != WIFI_APMODE && wifi_state != WIFI_CONNECTED)
        return;

    SondeInfo *si = &sonde.sondeList[sondeIndex];
    const char *type = sondeTypeStr[si->type];
    const char *launchsite = si->launchsite;
    float mhz = si->freq;
    int num = sondeIndex + 1;

    char payload[256];
    snprintf(
        payload, 256,
        "{\"channel\": %d, \"type\": \"%s\", \"site\": \"%s\", \"freq\": %.3f}",
        num, type, launchsite, mhz);
    LOG_D(TAG, "updateQRG: sending %s\n", payload);

    udp.beginPacket(sonde.config.ss.host, sonde.config.ss.port);
    udp.write((const uint8_t *)payload, strlen(payload));
    udp.endPacket();
}

String ConnSondeseeker::getStatus() {
    if(!sonde.config.ss.active) return String("disabled");
    char info[100];
    snprintf(info, 100, "active [%s:%d]", sonde.config.ss.host, sonde.config.ss.port);
    return String(info);
}

String ConnSondeseeker::getName() {
    return String("Sondeseeker");
}

ConnSondeseeker connSondeseeker;
#endif

#include "logger.h"
#include <stdio.h>
#include "colors.h"

#include <Arduino.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "netmgr.h"

/* Compile-time UDP log mirror for remote debugging (no USB).
 * Off by default. Enable and set LOG_UDP_HOST to your collector IP.
 * Sends only while Wi-Fi is up; uses a dedicated socket (not the APRS udp).
 */
#define USE_UDP_LOG 0
#define LOG_UDP_HOST "192.168.1.91"
#define LOG_UDP_PORT 12345

extern boolean connected;

extern const char *version_name;
extern const char *version_id;

static const char *lvlch[] = { "E", "W", "I", "D" };
static const char *lvlcol[] = { RED, YEL, GRN, BLU };

bool inimprov = false;

extern void enableNetwork(bool enable); // in RX_FSK.ino

#if USE_UDP_LOG
static WiFiUDP logUdp;
static SemaphoreHandle_t logUdpMutex = NULL;
static bool logUdpReady = false;

static void udpLogSend(const uint8_t *data, size_t len) {
	if (!logUdpReady || !connected) return;
	if (netmgr.status() != NetStatus::Uplink) return;
	if (!logUdpMutex) return;
	if (xSemaphoreTake(logUdpMutex, 0) != pdTRUE) return;
	if (logUdp.beginPacket(LOG_UDP_HOST, LOG_UDP_PORT)) {
		logUdp.write(data, len);
		logUdp.endPacket();
	}
	xSemaphoreGive(logUdpMutex);
}
#endif

void Logger::init() {
	sonde.config.debug = 3; // Use as initial value until config file is read from file system
#if USE_UDP_LOG
	if (!logUdpMutex) {
		logUdpMutex = xSemaphoreCreateMutex();
	}
#endif
}

void Logger::netUp() {
#if USE_UDP_LOG
	if (!logUdpMutex) {
		logUdpMutex = xSemaphoreCreateMutex();
	}
	if (logUdpMutex && xSemaphoreTake(logUdpMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		logUdp.begin(0);
		logUdpReady = true;
		xSemaphoreGive(logUdpMutex);
	}
#endif
}

void Logger::netDown() {
#if USE_UDP_LOG
	if (logUdpMutex && xSemaphoreTake(logUdpMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		logUdpReady = false;
		logUdp.stop();
		xSemaphoreGive(logUdpMutex);
	} else {
		logUdpReady = false;
	}
#endif
}

void Logger::logf(LOGLEVEL lvl, const char *module, const char *fmt, ...) {
	int color = sonde.config.debug >= 10 ? 1 : 0;
	int cfglvl = sonde.config.debug % 10;
	if (cfglvl > 3) cfglvl = 3;
	if (lvl > cfglvl) return;

	char buf[256];
	int n;
	if (color) {
		n = snprintf(buf, sizeof(buf), "%s%s%s (%6lu) %s%-14s%s: ",
			lvlcol[lvl], lvlch[lvl], COLOR_RESET,
			(unsigned long)millis(),
			lvlcol[lvl], module ? module : "?", COLOR_RESET);
	} else {
		n = snprintf(buf, sizeof(buf), "%s (%6lu) %-14s: ",
			lvlch[lvl], (unsigned long)millis(), module ? module : "?");
	}
	if (n < 0) n = 0;
	if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;

	va_list ptr;
	va_start(ptr, fmt);
	int m = vsnprintf(buf + n, sizeof(buf) - n, fmt, ptr);
	va_end(ptr);
	(void)m;

	size_t len = strlen(buf);
	while (len && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
		buf[--len] = 0;
	}
	if (len + 1 < sizeof(buf)) {
		buf[len++] = '\n';
		buf[len] = 0;
	}

	if (!inimprov) Serial.print(buf);
#if USE_UDP_LOG
	udpLogSend((const uint8_t *)buf, len);
#endif
}

Logger Log;


// Let's put this here as it is serial communication as well?

void Logger::sendImprov(int type, int len, const char *data) {
    char buf[100];
    Serial.write("\n", 1);
    strcpy(buf, "IMPROV\x01");
    buf[7] = type;
    buf[8] = len;
    memcpy(buf+9, data, len);
    int crc = 0;
    for(int i=0; i<9+len; i++) crc += buf[i];
    buf[9+len] = crc & 0xff;
    buf[9+len+1] = '\n';
    buf[9+len+2] = 0;
    Serial.write(buf, 9+len+2);
#if USE_UDP_LOG
    udpLogSend((const uint8_t *)"Reply:", 6);
    udpLogSend((const uint8_t *)buf, 9+len+2);
#endif
}

void Logger::sendImprovResult(int replyto, const char *strings[]) {
    char buf[512];
    Serial.write("\n", 1);
    strcpy(buf, "IMPROV\x01");
    buf[7] = 0x04; // type: RPC Response
    //buf[8] will be length, set at the end
    buf[9] = replyto;
    //buf[10] will be length, set at the end
    int i = 11;
    int nstr = 0;
    while(*strings) {
        int len = strlen(*strings);
        buf[i] = len;
        memcpy(buf+i+1, *strings, len);
        i += len+1;
        strings++;
        nstr++;
    }
    buf[8] = i-9;
    buf[10] = i-11;

    int crc = 0;
    for(int j=0; j<i; j++) crc += buf[j];
    buf[i++] = crc & 0xff;
    buf[i++] = '\n';
    buf[i++] = 0;
    Serial.write(buf, i-1);
#if USE_UDP_LOG
    udpLogSend((const uint8_t *)"Reply:", 6);
    udpLogSend((const uint8_t *)buf, i-1);
#endif
}

int cmdlen = 0;
char cmd[50];

#if 0
void Logger::handleImprovByte(char c) {
    cmd[cmdlen++] = c;
    if(cmdlen==1)
	return c == 'I';
    else if(cmdlen==2)
	return c == 'M';
    else if(cmdlen==3)
	return c == 'P';
    else if(cmdlen==4)
	return c == 'R';
    else if(cmdlen==5)
	return c == 'O';
    else if(cmdlen==6)
	return c == 'V';
    else if(cmdlen==7)
	return c == 1; // version number
    else if(cmdlen<10)
	return 1;      // read type and len
    // type is cmd[7]
    // len is cmd[8]
    // wait to read 'len' more bytes...
    if(cmdlen < 8 + cmd[8] + 1) return 1;
}
#endif

static void improv_scan_cb(const char *ssid, int rssi, void *arg)
{
	(void)arg;
	char rssi_s[8];
	snprintf(rssi_s, sizeof(rssi_s), "%d", rssi);
	const char *info[4] = { ssid, rssi_s, "YES", NULL };
	Log.sendImprovResult(0x04, info);
}

void Logger::handleImprov() {
    while(Serial.available()) {
        cmd[cmdlen] = Serial.read();
        if(cmd[cmdlen] == '\n') { // check if command
#if USE_UDP_LOG
            udpLogSend((const uint8_t *)cmd, cmdlen+1);
#endif
            if(strncmp(cmd, "IMPROV", 6)==0) {  // we have a command
                inimprov = true;
                // TODO: CHeck CRC
                if(cmd[7]==0x03 && cmd[9]==0x03) { // RPC, get info
		    const char *info[]={version_name, version_id, ESP.getChipModel(), "rdzSonde", NULL};
                    sendImprovResult(0x03, info);
                }
		if(cmd[7]==0x03 && cmd[9]==0x02) { // request state
		    if(netmgr.status() == NetStatus::Down ||
		       netmgr.status() == NetStatus::Connecting) {
		        sendImprov(0x01, 1, "\x02"); // current state: ready
		    } else {
			sendImprov(0x01, 1, "\x04"); // provisioned
			char url[50];
			char ip[24];
			const char *i[]={url, NULL};
			if (netmgr.ipv4(ip, sizeof(ip)))
				snprintf(url, 50, "http://%s/", ip);
			else
				url[0] = 0;
			sendImprovResult(0x02, i);
		    }
		}
		if(cmd[7]==0x03 && cmd[9]==0x04) { // wifi scan
		    netmgr.scan(improv_scan_cb, nullptr);
		    const char *info[4]={NULL};
		    sendImprovResult(0x04, info);
		}
		if(cmd[7]==0x03 && cmd[9]==0x01) { // send Wi-Fi settings
		    String ssid = String(cmd+12, cmd[11]);
		    int pwpos = 12+cmd[11];
		    String pw = String(cmd+pwpos+1, cmd[pwpos]);
		    netmgr.connectSta(ssid.c_str(), pw.c_str());
		    for(int i=0; i<20; i++) {
                        if(netmgr.status() == NetStatus::Uplink) {
			    if (!connected)
				enableNetwork(true);
			    char url[50];
			    char ip[24];
			    const char *i[]={url, NULL};
			    if (netmgr.ipv4(ip, sizeof(ip)))
				snprintf(url, 50, "http://%s/", ip);
			    else
				url[0] = 0;
			    sendImprovResult(0x02, i);
			    break;
                        }
                        delay(500);
                    }
		    const char *i[]={NULL};
                    sendImprovResult(0x02, i);
		}
            }
            cmdlen = 0;
        } else {
            cmdlen++;
            if(cmdlen>=50) cmdlen = 0;  // avoid overflow...
        }
    }
}

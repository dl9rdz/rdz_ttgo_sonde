#include "../features.h"

#if FEATURE_APRS

#include "conn-aprs.h"
#include "aprs.h"
#include "posinfo.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#include <sys/socket.h>
#include <lwip/dns.h>

#include <ESPAsyncWebServer.h>

// KISS over TCP for communicating with APRSdroid
static WiFiServer tncserver(14580);
static WiFiClient tncclient;

// APRS over TCP for radiosondy.info etc
static int tcpclient = 0;
enum { TCS_DISCONNECTED, TCS_DNSLOOKUP, TCS_DNSRESOLVED, TCS_CONNECTING, TCS_LOGIN, TCS_CONNECTED };
static uint8_t tcpclient_state = TCS_DISCONNECTED;
ip_addr_t tcpclient_ipaddr;

extern const char *version_name;
extern const char *version_id;

extern WiFiUDP udp;

static unsigned long last_in = 0;

void tcpclient_fsm();


void ConnAPRS::init() {
	aprs_gencrctab();
}

void ConnAPRS::netsetup() {
    // Setup for KISS TCP server
    if(sonde.config.kisstnc.active) {
        MDNS.addService("kiss-tnc", "tcp", 14580);
        tncserver.begin();
    }

    if(sonde.config.tcpfeed.active) {
        // start the FSM
        tcpclient_fsm();
    }
}

void ConnAPRS::updateSonde( SondeInfo *si ) {
    // prepare data (for UDP and TCP output)
    char *str = aprs_senddata(si, sonde.config.call, sonde.config.objcall, sonde.config.udpfeed.symbol);

    // Output via AXUDP
    if(sonde.config.udpfeed.active) {
        char raw[201];
        int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
        Serial.println("Sending AXUDP");
        //Serial.println(raw);
         udp.beginPacket(sonde.config.udpfeed.host, sonde.config.udpfeed.port);
         udp.write((const uint8_t *)raw, rawlen);
         udp.endPacket();
    }
    // KISS via TCP (incoming connection, e.g. from APRSdroid
    if (tncclient.connected()) {
        Serial.println("Sending position via TCP");
        char raw[201];
        int rawlen = aprsstr_mon2kiss(str, raw, APRS_MAXLEN);
        Serial.print("sending: "); Serial.println(raw);
        tncclient.write(raw, rawlen);
    }
    // APRS via TCP (outgoing connection to aprs-is, e.g. radiosonde.info or wettersonde.net
    if (sonde.config.tcpfeed.active) {
        static unsigned long lasttcp = 0;
        tcpclient_fsm();
        if(tcpclient_state == TCS_CONNECTED) {
            unsigned long now = millis();
            long tts =  sonde.config.tcpfeed.highrate * 1000L - (now-lasttcp);
            Serial.printf("aprs: now-last = %ld\n", (now - lasttcp));
            if ( tts < 0 ) {
                strcat(str, "\r\n");
                Serial.printf("Sending APRS: %s",str);
                write(tcpclient, str, strlen(str));
                lasttcp = now;
            } else {
                Serial.printf("Sending APRS in %d s\n", (int)(tts/1000));
            }
        }
    }
}

#define APRS_TIMEOUT 25000

void ConnAPRS::updateStation( PosInfo *pi ) {
    // This funciton is called peridocally.

    // We check for stalled connection and possibly close it
    Serial.printf("last_in - now: %ld\n", millis() - last_in);
    if ( sonde.config.tcpfeed.timeout > 0) {
        if ( last_in && ( (millis() - last_in) > sonde.config.tcpfeed.timeout*1000 ) ) {
            Serial.println("APRS timeout - closing connection");
            close(tcpclient);
            tcpclient_state = TCS_DISCONNECTED;
        }
    }

    // If available, read data from tcpclient; then send update (if its time for that)
    tcpclient_fsm();
    if(sonde.config.tcpfeed.active) {
        aprs_station_update();
    }

    // We check for new connections or new data (tnc port) 
    if (!tncclient.connected()) {
        tncclient = tncserver.available();
        if (tncclient.connected()) {
           Serial.println("new TCP KISS connection");
        }
    }
    if (tncclient.available()) {
        Serial.print("TCP KISS socket: recevied ");
        while (tncclient.available()) {
           Serial.print(tncclient.read());  // Check if we receive anything from from APRSdroid
        }
        Serial.println("");
    }
}

void ConnAPRS::aprs_station_update() {
  int chase = sonde.config.chase;
  // automatically decided if CHASE or FIXED mode is used (for config AUTO)
  if (chase == SH_LOC_AUTO) {
    if (posInfo.chase) chase = SH_LOC_CHASE; else chase = SH_LOC_FIXED;
  }
  unsigned long time_now = millis();
  unsigned long time_delta = time_now - time_last_aprs_update;
  unsigned long update_time = (chase == SH_LOC_CHASE) ? APRS_MOBILE_STATION_UPDATE_TIME : APRS_STATION_UPDATE_TIME;
  long tts = update_time - time_delta;
  Serial.printf("aprs_statio_update due in %d s", (int)(tts/1000));
  if (tts>0) return;

  float lat, lon;
  if (chase == SH_LOC_FIXED) {
    // fixed location
    lat = sonde.config.rxlat;
    lon = sonde.config.rxlon;
    if (isnan(lat) || isnan(lon)) return;
  } else {
    if (gpsPos.valid) {
      lat = gpsPos.lat;
      lon = gpsPos.lon;
    } else {
      return;
    }
  }
  char *bcn = aprs_send_beacon(sonde.config.call, lat, lon, sonde.config.beaconsym + ((chase == SH_LOC_CHASE) ? 2 : 0), sonde.config.comment);
  tcpclient_fsm();
  if(tcpclient_state == TCS_CONNECTED) {
    strcat(bcn, "\r\n");
    Serial.printf("APRS TCP BEACON: %s", bcn);
    write(tcpclient, bcn, strlen(bcn));
    time_last_aprs_update = time_now;
  }
}

static void _tcp_dns_found(const char * name, const ip_addr_t *ipaddr, void * arg) {
    if (ipaddr) {
        tcpclient_ipaddr = *ipaddr;
        tcpclient_state = TCS_DNSRESOLVED;    // DNS lookup success
    } else {
        memset(&tcpclient_ipaddr, 0, sizeof(tcpclient_ipaddr));
        tcpclient_state = TCS_DISCONNECTED;   // DNS lookup failed
    }
}

void tcpclient_sendlogin() {
    char buf[128];
    snprintf(buf, 128, "user %s pass %d vers %s %s\r\n", sonde.config.call, sonde.config.passcode, version_name, version_id);
    int res = write(tcpclient, buf, strlen(buf));
    Serial.printf("APRS login: %s, res=%d\n", buf, res);
    last_in = millis();
    if(res<=0) {
        close(tcpclient);
        tcpclient_state = TCS_DISCONNECTED;
    }
}

void tcpclient_fsm() {
    if(!sonde.config.tcpfeed.active)
        return;
    Serial.printf("TCS: %d\n", tcpclient_state);

    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(tcpclient, &fdset);
    fd_set fdeset;
    FD_ZERO(&fdeset);
    FD_SET(tcpclient, &fdeset);

    struct timeval selto = {0};
    int res;

    switch(tcpclient_state) {
    case TCS_DISCONNECTED: 
        /* We are disconnected. Try to connect, starting with a DNS lookup */
      {
        // Restart timeout
        last_in = millis();
        err_t res = dns_gethostbyname( sonde.config.tcpfeed.host, &tcpclient_ipaddr, /*(dns_found_callback)*/_tcp_dns_found, NULL );

        if(res == ERR_OK) {   // Returns immediately of host is IP or in cache
            tcpclient_state = TCS_DNSRESOLVED;
            /* fall through */
        } else if(res == ERR_INPROGRESS) {
            tcpclient_state = TCS_DNSLOOKUP;
            break;
        } else {  // failed
            tcpclient_state = TCS_DISCONNECTED;
            break;
        }
      }

    case TCS_DNSRESOLVED:
      {
        /* We have got the IP address, start the connection */
        tcpclient = socket(AF_INET, SOCK_STREAM, 0);
        int flags = fcntl(tcpclient, F_GETFL);
        if (fcntl(tcpclient, F_SETFL, flags | O_NONBLOCK) == -1) {
            Serial.println("Setting O_NONBLOCK failed");
        }

        struct sockaddr_in sock_info;
        memset(&sock_info, 0, sizeof(struct sockaddr_in));
        sock_info.sin_family = AF_INET;
        sock_info.sin_addr.s_addr = tcpclient_ipaddr.u_addr.ip4.addr;
        sock_info.sin_port = htons( sonde.config.tcpfeed.port );
        err_t res = connect(tcpclient, (struct sockaddr *)&sock_info, sizeof(sock_info));
        if(res) {
            if (errno == EINPROGRESS) { // Should be the usual case, go to connecting state
                tcpclient_state = TCS_CONNECTING;
            } else {
                close(tcpclient);
                tcpclient_state = TCS_DISCONNECTED;
            }
        } else {
            tcpclient_state = TCS_CONNECTED;
            tcpclient_sendlogin();
        }
      }
      break;
    case TCS_CONNECTING: 
      {
        // Poll to see if we are now connected 
        res = select(tcpclient+1, NULL, &fdset, &fdeset, &selto);
        if(res<0) {
            Serial.println("TNS_CONNECTING: select error");
            goto error;
        } else if (res==0) { // still pending
            break;
        }
        // Socket has become ready (or something went wrong, check for error first)
        
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        if (getsockopt(tcpclient, SOL_SOCKET, SO_ERROR, (void*)(&sockerr), &len) < 0) {
            goto error;
        }
        Serial.printf("select returing %d. isset:%d iseset:%d sockerr:%d\n", res, FD_ISSET(tcpclient, &fdset), FD_ISSET(tcpclient, &fdeset), sockerr);
        if(sockerr) {
            Serial.printf("APRS connect error: %s\n", strerror(sockerr));
            goto error;
        }
        tcpclient_state = TCS_CONNECTED;
        tcpclient_sendlogin();
      }
      break;
        
    case TCS_CONNECTED:
      {
        res = select(tcpclient+1, &fdset, NULL, NULL, &selto);
        if(res<0) {
            Serial.println("TCS_CONNECTING: select error");
            goto error;
        } else if (res==0) { // still pending
            break;
        }
        // Read data
        char buf[512+1];
        res = read(tcpclient, buf, 512);
        if(res<=0) {
            close(tcpclient);
            tcpclient_state = TCS_DISCONNECTED;
        } else {
            buf[res] = 0;
            Serial.printf("tcpclient data (len=%d):", res);
            Serial.write( (uint8_t *)buf, res );
            last_in = millis();    
        }
      }
      break;

    case TCS_DNSLOOKUP:
        Serial.println("DNS lookup in progress");
        break;   // DNS lookup in progress, do not do anything until callback is called, updating the state
    }
    return;

error:
    close(tcpclient);
    tcpclient_state = TCS_DISCONNECTED;
    return;
}


ConnAPRS connAPRS;

#endif

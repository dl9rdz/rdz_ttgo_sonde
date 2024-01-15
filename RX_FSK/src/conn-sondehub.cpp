#include "../features.h"

#if FEATURE_SONDEHUB

#include "conn-sondehub.h"
#include "posinfo.h"
#include "../core.h"
#include "DFM.h"
#include "RS41.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include "ShFreqImport.h"

#include <sys/socket.h>
#include <lwip/dns.h>

#include <ESPAsyncWebServer.h>

extern const char *version_name;
extern const char *version_id;

#define SONDEHUB_STATION_UPDATE_TIME (60*60*1000) // 60 min
#define SONDEHUB_MOBILE_STATION_UPDATE_TIME (30*1000) // 30 sec
WiFiClient shclient;    // Sondehub v2
int shImportInterval = 0;
char shImport = 0;
unsigned long time_last_update = 0;

void ConnSondehub::init() {
}

void ConnSondehub::netsetup() {
    if (sonde.config.sondehub.active && wifi_state != WIFI_APMODE) {
        time_last_update = millis() + 1000; /* force sending update */
        sondehub_station_update();

	// SH import: initial refresh on connect, even if configured interval is longer
        shImportInterval = 5;   // refresh now in 5 seconds
    } 
}

// Imitating the old non-modular code
// updateSonde is called once per second
// old code called
//   each second: sondehub_reply_handler
//   each second, if good decode: sondehub_send_data
//   each second, if no good decode: sondehub_finish_data
void ConnSondehub::updateSonde( SondeInfo *si ) {
    sondehub_reply_handler();
    if(si==NULL) {
        sondehub_finish_data();
    } else {
	sondehub_send_data(si);
    }
}


void ConnSondehub::updateStation( PosInfo *pi ) {
    // TODO: station_update should be here and not in netsetup()
    // Currently, interlnal reply_handler does this, using gpsInfo global variable instead of this pi
}

String ConnSondehub::getStatus() {
	return String("");
}

/*** Code moved from RX_FSK to here ****/

// Sondehub v2 DB related codes
/*
        Update station data to the sondehub v2 DB
*/
/* which_pos: 0=none, 1=fixed, 2=gps */
void ConnSondehub::sondehub_station_update() {
  struct st_sondehub *conf = &sonde.config.sondehub;
#define STATION_DATA_LEN 300
  char data[STATION_DATA_LEN];
  char *w;

  // If there is no connection to some WiFi AP, we cannot upload any data at all....
  if ( wifi_state != WIFI_CONNECTED ) return;

  unsigned long time_now = millis();
  // time_delta will be correct, even if time_now overflows
  unsigned long time_delta = time_now - time_last_update;

  int chase = conf->chase;
  // automatically decided if CHASE or FIXED mode is used (for config AUTO)
  if (chase == SH_LOC_AUTO) {
    if (posInfo.chase) chase = SH_LOC_CHASE; else chase = SH_LOC_FIXED;
  }

  // Use 30sec update time in chase mode, 60 min in station mode.
  unsigned long update_time = (chase == SH_LOC_CHASE) ? SONDEHUB_MOBILE_STATION_UPDATE_TIME : SONDEHUB_STATION_UPDATE_TIME;

  // If it is not yet time to send another update. do nothing....
  if ( (time_delta <= update_time) ) return;

  Serial.println("sondehub_station_update()");
  time_last_update = time_now;

  if (!shclient.connected()) {
    if (!shclient.connect(conf->host, 80)) {
      Serial.println("Connection FAILED");
      return;
    }
  }

  w = data;
  // not necessary...  memset(w, 0, STATION_DATA_LEN);

  sprintf(w,
          "{"
          "\"software_name\": \"%s\","
          "\"software_version\": \"%s\","
          "\"uploader_callsign\": \"%s\",",
          version_name, version_id, conf->callsign);
  w += strlen(w);

  // Only send email if provided
  if (strlen(conf->email) != 0) {
    sprintf(w, "\"uploader_contact_email\": \"%s\",", conf->email);
    w += strlen(w);
  }

 // Only send antenna if provided
  if (strlen(conf->antenna) != 0) {
    sprintf(w, "\"uploader_antenna\": \"%s\",", conf->antenna);
    w += strlen(w);
  }

  // We send GPS position: (a) in CHASE mode, (b) in AUTO mode if no fixed location has been specified in config
  if (chase == SH_LOC_CHASE) {
    if (gpsPos.valid) {
      sprintf(w,
              "\"uploader_position\": [%.6f,%.6f,%d],"
              "\"mobile\": true",
              gpsPos.lat, gpsPos.lon, gpsPos.alt);
    } else {
      sprintf(w, "\"uploader_position\": [null,null,null]");
    }
    w += strlen(w);
  }
  // Otherweise, in FIXED mode we send the fixed position from config (if specified)
  else if (chase == SH_LOC_FIXED) {
    if ((!isnan(sonde.config.rxlat)) && (!isnan(sonde.config.rxlon))) {
      if (isnan(sonde.config.rxalt))
        sprintf(w, "\"uploader_position\": [%.6f,%.6f,null]", sonde.config.rxlat, sonde.config.rxlon);
      else
        sprintf(w, "\"uploader_position\": [%.6f,%.6f,%d]", sonde.config.rxlat, sonde.config.rxlon, (int)sonde.config.rxalt);
    } else {
      sprintf(w, "\"uploader_position\": [null,null,null]");
    }
    w += strlen(w);
  } else {
    sprintf(w, "\"uploader_position\": [null,null,null]");
    w += strlen(w);
  }

  // otherwise (in SH_LOC_NONE mode) we dont include any position info
  sprintf(w, "}");

  shclient.println("PUT /listeners HTTP/1.1");
  shclient.print("Host: ");
  shclient.println(conf->host);
  shclient.println("accept: text/plain");
  shclient.println("Content-Type: application/json");
  shclient.print("Content-Length: ");
  shclient.println(strlen(data));
  shclient.println();
  shclient.println(data);
  Serial.println(strlen(data));
  Serial.println(data);
  Serial.println("Waiting for response");
  // TODO: better do this asynchronously
  // At least, do this safely. See Notes-on-Using-WiFiClient.txt for details
  // If any of the shclient.print failed before (remote end closed connection),
  // then calling client->read will cause a LoadProhibited exception
  if (shclient.connected()) {
    String response = shclient.readString();
    Serial.println(response);
    Serial.println("Response done...");
  } else {
    Serial.println("SH client connection closed\n");
  }
  //client->stop();
}

/*
        Update sonde data to the sondehub v2 DB
*/
enum SHState { SH_DISCONNECTED, SH_CONNECTING, SH_CONN_IDLE, SH_CONN_APPENDING, SH_CONN_WAITACK };

SHState shState = SH_DISCONNECTED;
time_t shStart = 0;



void ConnSondehub::sondehub_reply_handler() {
  // sondehub handler for tasks to be done even if no data is to be sent:
  //   process response messages from sondehub
  //   request frequency list (if active)
#define MSG_SIZE 1000
  char rs_msg[MSG_SIZE];

  if (shImport == 1) { // we are waiting for a reply to a sondehub frequency import request
    // while we are waiting, we do nothing else with sondehub...
    int res = ShFreqImport::shImportHandleReply(&shclient);
    Serial.printf("ret: %d\n", res);
    // res==0 means more data is expected, res==1 means complete reply received (or error)
    if (res == 1) {
      shImport = 2; // finished
      shImportInterval = sonde.config.sondehub.fiinterval * 60;
    }
  }
  else {
    // any reply here belongs to normal telemetry upload, lets just print it.
    // and wait for a valid HTTP response
    int cnt = 0;
    while (shclient.available() > 0) {
      // data is available from remote server, process it...
      // readBytesUntil may wait for up to 1 second if enough data is not available...
      // int cnt = shclient.readBytesUntil('\n', rs_msg, MSG_SIZE - 1);
      int c = shclient.read();
      if (c < 0) break; // should never happen in available() returned >0 right before....
      rs_msg[cnt++] = c;
      if (c == '\n') {
        rs_msg[cnt] = 0;
        Serial.println(rs_msg);
        // If something that looks like a valid HTTP response is received, we are ready to send the next data item
        if (shState == SH_CONN_WAITACK && cnt > 11 && strncmp(rs_msg, "HTTP/1", 6) == 0) {
          shState = SH_CONN_IDLE;
        }
        cnt = 0;
      }
      if (cnt >= MSG_SIZE - 1) {
        cnt = 0;
        Serial.println("(overlong line from network, ignoring)");
      }
    }
    if (cnt > 0) {
      rs_msg[cnt + 1] = 0;
      Serial.println(rs_msg);
    }
  }
  // send import requests if needed
  if (sonde.config.sondehub.fiactive)  {
    if (shImport == 2) {
      Serial.printf("next sondehub frequncy import in %d seconds\n", shImportInterval);
      shImportInterval --;
      if (shImportInterval <= 0) {
        shImport = 0;
      }
    }
    else if (shImport == 0) {
      if (shState == SH_CONN_APPENDING || shState == SH_CONN_WAITACK)
        Serial.printf("Time to request next sondehub import.... but still busy with upload request");
      else
        sondehub_send_fimport();
    }
  }

  // also handle periodic station updates here...
  // interval check moved to sondehub_station_update to avoid having to calculate distance in auto mode twice
  if (sonde.config.sondehub.active) {
    if (shState == SH_CONN_IDLE || shState == SH_DISCONNECTED ) {
      // (do not set station update while a telemetry report is being sent
      sondehub_station_update();
    }
  }
}

void ConnSondehub::sondehub_send_fimport() {
  if (shState == SH_CONN_APPENDING || shState == SH_CONN_WAITACK) {
    // Currently busy with SondeHub data upload
    // So do nothing here.
    // sond_fimport will be re-sent later, when shState becomes SH_CONN_IDLE
    return;
  }
  // It's time to run, so check prerequisites
  float lat = sonde.config.rxlat, lon = sonde.config.rxlon;
  if (gpsPos.valid) {
    lat = gpsPos.lat;
    lon = gpsPos.lon;
  }

  int maxdist = sonde.config.sondehub.fimaxdist;      // km
  int maxage = sonde.config.sondehub.fimaxage * 60;   // fimaxage is hours, shImportSendRequest uses minutes
  int fiinterval = sonde.config.sondehub.fiinterval;
  Serial.printf("shimp : %f %f %d %d %d\n", lat, lon, maxdist, maxage, shImportInterval);
  if ( !isnan(lat) && !isnan(lon) && maxdist > 0 && maxage > 0 && fiinterval > 0 ) {
    int res = ShFreqImport::shImportSendRequest(&shclient, lat, lon, maxdist, maxage);
    if (res == 0) shImport = 1; // Request OK: wait for response
    else shImport = 2;        // Request failed: wait interval, then retry
  }
}


// in hours.... max allowed diff UTC <-> sonde time
#define SONDEHUB_TIME_THRESHOLD (3)
void ConnSondehub::sondehub_send_data(SondeInfo * s) {
  struct st_sondehub *conf = &sonde.config.sondehub;

  Serial.println("sondehub_send_data()");
  Serial.printf("shState = %d\n", shState);

  // max age of data in JSON request (in seconds)
#define SONDEHUB_MAXAGE 15

  char rs_msg[MSG_SIZE];
  char *w;
  struct tm ts;
  // config setting M10 and M20 will both decode both types, so use the real type that was decoded
  uint8_t realtype = sonde.realType(s);

  // For DFM, s->d.time is data from subframe DAT8 (gps date/hh/mm), and sec is from DAT1 (gps sec/usec)
  // For all others, sec should always be 0 and time the exact time in seconds
  time_t t = s->d.time;

  int chase = conf->chase;
  // automatically decided if CHASE or FIXED mode is used (for config AUTO)
  if (chase == SH_LOC_AUTO) {
    if (posInfo.chase) chase = SH_LOC_CHASE; else chase = SH_LOC_FIXED;
  }


  struct tm timeinfo;
  time_t now;
  time(&now);
  gmtime_r(&now, &timeinfo);
  if (timeinfo.tm_year <= (2016 - 1900)) {
    Serial.println("Failed to obtain time");
    return;
  }

  // Check if current sonde data is valid. If not, don't do anything....
  if (*s->d.ser == 0 || s->d.validID == 0 ) return;     // Don't send anything without serial number
  if (((int)s->d.lat == 0) && ((int)s->d.lon == 0)) return;     // Sometimes these values are zeroes. Don't send those to the sondehub
  if ((int)s->d.alt > 50000) return;    // If alt is too high don't send to SondeHub
  // M20 data does not include #sat information
  if ( realtype != STYPE_M20 && (int)s->d.sats < 4) return;     // If not enough sats don't send to SondeHub

  // If not connected to sondehub, try reconnecting.
  // TODO: do this outside of main loop
  if (!shclient.connected()) {
    Serial.println("NO CONNECTION");
    shState = SH_DISCONNECTED;
    if (!shclient.connect(conf->host, 80)) {
      Serial.println("Connection FAILED");
      return;
    }
    shclient.Client::setTimeout(0);  // does this work?
    shState = SH_CONN_IDLE;
  }

  if ( shState == SH_CONN_WAITACK ) {
    Serial.println("Previous SH-frame not yet ack'ed, not sending new data");
    return;
  }
 if ( abs(now - (time_t)s->d.time) > (3600 * SONDEHUB_TIME_THRESHOLD) ) {
    Serial.printf("Sonde time %d too far from current UTC time %ld", s->d.time, now);
    return;
  }

  //  DFM uses UTC. Most of the other radiosondes use GPS time
  // SondeHub expect datetime to be the same time sytem as the sonde transmits as time stamp
  if ( realtype == STYPE_RS41 || realtype == STYPE_RS92 || realtype == STYPE_M20 ) {
    t += 18;    // convert back to GPS time from UTC time +18s
  }

  gmtime_r(&t, &ts);

  memset(rs_msg, 0, MSG_SIZE);
  w = rs_msg;

  sprintf(w,
          " {"
          "\"software_name\": \"%s\","
          "\"software_version\": \"%s\","
          "\"uploader_callsign\": \"%s\","
          "\"time_received\": \"%04d-%02d-%02dT%02d:%02d:%02d.000Z\","
          "\"manufacturer\": \"%s\","
          "\"serial\": \"%s\","
          "\"datetime\": \"%04d-%02d-%02dT%02d:%02d:%02d.000Z\","
          "\"lat\": %.5f,"
          "\"lon\": %.5f,"
          "\"alt\": %.5f,"
          "\"frequency\": %.3f,"
          "\"vel_h\": %.5f,"
          "\"vel_v\": %.5f,"
          "\"heading\": %.5f,"
          "\"rssi\": %.1f,"
          "\"frame\": %d,"
          "\"type\": \"%s\",",
          version_name, version_id, conf->callsign,
          timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
          manufacturer_string[realtype], s->d.ser,
          ts.tm_year + 1900, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec,
          (float)s->d.lat, (float)s->d.lon, (float)s->d.alt, (float)s->freq, (float)s->d.hs, (float)s->d.vs,
          (float)s->d.dir, -((float)s->rssi / 2), s->d.vframe, sondeTypeStrSH[realtype]
         );
  w += strlen(w);

  // Only send sats if not M20
  if (realtype != STYPE_M20) {
    sprintf(w, "\"sats\": %d,", (int)s->d.sats);
    w += strlen(w);
  }

  /* if there is a subtype (DFM only) */
  if ( TYPE_IS_DFM(s->type) && s->d.subtype > 0 ) {
    if ( (s->d.subtype & 0xF) != DFM_UNK) {
      const char *t = dfmSubtypeLong[s->d.subtype & 0xF];
      sprintf(w, "\"subtype\": \"%s\",", t);
    }
    else {
      sprintf(w, "\"subtype\": \"DFMx%X\",", s->d.subtype >> 4); // Unknown subtype
    }
    w += strlen(w);
  } else if ( s->type == STYPE_RS41 ) {
    char buf[11];
    if (RS41::getSubtype(buf, 11, s) == 0) {
      sprintf(w, "\"subtype\": \"%s\",", buf);
      w += strlen(w);
    }
  }

  // Only send temp if provided
  if (!isnan(s->d.temperature)) {
    sprintf(w, "\"temp\": %.1f,", s->d.temperature);
    w += strlen(w);
  }

  // Only send humidity if provided
  if (!isnan(s->d.relativeHumidity)) {
    sprintf(w, "\"humidity\": %.1f,", s->d.relativeHumidity);
    w += strlen(w);
  }

  // Only send pressure if provided
  if (!isnan(s->d.pressure)) {
    sprintf(w, "\"pressure\": %.2f,", s->d.pressure);
    w += strlen(w);
  }

  // Only send burst timer if RS41 and fresh within the last 51s
  if ((realtype == STYPE_RS41) && (s->d.crefKT > 0) && (s->d.vframe - s->d.crefKT < 51)) {
    sprintf(w, "\"burst_timer\": %d,", (int)s->d.countKT);
    w += strlen(w);
  }

  // Only send battery if provided
  if (s->d.batteryVoltage > 0) {
    sprintf(w, "\"batt\": %.2f,", s->d.batteryVoltage);
    w += strlen(w);
  }

  // Only send antenna if provided
  if (strlen(conf->antenna) != 0) {
    sprintf(w, "\"uploader_antenna\": \"%s\",", conf->antenna);
    w += strlen(w);
  }

  // We send GPS position: (a) in CHASE mode, (b) in AUTO mode if no fixed location has been specified in config
  if (chase == SH_LOC_CHASE) {
    if (gpsPos.valid) {
      sprintf(w, "\"uploader_position\": [%.6f,%.6f,%d]", gpsPos.lat, gpsPos.lon, gpsPos.alt);
    } else {
      sprintf(w, "\"uploader_position\": [null,null,null]");
    }
    w += strlen(w);
  }
  // Otherweise, in FIXED mode we send the fixed position from config (if specified)
  else if (chase == SH_LOC_FIXED) {
    if ((!isnan(sonde.config.rxlat)) && (!isnan(sonde.config.rxlon))) {
      if (isnan(sonde.config.rxalt))
        sprintf(w, "\"uploader_position\": [%.6f,%.6f,null]", sonde.config.rxlat, sonde.config.rxlon);
      else
        sprintf(w, "\"uploader_position\": [%.6f,%.6f,%d]", sonde.config.rxlat, sonde.config.rxlon, (int)sonde.config.rxalt);
    } else {
      sprintf(w, "\"uploader_position\": [null,null,null]");
    }
    w += strlen(w);
  } else {
    sprintf(w, "\"uploader_position\": [null,null,null]");
    w += strlen(w);
  }

  // otherwise (in SH_LOC_NONE mode) we dont include any position info
  sprintf(w, "}");

  if (shState != SH_CONN_APPENDING) {
    sondehub_send_header(s, &timeinfo);
    sondehub_send_next(s, rs_msg, strlen(rs_msg), 1);
    shState = SH_CONN_APPENDING;
    shStart = now;
  } else {
    sondehub_send_next(s, rs_msg, strlen(rs_msg), 0);
  }
  if (now - shStart > SONDEHUB_MAXAGE) { // after MAXAGE seconds
    sondehub_send_last();
    shState = SH_CONN_WAITACK;
    shStart = 0;
  }
  //client->println(rs_msg);
  //Serial.println(rs_msg);
  //String response = client->readString();
  //Serial.println(response);
}

void ConnSondehub::sondehub_finish_data() {
  // If there is an "old" pending collection of JSON data sets, send it even if no now data is received
  if (shState == SH_CONN_APPENDING) {
    time_t now;
    time(&now);
    if (now - shStart > SONDEHUB_MAXAGE + 3) { // after MAXAGE seconds
      sondehub_send_last();
      shState = SH_CONN_WAITACK;
      shStart = 0;
    }
  }
}

static const char *DAYS[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
static const char *MONTHS[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Noc", "Dec"};

void ConnSondehub::sondehub_send_header(SondeInfo * s, struct tm * now) {
  struct st_sondehub *conf = &sonde.config.sondehub;
  Serial.print("PUT /sondes/telemetry HTTP/1.1\r\n"
               "Host: ");
  Serial.println(conf->host);
  Serial.print("accept: text/plain\r\n"
               "Content-Type: application/json\r\n"
               "Transfer-Encoding: chunked\r\n");

  shclient.print("PUT /sondes/telemetry HTTP/1.1\r\n"
                "Host: ");
  shclient.println(conf->host);
  shclient.print("accept: text/plain\r\n"
                "Content-Type: application/json\r\n"
                "Transfer-Encoding: chunked\r\n");
  if (now) {
    Serial.printf("Date: %s, %02d %s %04d %02d:%02d:%02d GMT\r\n",
                  DAYS[now->tm_wday], now->tm_mday, MONTHS[now->tm_mon], now->tm_year + 1900,
                  now->tm_hour, now->tm_min, now->tm_sec);
    shclient.printf("Date: %s, %02d %s %04d %02d:%02d:%02d GMT\r\n",
                   DAYS[now->tm_wday], now->tm_mday, MONTHS[now->tm_mon], now->tm_year + 1900,
                   now->tm_hour, now->tm_min, now->tm_sec);
  }
  shclient.print("User-agent: ");
  shclient.print(version_name);
  shclient.print("/");
  shclient.println(version_id);
  shclient.println(""); // another cr lf as indication of end of header
}
void ConnSondehub::sondehub_send_next(SondeInfo * s, char *chunk, int chunklen, int first) {
  // send next chunk of JSON request
  shclient.printf("%x\r\n", chunklen + 1);
  shclient.write(first ? "[" : ",", 1);
  shclient.write(chunk, chunklen);
  shclient.print("\r\n");

  Serial.printf("%x\r\n", chunklen + 1);
  Serial.write((const uint8_t *)(first ? "[" : ","), 1);
  Serial.write((const uint8_t *)chunk, chunklen);
  Serial.print("\r\n");
}
void ConnSondehub::sondehub_send_last() {
  // last chunk. just the closing "]" of the json request
  shclient.printf("1\r\n]\r\n0\r\n\r\n");
  Serial.printf("1\r\n]\r\n0\r\n\r\n");
}






ConnSondehub connSondehub;

#endif





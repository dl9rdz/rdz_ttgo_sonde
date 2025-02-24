#include "../features.h"
#include <sstream>
#include <iomanip>

#define TAG "conn-sondehub"
#include "logger.h"

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

//#include <ESPAsyncWebServer.h>

extern const char *version_name;
extern const char *version_id;

#define SONDEHUB_STATION_UPDATE_TIME (60*60*1000) // 60 min
#define SONDEHUB_MOBILE_STATION_UPDATE_TIME (30*1000) // 30 sec

#define ERROR_RETRY_DELAY 10  // seconds

int shclient;    // Sondehub v2
ip_addr_t shclient_ipaddr;

unsigned long time_next_import = 0;
unsigned long time_last_update = 0;

enum SHState { SH_DISCONNECTED, SH_DNSLOOKUP, SH_DNSRESOLVED, SH_CONNECTING, SH_CONN_IDLE, SH_CONN_APPENDING, SH_CONN_WAITACK, SH_CONN_WAITIMPORTRES, SH_ERROR_RETRY };

SHState shclient_state = SH_DISCONNECTED;
time_t shStart = 0;

#define MSG_SIZE 1000
static char rs_msg[MSG_SIZE];
int rs_msg_len = 0;

static String response;

static const char *state2str(SHState state) {
  switch(state) {
  case SH_DISCONNECTED: return "Disconnected";
  case SH_DNSLOOKUP: return "DNS lookup";
  case SH_DNSRESOLVED: return "DNS resolved";
  case SH_CONNECTING: return "Connecting";
  case SH_CONN_IDLE: return "Connected: Idle";
  case SH_CONN_APPENDING: return "Connected: Sending data";
  case SH_CONN_WAITACK: return "Connected: Waiting for ACK from server";
  case SH_CONN_WAITIMPORTRES: return "Connected: Waiting for import reply";
  case SH_ERROR_RETRY: return "Failure, retrying in a few seconds";
  default: return "??";
  }
}


void ConnSondehub::init() {
}

void ConnSondehub::netsetup() {
    if ((sonde.config.sondehub.active||sonde.config.sondehub.fiactive) && wifi_state != WIFI_APMODE) {
        // start connecting...
        sondehub_client_fsm();
        time_last_update = 0; /* force sending update */ 

	// SH import: initial refresh on connect, even if configured interval is longer
        time_next_import = millis() + 5000;
    } 
}

void ConnSondehub::netshutdown() {
    close(shclient);
    shclient_state = SH_DISCONNECTED;
}

// Imitating the old non-modular code
// updateSonde is called once per second
// old code called
//   each second: sondehub_reply_handler
//   each second, if good decode: sondehub_send_data
//   each second, if no good decode: sondehub_finish_data
void ConnSondehub::updateSonde( SondeInfo *si ) {
    sondehub_client_fsm();
    if (!sonde.config.sondehub.active)
        return;
    LOG_D(TAG, "updateSonde: si:%s, SH_FSM in state %d (%s), next import in %d s\n", si?"yes":"null", shclient_state, state2str(shclient_state), (time_next_import-millis())/1000);

    if(si==NULL) {
        sondehub_finish_data();
    } else {
	sondehub_send_data(si);
    }
}


static void _sh_dns_found(const char * name, const ip_addr_t *ipaddr, void * /*arg*/) {
    LOG_I(TAG, "dns callback for %s\n", name);
    if (ipaddr) {
        shclient_ipaddr = *ipaddr;
        LOG_I(TAG, "dns addr: %x (%d)\n", ipaddr->u_addr.ip4, ipaddr->type);
        shclient_state = SH_DNSRESOLVED;    // DNS lookup success
    } else {
        memset(&shclient_ipaddr, 0, sizeof(shclient_ipaddr));
        shclient_state = SH_ERROR_RETRY;   // DNS lookup failed
        shStart = 0;
        // TODO: set "reply messge" to "DNS lookup failed"
        LOG_I(TAG, "dns_failed for %s\n", name);
	snprintf(rs_msg, MSG_SIZE, "DNS lookup failed for %s", name);
    }
}

// Sondehub client asynchronous FSM...
void ConnSondehub::sondehub_client_fsm() {
    fd_set fdset, fdeset;
    FD_ZERO(&fdset);
    FD_SET(shclient, &fdset);
    FD_ZERO(&fdeset);
    FD_SET(shclient, &fdeset);
    struct timeval selto = {0};
 
    LOG_I(TAG, "SH_FSM in state %d (%s), next import in %d s\n", shclient_state, state2str(shclient_state), (time_next_import-millis())/1000);

    switch(shclient_state) {
    case SH_ERROR_RETRY:
    {
        time_t now;
        time(&now);
        // Something went wrong. Wait for ERROR_RETRY_DELAY, then restart in state DISCONNECTED
        if(shStart == 0) {
            shStart = now;
        } else if (now - shStart > ERROR_RETRY_DELAY ) {
            shclient_state = SH_DISCONNECTED;
        }
        break;
    }
    case SH_DISCONNECTED:
    {
        // We are disconnected. Try to connect, starting with a DNS lookup
        shclient_state = SH_DNSLOOKUP;  // Set state already here to avoid potential race with callback
        err_t res = dns_gethostbyname_addrtype( sonde.config.sondehub.host, &shclient_ipaddr, _sh_dns_found, NULL, LWIP_DNS_ADDRTYPE_IPV4 );
        if(res == ERR_OK) { // returns immediately if host is IP or in cache
            shclient_state = SH_DNSRESOLVED;
            // fall through to next switch case
        } else if(res == ERR_INPROGRESS) { 
            // state is DNSLOOKUP, so wait for response....
            break;
        } else {
            shclient_state = SH_ERROR_RETRY;
            shStart = 0;
            break;
        }
    }
    case SH_DNSLOOKUP:
    {
        // DNS lookup still in progress. callback should switch to DNSRESOLVED or ERROR_RETRY, so just wait
        // TODO: Maybe in case of stuck here, abort and retry?
        LOG_I(TAG, "SH_FSM: Waiting for DNS response\n");
        break;
    }
    case SH_DNSRESOLVED:
    {
        // We have got the IP address, start the connection (asynchronously)
        shclient = socket(AF_INET, SOCK_STREAM, 0);
        int flags  = fcntl(shclient, F_GETFL);
        if (fcntl(shclient, F_SETFL, flags | O_NONBLOCK) == -1) {
            LOG_E(TAG, "Setting O_NONBLOCK failed\n");
        }

        struct sockaddr_in sock_info;
        memset(&sock_info, 0, sizeof(struct sockaddr_in));
        sock_info.sin_family = AF_INET;
        sock_info.sin_addr.s_addr = shclient_ipaddr.u_addr.ip4.addr;
        sock_info.sin_port = htons( 80 );
        err_t res = connect(shclient, (struct sockaddr *)&sock_info, sizeof(sock_info));
        if(res) {
            if (errno == EINPROGRESS) { // Should be the usual case, go to connecting state
                shclient_state = SH_CONNECTING;
            } else {
                close(shclient);
                shclient_state = SH_ERROR_RETRY;
                shStart = 0;
            }
        } else {
            shclient_state = SH_CONN_IDLE;
            // ok, ready to send data...
        }
    }
    break;

    case SH_CONNECTING:
    {
        // Poll to see if we are now connected
// Poll to see if we are now connected 
        int res = select(shclient+1, NULL, &fdset, &fdeset, &selto);
        if(res<0) {
            LOG_E(TAG, "SH_CONNECTING: select error\n");
            goto error;
        } else if (res==0) { // still pending
            break;
        }
        // Socket has become ready (or something went wrong, check for error first)
        
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        if (getsockopt(shclient, SOL_SOCKET, SO_ERROR, (void*)(&sockerr), &len) < 0) {
            goto error;
        }
        LOG_D(TAG, "select returing %d. isset:%d iseset:%d sockerr:%d\n", res, FD_ISSET(shclient, &fdset), FD_ISSET(shclient, &fdeset), sockerr);
        if(sockerr) {
            LOG_E(TAG, "SH connect error: %s\n", strerror(sockerr));
            goto error;
        }
        shclient_state = SH_CONN_IDLE;
        // ok, ready to send data...
    }
    break;

    case SH_CONN_IDLE:
	// if idle, check if we might want to send freq import request
        if (sonde.config.sondehub.fiactive)  {
            unsigned long now = millis();
            if(now > time_next_import) {
                sondehub_send_fimport();
            }
        }

	// Intentional fall-through: in idle state, read any data out of connection
	// in CONN_WAITACK we switch to IDLE as soon as we see the HTTP header

    case SH_CONN_WAITACK:
    case SH_CONN_WAITIMPORTRES:
    {
        // In CONN_WAITACK:
        // If data starts with HTTP/1 this is the expected response, move to state CONN_IDLE
        //   noise tolerant - should not be needed:
        //   if the data contains HTTP/1 copy that to the start of the buffer, ignore anything up to that point
        //   if not find the last \0 and append next response after the part afterwards
      for(int k=0; k<10; k++) { // read more data...
        int res = select(shclient+1, &fdset, NULL, NULL, &selto);
        if(res<0) {
            LOG_E(TAG, "SH_CONN_IDLE: select error\n");
            goto error;
        } else if (res==0) { //  no data
            break;
        }
        // Read data
        char buf[512+1]; 
        res = read(shclient, buf, 512);
        if(res<=0) {
            close(shclient);
            shclient_state = SH_ERROR_RETRY;
            shStart = 0;
        } else {
            // Copy to reponse
            for(int i=0; i<res; i++) {
               if(rs_msg_len<MSG_SIZE-1) {
                   rs_msg[rs_msg_len] = buf[i];
                   rs_msg_len++;
               }
	       if(shclient_state == SH_CONN_WAITACK) { 
      		  if(buf[i]=='\n') { 
        	    // We still wait for the beginning of the ACK
        	    // so check if we got that. if yes, all good, continue reading :)
        	    // If not, ignore everything we have read so far...
        	    if(strncmp(rs_msg, "HTTP/1", 6)==0) { shclient_state = SH_CONN_IDLE; }
        	    else rs_msg_len = 0;
                 }
               }
            }
            if( shclient_state == SH_CONN_WAITIMPORTRES ) {   // we are waiting for a reply to a sondehub frequency import request
                int import_res = ShFreqImport::shImportHandleReply(buf, res);
    		LOG_D(TAG, "shImportHandleReply: ret=%d\n", import_res);
    		// import_ress==0 means more data is expected, import_res==1 means complete reply received (or error)
    		if (import_res == 1) {
      			shclient_state = SH_CONN_IDLE;
			time_next_import = millis() + sonde.config.sondehub.fiinterval * 60000;
    		}
            }
	}
        rs_msg[rs_msg_len] = 0;
	buf[res] = 0;

        LOG_D(TAG, "client_fsm: got data (len=%d): %s\n", res, (char *)buf);
	// TODO: Maybe timestamp last received data?
        // TODO: Maybe repeat
        // TODO: Add timeout to WAITACK...
      }//for k=0..10
    }
    break;

    default:
        LOG_I(TAG, "SH_FSM in state %d (%s)\n", shclient_state, state2str(shclient_state));
        LOG_E(TAG, "UNHANLDED CASE: SHOULD NOT HAPPEN*****");
    }
    LOG_I(TAG, "FSM RETRUNING*****");
    return;

error:
    close(shclient);
    shclient = SH_ERROR_RETRY;
    shStart = 0;
}


/*
        Update station data to the sondehub v2 DB
*/
/* which_pos: 0=none, 1=fixed, 2=gps */
void ConnSondehub::updateStation( PosInfo *pi ) {
    sondehub_client_fsm();
    if (!sonde.config.sondehub.active)
        return;
    LOG_D(TAG, "updateStation\n");
    // Currently, internal handler uses gpsInfo global variable instead of this pi

  struct st_sondehub *conf = &sonde.config.sondehub;
#define STATION_DATA_LEN 300
  char data[STATION_DATA_LEN];
  char *w;

  sondehub_client_fsm();  // let's handle the connection state..
  if(shclient_state != SH_CONN_IDLE) return;  // Only if connected and idle can we send data...

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

  LOG_D(TAG, "tlu:%d  delta:%d upd=%d\n", time_last_update, time_delta, update_time);
  // If it is not yet time to send another update. do nothing....
  if(time_last_update != 0) {  // if 0, force update
      if ( (time_delta <= update_time) ) return;
  }

  LOG_D(TAG, "sondehub_station_update: send update\n");
  time_last_update = time_now;

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

  dprintf( shclient, "PUT /listeners HTTP/1.1\r\n"
      "Host: %s\r\n"
      "accept: text/plain\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %d\r\n\r\n%s",
      conf->host, strlen(data), data);

  LOG_I(TAG, "PUT /listeners HTTP/1.1\n"
      "Host: %s\n"
      "accept: text/plain\n"
      "Content-Type: application/json\n"
      "Content-Length: %d\n\n%s",
      conf->host, strlen(data), data);

  LOG_D(TAG, "Waiting for response");
  // Now we do this asychronously
  shclient_state = SH_CONN_WAITACK;
  rs_msg_len = 0;   // wait for new msg: 

  sondehub_client_fsm();
}

/*
        Update sonde data to the sondehub v2 DB
*/

void ConnSondehub::sondehub_send_fimport() {
  if (shclient_state != SH_CONN_IDLE) {
    // Only send import if connection is idle (connected but no reply pending or partial request sent)
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
  int time_to_next_import = time_next_import - millis();
  LOG_D(TAG, "shimp : %f %f %d %d %d\n", lat, lon, maxdist, maxage, time_to_next_import/1000);
  if ( !isnan(lat) && !isnan(lon) && maxdist > 0 && maxage > 0 && time_to_next_import < 0 ) {
    int res = ShFreqImport::shImportSendRequest(shclient, lat, lon, maxdist, maxage);
    if (res == 0) { 
        shclient_state = SH_CONN_WAITIMPORTRES;
    }
  }
}


// in hours.... max allowed diff UTC <-> sonde time
#define SONDEHUB_TIME_THRESHOLD (3)
void ConnSondehub::sondehub_send_data(SondeInfo * s) {
  struct st_sondehub *conf = &sonde.config.sondehub;

  LOG_D(TAG, "sondehub_send_data: shclient_state = %d\n", shclient_state);

  sondehub_client_fsm();
  // Only send data when in idle or appending state....
  if(shclient_state != SH_CONN_IDLE && shclient_state != SH_CONN_APPENDING) { 
    LOG_W(TAG, "Not in right state for sending next request...\n");
    return;
  }

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
    LOG_E(TAG, "Failed to obtain time\n");
    return;
  }

  // Check if current sonde data is valid. If not, don't do anything....
  if (*s->d.ser == 0 || s->d.validID == 0 ) return;     // Don't send anything without serial number
  if (((int)s->d.lat == 0) && ((int)s->d.lon == 0)) return;     // Sometimes these values are zeroes. Don't send those to the sondehub
  if ((int)s->d.alt > 50000) return;    // If alt is too high don't send to SondeHub
  // M20 data does not include #sat information
  if ( realtype != STYPE_M20 && (int)s->d.sats < 4) return;     // If not enough sats don't send to SondeHub

 if ( abs(now - (time_t)s->d.time) > (3600 * SONDEHUB_TIME_THRESHOLD) ) {
    LOG_E(TAG, "Sonde time %d too far from current UTC time %ld", s->d.time, now);
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

  if (shclient_state != SH_CONN_APPENDING) {
    sondehub_send_header(s, &timeinfo);
    sondehub_send_next(s, rs_msg, strlen(rs_msg), 1);
    shclient_state = SH_CONN_APPENDING;
    shStart = now;
  } else {
    sondehub_send_next(s, rs_msg, strlen(rs_msg), 0);
  }
  if (now - shStart > SONDEHUB_MAXAGE) { // after MAXAGE seconds
    sondehub_send_last();
    shclient_state = SH_CONN_WAITACK;
    rs_msg_len = 0;   // wait for new msg: 
    shStart = 0;
  }
}

void ConnSondehub::sondehub_finish_data() {
  // If there is an "old" pending collection of JSON data sets, send it even if no now data is received
  if (shclient_state == SH_CONN_APPENDING) {
    time_t now;
    time(&now);
    if (now - shStart > SONDEHUB_MAXAGE + 3) { // after MAXAGE seconds
      sondehub_send_last();
      shclient_state = SH_CONN_WAITACK;
    rs_msg_len = 0;   // wait for new msg: 
      shStart = 0;
    }
  }
}

static const char *DAYS[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
static const char *MONTHS[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Noc", "Dec"};

void ConnSondehub::sondehub_send_header(SondeInfo * s, struct tm * now) {
  struct st_sondehub *conf = &sonde.config.sondehub;
  dprintf(shclient, "PUT /sondes/telemetry HTTP/1.1\r\n"
                "Host: %s\n"
                "accept: text/plain\r\n"
                "Content-Type: application/json\r\n"
                "Transfer-Encoding: chunked\r\n", conf->host);
  LOG_D(TAG, "PUT /sondes/telemetry HTTP/1.1\r\n"
                "Host: %s\n"
                "accept: text/plain\r\n"
                "Content-Type: application/json\r\n"
                "Transfer-Encoding: chunked\r\n", conf->host);
  if (now) {
    LOG_D(TAG,        "Date: %s, %02d %s %04d %02d:%02d:%02d GMT\r\n",
                  DAYS[now->tm_wday], now->tm_mday, MONTHS[now->tm_mon], now->tm_year + 1900,
                  now->tm_hour, now->tm_min, now->tm_sec);
    dprintf(shclient, "Date: %s, %02d %s %04d %02d:%02d:%02d GMT\r\n",
                   DAYS[now->tm_wday], now->tm_mday, MONTHS[now->tm_mon], now->tm_year + 1900,
                   now->tm_hour, now->tm_min, now->tm_sec);
  }
  dprintf(shclient, "User-agent: %s/%s\n\n", version_name, version_id);
  // another cr lf as indication of end of header
}
void ConnSondehub::sondehub_send_next(SondeInfo * s, char *chunk, int chunklen, int first) {
  // send next chunk of JSON request
  dprintf(shclient, "%x\r\n%c", chunklen + 1, first ? '[' : ',');
  write(shclient, chunk, chunklen);
  write(shclient, "\r\n", 2);

  LOG_D(TAG, "%x\r\n%c", chunklen + 1, first ? "[" : ",");
  Serial.write((const uint8_t *)chunk, chunklen);
  LOG_D(TAG, "\r\n");
}
void ConnSondehub::sondehub_send_last() {
  // last chunk. just the closing "]" of the json request
  dprintf(shclient, "1\r\n]\r\n0\r\n\r\n");
  LOG_D(TAG, "1\r\n]\r\n0\r\n\r\n");
}


String ConnSondehub::getStatus() {
  char info[1200];
  time_t now;
  if(sonde.config.sondehub.active==0) {
    strcpy(info, "disabled");
  } else {
  time(&now);
  if(shStart==0) now=-1;
  snprintf(info, 1200, "State: %s. Last upload start: %ld s ago<br>Last reply: ",
      state2str(shclient_state), (uint32_t)(now-shStart));
  int n = strlen(info);
  escapeJson(info+n, rs_msg, 1200-n);
  }
  return String(info);
}

String ConnSondehub::getName() {
  return String("Sondehub");
}



ConnSondehub connSondehub;

#endif





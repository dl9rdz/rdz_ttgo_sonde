#include "posinfo.h"

#include <FS.h>
#include <LittleFS.h>
#include <MicroNMEA.h>

#define TAG "GPS"
#include "logger.h"

// Sation position obtained from GPS (if available)
struct StationPos gpsPos;

// Station position to use (from GPS or fixed)
struct StationPos posInfo;

static int gotNMEA = 0;

/* SH_LOC_OFF: never send position information to SondeHub
   SH_LOC_FIXED: send fixed position (if specified in config) to sondehub
   SH_LOC_CHASE: always activate chase mode and send GPS position (if available)
   SH_LOC_AUTO: if there is no valid GPS position, or GPS position < MIN_LOC_AUTO_DIST away from known fixed position: use FIXED mode
                otherwise, i.e. if there is a valid GPS position and (either no fixed position in config, or GPS position is far away from fixed position), use CHASE mode.
*/
// same constants used for SondeHub and APRS

/* auto mode is chase if valid GPS position and (no fixed location entered OR valid GPS position and distance in lat/lon deg to fixed location > threshold) */
//#define MIN_LOC_AUTO_DIST 200   /* meter */
//#define SH_LOC_AUTO_IS_CHASE ( gpsPos.valid && ( (isnan(sonde.config.rxlat) || isnan(sonde.config.rxlon) ) ||  \
//                               calcLatLonDist( gpsPos.lat, gpsPos.lon, sonde.config.rxlat, sonde.config.rxlon ) > MIN_LOC_AUTO_DIST ) )
//extern float calcLatLonDist(float lat1, float lon1, float lat2, float lon2);

/////
// set fixed position based on config
void fixedToPosInfo() {
    memset(&posInfo, 0, sizeof(posInfo));
    if( isnan(sonde.config.rxlat) || isnan(sonde.config.rxlon) )
        return;
    posInfo.lat = sonde.config.rxlat;
    posInfo.lon = sonde.config.rxlon;
    posInfo.alt = sonde.config.rxalt;
    posInfo.valid = 1;
}


///// GPS handling functions

static char buffer[85];
static MicroNMEA nmea(buffer, sizeof(buffer));

static int badNMEA = 0, totalNMEA = 0;

template<typename T>
void badChecksumHandler(T nmea) {
    badNMEA++;    
}

/// Arrg. MicroNMEA changes type definition... so lets auto-infer type
template<typename T>
//void unkHandler(const MicroNMEA& nmea) {
void unkHandler(T nmea) {
  //Serial.println(nmea.getSentence());
  if (strcmp(nmea.getMessageID(), "VTG") == 0) {
    const char *s = nmea.getSentence();
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    if (*s == ',') return; /// no new course data
    int lastCourse = nmea.parseFloat(s, 0, NULL);
    Serial.printf("Course update: %d\n", lastCourse);
  } else if (strcmp(nmea.getMessageID(), "GST") == 0) {
    // get horizontal accuracy for android app on devices without gps
    // GPGST,time,rms,-,-,-,stdlat,stdlon,stdalt,cs
    const char *s = nmea.getSentence();
    while (*s && *s != ',') s++;  // #0: GST
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;  // #1: time: skip
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;  // #1: rms: skip
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;  // #1: (-): skip
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;  // #1: (-): skip
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;  // #1: (-): skip
    if (*s == ',') s++; else return;
    // stdlat
    int stdlat = nmea.parseFloat(s, 1, NULL);
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    // stdlong
    int stdlon = nmea.parseFloat(s, 1, NULL);
    // calculate position error as 1-signma horizontal RMS
    // I guess that is equivalent to Androids getAccurac()?
    int poserr = 0;
    if (stdlat < 10000 && stdlon < 10000) { // larger errors: no GPS fix, avoid overflow in *
      poserr = (int)(sqrt(0.5 * (stdlat * stdlat + stdlon * stdlon)));
    }
    //Serial.printf("\nHorizontal accuracy: %d, %d => %.1fm\n", stdlat, stdlon, 0.1*poserr);
    gpsPos.accuracy = poserr;
  }
}

// 1 deg = aprox. 100 km  ==> approx. 200m
#define AUTO_CHASE_THRESHOLD 0.002

//#define DEBUG_GPS
static bool gpsCourseOld;
static int lastCourse;
static char lastnmea[101];
void gpsTask(void *parameter) {
  nmea.setUnknownSentenceHandler(unkHandler);

  while (1) {
    while (Serial2.available()) {
      if(gotNMEA == 0) gotNMEA = -1; // at least we got *something* 
      char c = Serial2.read();
#if DEBUG_GPS
      Serial.print(c);
#endif
      if (nmea.process(c)) {
        const char *nmeastring = nmea.getSentence();
        if(nmeastring[0]=='$') { // looks like a nmea string
            totalNMEA++;
            gotNMEA = 1;
        }
        if(strncmp(nmeastring+3, "GGA", 3)==0 || strncmp(nmeastring+3, "RMC", 3)==0) {
            strncpy(lastnmea, nmeastring, 100);
	    //Serial.printf("GPS: last position nmea: %s\n", lastnmea);
 	}
 	else  {
	    //Serial.printf("GPS: last nmea: %s\n", nmeastring);
	}
        // TODO: This runs on each valid sentance. We should process position updates
        // only once per second. 
        gpsPos.valid = nmea.isValid();
        if (gpsPos.valid) {
          gpsPos.lon = nmea.getLongitude() * 0.000001;
          gpsPos.lat = nmea.getLatitude() * 0.000001;
          gpsPos.speed = nmea.getSpeed() / 1000.0 * 0.514444; // speed is in m/s nmea.getSpeed is in 0.001 knots
          long alt = 0;
          nmea.getAltitude(alt);
          gpsPos.alt = (int)(alt / 1000);
          gpsPos.course = (int)(nmea.getCourse() / 1000);
          gpsCourseOld = false;
          if (gpsPos.course == 0) {
            // either north or not new
            if (lastCourse != 0) // use old value...
            {
              gpsCourseOld = true;
              gpsPos.course = lastCourse;
            }
          }
          if (gpsPos.lon == 0 && gpsPos.lat == 0) gpsPos.valid = false;
        }
        /* Check if home */
        if(gpsPos.valid) {
            if(isnan(sonde.config.rxlon)||isnan(sonde.config.rxlat)) {
                 // no fixed position => always use GPS position
                 posInfo = gpsPos;
                 posInfo.chase = 1;
            } else {
                float d = fabs(gpsPos.lon - sonde.config.rxlon);
                d += fabs(gpsPos.lat - sonde.config.rxlat);
	        Serial.printf("pos diff is %f (%f %f %f %f)\n", d, gpsPos.lon , sonde.config.rxlon , gpsPos.lat, sonde.config.rxlat);

		// Logic for auto chase mode:
		// If close to home (below half of chase threshold): switch to fixed position
		// If away from home (above chase threshold): switch to chase mode
		// Otherwise, stay in current mode (update with GPS pos if in chase mode only)
		//
		if ( posInfo.chase && d < AUTO_CHASE_THRESHOLD/2 ) {
		    // Stop GPS chase mode very close to home (fixeedToPosInfo sets chase to 0)
		    fixedToPosInfo();	
		    LOG_I(TAG, "Stopping chase mode (d=%f)\n", d);
		} else if(d > AUTO_CHASE_THRESHOLD || posInfo.chase) {
		    if(!posInfo.chase)
			LOG_I(TAG, "Activating chase mode (d=%f)\n", d);
		    // use / activate chase mode
		    posInfo = gpsPos;
		    posInfo.chase = 1;
		} else {
                    // Serial.printf("Not updating gpspos / chase (chase is: %d)\n", posInfo.chase);
                }
            }
        }

	//TODO: updating gpsPos is better done ebfore settings posInfo to gpsPos.....
        gpsPos.hdop = nmea.getHDOP();
        gpsPos.sat = nmea.getNumSatellites();
        gpsPos.speed = nmea.getSpeed() / 1000.0 * 0.514444; // speed is in m/s  nmea.getSpeed is in 0.001 knots
        snprintf(gpsPos.time, sizeof(gpsPos.time),
                 "%04d-%02d-%02d %02d:%02d:%02d", nmea.getYear(),
                 nmea.getMonth(), nmea.getDay(), nmea.getHour(),
                 nmea.getMinute(), nmea.getSecond());
#ifdef DEBUG_GPS
        uint8_t hdop = nmea.getHDOP();
        Serial.printf(" =>: valid: %d  N %f  E %f  alt %d  course:%d dop:%d\n", gpsPos.valid ? 1 : 0, gpsPos.lat, gpsPos.lon, gpsPos.alt, gpsPos.course, hdop);
#endif
      }
    }
    delay(50);
  }
}


#define UBX_SYNCH_1 0xB5
#define UBX_SYNCH_2 0x62
uint8_t ubx_set9k6[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x8F};
uint8_t ubx_factorydef[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x09, 13, 0, 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x13, 0x7c };
uint8_t ubx_hardreset[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x04, 4, 0, 0xff, 0xff, 0, 0, 0x0C, 0x5D };
// GPGST: Class 0xF0 Id 0x07
uint8_t ubx_enable_gpgst[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x01, 3, 0, 0xF0, 0x07, 2, 0x03, 0x1F};

void dumpGPS() {
  while (Serial2.available()) {
    char c = Serial2.read();
    char d = (c&127)<20 ? '?' : c;
    Serial.printf("%02x[%c] ", (uint8_t)c, d);
  }
}

static char h2i(char c) {
   if(c>='0'&&c<='9') return c-'0';
   else if (c>='a'&&c<='f') return c-'a'+10;
   else if (c>='A'&&c<='F') return c-'A'+10;
   else return 0;
}
static uint8_t cs1, cs2;
static void sendChar(char c) {
   cs1 += c;
   cs2 += cs1;
   char d = (c&127)<20 ? '?' : c;
   Serial.printf("%02x[%c] ", (uint8_t)c, d);
   Serial2.write(c);
}
static void sendString(const char *p) {
    bool ubx = false;
    if(p[0]=='u') { ubx=true; p++; }
    p++; // skip "
    if(ubx) { sendChar(UBX_SYNCH_1); sendChar(UBX_SYNCH_2); }
    cs1 = cs2 = 0;
    while(*p && *p!='"') {
        if(*p=='\\') {
            p++;
            if(*p=='x') { sendChar( (unsigned char)( 16*h2i(p[1]) + h2i(p[2]) ) ); p+=2; }
            else if(*p=='n') sendChar('\n');
            else if (*p=='r') sendChar('\r');
            else sendChar(*p);
        } 
        else sendChar(*p);
        p++;
    }
    if(ubx) { 
       uint8_t _cs1 = cs1, _cs2 = cs2;
       sendChar(_cs1); sendChar(_cs2);
    }
}
 

static int initGPSfromFile(bool reset) {
    File initgps = LittleFS.open("/gpsinit.txt", FILE_READ);
    if(!initgps) { LOG_W(TAG, "no gpsinit.txt found, skipping GPS init\n"); return -1; }

    uint16_t lines[100]; // max. 100 lines... (hard coded)
    uint8_t labels[10];  // max 10 labels (hard coded)
    char jumpto = 0;
    while(initgps.available()) {
        String l = initgps.readStringUntil('\n');
        const char *p = l.c_str();
        Serial.printf("\nGPSINIT string '%s'\n", p);
        if(jumpto > 0) {
            Serial.printf("is jumpto? %d, %d vs %d\n", p[0], p[1], jumpto);
            if(*p==':' && p[1] == jumpto) jumpto = 0;
            continue;
        }
        switch(p[0]) {
        case 'n':  // conditional jump if no reset
        {
            if(!reset) {
                jumpto = p[1];
		Serial.printf("Jumping to %c\n", jumpto);
            }
	    break;
        }
        case 'w':  // wait
	{
            int t = atoi(p+1);
	    Serial.printf("Waiting %d ms\n", t);
            delay(t);
            dumpGPS();
            break;
            // TODO: JUMP
	}
        case 'b':  // set baud
	{
            int baud = atoi(p+1);
	    Serial.printf("Setting baud to %d\n", baud);
            Serial2.begin(baud, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
            break;
	}
        case 's':  // send string
	{
            Serial.printf("Sending string to GPS: ");
            sendString(p+1);
            break;
        }}
    }
    return 0;
}

void initGPS() {
  if (sonde.config.gps_rxd < 0) return; // GPS disabled
  if (sonde.config.gps_txd >= 0) {  // TX enable, thus try setting baud to 9600 and do a factory reset
    File testfile = LittleFS.open("/GPSRESET", FILE_READ);
    bool reset = false;
    if (testfile && !testfile.isDirectory()) {
      reset = true;
      testfile.close();
      LittleFS.remove("/GPSRESET");
    } else {
      Serial.println("GPS reset file: not found/isdir");
      if(testfile) testfile.close();
    }
    if(initGPSfromFile(reset)<0) { // failed, just set to 9k6
      Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
    }
  } else {
    //TOOD: Also use last baud statement from gps init file...
    Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
  }
  xTaskCreate( gpsTask, "gpsTask",
               5000, /* stack size */
               NULL, /* paramter */
               1, /* priority */
               NULL);  /* task handle*/
}



// Getting GPS data from App (phone)

void parseGpsJson(char *data, int len) {
  char *key = NULL;
  char *value = NULL;
  // very simple json parser: look for ", then key, then ", then :, then number, then , or } or \0
  for (int i = 0; i < len; i++) {
    if (key == NULL) {
      if (data[i] != '"') continue;
      key = data + i + 1;
      i += 2;
      continue;
    }
    if (value == NULL) {
      if (data[i] != ':') continue;
      value = data + i + 1;
      i += 2;
      continue;
    }
    if (data[i] == ',' || data[i] == '}' || data[i] == 0) {
      // get value
      double val = strtod(value, NULL);
      // get data
      if (strncmp(key, "lat", 3) == 0) {
        gpsPos.lat = val;
      }
      else if (strncmp(key, "lon", 3) == 0) {
        gpsPos.lon = val;
      }
      else if (strncmp(key, "alt", 3) == 0) {
        gpsPos.alt = (int)val;
      }
      else if (strncmp(key, "course", 6) == 0) {
        gpsPos.course = (int)val;
      }
      gpsPos.valid = true;

      // next item:
      if (data[i] != ',') break;
      key = NULL;
      value = NULL;
    }
  }
  if (gpsPos.lat == 0 && gpsPos.lon == 0) gpsPos.valid = false;
  Serial.printf("Parse result: lat=%f, lon=%f, alt=%d, valid=%d\n", gpsPos.lat, gpsPos.lon, gpsPos.alt, gpsPos.valid);
}



// We implement the interface for showing the status only, the other functions remain empty...
void ConnGPS::init() { }
void ConnGPS::netsetup() { }
void ConnGPS::updateSonde( SondeInfo *si ) { }
void ConnGPS::updateStation( PosInfo *pi ) { }

String ConnGPS::getName() {
  return String("GPS");
}

String ConnGPS::getStatus() {
    char status[256];

    strlcpy(status, "On-board GPS: ", 256);
    if(sonde.config.gps_rxd==-1) strlcat(status, "disabled<br>", 256);
    else if(gotNMEA==0) strlcat(status, "no data<br>", 256);
    else if(gotNMEA<0) strlcat(status, "no NMEA data<br>", 256);
    else {
        int l = strlen(status);
        snprintf(status+l, 256-l, "ok (%d NMEA, %d bad)<br>", totalNMEA, badNMEA);
    }
    int pos = strlen(status);
    snprintf(status + pos, 256-pos, "GPS: valid=%d lat=%.6f lon=%.6f alt=%d<br>", gpsPos.valid, gpsPos.lat, gpsPos.lon, gpsPos.alt);
    pos = strlen(status);
    snprintf(status + pos, 256-pos, "Using station position: valid=%d lat=%.6f lon=%.6f<br>", posInfo.valid, posInfo.lat, posInfo.lon);
    pos = strlen(status);
    snprintf(status + pos, 256-pos, "Current NMEA: %s", lastnmea);
    return String(status);
}


ConnGPS connGPS;

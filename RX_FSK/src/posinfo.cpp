#include "posinfo.h"

#include <MicroNMEA.h>


// Sation position obtained from GPS (if available)
struct StationPos gpsPos;

// Station position to use (from GPS or fixed)
struct StationPos posInfo;


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


/// Arrg. MicroNMEA changes type definition... so lets auto-infer type
template<typename T>
//void unkHandler(const MicroNMEA& nmea) {
void unkHandler(T nmea) {
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
void gpsTask(void *parameter) {
  nmea.setUnknownSentenceHandler(unkHandler);

  while (1) {
    while (Serial2.available()) {
      char c = Serial2.read();
      //Serial.print(c);
      if (nmea.process(c)) {
        gpsPos.valid = nmea.isValid();
        if (gpsPos.valid) {
          gpsPos.lon = nmea.getLongitude() * 0.000001;
          gpsPos.lat = nmea.getLatitude() * 0.000001;
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
	    float d = fabs(gpsPos.lon - sonde.config.rxlon);
	    d += fabs(gpsPos.lat - sonde.config.rxlat);
	    if(!posInfo.chase && d > AUTO_CHASE_THRESHOLD) {
		posInfo = gpsPos;
                posInfo.chase = 1;
            } else if ( posInfo.chase && d < AUTO_CHASE_THRESHOLD/2 ) {
		fixedToPosInfo();
            }
	}

        gpsPos.hdop = nmea.getHDOP();
        gpsPos.sat = nmea.getNumSatellites();
        gpsPos.speed = nmea.getSpeed() / 1000.0 * 0.514444; // speed is in m/s  nmea.getSpeed is in 0.001 knots
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
    Serial.printf("%02x ", (uint8_t)c);
  }
}


void initGPS() {
  if (sonde.config.gps_rxd < 0) return; // GPS disabled
  if (sonde.config.gps_txd >= 0) {  // TX enable, thus try setting baud to 9600 and do a factory reset
    File testfile = SPIFFS.open("/GPSRESET", FILE_READ);
    if (testfile && !testfile.isDirectory()) {
      testfile.close();
      Serial.println("GPS resetting baud to 9k6...");
      /* TODO: debug:
          Sometimes I have seen the Serial2.begin to cause a reset
          Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1)
         Backtrace: 0x40081d2f:0x3ffc11b0 0x40087969:0x3ffc11e0 0x4000bfed:0x3ffb1db0 0x4008b7dd:0x3ffb1dc0 0x4017afee:0x3ffb1de0 0x4017b04b:0x3ffb1e20 0x4010722b:0x3ffb1e50 0x40107303:0x3ffb1e70 0x4010782d:0x3ffb1e90 0x40103814:0x3ffb1ed0 0x400d8772:0x3ffb1f10 0x400d9057:0x3ffb1f60 0x40107aca:0x3ffb1fb0 0x4008a63e:0x3ffb1fd0
         #0  0x40081d2f:0x3ffc11b0 in _uart_isr at /Users/hansi/.platformio/packages/framework-arduinoespressif32/cores/esp32/esp32-hal-uart.c:464
         #1  0x40087969:0x3ffc11e0 in _xt_lowint1 at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/freertos/xtensa_vectors.S:1154
         #2  0x4000bfed:0x3ffb1db0 in ?? ??:0
         #3  0x4008b7dd:0x3ffb1dc0 in vTaskExitCritical at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/freertos/tasks.c:3507
         #4  0x4017afee:0x3ffb1de0 in esp_intr_alloc_intrstatus at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/esp32/intr_alloc.c:784
         #5  0x4017b04b:0x3ffb1e20 in esp_intr_alloc at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/esp32/intr_alloc.c:784
         #6  0x4010722b:0x3ffb1e50 in uartEnableInterrupt at /Users/hansi/.platformio/packages/framework-arduinoespressif32/cores/esp32/esp32-hal-uart.c:464
         #7  0x40107303:0x3ffb1e70 in uartAttachRx at /Users/hansi/.platformio/packages/framework-arduinoespressif32/cores/esp32/esp32-hal-uart.c:464
         #8  0x4010782d:0x3ffb1e90 in uartBegin at /Users/hansi/.platformio/packages/framework-arduinoespressif32/cores/esp32/esp32-hal-uart.c:464
         #9  0x40103814:0x3ffb1ed0 in HardwareSerial::begin(unsigned long, unsigned int, signed char, signed char, bool, unsigned long) at /Users/hansi/.platformio/packages/framework-arduinoespressif32/cores/esp32/HardwareSerial.cpp:190
      */
      Serial2.begin(115200, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_set9k6, sizeof(ubx_set9k6));
      delay(200);
      Serial2.begin(38400, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_set9k6, sizeof(ubx_set9k6));
      delay(200);
      Serial2.begin(19200, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_set9k6, sizeof(ubx_set9k6));
      Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      delay(1000);
      dumpGPS();
      Serial.println("GPS factory reset...");
      Serial2.write(ubx_factorydef, sizeof(ubx_factorydef));
      delay(1000);
      dumpGPS();
      delay(1000);
      dumpGPS();
      delay(1000);
      dumpGPS();
      SPIFFS.remove("/GPSRESET");
    } else if (testfile) {
      Serial.println("GPS reset file: not found/isdir");
      testfile.close();
      Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
    }
    // Enable GPGST messages
    Serial2.write(ubx_enable_gpgst, sizeof(ubx_enable_gpgst));
  } else {
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

#include "features.h"
#include "version.h"
#include "core.h"

#define TAG "RX_FSK"
#include "src/logger.h"

#include <dirent.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include "src/httpd.h"

#include <LittleFS.h>

#include <SPI.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <Ticker.h>
#include "esp_heap_caps.h"
//#include <rtc_wdt.h>
//#include "soc/timer_group_struct.h"
//#include "soc/timer_group_reg.h"

#include "src/Radio.h"
#include "src/Sonde.h"
#include "src/Display.h"
#include "src/Scanner.h"
#if FEATURE_RS92
#include "src/geteph.h"
#include "src/rs92gps.h"
#endif

#include "src/ShFreqImport.h"
#include "src/RS41.h"
#include "src/DFM.h"
#include "src/json.h"
#include "src/posinfo.h"

#include "src/pmu.h"
#include "src/user.h"
#include "src/Keyboard.h"
#include "src/Startup.h"


/* Data exchange connectors */
#if FEATURE_SONDESEEKER
#include "src/conn-sondeseeker.h"
#endif
#if FEATURE_CHASEMAPPER
#include "src/conn-chasemapper.h"
#endif
#if FEATURE_MQTT
#include "src/conn-mqtt.h"
#endif
#if FEATURE_SDCARD
#include "src/conn-sdcard.h"
#endif
#if FEATURE_APRS
#include "src/conn-aprs.h"
#endif
#if FEATURE_SONDEHUB
#include "src/conn-sondehub.h"
#endif

#include "src/conn-system.h"

extern SemaphoreHandle_t globalLock;

Conn *connectors[] = { &connSystem,
&connGPS,
#if FEATURE_APRS
&connAPRS,
#endif
#if FEATURE_SONDEHUB
&connSondehub,
#endif
#if FEATURE_CHASEMAPPER
&connChasemapper,
#endif
#if FEATURE_SONDESEEKER
&connSondeseeker,
#endif
#if FEATURE_MQTT
&connMQTT,
#endif
#if FEATURE_SDCARD
&connSDCard,
#endif
NULL };

//#define ESP_MEM_DEBUG 1
//int e;

enum MainState { ST_DECODER, ST_SPECTRUM, ST_WIFISCAN, ST_UPDATE, ST_TOUCHCALIB, ST_RINEX_UPDATE, ST_FORMAT_SD };
static MainState mainState = ST_WIFISCAN;
const char *mainStateStr[] = {"DECODER", "SPECTRUM", "WIFISCAN", "UPDATE", "TOUCHCALIB", "RINEXUPDATE", "FORMATSD" };


PMU *pmu = NULL;
SemaphoreHandle_t axpSemaphore;
extern uint8_t pmu_irq;

const char *updateHost = "rdzsonde.org";
int updatePort = 80;

const char *updatePrefixM = "/main/";
const char *updatePrefixD = "/dev2/";
const char *updatePrefix = updatePrefixM;
const char *updateFs = "update.fs.bin";
const char *updateIno = "update.ino.bin";

#define LOCALUDPPORT 9002
//Get real UTC time from NTP server
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0; //UTC
const int   daylightOffset_sec = 0; //UTC

// Authentication management
// defaultUserLevel is read from user.txt
uint8_t defaultUserLevel = 2;

boolean connected = false;
WiFiUDP udp;
WiFiClient client;

/* Sonde.h: enum SondeType { STYPE_DFM,, STYPE_RS41, STYPE_RS92, STYPE_M10M20, STYPE_M10, STYPE_M20, STYPE_MP3H }; */
const char *sondeTypeStrSH[NSondeTypes] = { "DFM", "RS41", "RS92", "Mxx"/*never sent*/, "M10", "M20", "MRZ" };


// moved to connSondehub.cpp
//#if FEATURE_SONDEHUB
//#define SONDEHUB_STATION_UPDATE_TIME (60*60*1000) // 60 min
//#define SONDEHUB_MOBILE_STATION_UPDATE_TIME (30*1000) // 30 sec
//WiFiClient shclient;	// Sondehub v2
//int shImportInterval = 0;
//char shImport = 0;
//unsigned long time_last_update = 0;
//#endif

// JSON over TCP for communicating with the rdzSonde (rdzwx-go) Android app
WiFiServer rdzserver(14570);
WiFiClient rdzclient;

// If a file "localupd.txt" exists, firmware can be updated from a custom IP address read from this file, stored in localUpdates.
// By default (localUpdates==NULL) this is disabled to prevent abuse
// Note: by enabling this, someone with access to the web interface can replace the firmware arbitrarily!
// Make sure that only trustworthy persons have access to the web interface...
char *localUpdates = NULL;

boolean forceReloadScreenConfig = false;

static int lastDisplay = 1;
static int currentDisplay = 1;

// timestamp when spectrum display was activated
static unsigned long specTimer;

void enterMode(int mode);
void WiFiEvent(WiFiEvent_t event);


// Possibly we will need more fine grained permissions in the future...
// For now, disallow arbitrary firmware updates on standard installations
// development installations can add a file "localupd.txt" which enables updates from arbitrary locations
int checkAllowed(const char *filename) {
    if(!localUpdates && (strstr(filename, "localupd.txt") != NULL)) return 0;
    return 1;
}


// Read line from file, independent of line termination (LF or CR LF)
String readLine(Stream &stream) {
  String s = stream.readStringUntil('\n');
  int len = s.length();
  if (len == 0) return s;
  if (s.charAt(len - 1) == '\r') s.remove(len - 1);
  return s;
}

// Read line from file, without using dynamic memory allocation (String class)
// returns length line.
int readLine(Stream &stream, char *buffer, int maxlen) {
  int n = stream.readBytesUntil('\n', buffer, maxlen);
  buffer[n] = 0;
  if (n <= 0) return 0;
  if (buffer[n - 1] == '\r') {
    buffer[n - 1] = 0;
    n--;
  }
  return n;
}


const String sondeTypeSelect(int activeType) {
  String sts = "";
  for (int i = 0; i < NSondeTypes; i++) {
    sts += "<option value=\"";
    sts += sondeTypeLongStr[i];
    sts += "\"";
    if (activeType == i) {
      sts += " selected";
    }
    sts += ">";
    sts += sondeTypeLongStr[i];
    sts += "</option>";
  }
  return sts;
}


//trying to work around
//"assertion "heap != NULL && "free() target pointer is outside heap areas"" failed:"
// which happens if request->send is called in createQRGForm!?!??
char message[10240 * 3 - 2048]; //needs to be large enough for all forms (not checked in code)
// QRG form is currently about 24kb with 100 entries

///////////////////////// Functions for Reading / Writing QRG list from/to qrg.txt

void setupChannelList() {
  File file = LittleFS.open("/qrg.txt", "r");
  if (!file) {
    LOG_E(TAG, "There was an error opening the file '/qrg.txt' for reading");
    return;
  }
  int i = 0;
  char launchsite[17] = "                ";
  sonde.clearSonde();
  LOG_I(TAG, "Reading qrg.txt:");
  while (file.available()) {
    String line = readLine(file);
    String sitename;
    if (line[0] == '#') continue;
    char *space = strchr(line.c_str(), ' ');
    if (!space) continue;
    *space = 0;
    float freq = atof(line.c_str());
    SondeType type;
    if (space[1] == '4') {
      type = STYPE_RS41;
    } else if (space[1] == 'R') {
      type = STYPE_RS92;
    }
    else if (space[1] == 'D' || space[1] == '9' || space[1] == '6') {
      type = STYPE_DFM;
    }
    else if (space[1] == 'M') {
      type = STYPE_M10M20;
    }
    else if (space[1] == '2') {
      type = STYPE_M10M20;
    }
    else if (space[1] == '3') {
      type = STYPE_MP3H;
    }
    else continue;
    int active = space[3] == '+' ? 1 : 0;
    if (space[4] == ' ') {
      memset(launchsite, ' ', 16);
      strncpy(launchsite, space + 5, 16);
      if (sonde.config.debug == 1) {
        LOG_D(TAG, "Add %f - sondetype: %d (on/off: %d) - site #%d - name: %s\n ", freq, type, active, i, launchsite);
      }
    }
    sonde.addSonde(freq, type, active, launchsite);
    i++;
  }
  file.close();
  sonde.listReloadMs = millis();
}

const char *HTMLHEAD = "<!DOCTYPE html><html><head> <meta charset=\"UTF-8\"> <link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\">";
void HTMLBODY_OS(char *ptr, const char *which, const char *onsubmit) {
  strcat(ptr, "<body><form class=\"wrapper\" action=\"");
  strcat(ptr, which);
  if(onsubmit) {
     strcat(ptr, "\" onsubmit=\"");
     strcat(ptr, onsubmit);
  }
  strcat(ptr, "\" method=\"post\"><div class=\"content\">");
}
void HTMLBODY(char *ptr, const char *which) { HTMLBODY_OS(ptr, which, NULL); }
void HTMLBODYEND(char *ptr) {
  strcat(ptr, "</div></form></body></html>");
}
void HTMLSAVEBUTTON(char *ptr) {
  strcat(ptr, "</div><div class=\"footer\"><input type=\"submit\" class=\"save\" value=\"Save changes\"/>"
         "<span class=\"ttgoinfo\">rdzTTGOserver ");
  strcat(ptr, version_id);
  strcat(ptr, "</span>");
}

const char *getQRGAsJson() {
  char *ptr = message;
  strcpy(ptr, "{\"channels\":[");
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    SondeInfo *si = &sonde.sondeList[i];
    if (i > 0) {
      strcat(ptr, ",");
    }
    sprintf(ptr + strlen(ptr),
            "{\"channel\":%d, \"active\":%d, \"freq\":%.3f, \"launchsite\":\"%s\", \"type\":\"%s\"}",
            i+1, si->active, si->freq, si->launchsite, sondeTypeStr[si->type]);
  }
  strcat(ptr, "]}");
  return message;
}

const char *createQRGForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "<script src=\"rdz.js\"></script></head>");
  HTMLBODY(ptr, "qrg.html");
  //strcat(ptr, "<body><form class=\"wrapper\" action=\"qrg.html\" method=\"post\"><div class=\"content\"><table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Launchsite</th><th>Mode</th></tr>");
  strcat(ptr, "<script>\nvar qrgs = [];\n");
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    SondeInfo *si = &sonde.sondeList[i];
    sprintf(ptr + strlen(ptr), "qrgs.push([%d, \"%.3f\", \"%s\", \"%c\"]);\n", si->active, si->freq, si->launchsite, sondeTypeChar[si->type] );
  }
  strcat(ptr, "</script>\n");
  strcat(ptr, "<div id=\"divTable\"></div>");
  strcat(ptr, "<script> qrgTable() </script>\n");
  //</div><div class=\"footer\"><input type=\"submit\" class=\"update\" value=\"Update\"/>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  LOG_D(TAG, "QRG form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleQRGPost(void) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
  File file = LittleFS.open("/qrg.txt", "w");
  if (!file) {
    LOG_E(TAG, "Error while opening '/qrg.txt' for writing");
    return "Error while opening '/qrg.txt' for writing";
  }
  LOG_D(TAG, "Handling post request");
  for (int i = 1; i <= sonde.config.maxsonde; i++) {
    snprintf(label, 10, "A%d", i);
    const char *active = formGet(label);
    snprintf(label, 10, "F%d", i);
    const char *fstr = formGet(label);
    snprintf(label, 10, "S%d", i);
    const char *sstr = formGet(label);
    if (!fstr) continue;
    snprintf(label, 10, "T%d", i);
    const char *tstr = formGet(label);
    if (!tstr) continue;
    if (!sstr) sstr = "";
    if (*tstr == '6' || *tstr == '9') tstr = "D";
    LOG_D(TAG, "Processing a=%s, f=%s, t=%s, site=%s\n", active ? "YES" : "NO", fstr, tstr, sstr);
    char typech = tstr[0];
    file.printf("%3.3f %c %c %s\n", atof(fstr), typech, active ? '+' : '-', sstr);
  }
  file.close();

  LOG_D(TAG, "Channel setup finished\n");
  setupChannelList();
  return "";
}


/////////////////// Functions for reading/writing Wifi networks from networks.txt

#define MAX_WIFI 10
int nNetworks;
struct {
  String id;
  String pw;
} networks[MAX_WIFI];


// used by improv wifi
int updateWiFi(String ssid, String pw) {
        networks[1].id = ssid;
        networks[1].pw = pw;
	if(nNetworks<2) nNetworks = 2;
        File file = LittleFS.open("/networks.txt", "w");
        if(!file) return -1;
        for(int i=0; i<nNetworks; i++) {
                if(networks[i].id && networks[i].pw) {
                        file.printf("%s\n%s\n", networks[i].id.c_str(), networks[i].pw.c_str());
                }
        }
        file.close(); 
	return 0;
}       



// FIXME: For now, we don't uspport wifi networks that contain newline or null characters
// ... would require a more sophisicated file format (currently one line SSID; one line Password
void setupWifiList() {
  File file = LittleFS.open("/networks.txt", "r");
  if (!file) {
    LOG_E(TAG, "There was an error opening the file '/networks.txt' for reading");
    networks[0].id = "RDZsonde";
    networks[0].pw = "RDZsonde";
    return;
  }
  int i = 0;

  while (file.available()) {
    String line = readLine(file);  //file.readStringUntil('\n');
    if (!file.available()) break;
    networks[i].id = line;
    networks[i].pw = readLine(file); // file.readStringUntil('\n');
    i++;
  }
  nNetworks = i;
  LOG_I(TAG, "%d networks in networks.txt\n", i);
  for (int j = 0; j < i; j++) {
    LOG_I(TAG, "%s: %s\n", networks[j].id.c_str(), networks[j].pw.c_str());
  }
}

// copy string, replacing '"' with '&quot;'
// max string length is 31 characters
const String quoteString(const char *s) {
   char buf[6*32];
   uint16_t i = 0, o = 0;
   int len = strlen(s);
   if(len>31) len=31;
   while(i<len) {
      if(s[i]=='"') { strcpy(buf+o, "&quot;"); o+=6; }
      else buf[o++] = s[i];
      i++;
   }
   buf[o] = 0;
   return String(buf);
}

const char *createWIFIForm() {
  char *ptr = message;
  char tmp[4];
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "<script src=\"rdz.js\"></script></head>");
  HTMLBODY(ptr, "wifi.html");
  strcat(ptr, "<table><tr><th>Nr</th><th>SSID</th><th>Password</th></tr>");
  for (int i = 0; i < MAX_WIFI; i++) {
    String pw = i < nNetworks ? quoteString( networks[i].pw.c_str() ) : "";
    sprintf(tmp, "%d", i);
    sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"S%d\" type=\"text\" value=\"%s\"/></td>"
            "<td><input name=\"P%d\" type=\"text\" value=\"%s\"/></td>",
            i == 0 ? "<b>AP</b>" : tmp,
            i + 1, i < nNetworks ? networks[i].id.c_str() : "",
            i + 1, pw.c_str() );
  }
  strcat(ptr, "</table><script>footer()</script>");
  //</div><div class=\"footer\"><input type=\"submit\" class=\"update\" value=\"Update\"/>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  LOG_D(TAG, "WIFI form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleWIFIPost(void) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File f = LittleFS.open("/networks.txt", "w");
  if (!f) {
    LOG_E(TAG, "Error while opening '/networks.txt' for writing");
    return "Error while opening '/networks.txt' for writing";
  }
#endif
  LOG_D(TAG, "Handling post request");
  for (int i = 1; i <= MAX_WIFI; i++) {
    snprintf(label, 10, "S%d", i);
    const char *sstr = formGet(label);
    if (!sstr) continue;
    snprintf(label, 10, "P%d", i);
    const char *pstr = formGet(label);
    if (!pstr) continue;
    if (strlen(sstr) == 0) continue;
    LOG_D(TAG, "Processing S=%s, P=%s\n", sstr, pstr);
    f.printf("%s\n%s\n", sstr, pstr);
  }
  f.close();
  setupWifiList();
  return "";
}

/* decoder.json helpers: unsigned millis compare safe across wrap for gaps < ~24d */
static bool ms_after(uint32_t t, uint32_t since) {
  return (int32_t)(t - since) > 0;
}

static void json_append_escaped(char *dst, const char *src) {
  char *p = dst + strlen(dst);
  for (; *src; src++) {
    unsigned char c = (unsigned char)*src;
    if (c == '"' || c == '\\') {
      *p++ = '\\';
      *p++ = (char)c;
    } else if (c >= 0x20) {
      *p++ = (char)c;
    }
  }
  *p = 0;
}

static void appendDecoderEntry(char *ptr, int i, bool empty) {
  SondeInfo *s = &sonde.sondeList[i];
  const char *typestr = getType(s);
  uint32_t now = millis();
  uint32_t age = (s->lastRx == 0) ? 0 : (now - s->lastRx) / 1000;

  strcat(ptr, "{");
  sprintf(ptr + strlen(ptr), "\"ch\":%d,\"freq\":%.3f,\"type\":\"", i + 1, s->freq);
  json_append_escaped(ptr, typestr);
  sprintf(ptr + strlen(ptr), "\",\"age\":%u,\"res\":%d,\"rssi\":%d,\"validPos\":%d,\"validId\":%d",
          (unsigned)age, (int)s->rxStat[0], s->rssi, (int)s->d.validPos, s->d.validID ? 1 : 0);
  if (empty) {
    strcat(ptr, ",\"empty\":true");
  }
  if (s->d.id[0]) {
    strcat(ptr, ",\"id\":\"");
    json_append_escaped(ptr, s->d.id);
    strcat(ptr, "\"");
  }
  if (s->d.ser[0]) {
    strcat(ptr, ",\"ser\":\"");
    json_append_escaped(ptr, s->d.ser);
    strcat(ptr, "\"");
  }
  {
    char site[18];
    strncpy(site, s->launchsite, 17);
    site[17] = 0;
    // trim trailing spaces
    for (int k = (int)strlen(site) - 1; k >= 0 && site[k] == ' '; k--) site[k] = 0;
    if (site[0]) {
      strcat(ptr, ",\"launchsite\":\"");
      json_append_escaped(ptr, site);
      strcat(ptr, "\"");
    }
  }
  if (!isnan(s->d.lat)) sprintf(ptr + strlen(ptr), ",\"lat\":%.6f", s->d.lat);
  if (!isnan(s->d.lon)) sprintf(ptr + strlen(ptr), ",\"lon\":%.6f", s->d.lon);
  if (!isnan(s->d.alt)) sprintf(ptr + strlen(ptr), ",\"alt\":%.0f", s->d.alt);
  if (!isnan(s->d.vs)) sprintf(ptr + strlen(ptr), ",\"vs\":%.1f", s->d.vs);
  if (!isnan(s->d.hs)) sprintf(ptr + strlen(ptr), ",\"hs\":%.1f", s->d.hs);
  if (!isnan(s->d.dir)) sprintf(ptr + strlen(ptr), ",\"dir\":%.0f", s->d.dir);
  if (!isnan(s->d.temperature)) sprintf(ptr + strlen(ptr), ",\"temp\":%.1f", s->d.temperature);
  if (!isnan(s->d.relativeHumidity)) sprintf(ptr + strlen(ptr), ",\"humidity\":%.1f", s->d.relativeHumidity);
  if (!isnan(s->d.pressure)) sprintf(ptr + strlen(ptr), ",\"pressure\":%.1f", s->d.pressure);
  if (s->d.batteryVoltage > 0) sprintf(ptr + strlen(ptr), ",\"batt\":%.1f", s->d.batteryVoltage);
  sprintf(ptr + strlen(ptr), ",\"sats\":%d,\"frame\":%u,\"time\":%u",
          (int)s->d.sats, (unsigned)s->d.frame, (unsigned)s->d.time);
  if (s->type == STYPE_RS41 && (s->d.burstKT || s->d.launchKT || s->d.countKT)) {
    sprintf(ptr + strlen(ptr), ",\"burstKT\":%d,\"launchKT\":%d,\"countKT\":%d",
            (int)s->d.burstKT, (int)s->d.launchKT, (int)s->d.countKT);
  }
  strcat(ptr, "}");
}

const char *createDecoderJson(bool use_since, uint32_t since) {
  char *ptr = message;
  uint32_t now = millis();
  /* Full snapshot if no since, QRG reloaded, or since is ahead of now (device reboot). */
  bool full = !use_since || ms_after(sonde.listReloadMs, since) || (int32_t)(now - since) < 0;

  SondeInfo *cur = &sonde.sondeList[sonde.currentSonde];
  const char *typestr = sondeTypeLongStr[sonde.realType(cur)];
  const char *state;
  char detail[160];
  detail[0] = 0;

  if (mainState == ST_SPECTRUM) {
    state = "spectrum";
    snprintf(detail, sizeof(detail), "Spectrum scan");
  } else if (mainState != ST_DECODER) {
    state = "disabled";
    const char *label = "Idle";
    switch (mainState) {
      case ST_WIFISCAN: label = "WiFi scan"; break;
      case ST_UPDATE: label = "Update"; break;
      case ST_TOUCHCALIB: label = "Touch calib"; break;
      case ST_RINEX_UPDATE: label = "RINEX update"; break;
      case ST_FORMAT_SD: label = "Format SD"; break;
      default: break;
    }
    snprintf(detail, sizeof(detail), "%s", label);
  } else if (disp.layoutIdx == sonde.config.display[0]) {
    state = "scan";
    snprintf(detail, sizeof(detail), "Scan Ch %d: %.3f MHz %s",
             sonde.currentSonde + 1, cur->freq, typestr);
  } else {
    state = "decode";
    if (cur->d.validID && cur->d.id[0]) {
      snprintf(detail, sizeof(detail), "Decoding Ch %d: %.3f MHz %s %s",
               sonde.currentSonde + 1, cur->freq, typestr, cur->d.id);
    } else if (cur->d.ser[0]) {
      snprintf(detail, sizeof(detail), "Decoding Ch %d: %.3f MHz %s %s",
               sonde.currentSonde + 1, cur->freq, typestr, cur->d.ser);
    } else {
      snprintf(detail, sizeof(detail), "Decoding Ch %d: %.3f MHz %s",
               sonde.currentSonde + 1, cur->freq, typestr);
    }
  }

  strcpy(ptr, "{");
  sprintf(ptr + strlen(ptr),
          "\"now\":%u,\"full\":%s,\"state\":\"%s\",\"detail\":\"",
          (unsigned)now, full ? "true" : "false", state);
  json_append_escaped(ptr, detail);
  sprintf(ptr + strlen(ptr),
          "\",\"ch\":%d,\"freq\":%.3f,\"type\":\"",
          sonde.currentSonde + 1, cur->freq);
  json_append_escaped(ptr, typestr);
  strcat(ptr, "\"");

  strcat(ptr, ",\"entries\":[");
  bool first = true;
  int decoded = 0;
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    SondeInfo *s = &sonde.sondeList[i];
    if (!s->active || s->lastRx == 0) continue;
    decoded++;
    if (!full && !ms_after(s->lastRx, since)) continue;
    if (!first) strcat(ptr, ",");
    first = false;
    appendDecoderEntry(ptr, i, false);
  }
  if (full && decoded == 0) {
    appendDecoderEntry(ptr, sonde.currentSonde, true);
  }
  strcat(ptr, "]");

  if (mainState == ST_SPECTRUM && spectrumLen > 0) {
    int n = spectrumLen;
    if (n > SPECTRUM_MAX_BINS) n = SPECTRUM_MAX_BINS;
    sprintf(ptr + strlen(ptr),
            ",\"spectrum\":{\"start\":%.3f,\"step\":%.5f,\"peak\":%.3f,\"peakRssi\":%d,\"noisefloor\":%d,\"plotMax\":%d,\"rssi\":[",
            spectrumStartMHz, spectrumStepMHz, spectrumPeakMHz, spectrumPeakRssi,
            sonde.config.noisefloor, spectrumPlotMax);
    for (int i = 0; i < n; i++) {
      sprintf(ptr + strlen(ptr), "%s%d", i ? "," : "", scandisp[i]);
    }
    strcat(ptr, "]}");
  }

  strcat(ptr, "}");
  LOG_D(TAG, "decoder.json: size=%d bytes full=%d\n", (int)strlen(message), (int)full);
  return message;
}

const char *createLiveJson() {
  char *ptr = message;
  SondeInfo *s = &sonde.sondeList[sonde.currentSonde];

  strcpy(ptr, "{\"sonde\": {");
  // use the same JSON format here as for MQTT and for the Android App
  sonde2json( ptr + strlen(ptr), 1024, s );
#if 0
  sprintf(ptr + strlen(ptr), "\"sonde\": {\"rssi\": %d, \"vframe\": %d, \"time\": %d,\"id\": \"%s\", \"freq\": %3.3f, \"type\": \"%s\"",
          s->rssi, s->d.vframe, s->d.time, s->d.id, s->freq, sondeTypeStr[sonde.realType(s)]);

  if ( !isnan(s->d.lat) && !isnan(s->d.lon) )
    sprintf(ptr + strlen(ptr), ", \"lat\": %.6f, \"lon\": %.6f", s->d.lat, s->d.lon);
  if ( !isnan(s->d.alt) )
    sprintf(ptr + strlen(ptr), ", \"alt\": %.0f", s->d.alt);
  if ( !isnan(s->d.dir) )
    sprintf(ptr + strlen(ptr), ", \"dir\": %.0f", s->d.dir);
  if ( !isnan(s->d.vs) )
    sprintf(ptr + strlen(ptr), ", \"climb\": %.1f", s->d.vs);
  if ( !isnan(s->d.hs) )
    sprintf(ptr + strlen(ptr), ", \"speed\": %.1f", s->d.hs);

  sprintf(ptr + strlen(ptr), ", \"launchsite\": \"%s\", \"res\": %d }", s->launchsite, s->rxStat[0]);
#endif
  strcat(ptr, " }");

  if (posInfo.valid) {
    sprintf(ptr + strlen(ptr), ", \"gps\": {\"lat\": %.8g, \"lon\": %.8g, \"alt\": %d, \"sat\": %d, \"speed\": %g, \"dir\": %d, \"hdop\": %d }", posInfo.lat, posInfo.lon, posInfo.alt, posInfo.sat, posInfo.speed, posInfo.course, posInfo.hdop);
    //}
  }

  strcat(ptr, "}");
  return message;
}

///////////////////// Config form


void setupConfigData() {
  File file = LittleFS.open("/config.txt", "r");
  if (!file) {
    LOG_E(TAG, "There was an error opening the file '/config.txt' for reading");
    return;
  }
  while (file.available()) {
    String line = readLine(file);  //file.readStringUntil('\n');
    sonde.setConfig(line.c_str());
  }
  sonde.checkConfig(); // eliminate invalid entries
}


struct st_configitems config_list[] = {
  /* General config settings */
  {"wifi", 0, &sonde.config.wifi},
  {"debug", 0, &sonde.config.debug},
  {"maxsonde", 0, &sonde.config.maxsonde},
  {"rxlat", -7, &sonde.config.rxlat},
  {"rxlon", -7, &sonde.config.rxlon},
  {"rxalt", -7, &sonde.config.rxalt},
  {"b2mute", 0, &sonde.config.b2mute},
  {"screenfile", 0, &sonde.config.screenfile},
  {"display", -6, sonde.config.display},
  {"dispsaver", 0, &sonde.config.dispsaver},
  {"dispcontrast", 0, &sonde.config.dispcontrast},
  /* Spectrum display settings */
  {"spectrum", 0, &sonde.config.spectrum},
  {"startfreq", 0, &sonde.config.startfreq},
  {"channelbw", 0, &sonde.config.channelbw},
  {"marker", 0, &sonde.config.marker},
  {"noisefloor", 0, &sonde.config.noisefloor},
  /* decoder settings */
  {"freqofs", 0, &sonde.config.freqofs},
  {"rs41.agcbw", 0, &sonde.config.rs41.agcbw},
  {"rs41.rxbw", 0, &sonde.config.rs41.rxbw},
  {"rs92.rxbw", 0, &sonde.config.rs92.rxbw},
  {"rs92.alt2d", 0, &sonde.config.rs92.alt2d},
  {"dfm.agcbw", 0, &sonde.config.dfm.agcbw},
  {"dfm.rxbw", 0, &sonde.config.dfm.rxbw},
  {"m10m20.agcbw", 0, &sonde.config.m10m20.agcbw},
  {"m10m20.rxbw", 0, &sonde.config.m10m20.rxbw},
  {"mp3h.agcbw", 0, &sonde.config.mp3h.agcbw},
  {"mp3h.rxbw", 0, &sonde.config.mp3h.rxbw},
  {"ephftp", 79, &sonde.config.ephftp},
  /* APRS settings */
  {"call", 9, sonde.config.call},
  {"passcode", 0, &sonde.config.passcode},
  /* KISS tnc settings */
  {"kisstnc.active", 0, &sonde.config.kisstnc.active},
#if FEATURE_APRS
  /* AXUDP settings */
  {"axudp.active", -3, &sonde.config.udpfeed.active},
  {"axudp.host", 63, sonde.config.udpfeed.host},
  {"axudp.ratelimit", 0, &sonde.config.udpfeed.ratelimit},
  /* APRS TCP settings */
  {"tcp.active", -3, &sonde.config.tcpfeed.active},
  {"tcp.timeout", 0, &sonde.config.tcpfeed.timeout},
  {"tcp.host", 63, sonde.config.tcpfeed.host},
  {"tcp.host2", 63, &sonde.config.tcpfeed.host2},
  {"tcp.chase", 0, &sonde.config.chase},
  {"tcp.comment", 30, sonde.config.comment},
  {"tcp.objcall", 9, sonde.config.objcall},
  {"tcp.beaconsym", 4, sonde.config.beaconsym},
  {"tcp.highrate", 0, &sonde.config.tcpfeed.highrate},
#endif
#if FEATURE_CHASEMAPPER
  /* Chasemapper settings */
  {"cm.active", -3, &sonde.config.cm.active},
  {"cm.host", 63, &sonde.config.cm.host},
  {"cm.port", 0, &sonde.config.cm.port},
#endif
#if FEATURE_SONDESEEKER
   /* Sondeseeker settings */
   {"ss.active", -3, &sonde.config.ss.active},
   {"ss.host", 63, &sonde.config.ss.host},
   {"ss.port", 0, &sonde.config.ss.port},
#endif
#if FEATURE_MQTT
  /* MQTT */
  {"mqtt.active", 0, &sonde.config.mqtt.active},
  {"mqtt.id", 63, &sonde.config.mqtt.id},
  {"mqtt.host", 63, &sonde.config.mqtt.host},
  {"mqtt.port", 0, &sonde.config.mqtt.port},
  {"mqtt.username", 63, &sonde.config.mqtt.username},
  {"mqtt.password", 63, &sonde.config.mqtt.password},
  {"mqtt.prefix", 63, &sonde.config.mqtt.prefix},
  {"mqtt.report_interval", 0, &sonde.config.mqtt.report_interval},
#endif
#if FEATURE_SDCARD
  /* SD-Card settings */
  {"sd.cs", 0, &sonde.config.sd.cs},
  {"sd.miso", 0, &sonde.config.sd.miso},
  {"sd.mosi", 0, &sonde.config.sd.mosi},
  {"sd.clk", 0, &sonde.config.sd.clk},
  {"sd.sync", 0, &sonde.config.sd.sync},
  {"sd.name", 0, &sonde.config.sd.name},
  {"sd.speed", 0, &sonde.config.sd.speed},
#endif
  /* Hardware dependeing settings */
  {"disptype", 0, &sonde.config.disptype},
  {"norx_timeout", 0, &sonde.config.norx_timeout},
  {"oled_sda", 0, &sonde.config.oled_sda},
  {"oled_scl", 0, &sonde.config.oled_scl},
  {"oled_rst", 0, &sonde.config.oled_rst},
  {"tft_rs", 0, &sonde.config.tft_rs},
  {"tft_cs", 0, &sonde.config.tft_cs},
  {"tft_orient", 0, &sonde.config.tft_orient},
  {"tft_spifreq", 0, &sonde.config.tft_spifreq},
  {"button_pin", -4, &sonde.config.button_pin},
  {"button2_pin", -4, &sonde.config.button2_pin},
  {"button2_axp", 0, &sonde.config.button2_axp},
  {"touch_thresh", 0, &sonde.config.touch_thresh},
  {"power_pout", 0, &sonde.config.power_pout},
  {"led_pout", 0, &sonde.config.led_pout},
  {"gps_rxd", 0, &sonde.config.gps_rxd},
  {"gps_txd", 0, &sonde.config.gps_txd},
  {"batt_adc", 0, &sonde.config.batt_adc},
#if 1
  {"sx1278_ss", 0, &sonde.config.sx1278_ss},
  {"sx1278_miso", 0, &sonde.config.sx1278_miso},
  {"sx1278_mosi", 0, &sonde.config.sx1278_mosi},
  {"sx1278_sck", 0, &sonde.config.sx1278_sck},
#endif
  {"mdnsname", 14, &sonde.config.mdnsname},

#if FEATURE_SONDEHUB
  /* SondeHub settings */
  {"sondehub.active", 0, &sonde.config.sondehub.active},
  {"sondehub.chase", 0, &sonde.config.sondehub.chase},
  {"sondehub.host", 63, &sonde.config.sondehub.host},
  {"sondehub.callsign", 63, &sonde.config.sondehub.callsign},
  {"sondehub.antenna", 63, &sonde.config.sondehub.antenna},
  {"sondehub.email", 63, &sonde.config.sondehub.email},
  {"sondehub.fiactive", 0, &sonde.config.sondehub.fiactive},
  {"sondehub.fiinterval", 0, &sonde.config.sondehub.fiinterval},
  {"sondehub.fimaxdist", 0, &sonde.config.sondehub.fimaxdist},
  {"sondehub.fimaxage", -7, &sonde.config.sondehub.fimaxage},
#endif
};

const int N_CONFIG = (sizeof(config_list) / sizeof(struct st_configitems));

const char *createConfigForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "<script src=\"rdz.js\"></script></head>");
  HTMLBODY_OS(ptr, "config.html", "return checkForDuplicates()");
  strcat(ptr, "<div id=\"cfgtab\"></div>");
  strcat(ptr, "<script src=\"cfg.js\"></script>");
  strcat(ptr, "<script>\n");
  sprintf(ptr + strlen(ptr), "var scr=\"Using /screens%d.txt", Display::getScreenIndex(sonde.config.screenfile));
  for (int i = 0; i < disp.nLayouts; i++) {
    sprintf(ptr + strlen(ptr), "<br>%d=%s", i, disp.layouts[i].label);
  }
  strcat(ptr, "\";\n");
  strcat(ptr, "var cf=new Map();\n");
  for (int i = 0; i < N_CONFIG; i++) {
    sprintf(ptr + strlen(ptr), "cf.set(\"%s\", \"", config_list[i].name);
    switch (config_list[i].type) {
      case -4:
      case -3:
      case -2:
      case 0:
        sprintf(ptr + strlen(ptr), "%d", *(int *)config_list[i].data);
        LOG_D(TAG, "Config for %s is %d\n", config_list[i].name, *(int *)config_list[i].data);
        break;
      case -6: // list
        {
          int8_t *l = (int8_t *)config_list[i].data;
          if (*l == -1) strcat(ptr, "0");
          else {
            sprintf(ptr + strlen(ptr), "%d", l[0]);
            l++;
          }
          while (*l != -1) {
            sprintf(ptr + strlen(ptr), ",%d", *l);
            l++;
          }
        }
        break;
      case -7: // double
        if (!isnan(*(double *)config_list[i].data))
          sprintf(ptr + strlen(ptr), "%.8g", *(double *)config_list[i].data);
        break;
      default: // string
        strcat(ptr, (char *)config_list[i].data);
    }
    strcat(ptr, "\");\n");
  }
  strcat(ptr, "configTable();\n </script>");
  strcat(ptr, "<script>footer()</script>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  LOG_D(TAG, "Config form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleConfigPost(void) {
  // parameters: a_i, f_1, t_i  (active/frequency/type)
  LOG_D(TAG, "Handling config post request");
#if 1
  File f = LittleFS.open("/config.txt", "w");
  if (!f) {
    LOG_E(TAG, "Error while opening '/config.txt' for writing");
    return "Error while opening '/config.txt' for writing";
  }
#endif
  LOG_D(TAG, "File open for writing.");
  int params = formCount();
  for (int i = 0; i < params; i++) {
    const char *label = formNameAt(i);
    if (!label || label[0] == '\0') continue;
    if (label[strlen(label) - 1] == '#') continue;
    const char *value = formValueAt(i);
    if (!value) continue;
    char valuebuf[128];
    strncpy(valuebuf, value, sizeof(valuebuf) - 1);
    valuebuf[sizeof(valuebuf) - 1] = '\0';
    if ( strcmp(label, "button_pin") == 0 ||
         strcmp(label, "button2_pin") == 0) {
      char touchkey[48];
      snprintf(touchkey, sizeof(touchkey), "%s#", label);
      const char *touch = formGet(touchkey);
      if (touch) {
        int iv = atoi(valuebuf);
        if (iv != -1 && iv != 255) iv += 128;
        snprintf(valuebuf, sizeof(valuebuf), "%d", iv);
      }
    }
    LOG_D(TAG, "Processing %s=%s\n", label, valuebuf);
    int wlen = f.printf("%s=%s\n", label, valuebuf);
    LOG_D(TAG, "Written bytes: %d\n", wlen);
  }
  LOG_D(TAG, "Flushing file\n");
  f.flush();
  LOG_D(TAG, "Closing file\n");
  f.close();
  LOG_D(TAG, "Re-reading file file\n");
  setupConfigData();
  if (!gpsPos.valid) fixedToPosInfo();
  // TODO: Check if this is better done elsewhere?
  // Use new config (whereever this is feasible without a reboot)
  disp.setContrast();
  return "";
}

const char *ctrlid[] = {"rx", "scan", "spec", "wifi", "rx2", "scan2", "spec2", "wifi2",
#if FEATURE_RS92
	"rinex",
#endif
#if FEATURE_SDCARD
	"format",
#endif
        "reboot"};

const char *ctrllabel[] = {"Receiver/next freq. (short keypress)", "Scanner (double keypress)", "Spectrum (medium keypress)", "WiFi (long keypress)",
                           "Button 2/next screen (short keypress)", "Button 2 (double keypress)", "Button 2 (medium keypress)", "Button 2 (long keypress)",
#if FEATURE_RS92
                           "Update RS92 RINEX eph",
#endif
#if FEATURE_SDCARD
			   "Format SD Card",
#endif
			   "Reboot"
                          };

const char *createControlForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "</head>");
  HTMLBODY(ptr, "control.html");
  for (int i = 0; i < sizeof(ctrllabel)/sizeof((ctrllabel)[0]); i++) {
    strcat(ptr, "<input class=\"ctlbtn\" type=\"submit\" name=\"");
    strcat(ptr, ctrlid[i]);
    strcat(ptr, "\" value=\"");
    strcat(ptr, ctrllabel[i]);
    strcat(ptr, "\"></input>");
    if (i == 3 || i == 7 ) {
      strcat(ptr, "<p></p>");
    }
  }
  strcat(ptr, "</div><div class=\"footer\"><span></span>"
         "<span class=\"ttgoinfo\">rdzTTGOserver ");
  strcat(ptr, version_id);
  strcat(ptr, "</span>");
  HTMLBODYEND(ptr);
  LOG_I(TAG, "Control form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleControlPost(void) {
  LOG_D(TAG, "Handling control post request");
  int params = formCount();
  for (int i = 0; i < params; i++) {
    const char *param = formNameAt(i);
    if (!param) continue;
    LOG_D(TAG, "Contral post: %s\n", param);
    if (strcmp(param, "rx") == 0) {
      keyboardInject(EVT_KEY1SHORT);
    }
    else if (strcmp(param, "scan") == 0) {
      keyboardInject(EVT_KEY1DOUBLE);
    }
    else if (strcmp(param, "spec") == 0) {
      keyboardInject(EVT_KEY1MID);
    }
    else if (strcmp(param, "wifi") == 0) {
      keyboardInject(EVT_KEY1LONG);
    }
    else if (strcmp(param, "rx2") == 0) {
      keyboardInject(EVT_KEY2SHORT);
    }
    else if (strcmp(param, "scan2") == 0) {
      keyboardInject(EVT_KEY2DOUBLE);
    }
    else if (strcmp(param, "spec2") == 0) {
      keyboardInject(EVT_KEY2MID);
    }
    else if (strcmp(param, "wifi2") == 0) {
      keyboardInject(EVT_KEY2LONG);
    }
    else if (strcmp(param, "rinex") == 0) {
      keyboardInject(EVT_RINEX);
    }
    else if (strcmp(param, "format") == 0) {
      keyboardInject(EVT_FORMAT);
    }
    else if (strcmp(param, "reboot") == 0) {
      ESP.restart();
    }
  }
  return "";
}

/* IDF httpd entry points  */
const char *httpdHandleQRGPost(void) { return handleQRGPost(); }
const char *httpdHandleWIFIPost(void) { return handleWIFIPost(); }
const char *httpdHandleConfigPost(void) { return handleConfigPost(); }
const char *httpdHandleControlPost(void) { return handleControlPost(); }

// will be removed. its now in data/upd.html (for GET; POST to update.html still handled here)
const char *createUpdateForm(boolean run) {
  char *ptr = message;
  strcpy(ptr, "<!DOCTYPE html><html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"update.html\" method=\"post\">");
  if (run) {
    strcat(ptr, "<p>Doing update, wait until reboot</p>");
  } else {
    sprintf(ptr + strlen(ptr), "<p>Currently installed: %s-%c%d</p>\n", version_id, FS_MAJOR + 'A' - 1, FS_MINOR);
    strcat(ptr, "<p>Available main: <iframe src=\"http://rdzsonde.org/main/update-info.html\" style=\"height:40px;width:400px\"></iframe><br>"
           "Available devel: <iframe src=\"http://rdzsonde.org/dev2/update-info.html\" style=\"height:40px;width:400px\"></iframe></p>");
    strcat(ptr, "<input type=\"submit\" name=\"main\" value=\"Main-Update\"></input><br><input type=\"submit\" name=\"dev\" value=\"Devel-Update\">");
    strcat(ptr, "<br><p>Note: If suffix is the same, update should work fully. If the number is different, update contains changes in the file system. A full re-flash is required to get all new features, but the update should not break anything. If the letter is different, a full re-flash is mandatory, update will not work</p>");
  }
  strcat(ptr, "</form></body></html>");
  LOG_D(TAG, "Update form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleUpdatePost(void) {
  Serial.println("Handling post request");
  bool updateFromURL = false;
  int params = formCount();
  for (int i = 0; i < params; i++) {
    const char *param = formNameAt(i);
    if (!param) continue;
    Serial.println(param);
    if (strcmp(param, "dev2") == 0) {
      Serial.println("equals devel");
      updatePrefix = updatePrefixD;
    }
    else if (strcmp(param, "main") == 0) {
      Serial.println("equals main");
      updatePrefix = updatePrefixM;
    }
    else if (localUpdates && strcmp(param, "local") == 0) {
      // Local updates permitted. Expect URL as url parameter...
      updateFromURL = true;
    }
    else if (updateFromURL && strcmp(param, "url") == 0) {
      // Note: strdup allocates memory that is never free'd.
      // Let's not care about this as we are going to reboot after the update anyway...
      const char *raw = formValueAt(i);
      if (!raw) continue;
      char localsrc[160];
      strncpy(localsrc, raw, sizeof(localsrc) - 1);
      localsrc[sizeof(localsrc) - 1] = '\0';
      char *pos;

      if ((pos = strstr(localsrc, "://")) != nullptr) {
        // strip off http://
        memmove(localsrc, pos + 3, strlen(pos + 3) + 1);
      }

      if ((pos = strchr(localsrc, '/')) != nullptr) {
        // see if there's a directory or updates are in the root
        updatePrefix = strdup(pos);
        *pos = '\0';
      } else {
         updatePrefix = strdup("/");
      }

      if ((pos = strchr(localsrc, ':')) != nullptr) {
        // extract port
        *pos = '\0';
        updatePort = atoi(pos + 1);
        updateHost = strdup(localsrc);
      } else {
        updateHost = strdup(localsrc);
      }
    }
  }
  LOG_I(TAG, "Updating: %s%s from %s:%d\n", updatePrefix, updateIno, updateHost, updatePort);
  enterMode(ST_UPDATE);
  return "";
}

const char *httpdHandleUpdatePost(void) { return handleUpdatePost(); }
const char *httpdCreateUpdateForm(int run) { return createUpdateForm(run ? true : false); }

const char *createKMLLive(const char *myIP) {
  char *ptr = message;

  strcpy(ptr, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><NetworkLink><name>loads dynamic.kml</name><Link><href>http://");
  strcat(ptr, myIP);
  strcat(ptr, "/dynamic.kml</href><refreshMode>onInterval</refreshMode><refreshInterval>10</refreshInterval></Link></NetworkLink></kml>");

  return message;
}

void addSondeStatusKML(char *ptr, int i)
{
  SondeInfo *s = &sonde.sondeList[i];

  if (!s->d.validID)
  {
    return;
  }

  sprintf(ptr + strlen(ptr), "<Placemark id=\"%s\"><name>%s</name><Point><altitudeMode>absolute</altitudeMode><coordinates>%.6f,%.6f,%.0f</coordinates></Point><description>%3.3f MHz, Type: %s, h=%.0fm</description></Placemark>",
          s->d.id, s->d.id,
          s->d.lon, s->d.lat, s->d.alt,
          s->freq, sondeTypeStr[sonde.realType(s)], s->d.alt);
}

const char *createKMLDynamic() {
  char *ptr = message;

  strcpy(ptr, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>");

  for (int i = 0; i < sonde.config.maxsonde; i++) {
    int snum = (i + sonde.currentSonde) % sonde.config.maxsonde;
    if (sonde.sondeList[snum].active) {
      addSondeStatusKML(ptr, snum);
    }
  }

  strcat(ptr, "</Document></kml>");

  return message;
}


const char *createGPX(int index) {
  Serial.println("\n\n\n********GPX request\n\n");
  char *ptr = message;
  if (index < 0 || index >= MAXSONDE) {
    return "ERROR";
  }
  SondeInfo *si = &sonde.sondeList[index];
  strcpy(si->d.id, "test");
  si->d.lat = 48; si->d.lon = 11; si->d.alt = 500;
  snprintf(ptr, 10240, "<?xml version='1.0' encoding='UTF-8'?>\n"
           "<gpx version=\"1.1\" creator=\"http://rdzsonde.local\" xmlns=\"http://www.topografix.com/GPX/1/1\" "
           "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
           "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
           "<metadata>"
           "<name>Sonde #%d (%s)</name>\n"
           "<author>rdzTTGOsonde</author>\n"
           "</metadata>\n"
           "<wpt lat=\"%f\" lon=\"%f\">\n  <ele>%f</ele>\n  <name>%s</name>\n  <sym>Radio Beacon</sym><type>Sonde</type>\n"
           "</wpt></gpx>\n", index, si->d.id, si->d.lat, si->d.lon, si->d.alt, si->d.id);
  Serial.println(message);
  return message;
}

const char *httpdCreateKMLLive(void) {
  return createKMLLive(sonde.ipaddr.c_str());
}

const char *httpdCreateKMLDynamic(void) {
  return createKMLDynamic();
}

const char *httpdCreateGPX(int index) {
  return createGPX(index);
}

int fetchWifiIndex(const char *id) {
  for (int i = 0; i < nNetworks; i++) {
    if (strcmp(id, networks[i].id.c_str()) == 0) {
      LOG_D(TAG, "Match for %s at %d\n", id, i);
      return i;
    }
    //LOG_D(TAG, "No match: '%s' vs '%s'\n", id, networks[i].id.c_str());
    const char *cfgid = networks[i].id.c_str();
    int len = strlen(cfgid);
    if (strlen(id) > len) len = strlen(id);
  }
  return -1;
}

const char *fetchWifiSSID(int i) {
  return networks[i].id.c_str();
}
const char *fetchWifiPw(int i) {
  return networks[i].pw.c_str();
}

const char *fetchWifiPw(const char *id) {
  for (int i = 0; i < nNetworks; i++) {
    //Serial.print("Comparing '");
    //Serial.print(id);
    //Serial.print("' and '");
    //Serial.print(networks[i].id.c_str());
    //Serial.println("'");
    if (strcmp(id, networks[i].id.c_str()) == 0) return networks[i].pw.c_str();
  }
  return NULL;
}

// ISR won't work for SPI transfer, so forget about the following approach
///// Also initialized timers for sx1278 handling with interruts
///// fastest mode currentily is 4800 bit/s, i.e. 600 bytes/sec
///// 64 byte FIFO will last for at most about 106 ms.
///// lets use a timer every 20ms to handle sx1278 FIFO input, that should be fine.
// Instead create a tast...

Ticker ledFlasher;

const char *getStateStr(int what) {
  if (what < 0 || what >= (sizeof(mainStateStr) / sizeof(const char *)))
    return "--";
  else
    return mainStateStr[what];
}

void sx1278Task(void *parameter) {
  /* new strategy:
      background tasks handles all interactions with sx1278.
      implementation is decoder specific.
      This task is a simple infinit loop that
       (a) initially and after frequency or mode change calls <decoder>.setup()
       (b) then repeatedly calls <decoder>.receive() which should
           (1) update data in the Sonde structure (additional updates may be done later in main loop/waitRXcomplete)
           (2) set output flag receiveResult (success/error/timeout and keybord events)

  */
  while (1) {
    if (rxtask.activate >= 128) {
      // activating sx1278 background task...
      LOG_I(TAG, "RXtask: start DECODER for sonde %d (was %s)\n", rxtask.activate & 0x7f, getStateStr(rxtask.mainState));
      rxtask.mainState = ST_DECODER;
      rxtask.currentSonde = rxtask.activate & 0x7F;
      sonde.setup();
    } else if (rxtask.activate != -1) {
      LOG_I(TAG, "RXtask: start %s (was %s)\n", getStateStr(rxtask.activate), getStateStr(rxtask.mainState));
      rxtask.mainState = rxtask.activate;
    }
    rxtask.activate = -1;
    /* only if mainState is ST_DECODER */
    if (rxtask.mainState != ST_DECODER) {
      delay(100);
      continue;
    }
    sonde.receive();
    delay(20);
  }
}


static unsigned long t_muted = (unsigned long)-1;

void toggleLedMute() {
  if(t_muted==-1) t_muted = millis(); else t_muted = -1;
}

void ledOffCallback() {
  digitalWrite(sonde.config.led_pout, LOW);
}
void flashLed(int ms) {
  if (sonde.config.led_pout >= 0) {
    if(t_muted != -1) {
      LOG_D(TAG, "Muted at %d\n", t_muted);
      // t_muted was set by key press to mute LED / buzzer
      if(millis()-t_muted < sonde.config.b2mute * 60000L) return;
      else t_muted = -1;
      LOG_D(TAG, "Unmuted\n");
    }
    Serial.println("Not muted");
    digitalWrite(sonde.config.led_pout, HIGH);
    ledFlasher.once_ms(ms, ledOffCallback);
  }
}

extern int initlevels[40];

#ifdef ESP_MEM_DEBUG
typedef void (*esp_alloc_failed_hook_t) (size_t size, uint32_t caps, const char * function_name);
extern esp_err_t heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t callback);

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
  printf("%s was called but failed to allocate %d bytes with 0x%X capabilities. \n", function_name, requested_size, caps);
}
#endif

static void enableLocalUpdates() {
  localUpdates = NULL;
  File local = LittleFS.open("/localupd.txt", "r");
  if(local && local.available()) {
    localUpdates = strdup(readLine(local).c_str());
  } 
  LOG_I(TAG, "Local update server for development: %s\n", localUpdates?localUpdates:"<disabled>");
}

void setup()
{
  char buf[12];

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Log.init();

  for (int i = 0; i < 39; i++) {
    int v = gpio_get_level((gpio_num_t)i);
    LOG_D(TAG, "%d:%d ", i, v);
  }
  LOG_I(TAG, "sizeof long is %d\n", sizeof(long));

#ifndef REMOVE_ALL_FOR_TESTING

  Serial.println("");
#ifdef ESP_MEM_DEBUG
  esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);
#endif
  axpSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(axpSemaphore);

  for (int i = 0; i < 39; i++) {
    LOG_D(TAG, "%d:%d ", i, initlevels[i]);
  }
  Serial.println(" (before setup)");
  sonde.defaultConfig();  // including autoconfiguration

  delay(1000);
  Serial.println("Initializing LittleFS");
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  Serial.println("Reading initial configuration");
  setupConfigData();    // configuration must be read first due to OLED ports!!!
  WiFi.setHostname(sonde.config.mdnsname);
  //WiFi.enableIPv6();

  startupPower();

  keyboardInit();

  disp.init();
  delay(100);
  Serial.println("Showing welcome display");
  disp.rdis->welcome();
  delay(3000);
  Serial.println("Clearing display");
  sonde.clearDisplay();

  setupWifiList();
  LOG_D(TAG, "before disp.initFromFile... layouts is %p\n", disp.layouts);
  disp.initFromFile(sonde.config.screenfile);
  LOG_D(TAG, "disp.initFromFile... layouts is %p\n", disp.layouts);


#if 0
  if (1) { //testing cyd
    digitalWrite(sonde.config.sx1278_ss, HIGH);
    pinMode(sonde.config.sx1278_ss, OUTPUT);

    SPI.begin(18, 19, 23, -1);    // CKL, MISO, MOSI, CS)
  } else 
  if (sonde.config.type == TYPE_M5_CORE2) {
    // Core2 uses Pin 38 for MISO
    SPI.begin(18, 38, 23, -1);
  } else if (sonde.config.type == TYPE_M5_CORE) {
    SPI.begin(18, 19, 23, -1);
    // GPIO26 is reset
    pinMode(26, OUTPUT);
    digitalWrite(26, 0);
    delay(5);
    digitalWrite(26, 1);
    delay(5);
  } else {
    SPI.begin();
  }
  //Set most significant bit first
  SPI.setBitOrder(MSBFIRST);
  //Divide the clock frequency
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  //Set data mode
  SPI.setDataMode(SPI_MODE0);
#endif

  startupRadio();

  Serial.println("Setup finished");
  Serial.println();
  // int returnValue = pthread_create(&wifithread, NULL, wifiloop, (void *)0);

  //  if (returnValue) {
  //     Serial.println("An error has occurred");
  //  }
  //   xTaskCreate(mainloop, "MainServer", 10240, NULL, 10, NULL);


  // == setup default channel list if qrg.txt read fails =========== //
  sonde.clearSonde();
  setupChannelList();
  /// not here, done by sonde.setup(): rs41.setup();
  // == setup default channel list if qrg.txt read fails =========== //
#ifndef DISABLE_SX1278
  xTaskCreate( sx1278Task, "sx1278Task",
               10000, /* stack size */
               NULL, /* paramter */
               1, /* priority */
               NULL);  /* task handle*/
#endif
  sonde.setup();
  fixedToPosInfo();
  initGPS();
#if FEATURE_APRS
  connAPRS.init();
#endif
#if FEATURE_SDCARD
  connSDCard.init();
#endif

  enableLocalUpdates();   // check if local updates from other servers is allowed

  WiFi.onEvent(WiFiEvent);
  getKeyPressEvent();    // clear key buffer

#else
/// DEBUG ONLY
  WiFi.begin("Dinosauro", "03071975");
  while(WiFi.status() != WL_CONNECTED) { delay(500); Serial.print(":"); }
  Serial.println("... WiFi is connected!\n");
  defaultUserLevel = getDefaultAuthLevel();
  httpd_set_allowed_cb(checkAllowed);
  httpd_set_after_write_cb([](const char *name) {
    if (name && strncmp(name, "screens", 7) == 0) {
      forceReloadScreenConfig = true;
    }
  });
  httpd_start_server();
  sonde.config.sd.cs = 13;
  sonde.config.sd.clk = 14;
  sonde.config.sd.miso = 2;
  sonde.config.sd.mosi = 15;
  connSDCard.init();
#endif
}

void enterMode(int mode) {
  LOG_D(TAG, "enterMode(%d)\n", mode);
  // Backround RX task should only be active in mode ST_DECODER for now
  // (future changes might use RX background task for spectrum display as well)
  if (mode != ST_DECODER) {
    rxtask.activate = mode;
    while (rxtask.activate == mode) {
      delay(10);  // until cleared by RXtask -- rx task is deactivated
    }
  }
  mainState = (MainState)mode;
  if (mainState == ST_SPECTRUM) {
    Serial.println("Entering ST_SPECTRUM mode");
    sonde.clearDisplay();
    disp.rdis->setFont(FONT_SMALL);
    specTimer = millis();
    //scanner.init();
  } else if (mainState == ST_WIFISCAN || mainState == ST_RINEX_UPDATE || mainState == ST_FORMAT_SD) {
    sonde.clearDisplay();
  }

  if (mode == ST_DECODER) {
    // trigger activation of background task
    // currentSonde should be set before enterMode()
    rxtask.activate = ACT_SONDE(sonde.currentSonde);
    //
    Serial.println("clearing and updating display");
    sonde.clearDisplay();
    sonde.updateDisplay();
  }
  printf("enterMode ok\n");
}

static char text[40];
static const char *action2text(uint8_t action) {
  if (action == ACT_DISPLAY_DEFAULT) return "Default Display";
  if (action == ACT_DISPLAY_SPECTRUM) return "Spectrum Display";
  if (action == ACT_DISPLAY_WIFI) return "Wifi Scan Display";
  if (action == ACT_NEXTSONDE) return "Go to next sonde";
  if (action == ACT_PREVSONDE) return "presonde (not implemented)";
  if (action == ACT_RINEX_UPDATE) return "update RINEX eph data";
  if (action == ACT_FORMAT_SD) return "format SD card";
  if (action == ACT_NONE) return "none";
  if (action >= 128) {
    snprintf(text, 40, "Sonde=%d", action & 127);
  } else {
    snprintf(text, 40, "Display=%d", action);
  }
  return text;
}

#define RDZ_DATA_LEN 128
static char rdzData[RDZ_DATA_LEN];
static int rdzDataPos = 0;

void loopDecoder() {
  // sonde knows the current type and frequency, and delegates to the right decoder
  uint16_t res = sonde.waitRXcomplete();
  int action;
  //LOG_D(TAG, "waitRX result is %x\n", (int)res);
  action = (int)(res >> 8);
  // TODO: update displayed sonde?

#if 0
  static int i = 0;
  if (i++ > 20) {
    i = 0;
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    // Requires serial speed 921600, otherweise interrupt wdt will occur
    heap_caps_dump(MALLOC_CAP_8BIT);
  }
#endif

  if (action != ACT_NONE) {
    int newact = sonde.updateState(action);
    LOG_I(TAG, "loopDecoder: action %02x (%s) => %d  [current: main=%d, rxtask=%d]\n", action, action2text(action), newact, sonde.currentSonde, rxtask.currentSonde);
    action = newact;
    if (action != 255) {
      if (action == ACT_DISPLAY_SPECTRUM) {
        enterMode(ST_SPECTRUM);
        return;
      }
      else if (action == ACT_DISPLAY_WIFI) {
        enterMode(ST_WIFISCAN);
        return;
      }
#if FEATURE_RS92
      else if (action == ACT_RINEX_UPDATE) {
        enterMode(ST_RINEX_UPDATE);
        return;
      }
#endif
      else if (action == ACT_FORMAT_SD) {
        enterMode(ST_FORMAT_SD);
        return;
      }
    }
    // If action is to change to a different frequency, notify clients (MQTT and UDP-json for now)
    // TODO: do the same for TCP-json (rdzjson)
    if(ACT_IS_FREQ_CHANGE(action)) {
#if FEATURE_SONDESEEKER
      connSondeseeker.updateQRG(sonde.currentSonde);
#endif
#if FEATURE_MQTT
      connMQTT.updateQRG(sonde.currentSonde);
#endif
    }
  }

  if (rdzserver.hasClient()) {
    Serial.println("TCP JSON socket: new connection");
    rdzclient.stop();
    rdzclient = rdzserver.accept();
  }
  if (rdzclient.available()) {
    Serial.print("RDZ JSON socket: received ");
    while (rdzclient.available()) {
      char c = (char)rdzclient.read();
      Serial.print(c);
      if (c == '\n' || c == '}' || rdzDataPos >= RDZ_DATA_LEN) {
        // parse GPS position from phone
        rdzData[rdzDataPos] = c;
        if (rdzDataPos > 2) parseGpsJson(rdzData, rdzDataPos + 1);
        rdzDataPos = 0;
      }
      else {
        rdzData[rdzDataPos++] = c;
      }
    }
    Serial.println("");
  }

  // wifi active and good packet received => send packet
  SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
  if ((res & 0xff) == 0 && connected) {
    //Send a packet with position information
    // first check if ID and position lat+lonis ok

    if (s->d.validID && ((s->d.validPos & 0x03) == 0x03)) {
#if FEATURE_APRS
      connAPRS.updateSonde(s);
#endif
#if FEATURE_CHASEMAPPER
      connChasemapper.updateSonde( s );
#endif
#if FEATURE_SONDESEEKER
  connSondeseeker.updateSonde( s );
#endif
    }
#if FEATURE_SONDEHUB
    connSondehub.updateSonde( s );   // invoke sh_send_data....
#endif

#if FEATURE_MQTT
    connMQTT.updateSonde( s );      // send to MQTT if enabled
#endif
#if FEATURE_SDCARD
    connSDCard.updateSonde(s);
#endif
  } else {
#if FEATURE_SONDEHUB
    connSondehub.updateSonde( NULL );
#endif
  }

  // Send own position periodically
#if FEATURE_MQTT
  connMQTT.updateStation( NULL );
#endif
#if FEATURE_APRS
  connAPRS.updateStation( NULL );
#endif
#if FEATURE_SONDEHUB
  connSondehub.updateStation( NULL );
#endif
#if FEATURE_SDCARD
  connSDCard.updateStation( NULL );
#endif
  // always send data, even if not valid....
  if (rdzclient.connected()) {
    Serial.println("Sending position via TCP as rdzJSON");
    char raw[1024];
    char gps[128];
    const char *typestr = s->d.typestr;
    if (*typestr == 0) typestr = sondeTypeStr[sonde.realType(s)];
    // TODO: only if GPS is valid...
    if (gpsPos.valid) {
      snprintf(gps, 128, ", \"gpslat\": %f,"
               "\"gpslon\": %f,"
               "\"gpsalt\": %d,"
               "\"gpsacc\": %d,"
               "\"gpsdir\": %d",
               gpsPos.lat, gpsPos.lon, gpsPos.alt, gpsPos.accuracy, gpsPos.course);
    } else {
      *gps = 0;
    }
    //
    raw[0] = '{';
    // Use same JSON format as for MQTT and HTML map........
    sonde2json(raw + 1, 1023, s);
    sprintf(raw + strlen(raw),
            ",\"active\":%d"
            ",\"validId\":%d"
            ",\"validPos\":%d"
            " %s}\n",
            (int)s->active,
            s->d.validID,
            s->d.validPos,
            gps);
    int len = strlen(raw);


    //Serial.println("Writing rdzclient...");
    if (len > 1024) len = 1024;
    int wlen = rdzclient.write(raw, len);
    if (wlen != len) {
      Serial.println("Writing rdzClient not OK, closing connection");
      rdzclient.stop();
    }
    //Serial.println("Writing rdzclient OK");
  }
  LOG_D(TAG, "updateDisplay started\n");
  sonde.dispsavectlOFF( (res & 0xff) == 0 );  // handle screen saver (disp auto off)
  if (forceReloadScreenConfig) {
    disp.initFromFile(sonde.config.screenfile);
    sonde.clearDisplay();
    forceReloadScreenConfig = false;
  }
  int t = millis();
  sonde.updateDisplay();
  LOG_D(TAG, "updateDisplay done (after %d ms)\n", (int)(millis() - t));
}

void setCurrentDisplay(int value) {
  LOG_D(TAG, "setCurrentDisplay: setting index %d, entry %d\n", value, sonde.config.display[value]);
  currentDisplay = sonde.config.display[value];
}

void loopSpectrum() {
  int marker = 0;
  char buf[10];
  uint8_t dispw, disph, dispxs, dispys;
  disp.rdis->getDispSize(&disph, &dispw, &dispxs, &dispys);

  switch (getKeyPressEvent()) {
    case EVT_KEY1SHORT: /* move selection of peak, TODO */
      sonde.nextConfig(); // TODO: Should be set specific frequency
      enterMode(ST_DECODER);
      return;
    case EVT_KEY1MID: /* restart, TODO */ break;
    case EVT_KEY1LONG:
      Serial.println("loopSpectrum: KP_LONG");
      enterMode(ST_WIFISCAN);
      return;
    case EVT_KEY1DOUBLE:
      setCurrentDisplay(0);
      enterMode(ST_DECODER);
      return;
    default: break;
  }

  scanner.scan();
  scanner.plotResult();

  /*
    if(globalClient != NULL && globalClient->status() == WS_CONNECTED){
        String randomNumber = String(random(0,20));
        globalClient->text(randomNumber);
     }
  */

  if (sonde.config.spectrum > 0) {
    int remaining = sonde.config.spectrum - (millis() - specTimer) / 1000;
    LOG_D(TAG, "config.spectrum:%d  specTimer:%ld millis:%ld remaining:%d\n", sonde.config.spectrum, specTimer, millis(), remaining);
    if (sonde.config.marker != 0) {
      marker = 1;
    }
    snprintf(buf, 10, "%d Sec.", remaining);
    disp.rdis->drawString(0, dispys <= 1 ? (1 + marker) : (dispys + 1)*marker, buf);
    if (remaining <= 0) {
      setCurrentDisplay(0);
      enterMode(ST_DECODER);
    }
  }
}

void startSpectrumDisplay() {
  sonde.clearDisplay();
  disp.rdis->setFont(FONT_SMALL);
  disp.rdis->drawString(0, 0, "Spectrum Scan...");
  delay(500);
  enterMode(ST_SPECTRUM);
}

const char *translateEncryptionType(wifi_auth_mode_t encryptionType) {
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
    default:
      return "";
  }
}

// in core.h
//enum t_wifi_state { WIFI_DISABLED, WIFI_SCAN, WIFI_CONNECT, WIFI_CONNECT_GOT_DISCONNECT, WIFI_CONNECTED, WIFI_APMODE };

t_wifi_state wifi_state = WIFI_DISABLED;

uint32_t netup_time;

void enableNetwork(bool enable) {
  if (enable) {
    netup_time = esp_timer_get_time() / 1000000;
    MDNS.begin(sonde.config.mdnsname);
    defaultUserLevel = getDefaultAuthLevel();
    httpd_set_allowed_cb(checkAllowed);
    httpd_set_after_write_cb([](const char *name) {
      if (name && strncmp(name, "screens", 7) == 0) {
        forceReloadScreenConfig = true;
      }
    });
    httpd_start_server();
    udp.begin(WiFi.localIP(), LOCALUDPPORT);
    MDNS.addService("http", "tcp", 80);
    // => moved to conn-aprs  MDNS.addService("kiss-tnc", "tcp", 14580);
    MDNS.addService("jsonrdz", "tcp", 14570);
    //if (sonde.config.kisstnc.active) {
    //   tncserver.begin();
    rdzserver.begin();
    //}
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    connected = true;
#if FEATURE_MQTT
    connMQTT.netsetup();
#endif
#if FEATURE_SONDEHUB
    connSondehub.netsetup();
#endif
#if FEATURE_APRS
    connAPRS.netsetup();
#endif
  } else {
    httpd_stop_server();
    MDNS.end();
#if FEATURE_MQTT
    connMQTT.netshutdown();
#endif
#if FEATURE_SONDEHUB
    connSondehub.netshutdown();
#endif
#if FEATURE_APRS
    connAPRS.netshutdown();
#endif
    connected = false;
  }

  Serial.println("enableNetwork done");
}


// Events used only for debug output right now
void WiFiEvent(WiFiEvent_t event)
{
  LOG_D(TAG, "[WiFi-event] event: %d\n", event);

  switch (event) {
    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi interface ready");
      break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi client started");
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      Serial.println("WiFi clients stopped");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to access point");
      if (wifi_state == WIFI_CONNECT_GOT_DISCONNECT) {
        /* Connection came back on its own; don't run the disconnect+retry path */
        wifi_state = WIFI_CONNECT;
      }
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      if (wifi_state == WIFI_CONNECT) {
        // If we get a disconnect event while waiting for connection (as I do sometimes with my FritzBox),
        // just start from scratch with WiFi scan
        //wifi_state = WIFI_DISABLED;
        //%WiFi.disconnect(true);
	// lets try somethign else:
	/// This is not a good idea, lets check what if we do nothing:   WiFi.reconnect();
	Serial.println("WiFi State was connect");
        wifi_state = WIFI_CONNECT_GOT_DISCONNECT;
	break;
      }
      LOG_D(TAG, "Turning off (state is %d)\n", wifi_state);
      WiFi.mode(WIFI_MODE_NULL);
      break;
    case ARDUINO_EVENT_WIFI_OFF:
      Serial.println("WiFi is OFF");
      /* So let's retry? depending on mode... for now testing for mode 4*/
      if(sonde.config.wifi == 4) { wifiConnectDirect(1); }
      break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      Serial.println("Authentication mode of access point has changed");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      WiFi.STA.dnsIP(2, 0x08080808);
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("Lost IP address and IP address is reset to 0");
      break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
      Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
      Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
      Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_PIN:
      Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println("WiFi access point started");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println("WiFi access point  stopped");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("Client connected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println("Client disconnected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.println("Assigned IP address to client");
      break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
      Serial.println("Received probe request");
      break;
    case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
      Serial.println("AP IPv6 is preferred");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
      Serial.println("STA obtained IPv6");
      Serial.println(WiFi.STA.globalIPv6());
      Serial.println(WiFi.STA.linkLocalIPv6());
      break;
    case ARDUINO_EVENT_ETH_GOT_IP6:
      Serial.println("Ethernet IPv6 is preferred");
      break;
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet started");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address");
      break;
    default:
      break;
  }
}


void wifiConnect(int16_t res) {
  LOG_I(TAG, "WiFi scan result: found %d networks\n", res);

  // pick best network
  int bestEntry = -1;
  int bestRSSI = INT_MIN;
  uint8_t bestBSSID[6];
  int32_t bestChannel = 0;

  for (int8_t i = 0; i < res; i++) {
    String ssid_scan;
    int32_t rssi_scan;
    uint8_t sec_scan;
    uint8_t* BSSID_scan;
    int32_t chan_scan;
    WiFi.getNetworkInfo(i, ssid_scan, sec_scan, rssi_scan, BSSID_scan, chan_scan);
    int networkEntry = fetchWifiIndex(ssid_scan.c_str());
    if (networkEntry < 0) continue;
    if (rssi_scan <= bestRSSI) continue;
    bestEntry = networkEntry;
    bestRSSI = rssi_scan;
    bestChannel = chan_scan;
    memcpy((void*) &bestBSSID, (void*) BSSID_scan, sizeof(bestBSSID));
  }
  WiFi.scanDelete();
  if (bestEntry >= 0) {
    LOG_D(TAG, "WiFi Connecting BSSID: %02X:%02X:%02X:%02X:%02X:%02X SSID: %s PW %s Channel: %d (RSSI %d)\n",
      bestBSSID[0], bestBSSID[1], bestBSSID[2], bestBSSID[3], bestBSSID[4], bestBSSID[5],
      fetchWifiSSID(bestEntry), fetchWifiPw(bestEntry), bestChannel, bestRSSI);
    wifi_state = WIFI_CONNECT;
    WiFi.begin(fetchWifiSSID(bestEntry), fetchWifiPw(bestEntry), bestChannel, bestBSSID);
  } else {
    // rescan
    // wifiStart();
    WiFi.disconnect(true);
    wifi_state = WIFI_DISABLED;
  }
}

void wifiConnectDirect(int16_t index) {
  Serial.println("AP mode 4: trying direct reconnect");
  wifi_state = WIFI_CONNECT;
  WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
}

static int wifi_cto;

void loopWifiBackground() {
  LOG_D(TAG, "WifiBackground: state %d\n", wifi_state);
  // handle Wifi station mode in background
  if (sonde.config.wifi == 0 || sonde.config.wifi == 2) return; // nothing to do if disabled or access point mode

  if (wifi_state == WIFI_DISABLED) {  // stopped => start can
    if (sonde.config.wifi == 4) {  // direct connect to first network, supports hidden SSID
       wifiConnectDirect(1);
       wifi_cto = 0;
    } else {
      Serial.println("WiFi start scan");
      wifi_state = WIFI_SCAN;
      WiFi.scanNetworks(true); // scan in async mode
    }
  } else if (wifi_state == WIFI_SCAN) {
    int16_t res = WiFi.scanComplete();
    if (res == 0 || res == WIFI_SCAN_FAILED) {
      // retry
      Serial.println("WiFi restart scan");
      WiFi.disconnect(true);
      wifi_state = WIFI_DISABLED;
      return;
    }
    if (res == WIFI_SCAN_RUNNING) {
      return;
    }
    // Scan finished, try to connect
    wifiConnect(res);
    wifi_cto = 0;
  } else if (wifi_state == WIFI_CONNECT) {
    wifi_cto++;
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Wifi is connected\n");
      wifi_state = WIFI_CONNECTED;
      // update IP in display
      String localIPstr = WiFi.localIP().toString();
      LOG_I(TAG, "IP is %s\n", localIPstr.c_str());
      sonde.setIP(localIPstr.c_str(), false);
      sonde.updateDisplayIP();
      enableNetwork(true);
    }
    if (wifi_cto > 20) { // failed, restart scanning
      wifi_state = WIFI_DISABLED;
      WiFi.disconnect(true);
    }
  } else if (wifi_state == WIFI_CONNECTED) {
    //LOG_D(TAG, "status: %d\n", ((WiFiSTAClass)WiFi).status());
    if (WiFi.status() != WL_CONNECTED) {
      sonde.setIP("", false);
      sonde.updateDisplayIP();

      wifi_state = WIFI_DISABLED;  // restart scan
      enableNetwork(false);
      WiFi.disconnect(true);
    } //else Serial.println("WiFi still connected");
  }
}

void startAP() {
  Serial.println("Activating access point mode");
  wifi_state = WIFI_APMODE;
  WiFi.softAP(networks[0].id.c_str(), networks[0].pw.c_str());

  Serial.println("Wait 100 ms for AP_START...");
  delay(100);
  Serial.println(WiFi.softAPConfig(IPAddress (192, 168, 4, 1), IPAddress (0, 0, 0, 0), IPAddress (255, 255, 255, 0)) ? "Ready" : "Failed!");

  IPAddress myIP = WiFi.softAPIP();
  String myIPstr = myIP.toString();
  sonde.setIP(myIPstr.c_str(), true);
  sonde.updateDisplayIP();
  // enableNetwork(true); done later in WifiLoop.
}

void initialMode() {
  if (sonde.config.touch_thresh == 0) {
    enterMode(ST_TOUCHCALIB);
    return;
  }
  if (sonde.config.spectrum != -1) {    // enable Spectrum in config.txt: spectrum=number_of_seconds
    startSpectrumDisplay();
  } else {
    setCurrentDisplay(0);
    enterMode(ST_DECODER);
  }
}

void loopTouchCalib() {
  uint8_t dispw, disph, dispxs, dispys;
  disp.rdis->clear();
  disp.rdis->getDispSize(&disph, &dispw, &dispxs, &dispys);
  char num[10];

  while (1) {
    int t1 = touchRead(sonde.config.button_pin & 0x7f);
    int t2 = touchRead(sonde.config.button2_pin & 0x7f);
    disp.rdis->setFont(FONT_LARGE);
    disp.rdis->drawString(0, 0, "Touch calib.");
    disp.rdis->drawString(0, 3 * dispys, "Touch1: ");
    snprintf(num, 10, "%d  ", t1);
    disp.rdis->drawString(8 * dispxs, 3 * dispys, num);
    disp.rdis->drawString(0, 6 * dispys, "Touch2: ");
    snprintf(num, 10, "%d  ", t2);
    disp.rdis->drawString(8 * dispxs, 6 * dispys, num);
    delay(300);
  }
}

// Wifi modes
// 0: disabled. directly start initial mode (spectrum or scanner)
// 1: Station mode, new version: start with synchronous WiFi scan, then
//    - if button was pressed, switch to AP mode
//    - if connect successful, all good
//    - otherwise, continue with station mode in background
// 2: access point mode (wait for clients in background)
// 3: traditional sync. WifiScan. Tries to connect to a network, in case of failure activates AP.
// 4: Station mode/hidden AP: same as 1, but instead of scan, just call espressif method to connect (will connect to hidden AP as well
#define MAXWIFIDELAY 40
static const char* _scan[2] = {"/", "\\"};
void loopWifiScan() {
  getKeyPressEvent(); // Clear any old events
  enableNetwork(false);
  WiFi.disconnect(true);
  wifi_state = WIFI_DISABLED;
  disp.rdis->setFont(FONT_SMALL);
  uint8_t dispw, disph, dispxs, dispys;
  disp.rdis->getDispSize(&disph, &dispw, &dispys, &dispxs);
  int dwidth = dispw / dispxs;
  int lastl = (disph / dispys - 2) * dispys;
  int cnt = 0;
  char abort = 0; // abort on keypress
  int net_index = -1;

  switch(sonde.config.wifi) {
  case 0:  // no WiFi
    initialMode();
    return;
  case 2:  // AP mode, setup in background
    startAP();
    enableNetwork(true);
    initialMode();
    return;
  case 4:  // direct connect without scan, only first item in network list
    // Mode STN/DIRECT[4]: Connect directly (supports hidden AP)
    {
      disp.rdis->drawString(0, 0, "WiFi Connect...");
      disp.rdis->drawString(0, dispys * 2, fetchWifiSSID(1));
      wifiConnectDirect(1);
    }
    break;
  case 1:  // STATION mode (continue in BG if no connection)
  case 3:  // old AUTO mode (change to AP if no connection)
    // Mode STATION[1] or SETUP[3]: Scan for networks;
    disp.rdis->drawString(0, 0, "WiFi Scan...");
    int line = 0;
    WiFi.mode(WIFI_STA);
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      disp.rdis->drawString(0, dispys * (1 + line), ssid.c_str(), dwidth);
      line = (line + 1) % (disph / dispys);
      String mac = WiFi.BSSIDstr(i);
      const char *encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
      LOG_I(TAG, "Network %s: RSSI %d, MAC %s, enc: %s\n", ssid.c_str(), WiFi.RSSI(i), mac.c_str(), encryptionTypeDescription);
      int curidx = fetchWifiIndex(ssid.c_str());
      if (curidx >= 0 && net_index == -1) {
        net_index = curidx;
        LOG_I(TAG, "Match found at scan entry %d, config network %d\n", i, net_index);
      }
    }
    if (net_index >= 0) { // some network was found
      Serial.print("Connecting to: "); Serial.print(fetchWifiSSID(net_index));
      Serial.print(" with password "); Serial.println(fetchWifiPw(net_index));

      disp.rdis->drawString(0, lastl, "Conn:");
      disp.rdis->drawString(6 * dispxs, lastl, fetchWifiSSID(net_index));
      // TODO: wifi_state is used inconsistently
      wifi_state = WIFI_CONNECT;
      WiFi.begin(fetchWifiSSID(net_index), fetchWifiPw(net_index));
    } else {
      abort = 2;  // no network found in scan => abort right away
    }
  }
  while (WiFi.status() != WL_CONNECTED && cnt < MAXWIFIDELAY && !abort)  {
    delay(500);
    if(wifi_state == WIFI_CONNECT_GOT_DISCONNECT) {
      if(WiFi.status() == WL_CONNECTED) { /* connection came back, avoid tearing it down */
        wifi_state = WIFI_CONNECT;
      } else {
        int connectIndex = (sonde.config.wifi == 4) ? 1 : net_index;
        WiFi.disconnect(true);   // Disconnect and wait, allowing full dissassociation, then retry
        Serial.print("_d_");
        delay(2000); // 2sec delay
        handlePMUirq();
        abort = (getKeyPressEvent() != EVT_NONE);
        if(abort) break;
        WiFi.begin(fetchWifiSSID(connectIndex), fetchWifiPw(connectIndex));
        wifi_state = WIFI_CONNECT;
      }
    }
    Serial.print(".");
    disp.rdis->drawString(15 * dispxs, lastl + dispys, _scan[cnt & 1]);
    cnt++;
    handlePMUirq();    // Needed to react to PMU chip button
    abort = (getKeyPressEvent() != EVT_NONE);
  }
  // We reach this point for mode 1, 3, and 4
  // If connected (in any case) => all good, download eph if needed, all up and running
  // Otherwise, If key was pressed, switch to AP mode
  // Otherwise, if mode is 3 (old AUTO), switch to AP mode
  // Otherwise, no network yet, keep trying to activate network in background (loopWiFiBackground)
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected\nIP address:");
    String localIPstr = WiFi.localIP().toString();
    Serial.println(localIPstr);
    sonde.setIP(localIPstr.c_str(), false);
    sonde.updateDisplayIP();
    wifi_state = WIFI_CONNECTED;
#if FEATURE_RS92
    bool hasRS92 = false;
    for (int i = 0; i < MAXSONDE; i++) {
      if (sonde.sondeList[i].type == STYPE_RS92) hasRS92 = true;
    }
    if (hasRS92) {
      geteph();
      if (ephstate == EPH_PENDING) ephstate = EPH_ERROR;
      get_eph("/brdc");
    }
#endif
    enableNetwork(true);
    delay(3000);
  }
  else if(sonde.config.wifi == 3 || abort==1 ) {
    WiFi.disconnect(true);
    delay(1000);
    startAP();
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    disp.rdis->drawString(0, lastl, "AP:             ");
    disp.rdis->drawString(6 * dispxs, lastl + 1, networks[0].id.c_str());
    enableNetwork(true);
    delay(3000);
  }
  initialMode();
}

#if FEATURE_RS92
// Rinex update...
void execRinexUpdate() {
   Serial.println("Fetching update for RINEX data...\n");
   geteph();
   Serial.println("Reading RINEX data...\n");
   if (ephstate == EPH_PENDING) ephstate = EPH_ERROR;
   get_eph("/brdc");
   setCurrentDisplay(0);
   enterMode(ST_DECODER);
}
#endif

#if FEATURE_SDCARD
void execFormatSD() {
   Serial.println("format SD card\n");
   if ( ISOLED(sonde.config) ) {
       disp.rdis->setFont(FONT_SMALL);
   } else {
       disp.rdis->setFont(5);
   }
   disp.rdis->drawString(0, 0, "Format SD card");

   connSDCard.format();
   setCurrentDisplay(0);
   enterMode(ST_DECODER);
}
#endif

/// Testing OTA Updates
/// somewhat based on Arduino's AWS_S3_OTA_Update
// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}


// OTA Logic
void execOTA() {
  int contentLength = 0;
  bool isValidContentType = false;
  sonde.clearDisplay();
  uint8_t dispxs, dispys;
  Serial.printf("Updater connecting to: %s:%d%s\n", updateHost, updatePort, updatePrefix);

  if ( ISOLED(sonde.config) ) {
    disp.rdis->setFont(FONT_SMALL);
    dispxs = dispys = 1;
    char uh[17];
    strncpy(uh, updateHost, 17);
    uh[16] = 0;
    disp.rdis->drawString(0, 0, uh);
  } else {
    disp.rdis->setFont(5);
    dispxs = 18;
    dispys = 20;
    disp.rdis->drawString(0, 0, updateHost);
  }

  // Connect to Update host
  if (!client.connect(updateHost, updatePort)) {
    LOG_E(TAG, "Connection to %s:%d for fs update failed\n", updateHost, updatePort);
    enterMode(ST_DECODER);
  }

  // First, try update file system
  LOG_I(TAG, "Fetching fs update from '%s:%d' '%s' '%s'\n", updateHost, updatePort, updatePrefix, updateFs);
  disp.rdis->drawString(0, 1 * dispys, "Fetching fs...");
  client.printf("GET %s%s HTTP/1.1\r\n"
                "Host: %s:%d\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n", updatePrefix, updateFs, updateHost, updatePort);
  // see if we get some data....

  int type = 0;
  int res = fetchHTTPheader(&type);
  if (res < 0) {
    ; // no-op
  } else {
    // process data...
    while (client.available()) {
      // get header...
      char fn[128];
      fn[0] = '/';
      client.readBytesUntil('\n', fn + 1, 128);
      char *sz = strchr(fn, ' ');
      if (!sz) {
        client.stop();
        enterMode(ST_DECODER);
        return;
      }
      *sz = 0;
      int len = atoi(sz + 1);
      LOG_I(TAG, "Updating file %s (%d bytes)\n", fn, len);
      char fnstr[17];
      memset(fnstr, ' ', 16);
      strncpy(fnstr, fn, 16);
      fnstr[16] = 0;
      disp.rdis->drawString(0, 2 * dispys, fnstr);
      File f = LittleFS.open(fn, FILE_WRITE);
      // read sz bytes........
      while (len > 0) {
        unsigned char buf[1024];
        int r = client.read(buf, len > 1024 ? 1024 : len);
        if (r == -1) {
          client.stop();
          enterMode(ST_DECODER);
          return;
        }
        f.write(buf, r);
        len -= r;
      }
    }
    client.stop();
  }

  // Connect to Update host
  if (!client.connect(updateHost, updatePort)) {
    LOG_E(TAG, "Connection to %s:%d for app update failed\n", updateHost, updatePort);
    enterMode(ST_DECODER);
    return;
  }

  // Connection succeeded, fecthing the bin
  LOG_I(TAG, "Fetching bin update from '%s:%d' '%s' '%s'", updateHost, updatePort, updatePrefix, updateIno);
  disp.rdis->drawString(0, 3 * dispys, "Fetching update");

  // Get the contents of the bin file
  client.printf("GET %s%s HTTP/1.1\r\n"
                "Host: %s:%d\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n",
                updatePrefix, updateIno, updateHost, updatePort);

  // Check what is being sent
  //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
  //                 "Host: " + host + "\r\n" +
  //                 "Cache-Control: no-cache\r\n" +
  //                 "Connection: close\r\n\r\n");

  int validType = 0;
  contentLength = fetchHTTPheader( &validType );
  if (validType == 1) isValidContentType = true;

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.printf("contentLength : %d, isValidContentType : %d\n",contentLength, isValidContentType);
  disp.rdis->drawString(0, 4 * dispys, "Len: ");
  String cls = String(contentLength);
  disp.rdis->drawString(5 * dispxs, 4 * dispys, cls.c_str());

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      disp.rdis->drawString(0, 5 * dispys, "Starting update");
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
      }

      if (Update.end()) {
        Serial.println("OTA done!");
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting.");
          disp.rdis->drawString(0, 7 * dispys, "Rebooting....");
          delay(1000);
          ESP.restart();
        } else {
          Serial.println("Update not finished? Something went wrong!");
        }
      } else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
      client.clear();
    }
  } else {
    Serial.println("There was no content in the response");
    client.clear();
  }
  // Back to some normal state
  enterMode(ST_DECODER);
}

int fetchHTTPheader(int *validType) {
  int contentLength = -1;
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println("Client Timeout !");
      client.stop();
      return -1;
    }
  }
  // Once the response is available, check stuff

  /*
     Response Structure
      HTTP/1.1 200 OK
      x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
      x-amz-request-id: 2D56B47560B764EC
      Date: Wed, 14 Jun 2017 03:33:59 GMT
      Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
      ETag: "d2afebbaaebc38cd669ce36727152af9"
      Accept-Ranges: bytes
      Content-Type: application/octet-stream
      Content-Length: 357280
      Server: AmazonS3

      {{BIN FILE CONTENTS}}

  */
  while (client.available()) {
    // read line till \n
    String line = client.readStringUntil('\n');
    // remove space, to check if the line is end of headers
    line.trim();

    // if the the line is empty,
    // this is end of headers
    // break the while and feed the
    // remaining `client` to the
    // Update.writeStream();
    if (!line.length()) {
      //headers ended
      break; // and get the OTA started
    }

    // Check if the HTTP Response is 200
    // else break and Exit Update
    if (line.startsWith("HTTP/1.1")) {
      if (line.indexOf("200") < 0) {
        Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
        return -1;
      }
    }

    // extract headers here
    // Start with content length
    static const char *HEADER_CL = "Content-Length: ";
    if (strncasecmp( line.c_str(), HEADER_CL, strlen(HEADER_CL) ) == 0 ) {
      contentLength = atoi( line.c_str() + strlen(HEADER_CL) );
      Serial.println("Got " + String(contentLength) + " bytes from server");
    }

    // Next, the content type
    static const char *HEADER_CT = "Content-Type: ";
    if (strncasecmp( line.c_str(), HEADER_CT, strlen(HEADER_CT) ) == 0 ) {
      const char *contentType = line.c_str() + strlen(HEADER_CT);
      LOG_I(TAG, "Content type: %s\n", contentType);
      if (strcmp(contentType, "application/octet-stream")==0) {
        if (validType) *validType = 1;
      }
    }
  }
  return contentLength;
}



void loop() {
  LOG_I(TAG, "Running loop in state %d [currentDisp:%d, lastDisp:%d]. free heap: %d, unused stack: %d\n",
                mainState, currentDisplay, lastDisplay, ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(0));

  Log.handleImprov();

#ifndef REMOVE_ALL_FOR_TESTING
  switch (mainState) {
    case ST_DECODER:
#ifndef DISABLE_MAINRX
      loopDecoder();
#else
      delay(1000);
#endif
      break;
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_WIFISCAN: loopWifiScan(); break;
    case ST_UPDATE: execOTA(); break;
    case ST_TOUCHCALIB: loopTouchCalib(); break;
#if FEATURE_RS92
    case ST_RINEX_UPDATE: execRinexUpdate(); break;
#endif
#if FEATURE_SDCARD
    case ST_FORMAT_SD: execFormatSD(); break;
#endif
  }
#if 0
  int rssi = radio.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);

  int gain = radio.getLNAGain();
  Serial.print(" LNA Gain: "),
               Serial.println(gain);
#endif
  loopWifiBackground();
  if (currentDisplay != lastDisplay && (mainState == ST_DECODER)) {
    disp.setLayout(currentDisplay);
    sonde.clearDisplay();
    sonde.updateDisplay();
    lastDisplay = currentDisplay;
  }
#else
  delay(1000);
#endif
}



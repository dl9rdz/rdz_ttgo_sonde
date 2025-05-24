#include "features.h"
#include "version.h"
#include "core.h"

#define TAG "RX_FSK"
#include "src/logger.h"

#include <dirent.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>

#include <LittleFS.h>

#include <SPI.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <Ticker.h>
#include "esp_heap_caps.h"
//#include <rtc_wdt.h>
//#include "soc/timer_group_struct.h"
//#include "soc/timer_group_reg.h"

#include "src/SX1278FSK.h"
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


/* Data exchange connectors */
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

AsyncWebServer server(80);

PMU *pmu = NULL;
SemaphoreHandle_t axpSemaphore;
extern uint8_t pmu_irq;

const char *updateHost = "rdzsonde.mooo.com";
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
// BootID is embedded in index.html so client can invalidate auth cookie after ttgo reboot
// defaultUserLevel is read from user.txt
char bootid[8];
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

enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG, KP_RINEX, KP_FORMAT };

// "doublepress" is now also used to eliminate key glitch on TTGO T-Beam startup (SENSOR_VN/GPIO39)
struct Button {
  uint8_t pin;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long keydownTime;
  int8_t doublepress;
  bool isTouched;
};
Button button1 = {0, 0, KP_NONE, 0, -1, false};
Button button2 = {0, 0, KP_NONE, 0, -1, false};


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


// Replaces placeholder with LED state value
String processor(const String& var) {
  LOG_D(TAG, "%s\n", var.c_str());
  if (var == "MAPCENTER") {
#if 0
    double lat, lon;
    if (gpsPos.valid) {
      lat = gpsPos.lat;
      lon = gpsPos.lon;
    }
    else {
      lat = sonde.config.rxlat;
      lon = sonde.config.rxlon;
    }
    //if ( !isnan(lat) && !isnan(lon) ) {
#endif
    if ( posInfo.valid ) {
      char p[40];
      snprintf(p, 40, "%g,%g", posInfo.lat, posInfo.lon);
      return String(p);
    } else {
      return String("48,13");
    }
  }
  if (var == "BOOTID") {
    return String(bootid);
  }
  if (var == "VERSION_NAME") {
    return String(version_name);
  }
  if (var == "VERSION_ID") {
    return String(version_id);
  }
  if (var == "FULLNAMEID") {
    char tmp[128];
    snprintf(tmp, 128, "%s-%c%d", version_id, FS_MAJOR + 'A' - 1, FS_MINOR);
    return String(tmp);
  }
  if (var == "AUTODETECT_INFO") {
    char tmpstr[128];
    const char *fpstr;
    int i = 0;
    while (fingerprintValue[i] != sonde.fingerprint && fingerprintValue[i] != -1) i++;
    if (fingerprintValue[i] == -1) {
      fpstr = "Unknown board";
    } else {
      fpstr = fingerprintText[i];
    }
    snprintf(tmpstr, 128, "Fingerprint %d (%s)", sonde.fingerprint, fpstr);
    return String(tmpstr);
  }
  if (var == "EPHSTATE") {
#if FEATURE_RS92
    return String(ephtxt[ephstate]);
#else
    return String("Not supported");
#endif
  }
  if (var == "LOCAL_UPDATES") {
    if(localUpdates) return String(localUpdates);
    else return String();
  }
  if (var == "PREAUTH") {
    char preauth[COOKIE_SIZE];
    generateRandomCookie("preauth",preauth);
    storeCookie(preauth, -1);  // preauth value
    return String(preauth);
  }
  return String();
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

const char *handleLoginPost(AsyncWebServerRequest * request) {
  LOG_D(TAG, "Handling login POST request");

  AsyncWebParameter *userp = request->getParam("user", true, false);
  AsyncWebParameter *authp = request->getParam("auth", true, false);
  AsyncWebParameter *preauthp= request->getParam("preauth", true, false);
  if (!userp || !authp || !preauthp) {
    request->send(400, "text/plain", "Invalid Request");
    return nullptr;
  }

  String username = userp->value();
  String preauth = preauthp->value();
  String auth = authp->value();

  int ulvl = getUserPermissions(username.c_str(), preauth.c_str(), auth.c_str());
  if(ulvl> 0) {
    // Generate a new session cookie
    char cookie[COOKIE_SIZE];
    generateRandomCookie(username.c_str(), cookie);
    if(upgradeCookie(preauth.c_str(), cookie, ulvl)==0) {
      // Set cookie and redirect
      AsyncWebServerResponse *response = request->beginResponse(302);
      response->addHeader("Location","/index.html");
      response->addHeader("Set-Cookie", "SESSION=" + String(cookie) + "; Path=/; SameSite=Strict");
      request->send(response);
      return nullptr;
    }
  }
  request->send(401, "text/plain", "Invalid credentials or session expired");
  return nullptr;
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

const char *handleQRGPost(AsyncWebServerRequest * request) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
  File file = LittleFS.open("/qrg.txt", "w");
  if (!file) {
    LOG_E(TAG, "Error while opening '/qrg.txt' for writing");
    return "Error while opening '/qrg.txt' for writing";
  }
  Serial.println("Handling post request");
#if 0
  int params = request->params();
  for (int i = 0; i < params; i++) {
    String pname = request->getParam(i)->name();
    Serial.println(pname.c_str());
  }
#endif
  for (int i = 1; i <= sonde.config.maxsonde; i++) {
    snprintf(label, 10, "A%d", i);
    AsyncWebParameter *active = request->getParam(label, true);
    snprintf(label, 10, "F%d", i);
    AsyncWebParameter *freq = request->getParam(label, true);
    snprintf(label, 10, "S%d", i);
    AsyncWebParameter *launchsite = request->getParam(label, true);
    if (!freq) continue;
    snprintf(label, 10, "T%d", i);
    AsyncWebParameter *type = request->getParam(label, true);
    if (!type) continue;
    String fstring = freq->value();
    String tstring = type->value();
    String sstring = launchsite->value();
    const char *fstr = fstring.c_str();
    const char *tstr = tstring.c_str();
    const char *sstr = sstring.c_str();
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
                        file.printf("%s\n%s\n", networks[i].id, networks[i].pw);
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
  // Serial.print(i); Serial.println(" networks in networks.txt\n");
  for (int j = 0; j < i; j++) {
    LOG_I(TAG, "%s: %s\n", networks[j].id, networks[j].pw);
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

const char *handleWIFIPost(AsyncWebServerRequest * request) {
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
#if 0
  int params = request->params();
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
  }
#endif
  for (int i = 1; i <= MAX_WIFI; i++) {
    snprintf(label, 10, "S%d", i);
    AsyncWebParameter *ssid = request->getParam(label, true);
    if (!ssid) continue;
    snprintf(label, 10, "P%d", i);
    AsyncWebParameter *pw = request->getParam(label, true);
    if (!pw) continue;
    String sstring = ssid->value();
    String pstring = pw->value();
    const char *sstr = sstring.c_str();
    const char *pstr = pstring.c_str();
    if (strlen(sstr) == 0) continue;
    LOG_D(TAG, "Processing S=%s, P=%s\n", sstr, pstr);
    f.printf("%s\n%s\n", sstr, pstr);
  }
  f.close();
  setupWifiList();
  return "";
}

// Show current status
void addSondeStatus(char *ptr, int i)
{
  struct tm ts;
  SondeInfo *s = &sonde.sondeList[i];
  strcat(ptr, "<table class=\"stat\">");
  sprintf(ptr + strlen(ptr), "<tr><td id=\"sfreq\">%3.3f MHz, Type: %s</td><tr><td>ID: %s", s->freq, sondeTypeLongStr[sonde.realType(s)],
          s->d.validID ? s->d.id : "<?""?>");
  if (s->d.validID && (TYPE_IS_DFM(s->type) || TYPE_IS_METEO(s->type) || s->type == STYPE_MP3H) ) {
    sprintf(ptr + strlen(ptr), " (ser: %s)", s->d.ser);
  }
  sprintf(ptr + strlen(ptr), "</td></tr><tr><td>QTH: %.6f,%.6f h=%.0fm</td></tr>\n", s->d.lat, s->d.lon, s->d.alt);
  const time_t t = s->d.time;
  ts = *gmtime(&t);
  sprintf(ptr + strlen(ptr), "<tr><td>Frame# %u, Sats=%d, %04d-%02d-%02d %02d:%02d:%02d</td></tr>",
          s->d.frame, s->d.sats, ts.tm_year + 1900, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec);
  if (s->type == STYPE_RS41) {
    sprintf(ptr + strlen(ptr), "<tr><td>Burst-KT=%d Launch-KT=%d Countdown=%d (vor %ds)</td></tr>\n",
            s->d.burstKT, s->d.launchKT, s->d.countKT, ((uint16_t)s->d.frame - s->d.crefKT));
  }
  sprintf(ptr + strlen(ptr), "<tr><td><a target=\"_empty\" href=\"geo:%.6f,%.6f\">GEO-App</a> - ", s->d.lat, s->d.lon);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://radiosondy.info/sonde_archive.php?sondenumber=%s\">radiosondy.info</a> - ", s->d.id);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://tracker.sondehub.org/%s\">SondeHub Tracker</a> - ", s->d.ser);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://www.openstreetmap.org/?mlat=%.6f&mlon=%.6f&zoom=14\">OSM</a> - ", s->d.lat, s->d.lon);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://www.google.com/maps/search/?api=1&query=%.6f,%.6f\">Google</a></td></tr>", s->d.lat, s->d.lon);

  strcat(ptr, "</table>\n");
}

const char *createStatusForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "<meta http-equiv=\"refresh\" content=\"5\"></head>");
  HTMLBODY(ptr, "status.html");
  strcat(ptr, "<div class=\"content\">");

  for (int i = 0; i < sonde.config.maxsonde; i++) {
    int snum = (i + sonde.currentSonde) % sonde.config.maxsonde;
    if (sonde.sondeList[snum].active) {
      addSondeStatus(ptr, snum);
    }
  }
  strcat(ptr, "</div><div class=\"footer\"><span></span>"
         "<span class=\"ttgoinfo\">rdzTTGOserver ");
  strcat(ptr, version_id);
  strcat(ptr, "</span>");

  HTMLBODYEND(ptr);
  LOG_D(TAG, "Status form: size=%d bytes\n", strlen(message));
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
    sprintf(ptr + strlen(ptr), ", \"gps\": {\"lat\": %g, \"lon\": %g, \"alt\": %d, \"sat\": %d, \"speed\": %g, \"dir\": %d, \"hdop\": %d }", posInfo.lat, posInfo.lon, posInfo.alt, posInfo.sat, posInfo.speed, posInfo.course, posInfo.hdop);
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
          sprintf(ptr + strlen(ptr), "%g", *(double *)config_list[i].data);
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


const char *handleConfigPost(AsyncWebServerRequest * request) {
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
  int params = request->params();
#if 0
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
  }
#endif
  for (int i = 0; i < params; i++) {
    String strlabel = request->getParam(i)->name();
    const char *label = strlabel.c_str();
    if (label[strlen(label) - 1] == '#') continue;
    AsyncWebParameter *value = request->getParam(label, true);
    if (!value) continue;
    String strvalue = value->value();
    if ( strcmp(label, "button_pin") == 0 ||
         strcmp(label, "button2_pin") == 0) {
      AsyncWebParameter *touch = request->getParam(strlabel + "#", true);
      if (touch) {
        int i = atoi(strvalue.c_str());
        if (i != -1 && i != 255) i += 128;
        strvalue = String(i);
      }
    }
    LOG_D(TAG, "Processing %s=%s\n", label, strvalue.c_str());
    //int wlen = f.printf("%s=%s\n", config_list[idx].name, strvalue.c_str());
    int wlen = f.printf("%s=%s\n", label, strvalue.c_str());
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


const char *handleControlPost(AsyncWebServerRequest * request) {
  LOG_D(TAG, "Handling control post request");
  int params = request->params();
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
    if (param.equals("rx")) {
      Serial.println("equals rx");
      button1.pressed = KP_SHORT;
    }
    else if (param.equals("scan")) {
      Serial.println("equals scan");
      button1.pressed = KP_DOUBLE;
    }
    else if (param.equals("spec")) {
      Serial.println("equals spec");
      button1.pressed = KP_MID;
    }
    else if (param.equals("wifi")) {
      Serial.println("equals wifi");
      button1.pressed = KP_LONG;
    }
    else if (param.equals("rx2")) {
      Serial.println("equals rx2");
      button2.pressed = KP_SHORT;
    }
    else if (param.equals("scan2")) {
      Serial.println("equals scan2");
      button2.pressed = KP_DOUBLE;
    }
    else if (param.equals("spec2")) {
      Serial.println("equals spec2");
      button2.pressed = KP_MID;
    }
    else if (param.equals("wifi2")) {
      Serial.println("equals wifi2");
      button2.pressed = KP_LONG;
    }
    else if (param.equals("rinex")) {
      Serial.println("equals rinex");
      button2.pressed = KP_RINEX;
    }
    else if (param.equals("format")) {
      Serial.println("equals format");
      button2.pressed = KP_FORMAT;
    }
    else if (param.equals("reboot")) {
      Serial.println("equals reboot");
      ESP.restart();
    }
  }
  return "";
}

void handleUpload(AsyncWebServerRequest * request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static File file;
  if (!index) {
    const char *fn = filename.c_str();
    if(!checkAllowed(fn)) {
      LOG_E(TAG, "UploadStart: writing %s prohibited\n", fn);
      return;
    }
    LOG_D(TAG, "UploadStart: %s\n", filename.c_str());
    file = LittleFS.open("/" + filename, "w");
    if (!file) {
      LOG_E(TAG, "Error opening the file '/%s' for writing", fn);
    }
  }
  if (!file) return;
  for (size_t i = 0; i < len; i++) {
    file.write(data[i]);
  }
  if (final) {
    LOG_D(TAG, "UploadEnd: %s, %u B\n", filename.c_str(), index + len);
    file.close();
  }
}


int streamEditForm(int &state, File & file, String filename, char *buffer, size_t maxlen, size_t index) {
  LOG_D(TAG, "streamEdit: state=%d  max:%d idx:%d\n", state, maxlen, index);
  int i = 0;
  switch (state) {
    case 0: // header
      {
        // we optimistically assume that on first invocation, maxlen is large enough to handle the header.....
        strncpy(buffer, "<html><head><title>Editor</title></head><body><p>Edit: ", maxlen);
        i = strlen(buffer);
        strncpy(buffer + i, filename.c_str(), maxlen - i);
        i += strlen(buffer + i);
        strncpy(buffer + i, "</p><form action=\"edit.html?file=", maxlen - i);
        i += strlen(buffer + i);
        strncpy(buffer + i, filename.c_str(), maxlen - i);
        i += strlen(buffer + i);
        strncpy(buffer + i, "\" method=\"post\" enctype=\"multipart/form-data\"><textarea name=\"text\" cols=\"80\" rows=\"40\">", maxlen - i);
        i += strlen(buffer + i);
        if (i >= maxlen) {
          strncpy(buffer, "Out of memory", maxlen);
          state = 3;
          return strlen(buffer);
        }
        state++;
        LOG_D(TAG, "Wrote %d bytes. Header finished", i);
        return i;
        break;
      }
    case 1: // file content
      while (file.available()) {
        int cnt = readLine(file, buffer + i, maxlen - i - 1);
        i += cnt;
        buffer[i++] = '\n';
        buffer[i] = 0;
        if (i + 256 > maxlen) break; // max line length in file 256 chars
      }
      if (i > 0) return i;
      file.close();
      state++;  // intentional fall-through
    case 2:  // footer
      Serial.println("Appending footer\n");
      strncpy(buffer, "</textarea><input type=\"submit\" value=\"Save\"></input></form></body></html>", maxlen);
      state++;
      return strlen(buffer);
    case 3:  // end
      return 0;
  }
  return 0;
}

// bad idea. prone to buffer overflow. use at your own risk...
const char *createEditForm(String filename) {
  Serial.println("Creating edit form");
  char *ptr = message;
  File file = LittleFS.open("/" + filename, "r");
  if (!file) {
    Serial.println("There was an error opening the file '/config.txt' for reading");
    return "<html><head><title>File not found</title></head><body>File not found</body></html>";
  }

  strcpy(ptr, "<html><head><title>Editor ");
  strcat(ptr, filename.c_str());
  strcat(ptr, "</title></head><body><form action=\"edit.html?file=");
  strcat(ptr, filename.c_str());
  strcat(ptr, "\" method=\"post\" enctype=\"multipart/form-data\">");
  strcat(ptr, "<textarea name=\"text\" cols=\"80\" rows=\"40\">");
  while (file.available()) {
    String line = readLine(file);  //file.readStringUntil('\n');
    strcat(ptr, line.c_str()); strcat(ptr, "\n");
  }
  strcat(ptr, "</textarea><input type=\"submit\" value=\"Save\"></input></form></body></html>");
  LOG_D(TAG, "Edit form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleEditPost(AsyncWebServerRequest * request) {
  Serial.println("Handling post request");
  int params = request->params();
  LOG_D(TAG, "Post:, %d params\n", params);
  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    String name = p->name();
    String value = p->value();
    if (name.c_str() == NULL) {
      name = String("NULL");
    }
    if (value.c_str() == NULL) {
      value = String("NULL");
    }
    if (p->isFile()) {
      LOG_D(TAG, "_FILE[%s]: %s, size: %u\n", name.c_str(), value.c_str(), p->size());
    } else if (p->isPost()) {
      LOG_D(TAG, "_POST[%s]: %s\n", name.c_str(), value.c_str());
    } else {
      LOG_D(TAG, "_GET[%s]: %s\n", name.c_str(), value.c_str());
    }
  }

  AsyncWebParameter *filep = request->getParam("file");
  if (!filep) return NULL;

  String filename = filep->value();
  const char *fn = filename.c_str();
  if(!checkAllowed(fn)) {
    LOG_E(TAG, "handleEditPost: writing %s prohibited\n", fn);
    return NULL;
  }

  LOG_D(TAG, "Writing file <%s>\n", fn);
  AsyncWebParameter *textp = request->getParam("text", true);
  if (!textp) return NULL;
  LOG_D(TAG, "Parameter size is %d\n", textp->size());
  LOG_D(TAG, "Multipart: %d  contentlen=%d  \n",
                request->multipart(), request->contentLength());
  String content = textp->value();
  if (content.length() == 0) {
    Serial.println("File is empty. Not written.");
    return NULL;
  }
  File file = LittleFS.open("/" + filename, "w");
  if (!file) {
    Serial.println("There was an error opening the file '/" + filename + "'for writing");
    return "";
  }
  LOG_D(TAG, "File is open for writing, content is %d bytes\n", content.length());
  int len = file.print(content);
  file.close();
  LOG_D(TAG, "Written: %d bytes\n", len);
  if (strncmp(filename.c_str(), "screens", 7) == 0) {
    // screens update => reload
    forceReloadScreenConfig = true;
  }
  return "";
}

// will be removed. its now in data/upd.html (for GET; POST to update.html still handled here)
const char *createUpdateForm(boolean run) {
  char *ptr = message;
  strcpy(ptr, "<!DOCTYPE html><html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"update.html\" method=\"post\">");
  if (run) {
    strcat(ptr, "<p>Doing update, wait until reboot</p>");
  } else {
    sprintf(ptr + strlen(ptr), "<p>Currently installed: %s-%c%d</p>\n", version_id, FS_MAJOR + 'A' - 1, FS_MINOR);
    strcat(ptr, "<p>Available main: <iframe src=\"http://rdzsonde.mooo.com/main/update-info.html\" style=\"height:40px;width:400px\"></iframe><br>"
           "Available devel: <iframe src=\"http://rdzsonde.mooo.com/dev2/update-info.html\" style=\"height:40px;width:400px\"></iframe></p>");
    strcat(ptr, "<input type=\"submit\" name=\"main\" value=\"Main-Update\"></input><br><input type=\"submit\" name=\"dev\" value=\"Devel-Update\">");
    strcat(ptr, "<br><p>Note: If suffix is the same, update should work fully. If the number is different, update contains changes in the file system. A full re-flash is required to get all new features, but the update should not break anything. If the letter is different, a full re-flash is mandatory, update will not work</p>");
  }
  strcat(ptr, "</form></body></html>");
  LOG_D(TAG, "Update form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleUpdatePost(AsyncWebServerRequest * request) {
  Serial.println("Handling post request");
  int params = request->params();
  bool updateFromURL = false;
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
    if (param.equals("dev2")) {
      Serial.println("equals devel");
      updatePrefix = updatePrefixD;
    }
    else if (param.equals("main")) {
      Serial.println("equals main");
      updatePrefix = updatePrefixM;
    }
    else if (localUpdates && param.equals("local")) {
      // Local updates permitted. Expect URL as url parameter...
      updateFromURL = true;
    }
    else if (updateFromURL && param.equals("url")) {
      // Note: strdup allocates memory that is never free'd.
      // Let's not care about this as we are going to reboot after the update anyway...
      String localsrc = request->getParam(i)->value();
      int pos;

      if (-1 != (pos = localsrc.indexOf("://")) ){
        // strip off http://
        localsrc = localsrc.substring(pos+3);
      }

      if (-1 != (pos = localsrc.indexOf("/")) ){
        // see if there's a directory or updates are in the root
        String tmp = localsrc.substring(pos);
        updatePrefix = strdup(tmp.c_str());
        localsrc = localsrc.substring(0, pos);
      } else {
         updatePrefix = strdup("/");
      }

      if (-1 != (pos = localsrc.indexOf(":"))) {
        // extract port
        updatePort = atoi(localsrc.substring(pos+1).c_str());
        updateHost = strdup(localsrc.substring(0, pos).c_str());
      } else {
        updateHost = strdup(localsrc.c_str());
      }
    }
  }
  LOG_I(TAG, "Updating: %s%s from %s:%d\n", updatePrefix, updateIno, updateHost, updatePort);
  enterMode(ST_UPDATE);
  return "";
}

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


const char *sendGPX(AsyncWebServerRequest * request) {
  Serial.println("\n\n\n********GPX request\n\n");
  String url = request->url();
  int index = atoi(url.c_str() + 1);
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

bool isAuthenticated(AsyncWebServerRequest *request, int level) {
  if(defaultUserLevel >= level)
    return 1;
  if(request->hasHeader("Cookie")) {
    String cookieHdr = request->getHeader("Cookie")->value();
    int start = cookieHdr.indexOf("SESSION=") + strlen("SESSION=");
    if(start!=-1) {
      int end = cookieHdr.indexOf(';', start);
      String session = (end==-1) ? cookieHdr.substring(start) : cookieHdr.substring(start, end);
      session.trim();
      int ulvl = getCookieAuthLevel(session.c_str());
      if(ulvl >= level) {
        return 1;
      }
    }
  }
  request->send(401, "text/plain", "Permission denied");
  return false;
}

const char* PARAM_MESSAGE = "message";

void SetupAsyncServer() {
  Serial.println("SetupAsyncServer()\n");
  for(int i=0; i<7; i++) { bootid[i]=random(26)+'A'; }
  bootid[7] = 0;
  defaultUserLevel = getDefaultAuthLevel();
  server.reset();
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/index.html", String(), false, processor);
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/index.html", String(), false, processor);
  });

  server.on("/qrg.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createQRGForm());
  });
  server.on("/qrg.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    handleQRGPost(request);
    request->send(200, "text/html", createQRGForm());
  });

  server.on("/wifi.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    request->send(200, "text/html", createWIFIForm());
  });
  server.on("/wifi.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    handleWIFIPost(request);
    request->send(200, "text/html", createWIFIForm());
  });


  server.on("/config.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    request->send(200, "text/html", createConfigForm());
  });
  server.on("/config.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    handleConfigPost(request);
    request->send(200, "text/html", createConfigForm());
  });

  server.on("/status.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createStatusForm());
  });
  server.on("/live.json", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/json", createLiveJson());
  });
  server.on("/livemap.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/livemap.html", String(), false, processor);
  });
  server.on("/livemap.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/livemap.js", String(), false, processor);
  });
  server.on("/update.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createUpdateForm(0));
  });
  server.on("/update.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleUpdatePost(request);
    request->send(200, "text/html", createUpdateForm(1));
  });

  server.on("/control.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    request->send(200, "text/html", createControlForm());
  });
  server.on("/control.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    handleControlPost(request);
    request->send(200, "text/html", createControlForm());
  });

  server.on("/login.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/login.html", String(), false, processor);
  });
  server.on("/login.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleLoginPost(request);
  });

  server.on("/file", HTTP_GET,  [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    String url = request->url();
    const char *filename = url.c_str() + 5;
    if (*filename == 0) {
      request->send(400, "error");
      return;
    }
    request->send(LittleFS, filename, "text/plain");
  });
  
  server.on("/file", HTTP_POST,  [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    request->send(200);
  }, handleUpload);
#if FEATURE_SDCARD
  server.on("/sd/data.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
     Serial.println("Opening SD card file\n");
     const File SDFile = SD.open("/data.csv", FILE_READ);
     if(SDFile) { LOG_I(TAG, "SD file opened\n"); }
     else { LOG_I(TAG, "SD file does not exist"); request->send(404); return; }
     AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain", [SDFile](uint8_t *buf, size_t maxLen, size_t index) -> size_t {
       File sdlf = SDFile;
       // if(maxLen>1024) maxLen=1024;
       LOG_D(TAG, "[HTTP]\t[%d]\tINDEX [%d]\tBUFFER_MAX_LENGHT [%d]\r\n", index, sdlf.size(), maxLen);
       return sdlf.read(buf, maxLen);
     });
     request->send(response);
  });
  server.serveStatic("/sd/", SD, "/");
  server.on("/sd/files.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    DIR *dir = opendir("/sd/");
    struct dirent *dent;
    AsyncWebServerResponse *response = request->beginChunkedResponse("application/json", [dir, dent](uint8_t *buf, size_t maxLen, size_t index) mutable -> size_t {
      LOG_D(TAG, "[HTTP]\tINDEX [%d]\tBUFFER_MAX_LENGHT [%d]\r\n", index, maxLen);
      if(index==0) {
        if(!dir) {
		dent = NULL;
	}else {
         	dent = readdir(dir);
	}
        strcpy((char *)buf, "[ \n");
        if(dent==NULL) { strcpy( (char*)buf+2, "]"); return 3; }
        return 3;
      }
      if(dent) {
        char fname[128];
        struct stat attr;
        char ftim[50];
        snprintf(fname, 128, "/sd/%s", dent->d_name);
        stat(fname, &attr);
        strftime(ftim, 50, "%Y-%m-%dT%H:%M:%SZ", gmtime(&attr.st_mtime)); 
        snprintf((char *)buf, maxLen, "{\"name\":\"%s\", \"size\":%d, \"ts\":\"%s\"}", dent->d_name, attr.st_size, ftim);
        dent = readdir(dir);
        if(dent) strcat((char *)buf, ",\n");
        else strcat((char *)buf, "\n]\n");
        return strlen((char *)buf);
      } else { return 0; }
    });
    request->send(response);
  });
#endif

  server.on("/edit.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    // new version:
    // Open file
    // store file object in request->_tempObject
    //request->send(200, "text/html", createEditForm(request->getParam(0)->value()));
    AsyncWebParameter *param = request->getParam(0);
    if(!param) {
      request->send(404);
      return;
    }
    const String filename = param->value();
    File file = LittleFS.open("/" + filename, "r");
    int state = 0;
    request->send("text/html", 0, [state, file, filename](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t  {
      LOG_D(TAG, "******* send callback: %d %d %d\n", state, maxLen, index);
      return streamEditForm(state, file, filename, (char *)buffer, maxLen, index);
    });
  });
  server.on("/edit.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    if(!isAuthenticated(request, 2)) return;
    const char *ret = handleEditPost(request);
    if (ret == NULL)
      request->send(200, "text/html", "<html><head>ERROR</head><body><p>Something went wrong (probably ESP32 out of memory). Uploaded file is empty.</p></body></hhtml>");
    else {
      AsyncWebParameter *param = request->getParam(0);
      if(!param) {
         request->send(404);
         return;
      }
      String f = param->value();
      request->redirect("/edit.html?file=" + f);
    }
  },
  NULL,
  [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    LOG_D(TAG, "post data: index=%d len=%d total=%d\n", index, len, total);
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/style.css", "text/css");
    if(response) {
      response->addHeader("Cache-Control", "max-age=86400");
      request->send(response);
    } else {
      request->send(404);
    }
  });

  server.on("/live.kml", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "application/vnd.google-earth.kml+xml", createKMLLive(sonde.ipaddr.c_str()));
  });

  server.on("/dynamic.kml", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "application/vnd.google-earth.kml+xml", createKMLDynamic());
  });

  server.on("/upd.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/upd.html", String(), false, processor);
  });

  server.on("/status.json", HTTP_GET, [](AsyncWebServerRequest * request) {
   int nr = 0;
   AsyncWebServerResponse *response = request->beginChunkedResponse("application/json", [nr](uint8_t *buf, size_t maxLen, size_t index) mutable-> size_t {
       if(connectors[nr]==NULL) return 0;
       if(index==0) { strcpy((char *)buf, "{"); buf++; maxLen--; } else *buf=0;
       snprintf( (char *)buf, maxLen-2, "\"%s\": \"%s\"\n", connectors[nr]->getName().c_str(), connectors[nr]->getStatus().c_str() );
       nr++;
       strcat((char *)buf, connectors[nr]==NULL ? "}\n":",\n");
       return strlen((char *)buf);
     }); 
     request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest * request) {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      String url = request->url();
      if (url.endsWith(".gpx"))
        request->send(200, "application/gpx+xml", sendGPX(request));
      else {
        // TODO: set correct type for .js

        // Caching is an important work-around for a bug somewhere in the network stack that causes corrupt replies
        // with platform-espressif32 (some TCP segments simply get lost before being sent, so reply header and parts of data is missing)
        // This happens with concurrent requests, notably if a browser fetches rdz.js and cfg.js concurrently for config.html
        // With the cache, rdz.js is likely already in the cache...0
        LOG_D(TAG, "URL is %s\n", url.c_str());
	const char *type = "text/html";
        if(url.endsWith(".js")) type="text/javascript";
        LOG_D(TAG, "Responding with type %s (url %s)\n", type, url.c_str());
        AsyncWebServerResponse *response = request->beginResponse(LittleFS, url, type);
        if(response) {
          response->addHeader("Cache-Control", "max-age=900"); 
          request->send(response);
        } else {
          request->send(404);
        }
      }
    }
  });

  // Start server
  server.begin();
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

// It is not safe to call millis() in ISR!!!
// millis() does a division int64_t by 1000 for which gcc creates a library call
// on a 32bit system, and the called function has no IRAM_ATTR
// so doing it manually...
// Code adapted for 64 bits from https://www.hackersdelight.org/divcMore.pdf
static int64_t IRAM_ATTR divs10(int64_t n) {
  int64_t q, r;
  n = n + (n >> 63 & 9);
  q = (n >> 1) + (n >> 2);
  q = q + (q >> 4);
  q = q + (q >> 8);
  q = q + (q >> 16);
  q = q + (q >> 32);
  q = q >> 3;
  r = n - q * 10;
  return q + ((r + 6) >> 4);
  // return q + (r > 9);
}

static int64_t IRAM_ATTR divs1000(int64_t n) {
  return divs10(divs10(divs10(n)));
}

static unsigned long IRAM_ATTR my_millis()
{
  return divs1000(esp_timer_get_time());
}

static void checkTouchStatus();
static void touchISR();
static void touchISR2();

// ISR won't work for SPI transfer, so forget about the following approach
///// Also initialized timers for sx1278 handling with interruts
///// fastest mode currentily is 4800 bit/s, i.e. 600 bytes/sec
///// 64 byte FIFO will last for at most about 106 ms.
///// lets use a timer every 20ms to handle sx1278 FIFO input, that should be fine.
// Instead create a tast...

Ticker ticker;
Ticker ledFlasher;

#define IS_TOUCH(x) (((x)!=255)&&((x)!=-1)&&((x)&128))
void initTouch() {
  // also used for LED
  ticker.attach_ms(300, checkTouchStatus);

  if ( !(IS_TOUCH(sonde.config.button_pin) || IS_TOUCH(sonde.config.button2_pin)) ) return; // no touch buttons configured
  /*
   *  ** no. readTouch is not safe to use in ISR!
      so now using Ticker
    hw_timer_t *timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, checkTouchStatus, true);
    timerAlarmWrite(timer, 300000, true);
    timerAlarmEnable(timer);
  */

  if ( IS_TOUCH(sonde.config.button_pin) ) {
    touchAttachInterrupt(sonde.config.button_pin & 0x7f, touchISR, sonde.config.touch_thresh);
    LOG_I(TAG, "Initializing touch 1 on pin %d\n", sonde.config.button_pin & 0x7f);
  }
  if ( IS_TOUCH(sonde.config.button2_pin) ) {
    touchAttachInterrupt(sonde.config.button2_pin & 0x7f, touchISR2, sonde.config.touch_thresh);
    LOG_I(TAG, "Initializing touch 2 on pin %d\n", sonde.config.button2_pin & 0x7f);
  }
}



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


static void IRAM_ATTR touchISR() {
  if (!button1.isTouched) {
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) button1.doublepress = 1;
    else button1.doublepress = 0;
    button1.keydownTime = now;
    button1.isTouched = true;
  }
}

static void IRAM_ATTR touchISR2() {
  if (!button2.isTouched) {
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) button2.doublepress = 1;
    else button2.doublepress = 0;
    button2.keydownTime = now;
    button2.isTouched = true;
  }
}

// touchRead in ISR is also a bad idea. Now moved to Ticker task
static void checkTouchButton(Button & button) {
  if (button.isTouched) {
    int tmp = touchRead(button.pin & 0x7f);
    //LOG_D(TAG, "touch read %d: value is %d\n", button.pin & 0x7f, tmp);
    if (tmp > sonde.config.touch_thresh + 5) {
      button.isTouched = false;
      unsigned long elapsed = my_millis() - button.keydownTime;
      if (elapsed > 1500) {
        if (elapsed < 4000) {
          button.pressed = KP_MID;
        }
        else {
          button.pressed = KP_LONG;
        }
      } else if (button.doublepress) {
        button.pressed = KP_DOUBLE;
      } else {
        button.pressed = KP_SHORT;
      }
    }
  }
}

static unsigned long t_muted = (unsigned long)-1;

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

int doTouch = 0;
static void checkTouchStatus() {
  checkTouchButton(button1);
  checkTouchButton(button2);
}

unsigned long bdd1, bdd2;
static bool b1wasdown = false;
static void IRAM_ATTR buttonISR() {
  if (digitalRead(button1.pin) == 0) { // Button down
    b1wasdown = true;
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) {
      // Double press
      if (now - button1.keydownTime > 100)
        button1.doublepress = 1;
      bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button1.doublepress = 0;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  } else { //Button up
    if (!b1wasdown) return;
    b1wasdown = false;
    unsigned long now = my_millis();
    if (button1.doublepress == -1) return;   // key was never pressed before, ignore button up
    unsigned int elapsed = now - button1.keydownTime;
    if (elapsed > 1500) {
      if (elapsed < 4000) {
        button1.pressed = KP_MID;
      }
      else {
        button1.pressed = KP_LONG;
      }
    } else {
      if (button1.doublepress) button1.pressed = KP_DOUBLE;
      else button1.pressed = KP_SHORT;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  }
}

static void IRAM_ATTR button2ISR() {
  if (digitalRead(button2.pin) == 0) { // Button down
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) {
      // Double press
      if (now - button2.keydownTime > 100)
        button2.doublepress = 1;
      //bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button2.doublepress = 0;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  } else { //Button up
    unsigned long now = my_millis();
    if (button2.doublepress == -1) return;   // key was never pressed before, ignore button up
    unsigned int elapsed = now - button2.keydownTime;
    if (elapsed > 1500) {
      if (elapsed < 4000) {
        button2.pressed = KP_MID;
      }
      else {
        button2.pressed = KP_LONG;
      }
    } else {
      if (button2.doublepress) button2.pressed = KP_DOUBLE;
      else button2.pressed = KP_SHORT;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  }
}

int getKeyPress() {
  KeyPress p = button1.pressed;
  button1.pressed = KP_NONE;
#if 0
  int x = digitalRead(button1.pin);
  LOG_D(TAG, "Debug: bdd1=%ld, bdd2=%ld\n", bdd1, bdd2);
  LOG_D(TAG, "button1 press (dbl:%d) (now:%d): %d at %ld (%d)\n", button1.doublepress, x, p, button1.keydownTime, button1.numberKeyPresses);
#endif
  return p;
}

// called by arduino main loop (from Sonde::waitRXcomplete) as soon as pmu_irq is set
void handlePMUirq() {
  if (sonde.config.button2_axp) {
    // Use AXP power button as second button
    int key = pmu->handleIRQ();
    if (key > 0) {
      button2.pressed = (KeyPress)key;
      button2.keydownTime = my_millis();
    }
  } else {
    // WiFi loop intentionally calls this in order to react to PMU key press
    //Serial.println("handlePMIirq() called. THIS SHOULD NOT HAPPEN w/o button2_axp set");
    pmu_irq = 0;   // prevent main loop blocking
  }
}

int getKey2Press() {
  // TODO: Should be atomic
  KeyPress p = button2.pressed;
  button2.pressed = KP_NONE;
  //LOG_D(TAG, "button2 press: %d at %ld (%d)\n", p, button2.keydownTime, button2.numberKeyPresses);
  return p;
}

int getKeyPressEvent() {
  int p = getKeyPress();
  if (p == KP_NONE) {
    p = getKey2Press();
    if (p == KP_NONE)
      return EVT_NONE;
    if (p == KP_RINEX)
      return EVT_RINEX;
    if (p == KP_FORMAT)
      return EVT_FORMAT;
    LOG_D(TAG, "Key 2 was pressed [%d]\n", p + 4);
    // maybe not the best place, but easy to do: check for B2 medium keypress to mute LED
    if(p == KP_MID && sonde.config.b2mute > 0) {
       if(t_muted==-1) t_muted = millis(); else t_muted = -1;
    }
    return p + 4;
  }
  LOG_D(TAG, "Key 1 was pressed [%d]\n", p);
  return p;  /* map KP_x to EVT_KEY1_x / EVT_KEY2_x*/
}

#define SSD1306_ADDRESS 0x3c
bool ssd1306_found = false;
bool axp_found = false;

int scanI2Cdevice(void)
{
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println(" !");
      nDevices++;

      if (addr == SSD1306_ADDRESS) {
        ssd1306_found = true;
        Serial.println("ssd1306 display found");
      }
      if (addr == AXP192_SLAVE_ADDRESS) {  // Same for AXP2101
        axp_found = true;
        Serial.println("axp2101 PMU found");
      }
    } else if (err == 4) {
      Serial.print("Unknow error at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  return nDevices;
}

extern int initlevels[40];

extern SemaphoreHandle_t globalLock;

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

  // NOT TTGO v1 (fingerprint 64) or Heltec v1/v2 board (fingerprint 4)
  // and NOT TTGO Lora32 v2.1_1.6 (fingerprint 31/63)
  if ( ( (sonde.fingerprint & (64 + 31)) != 31) && ((sonde.fingerprint & 16) == 16) ) {
    // FOr T-Beam 1.0
    for (int i = 0; i < 10; i++) { // try multiple times
      Wire.begin(21, 22);
      // Make sure the whole thing powers up!?!?!?!?!?
      U8X8 *u8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(0, 22, 21);
      u8x8->initDisplay();
      delay(100);

      scanI2Cdevice();

      if (!pmu) {
        pmu = PMU::getInstance(Wire);
        if (pmu) {
          Serial.println("PMU found");
          pmu->init();
          if (sonde.config.button2_axp ) {
            //axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
            //axp.enableIRQ( AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, 1 );
            //axp.clearIRQ();
            pmu->disableAllIRQ();
            pmu->enableIRQ();
          }
          int ndevices = scanI2Cdevice();
          if (sonde.fingerprint != 17 || ndevices > 0) break; // only retry for fingerprint 17 (startup problems of new t-beam with oled)
          delay(100);
        }
      }
    }
  }
  if (sonde.config.batt_adc >= 0) {
    pinMode(sonde.config.batt_adc, INPUT);
  }
  if (sonde.config.power_pout >= 0) { // for a heltec v2, pull GPIO21 low for display power
    pinMode(sonde.config.power_pout & 127, OUTPUT);
    digitalWrite(sonde.config.power_pout & 127, sonde.config.power_pout & 128 ? 1 : 0);
  }

  if (sonde.config.led_pout >= 0) {
    pinMode(sonde.config.led_pout, OUTPUT);
    flashLed(1000); // testing
  }

  button1.pin = sonde.config.button_pin;
  button2.pin = sonde.config.button2_pin;
  if (button1.pin != 0xff) {
    if ( (button1.pin & 0x80) == 0 && button1.pin < 34 ) {
      Serial.println("Button 1 configured as input with pullup");
      pinMode(button1.pin, INPUT_PULLUP);
    } else
      pinMode(button1.pin, INPUT);  // configure as input if not disabled
  }
  if (button2.pin != 0xff) {
    if ( (button2.pin & 0x80) == 0 && button2.pin < 34 ) {
      Serial.println("Button 2 configured as input with pullup");
      pinMode(button2.pin, INPUT_PULLUP);
    } else
      pinMode(button2.pin, INPUT);  // configure as input if not disabled
  }
  // Handle button press
  if ( (button1.pin & 0x80) == 0) {
    attachInterrupt( button1.pin, buttonISR, CHANGE);
    LOG_D(TAG, "button1.pin is %d, attaching interrupt\n", button1.pin);
  }
  // Handle button press
  if ( (button2.pin & 0x80) == 0) {
    attachInterrupt( button2.pin, button2ISR, CHANGE);
    LOG_D(TAG, "button2.pin is %d, attaching interrupt\n", button2.pin);
  }
  initTouch();

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


#if 1

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

  sx1278.setup(globalLock);

  int i = 0;
  while (++i < 3) {
    // == check the radio chip by setting default frequency =========== //
    sx1278.ON();
    if (sx1278.setFrequency(402700000) == 0) {
      Serial.println(F("Setting freq: SUCCESS "));
    } else {
      Serial.println(F("Setting freq: ERROR "));
    }
    float f = sx1278.getFrequency();
    Serial.print("Frequency set to ");
    Serial.println(f);
    // == check the radio chip by setting default frequency =========== //
    if( f>402700000-1000 && f<402700000+1000 ) break;
    delay(500);
  }
#endif

  //sx1278.setLNAGain(-48);
  sx1278.setLNAGain(0);

  int gain = sx1278.getLNAGain();
  Serial.print("RX LNA Gain is ");
  Serial.println(gain);

  // Print a success message
  Serial.println(F("SX1278 configuration finished"));

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
  getKeyPress();    // clear key buffer

#else
/// DEBUG ONLY
  WiFi.begin("Dinosauro", "03071975");
  while(WiFi.status() != WL_CONNECTED) { delay(500); Serial.print(":"); }
  Serial.println("... WiFi is connected!\n");
  SetupAsyncServer();
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

  switch (getKeyPress()) {
    case KP_SHORT: /* move selection of peak, TODO */
      sonde.nextConfig(); // TODO: Should be set specific frequency
      enterMode(ST_DECODER);
      return;
    case KP_MID: /* restart, TODO */ break;
    case KP_LONG:
      Serial.println("loopSpectrum: KP_LONG");
      enterMode(ST_WIFISCAN);
      return;
    case KP_DOUBLE:
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
//enum t_wifi_state { WIFI_DISABLED, WIFI_SCAN, WIFI_CONNECT, WIFI_CONNECTED, WIFI_APMODE };

t_wifi_state wifi_state = WIFI_DISABLED;

uint32_t netup_time;

void enableNetwork(bool enable) {
  if (enable) {
    netup_time = esp_timer_get_time() / 1000000;
    MDNS.begin(sonde.config.mdnsname);
    SetupAsyncServer();
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
	break;
      }
      LOG_D(TAG, "Turning off (state is %d)\n", wifi_state);
      WiFi.mode(WIFI_MODE_NULL);
      break;
    case ARDUINO_EVENT_WIFI_OFF:
      Serial.println("WiFi is OFF");
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
    int t1 = touchRead(button1.pin & 0x7f);
    int t2 = touchRead(button2.pin & 0x7f);
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
      const char *ssid = fetchWifiSSID(1);
      WiFi.mode(WIFI_STA);
      WiFi.begin( ssid, fetchWifiPw(1) );
      disp.rdis->drawString(0, dispys * 2, ssid);
    }
    break;
  case 1:  // STATION mode (continue in BG if no connection)
  case 3:  // old AUTO mode (change to AP if no connection)
    // Mode STATION[1] or SETUP[3]: Scan for networks;
    disp.rdis->drawString(0, 0, "WiFi Scan...");
    int line = 0;
    int index = -1;
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
      if (curidx >= 0 && index == -1) {
        index = curidx;
        LOG_I(TAG, "Match found at scan entry %d, config network %d\n", i, index);
      }
    }
    if (index >= 0) { // some network was found
      Serial.print("Connecting to: "); Serial.print(fetchWifiSSID(index));
      Serial.print(" with password "); Serial.println(fetchWifiPw(index));

      disp.rdis->drawString(0, lastl, "Conn:");
      disp.rdis->drawString(6 * dispxs, lastl, fetchWifiSSID(index));
      // TODO: wifi_state is used inconsistently
      wifi_state = WIFI_CONNECT;
      WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
    } else {
      abort = 2;  // no network found in scan => abort right away
    }
  }
  while (WiFi.status() != WL_CONNECTED && cnt < MAXWIFIDELAY && !abort)  {
    delay(500);
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
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);

  int gain = sx1278.getLNAGain();
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



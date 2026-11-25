#include "features.h"
#include "version.h"
#include "core.h"

#define TAG "RX_FSK"
#include "src/logger.h"

#include <dirent.h>
#include <atomic>

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
#include "src/sonde_hist.h"
#if FEATURE_APRS
#include "src/conn-aprs.h"
#endif
#if FEATURE_SONDEHUB
#include "src/conn-sondehub.h"
#endif

#include "src/conn-system.h"
#include "src/netmgr.h"

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

/* /update.json snapshot — written by handleUpdatePost / enterMode(ST_UPDATE) / execOTA, read by HTTP. */
static const char *otaPhase = "idle";
static const char *otaMsg = "";
static unsigned otaFiles = 0;
static unsigned otaFsBytes = 0;
static unsigned otaBytes = 0;
static unsigned otaTotal = 0;

/* Set by the httpd task (handleUpdatePost), consumed by loop(): the update itself
 * (enterMode, RX task handshake, display) must run in the main task. */
static std::atomic<bool> otaRequested{false};

static void otaResetProgress() {
  otaPhase = "starting";
  otaMsg = "";
  otaFiles = otaFsBytes = otaBytes = otaTotal = 0;
}

#define LOCALUDPPORT 9002
//Get real UTC time from NTP server
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0; //UTC
const int   daylightOffset_sec = 0; //UTC

// Authentication management
// defaultUserLevel is read from user.txt
uint8_t defaultUserLevel = PERM_ADMIN;

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

// Local (custom server) updates: enabled by LOCAL_UPDATE_URL (features.h) or a non-empty first
// line in "localupd.txt"; localUpdates then holds the URL that pre-fills the update form.
// By default (localUpdates==NULL) this is disabled to prevent abuse
// Note: by enabling this, someone with access to the web interface can replace the firmware arbitrarily!
// Make sure that only trustworthy persons have access to the web interface...
const char *localUpdates = NULL;

boolean forceReloadScreenConfig = false;

static int lastDisplay = 1;
static int currentDisplay = 1;

// timestamp when spectrum display was activated
static unsigned long specTimer;

void enterMode(int mode);


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
  int n = stream.readBytesUntil('\n', buffer, maxlen-1);
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
            "{\"channel\":%d, \"active\":%d, \"freq\":%.3f, \"launchsite\":\"%s\", \"type\":\"%s\", \"typeCode\":\"%c\"}",
            i+1, si->active, si->freq, si->launchsite, sondeTypeStr[si->type], sondeTypeChar[si->type]);
  }
  strcat(ptr, "]}");
  return message;
}

/* Escape for JSON string values into dst (NUL-terminated). */
static void jsonEscapeInto(char *dst, size_t dst_cap, const char *src)
{
  if (dst_cap == 0) {
    return;
  }
  size_t o = 0;
  if (!src) {
    dst[0] = '\0';
    return;
  }
  while (*src && o + 1 < dst_cap) {
    unsigned char c = (unsigned char)*src++;
    if (c == '"' || c == '\\') {
      if (o + 2 >= dst_cap) {
        break;
      }
      dst[o++] = '\\';
      dst[o++] = (char)c;
    } else if (c < 0x20) {
      if (o + 6 >= dst_cap) {
        break;
      }
      snprintf(dst + o, dst_cap - o, "\\u%04x", c);
      o = strlen(dst);
    } else {
      dst[o++] = (char)c;
    }
  }
  dst[o] = '\0';
}

/* Read JSON string value for key "name" starting search at *pp; advance *pp past value. */
static bool jsonNextStringField(const char **pp, const char *name, char *out, size_t out_cap)
{
  if (!pp || !*pp || !name || out_cap == 0) {
    return false;
  }
  char keypat[48];
  snprintf(keypat, sizeof(keypat), "\"%s\"", name);
  const char *p = strstr(*pp, keypat);
  if (!p) {
    return false;
  }
  p += strlen(keypat);
  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
    p++;
  }
  if (*p != ':') {
    return false;
  }
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
    p++;
  }
  if (*p != '"') {
    return false;
  }
  p++;
  size_t o = 0;
  while (*p && *p != '"') {
    char c = *p++;
    if (c == '\\' && *p) {
      char e = *p++;
      if (e == 'n') {
        c = '\n';
      } else if (e == 'r') {
        c = '\r';
      } else if (e == 't') {
        c = '\t';
      } else {
        c = e;
      }
    }
    if (o + 1 < out_cap) {
      out[o++] = c;
    }
  }
  out[o] = '\0';
  if (*p == '"') {
    p++;
  }
  *pp = p;
  return true;
}

const char *getControlActionsJson(void)
{
  static char buf[160];
  strcpy(buf, "[\"rx\",\"scan\",\"spec\",\"wifi\",\"rx2\",\"scan2\",\"spec2\",\"wifi2\"");
#if FEATURE_RS92
  strcat(buf, ",\"rinex\"");
#endif
#if FEATURE_SDCARD
  strcat(buf, ",\"format\"");
#endif
  strcat(buf, ",\"reboot\"]");
  return buf;
}

const char *runControlCmd(const char *param)
{
  if (!param) {
    return "missing cmd";
  }
  if (strcmp(param, "rx") == 0) {
    keyboardInject(EVT_KEY1SHORT);
  } else if (strcmp(param, "scan") == 0) {
    keyboardInject(EVT_KEY1DOUBLE);
  } else if (strcmp(param, "spec") == 0) {
    keyboardInject(EVT_KEY1MID);
  } else if (strcmp(param, "wifi") == 0) {
    keyboardInject(EVT_KEY1LONG);
  } else if (strcmp(param, "rx2") == 0) {
    keyboardInject(EVT_KEY2SHORT);
  } else if (strcmp(param, "scan2") == 0) {
    keyboardInject(EVT_KEY2DOUBLE);
  } else if (strcmp(param, "spec2") == 0) {
    keyboardInject(EVT_KEY2MID);
  } else if (strcmp(param, "wifi2") == 0) {
    keyboardInject(EVT_KEY2LONG);
  } else if (strcmp(param, "rinex") == 0) {
    keyboardInject(EVT_RINEX);
  } else if (strcmp(param, "format") == 0) {
    keyboardInject(EVT_FORMAT);
  } else if (strcmp(param, "reboot") == 0) {
    // Restart only after the HTTP response is out (httpdRebootIfPending): a request that
    // gets no answer may be resent by the browser after the reboot -> boot loop.
    otaPhase = "reboot";
  } else {
    return "unknown cmd";
  }
  return "";
}

const char *handleControlJson(const char *body)
{
  char cmd[24];
  const char *p = body ? body : "";
  if (!jsonNextStringField(&p, "cmd", cmd, sizeof(cmd))) {
    return "bad cmd";
  }
  return runControlCmd(cmd);
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


/////////////////// Wi-Fi network list (owned by NetMgr)

#define MAX_WIFI 10

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
  int n = netmgr.count();
  for (int i = 0; i < MAX_WIFI; i++) {
    String pw = i < n ? quoteString( netmgr.password(i) ) : "";
    sprintf(tmp, "%d", i);
    sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"S%d\" type=\"text\" value=\"%s\"/></td>"
            "<td><input name=\"P%d\" type=\"text\" value=\"%s\"/></td>",
            i == 0 ? "<b>AP</b>" : tmp,
            i + 1, i < n ? netmgr.ssid(i) : "",
            i + 1, pw.c_str() );
  }
  strcat(ptr, "</table><script>footer()</script>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  LOG_D(TAG, "WIFI form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleWIFIPost(void) {
  char label[10];
  File f = LittleFS.open("/networks.txt", "w");
  if (!f) {
    LOG_E(TAG, "Error while opening '/networks.txt' for writing");
    return "Error while opening '/networks.txt' for writing";
  }
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
  netmgr.reload();
  return "";
}

const char *getWifiAsJson(void)
{
  char *ptr = message;
  strcpy(ptr, "{\"slots\":[");
  int n = netmgr.count();
  for (int i = 0; i < MAX_WIFI; i++) {
    char ssid[96];
    char pw[96];
    const char *id = (i < n) ? netmgr.ssid(i) : "";
    const char *pass = (i < n) ? netmgr.password(i) : "";
    jsonEscapeInto(ssid, sizeof(ssid), id);
    jsonEscapeInto(pw, sizeof(pw), pass);
    if (i > 0) {
      strcat(ptr, ",");
    }
    sprintf(ptr + strlen(ptr),
            "{\"role\":\"%s\",\"ssid\":\"%s\",\"password\":\"%s\"}",
            i == 0 ? "ap" : "sta", ssid, pw);
  }
  strcat(ptr, "]}");
  return message;
}

const char *handleWifiJsonPut(const char *body)
{
  if (!body) {
    return "bad body";
  }
  File f = LittleFS.open("/networks.txt", "w");
  if (!f) {
    return "Error while opening '/networks.txt' for writing";
  }
  const char *p = strstr(body, "\"slots\"");
  if (!p) {
    f.close();
    return "missing slots";
  }
  p = strchr(p, '[');
  if (!p) {
    f.close();
    return "bad slots";
  }
  p++;
  for (int i = 0; i < MAX_WIFI; i++) {
    const char *obj = strchr(p, '{');
    if (!obj) {
      break;
    }
    const char *end = strchr(obj, '}');
    if (!end) {
      break;
    }
    char objbuf[256];
    size_t n = (size_t)(end - obj + 1);
    if (n >= sizeof(objbuf)) {
      n = sizeof(objbuf) - 1;
    }
    memcpy(objbuf, obj, n);
    objbuf[n] = '\0';
    const char *scan = objbuf;
    char ssid[64];
    char pw[64];
    ssid[0] = pw[0] = '\0';
    if (!jsonNextStringField(&scan, "ssid", ssid, sizeof(ssid))) {
      p = end + 1;
      continue;
    }
    scan = objbuf;
    (void)jsonNextStringField(&scan, "password", pw, sizeof(pw));
    if (ssid[0]) {
      f.printf("%s\n%s\n", ssid, pw);
    }
    p = end + 1;
  }
  f.close();
  netmgr.reload();
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
  } else {
    /* Hz, same units as sonde2json / OLED (displayed as kHz). */
    sprintf(ptr + strlen(ptr), ",\"afc\":%ld", (long)s->afc);
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

  uint32_t lastChange = sonde.lastViewChangeMs;
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    SondeInfo *s = &sonde.sondeList[i];
    /* Include hist-seeded slots (validID) even before a live lastRx update. */
    if (!s->active || (s->lastRx == 0 && !s->d.validID)) continue;
    if (s->lastRx != 0 && (ms_after(s->lastRx, lastChange) || lastChange == 0)) {
      lastChange = s->lastRx;
    }
  }

  strcpy(ptr, "{");
  sprintf(ptr + strlen(ptr),
          "\"now\":%u,\"lastChange\":%u,\"full\":%s,\"state\":\"%s\",\"detail\":\"",
          (unsigned)now, (unsigned)lastChange, full ? "true" : "false", state);
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
    if (!s->active || (s->lastRx == 0 && !s->d.validID)) continue;
    decoded++;
    if (!full && s->lastRx != 0 && !ms_after(s->lastRx, since)) continue;
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
  /* LittleFS last RX / light tracks: 0=off 1=last_rx 2=last_rx+lt_* */
  {"hist.mode", 0, &sonde.config.hist.mode},
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

/* Keep a config value inside the cf.set("...", "...") script on /config.html. */
static void appendJsString(char *dst, const char *src) {
  char *p = dst + strlen(dst);
  if (!src) src = "";
  for (; *src; src++) {
    unsigned char c = (unsigned char)*src;
    if (c == '"' || c == '\\') {
      *p++ = '\\';
      *p++ = (char)c;
    } else if (c == '<') {
      memcpy(p, "\\u003c", 6);
      p += 6;
    } else if (c < 0x20) {
      p += sprintf(p, "\\u%04x", c);
    } else {
      *p++ = (char)c;
    }
  }
  *p = 0;
}

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
        appendJsString(ptr, (char *)config_list[i].data);
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

static int configRawValue(int i, char *dst, int cap)
{
  if (cap < 1) return -1;
  dst[0] = '\0';
  switch (config_list[i].type) {
    case -4:
    case -3:
    case -2:
    case 0:
      return snprintf(dst, cap, "%d", *(int *)config_list[i].data);
    case -6: {
      int8_t *l = (int8_t *)config_list[i].data;
      int n;
      if (*l == -1) return snprintf(dst, cap, "0");
      n = snprintf(dst, cap, "%d", l[0]);
      l++;
      while (*l != -1 && n >= 0 && n < cap - 1) {
        int k = snprintf(dst + n, cap - n, ",%d", *l);
        if (k < 0) return k;
        n += k;
        l++;
      }
      return n;
    }
    case -7:
      if (isnan(*(double *)config_list[i].data)) return 0;
      return snprintf(dst, cap, "%.8g", *(double *)config_list[i].data);
    default:
      return snprintf(dst, cap, "%s", (char *)config_list[i].data);
  }
}

static bool jsonQuote(char **p, char *end, const char *src)
{
  char *w = *p;
  if (w >= end) return false;
  *w++ = '"';
  if (!src) src = "";
  for (; *src; src++) {
    unsigned char c = (unsigned char)*src;
    if (c == '"' || c == '\\') {
      if (w + 2 >= end) return false;
      *w++ = '\\';
      *w++ = (char)c;
    } else if (c < 0x20) {
      continue;
    } else {
      if (w + 1 >= end) return false;
      *w++ = (char)c;
    }
  }
  if (w + 1 >= end) return false;
  *w++ = '"';
  *w = '\0';
  *p = w;
  return true;
}

const char *createConfigJson(void)
{
  char *p = message;
  char *end = message + sizeof(message);
  char value[160];
  int n = snprintf(p, end - p, "{\"screenIndex\":%d,\"layouts\":[",
                   Display::getScreenIndex(sonde.config.screenfile));
  if (n < 0 || n >= (int)(end - p)) return "{\"error\":\"truncated\"}";
  p += n;
  int nLay = disp.layouts ? disp.nLayouts : 0;
  for (int i = 0; i < nLay; i++) {
    if (i && p < end) *p++ = ',';
    if (!jsonQuote(&p, end, disp.layouts[i].label)) return "{\"error\":\"truncated\"}";
  }
  if (p + 12 >= end) return "{\"error\":\"truncated\"}";
  memcpy(p, "],\"values\":{", 12);
  p += 12;
  *p = '\0';
  for (int i = 0; i < N_CONFIG; i++) {
    if (p + 1 >= end) return "{\"error\":\"truncated\"}";
    if (i) *p++ = ',';
    if (!jsonQuote(&p, end, config_list[i].name)) return "{\"error\":\"truncated\"}";
    if (p >= end) return "{\"error\":\"truncated\"}";
    *p++ = ':';
    if (configRawValue(i, value, sizeof(value)) < 0) return "{\"error\":\"truncated\"}";
    if (!jsonQuote(&p, end, value)) return "{\"error\":\"truncated\"}";
  }
  if (p + 2 >= end) return "{\"error\":\"truncated\"}";
  *p++ = '}';
  *p++ = '}';
  *p = '\0';
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
    (void)runControlCmd(param);
  }
  return "";
}

/* IDF httpd entry points  */
const char *httpdHandleQRGPost(void) { return handleQRGPost(); }
const char *httpdHandleWIFIPost(void) { return handleWIFIPost(); }
const char *httpdHandleConfigPost(void) { return handleConfigPost(); }
const char *httpdHandleControlPost(void) { return handleControlPost(); }
const char *httpdGetWifiAsJson(void) { return getWifiAsJson(); }
const char *httpdHandleWifiJsonPut(const char *body) { return handleWifiJsonPut(body); }
const char *httpdHandleControlJson(const char *body) { return handleControlJson(body); }
const char *httpdGetControlActionsJson(void) { return getControlActionsJson(); }
/* Called by the httpd task after a control response was sent. Restarts from the httpd task,
 * so it works even if the main loop is stuck. The delay lets lwIP deliver the response. */
void httpdRebootIfPending(void) {
  if (strcmp(otaPhase, "reboot") == 0) {
    LOG_W(TAG, "Rebooting (requested via web)\n");
    delay(1000);
    ESP.restart();
  }
}

/* /update.json — OTA progress for the wait page. */
const char *createUpdateJson(void) {
  static char buf[192];
  unsigned pct = otaTotal ? (otaBytes * 100u / otaTotal) : 0;
  snprintf(buf, sizeof(buf),
           "{\"phase\":\"%s\",\"files\":%u,\"fsBytes\":%u,\"bytes\":%u,\"total\":%u,\"pct\":%u,\"msg\":\"%s\"}",
           otaPhase, otaFiles, otaFsBytes, otaBytes, otaTotal, pct, otaMsg ? otaMsg : "");
  return buf;
}

/* Reply after POST /update.html. The form itself is /upd.html. */
const char *createUpdateForm(void) {
  return
    "<!DOCTYPE html><html><head><meta charset=utf-8><meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<link rel=stylesheet href=style.css><title>Updating…</title></head><body>"
    "<div class=card style=\"max-width:28rem;margin:2rem auto;padding:1rem\">"
    "<h1>Firmware update</h1><p id=st>Starting…</p><pre id=dt style=\"white-space:pre-wrap\"></pre></div>"
    "<script>"
    "var seen=0;"
    "function tick(){"
    "fetch('/update.json',{credentials:'same-origin',cache:'no-store'}).then(function(r){return r.json()}).then(function(d){"
    "var p=d.phase||'idle',el=document.getElementById('st'),dt=document.getElementById('dt'),lines=[];"
    "if(p==='starting'||p==='fs'||p==='flash'||p==='reboot')seen=1;"
    "if(p==='error'){el.textContent=d.msg||'Update failed';dt.textContent='';return;}"
    "if(p==='idle'){if(seen){location.href='/';return;}el.textContent='Starting…';}"
    "else if(p==='reboot')el.textContent='Rebooting…';"
    "else if(p==='starting')el.textContent='Starting…';"
    "else if(p==='fs')el.textContent='Updating filesystem…';"
    "else if(p==='flash')el.textContent='Flashing firmware…';"
    "else el.textContent=p;"
    "if(d.files||d.fsBytes)lines.push('Filesystem: '+d.files+' files, '+d.fsBytes+' bytes');"
    "if(d.total)lines.push('Firmware: '+d.pct+'% ('+d.bytes+'/'+d.total+' bytes)');"
    "dt.textContent=lines.join('\\n');"
    "setTimeout(tick,1000);"
    "}).catch(function(){"
    "document.getElementById('st').textContent=seen?'Rebooting…':'Connection error — retrying…';"
    "document.getElementById('dt').textContent='';"
    "setTimeout(tick,1000);"
    "});"
    "}"
    "tick();"
    "</script></body></html>";
}

const char *httpdCreateUpdateJson(void) { return createUpdateJson(); }

// Reject an update request: no update starts; the progress page shows msg (phase "error").
static int updateReject(int status, const char *msg) {
  LOG_E(TAG, "Update request rejected: %s\n", msg);
  otaPhase = "error";
  otaMsg = msg;
  return status;
}

// Returns 0 if the update was started, otherwise an HTTP status (403/400).
int handleUpdatePost(void) {
  LOG_D(TAG, "Handling post request");
  // A second request must not change host/prefix while execOTA() uses them.
  if (otaRequested.load() || strcmp(otaPhase, "starting") == 0 || strcmp(otaPhase, "fs") == 0 ||
      strcmp(otaPhase, "flash") == 0) {
    LOG_E(TAG, "Update request rejected: update already running\n");
    return 409;  // keep otaPhase/otaMsg of the running update
  }
  // Prefer formGet so action/url order in the POST body does not matter
  // (url is listed before the local submit button in upd.html).
  if (formGet("local")) {
    // Never fall back to a main update if a local one was requested.
    if (!localUpdates) return updateReject(403, "Local updates are disabled");
    const char *raw = formGet("url");
    if (!raw || !raw[0]) return updateReject(400, "Update URL missing");
    // Note: strdup allocates memory that is never free'd.
    // Let's not care about this as we are going to reboot after the update anyway...
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
  } else if (formGet("dev2")) {
    updatePrefix = updatePrefixD;
  } else if (formGet("main")) {
    updatePrefix = updatePrefixM;
  } else {
    return updateReject(400, "Unknown update request");
  }
  LOG_I(TAG, "Updating: %s%s from %s:%d\n", updatePrefix, updateIno, updateHost, updatePort);
  otaResetProgress();  // progress page shows "Starting..." until loop() picks up the request
  otaRequested.store(true, std::memory_order_release);  // publishes updateHost/Prefix/Port
  return 0;
}

int httpdHandleUpdatePost(void) { return handleUpdatePost(); }
const char *httpdCreateUpdateForm(void) { return createUpdateForm(); }

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
  LOG_D(TAG, "GPX request");
  char *ptr = message;
  if (index < 0 || index >= MAXSONDE) {
    return "ERROR";
  }
  SondeInfo *si = &sonde.sondeList[index];
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
  LOG_D(TAG, "%s", message);
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
    delay(1);
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
    LOG_D(TAG, "Not muted");
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

#ifdef LOCAL_UPDATE_URL
#warning "LOCAL_UPDATE_URL is set - do not use for release builds"
static_assert(sizeof(LOCAL_UPDATE_URL) > 1, "LOCAL_UPDATE_URL must not be empty");
#endif

static void enableLocalUpdates() {
#ifdef LOCAL_UPDATE_URL
  localUpdates = LOCAL_UPDATE_URL;
#else
  localUpdates = NULL;
#endif
  File local = LittleFS.open("/localupd.txt", "r");
  if (local && local.available()) {
    String url = readLine(local);
    url.trim();
    if (url.length() > 0) localUpdates = strdup(url.c_str());  // file overrides the define
  }
  LOG_I(TAG, "Local update server for development: %s\n", localUpdates?localUpdates:"<disabled>");
}

void setup()
{
  char buf[12];

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  //Serial.begin(921600);
  Log.init();

  {
    char gbuf[256];
    int n = 0;
    for (int i = 0; i < 39 && n < (int)sizeof(gbuf) - 8; i++) {
      n += snprintf(gbuf + n, sizeof(gbuf) - n, "%d:%d ", i, gpio_get_level((gpio_num_t)i));
    }
    LOG_D(TAG, "%s", gbuf);
  }
  LOG_I(TAG, "sizeof long is %d\n", sizeof(long));

#ifndef REMOVE_ALL_FOR_TESTING

#ifdef ESP_MEM_DEBUG
  esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);
#endif
  axpSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(axpSemaphore);

  {
    char gbuf[256];
    int n = 0;
    for (int i = 0; i < 39 && n < (int)sizeof(gbuf) - 8; i++) {
      n += snprintf(gbuf + n, sizeof(gbuf) - n, "%d:%d ", i, initlevels[i]);
    }
    LOG_D(TAG, "%s", gbuf);
  }
  LOG_D(TAG, "(before setup)");
  sonde.defaultConfig();  // including autoconfiguration

  delay(1000);
  LOG_I(TAG, "Initializing LittleFS");
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    LOG_E(TAG, "An Error has occurred while mounting LittleFS");
    return;
  }

  LOG_I(TAG, "Reading initial configuration");
  setupConfigData();    // configuration must be read first due to OLED ports!!!

  startupPower();

  keyboardInit();

  disp.init();
  delay(100);
  LOG_D(TAG, "Showing welcome display");
  disp.rdis->welcome();
  delay(3000);
  LOG_D(TAG, "Clearing display");
  sonde.clearDisplay();

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

  LOG_I(TAG, "Setup finished");
  // int returnValue = pthread_create(&wifithread, NULL, wifiloop, (void *)0);

  //  if (returnValue) {
  //     Serial.println("An error has occurred");
  //  }
  //   xTaskCreate(mainloop, "MainServer", 10240, NULL, 10, NULL);


  // == setup default channel list if qrg.txt read fails =========== //
  sonde.clearSonde();
  setupChannelList();
  sondeHistLoad();
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

  getKeyPressEvent();    // clear key buffer

#else
/// DEBUG ONLY
  sonde.config.sd.cs = 13;
  sonde.config.sd.clk = 14;
  sonde.config.sd.miso = 2;
  sonde.config.sd.mosi = 15;
  connSDCard.init();
#endif
}

void enterMode(int mode) {
  LOG_D(TAG, "enterMode(%d)\n", mode);
  if (mode == ST_UPDATE) {
    otaResetProgress();
  }
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
    LOG_I(TAG, "Entering ST_SPECTRUM mode");
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
    LOG_D(TAG, "clearing and updating display");
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
static char rdzData[RDZ_DATA_LEN + 1];  // + NUL; rdzDataPos == RDZ_DATA_LEN marks an overlong line
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
    LOG_I(TAG, "TCP JSON socket: new connection");
    rdzclient.stop();
    rdzclient = rdzserver.accept();
  }
  if (rdzclient.available()) {
    LOG_D(TAG, "RDZ JSON socket: received data");
    while (rdzclient.available()) {
      char c = (char)rdzclient.read();
      if (c == '\n' || c == '}') {
        // parse GPS position from phone; an overlong (truncated) line is dropped, not parsed
        if (rdzDataPos > 2 && rdzDataPos < RDZ_DATA_LEN) {
          rdzData[rdzDataPos++] = c;
          rdzData[rdzDataPos] = '\0';
          parseGpsJson(rdzData, rdzDataPos);
        }
        rdzDataPos = 0;
      } else if (rdzDataPos < RDZ_DATA_LEN) {
        rdzData[rdzDataPos++] = c;
      }
    }
  }

  // wifi active and good packet received => send packet
  SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
  if ((res & 0xff) == 0) {
    /* Local LittleFS history does not need WiFi. */
    if (s->d.validID && VALIDPOS(s->d.validPos))
      sondeHistUpdate(s);
  }
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
  sondeHistTick();

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
    LOG_D(TAG, "Sending position via TCP as rdzJSON");
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
      LOG_W(TAG, "Writing rdzClient not OK, closing connection");
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
      LOG_D(TAG, "loopSpectrum: KP_LONG");
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
      if (name && strcmp(name, "user.txt") == 0) {
        defaultUserLevel = getDefaultAuthLevel();
      }
    });
    httpd_start_server();
    udp.begin(IPAddress((uint32_t)0), LOCALUDPPORT);
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("jsonrdz", "tcp", 14570);
    rdzserver.begin();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    connected = true;
    Log.netUp();
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
    Log.netDown();
    connected = false;
  }

  LOG_I(TAG, "enableNetwork done");
}

static void applyNetEdge(NetEdge e) {
  if (e == NetEdge::None) return;
  if (e == NetEdge::ServicesUp) {
    char ip[24];
    bool isAp = (netmgr.status() == NetStatus::Ap);
    if (netmgr.ipv4(ip, sizeof(ip))) {
      sonde.setIP(ip, isAp);
      sonde.updateDisplayIP();
    }
    enableNetwork(true);
  } else if (e == NetEdge::ServicesDown) {
    sonde.setIP("", false);
    sonde.updateDisplayIP();
    enableNetwork(false);
  }
}

void initialMode() {
  if (sonde.config.touch_thresh == 0) {
    enterMode(ST_TOUCHCALIB);
    return;
  }
  if (sonde.config.spectrum != -1) {
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

/* Wifi modes (config.wifi):
 * 0: off
 * 1/3: station (sync wait), AP+STA fallback on failure
 * 2: AP only
 * 4: direct connect cycle
 * 5: mode 4 with softAP first (key pressed before start; not stored in config)
 */
#define MAXWIFIDELAY 40
static const char* _scan[2] = {"/", "\\"};
void loopWifiScan() {
  getKeyPressEvent();
  enableNetwork(false);

  disp.rdis->setFont(FONT_SMALL);
  uint8_t dispw, disph, dispxs, dispys;
  disp.rdis->getDispSize(&disph, &dispw, &dispys, &dispxs);
  int lastl = (disph / dispys - 2) * dispys;

  int mode = sonde.config.wifi;
  if (mode == 0) {
    netmgr.start(0, sonde.config.mdnsname);
    initialMode();
    return;
  }

  /* Mode 4: one key check before start -> mode 5 */
  if (mode == 4) {
    bool key = (getKeyPressEvent() != EVT_NONE);
    if (key) mode = 5;
  }

  netmgr.reload();

  if (mode == 2 || mode == 5) {
    disp.rdis->drawString(0, 0, "WiFi...");
    disp.rdis->drawString(0, dispys * 2, "AP");
    if (netmgr.count() > 0)
      disp.rdis->drawString(0, dispys * 3, netmgr.ssid(0));
  } else if (mode == 4) {
    disp.rdis->drawString(0, 0, "WiFi...");
    disp.rdis->drawString(0, dispys * 2, "Direct");
    if (netmgr.count() > 1)
      disp.rdis->drawString(0, dispys * 3, netmgr.ssid(1));
  } else {
    disp.rdis->drawString(0, 0, "WiFi Scan...");
  }

  netmgr.start(mode, sonde.config.mdnsname);

  /* Modes that return immediately once AP/background is running */
  if (mode == 2) {
    int cnt = 0;
    while (netmgr.status() != NetStatus::Ap && cnt < 20) {
      applyNetEdge(netmgr.poll());
      delay(100);
      cnt++;
    }
    applyNetEdge(netmgr.poll());
    if (netmgr.status() == NetStatus::Ap && !connected)
      applyNetEdge(NetEdge::ServicesUp);
    initialMode();
    return;
  }

  /* Sync wait for station modes 1/3/4 (and 5 until AP is up) */
  int cnt = 0;
  while (cnt < MAXWIFIDELAY) {
    applyNetEdge(netmgr.poll());
    NetStatus st = netmgr.status();
    if (st == NetStatus::Uplink) {
      char ip[24];
      if (netmgr.ipv4(ip, sizeof(ip))) {
        LOG_I(TAG, "WiFi connected, IP %s", ip);
        sonde.setIP(ip, false);
        sonde.updateDisplayIP();
      }
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
      if (!connected)
        applyNetEdge(NetEdge::ServicesUp);
      delay(1000);
      initialMode();
      return;
    }
    if (st == NetStatus::Ap) {
      /* fallback / timed AP running — leave wait */
      char ip[24];
      if (netmgr.ipv4(ip, sizeof(ip))) {
        sonde.setIP(ip, true);
        sonde.updateDisplayIP();
        disp.rdis->drawString(0, lastl, "AP:");
        disp.rdis->drawString(6 * dispxs, lastl + 1, netmgr.ssid(0));
      }
      if (!connected)
        applyNetEdge(NetEdge::ServicesUp);
      delay(1000);
      initialMode();
      return;
    }
    disp.rdis->drawString(15 * dispxs, lastl + dispys, _scan[cnt & 1]);
    delay(500);
    handlePMUirq();
    cnt++;
  }

  /* Timeout: background continues inside netmgr */
  LOG_W(TAG, "WiFi startup wait timed out, continuing in background");
  initialMode();
}

#if FEATURE_RS92
// Rinex update...
void execRinexUpdate() {
   LOG_I(TAG, "Fetching update for RINEX data...");
   geteph();
   LOG_I(TAG, "Reading RINEX data...");
   if (ephstate == EPH_PENDING) ephstate = EPH_ERROR;
   get_eph("/brdc");
   setCurrentDisplay(0);
   enterMode(ST_DECODER);
}
#endif

#if FEATURE_SDCARD
void execFormatSD() {
   LOG_I(TAG, "format SD card");
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


static void otaFail(const char *msg) {
  otaPhase = "error";
  otaMsg = msg;
  enterMode(ST_DECODER);
}

// OTA Logic
void execOTA() {
  int contentLength = 0;
  bool isValidContentType = false;
  sonde.clearDisplay();
  uint8_t dispxs, dispys;
  LOG_I(TAG, "Updater connecting to: %s:%d%s\n", updateHost, updatePort, updatePrefix);

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
    otaFail("Update host unreachable");
    return;
  }

  // First, try update file system
  LOG_I(TAG, "Fetching fs update from '%s:%d' '%s' '%s'\n", updateHost, updatePort, updatePrefix, updateFs);
  disp.rdis->drawString(0, 1 * dispys, "Fetching fs...");
  // HTTP/1.0: the server must not use chunked transfer encoding (the parser below cannot handle it)
  client.printf("GET %s%s HTTP/1.0\r\n"
                "Host: %s:%d\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n", updatePrefix, updateFs, updateHost, updatePort);

  // Body: repeated "<name> <size>\n" + <size> bytes. Its end is given by Content-Length,
  // not by client.available() (which is 0 whenever the network pauses briefly).
  // Any failure aborts the whole update, so the firmware is never updated without the
  // matching web UI. A file system aborted part-way is fixed by running the update again.
  int type = 0;
  int total = fetchHTTPheader(&type);  // Content-Length; -1 on error or if missing
  if (total < 0) {
    client.stop();
    otaFail("Filesystem download failed");
    return;
  }
  otaPhase = "fs";
  while (total > 0) {
    // header line "name size": leave room for the leading '/' and the NUL
    char fn[128];
    fn[0] = '/';
    size_t n = client.readBytesUntil('\n', fn + 1, sizeof(fn) - 2);
    fn[1 + n] = '\0';
    total -= n + 1;
    char *sz = strchr(fn, ' ');
    if (!sz) {
      client.stop();
      otaFail("Filesystem update truncated");
      return;
    }
    *sz = 0;
    int len = atoi(sz + 1);
    total -= len;
    if (len < 0 || total < 0) {
      client.stop();
      otaFail("Filesystem update corrupt");
      return;
    }
    LOG_I(TAG, "Updating file %s (%d bytes)\n", fn, len);
    char fnstr[17];
    memset(fnstr, ' ', 16);
    strncpy(fnstr, fn, 16);
    fnstr[16] = 0;
    disp.rdis->drawString(0, 2 * dispys, fnstr);
    File f = LittleFS.open(fn, FILE_WRITE);  // closed by its destructor, also on return
    if (!f) {
      client.stop();
      otaFail("Cannot write file");
      return;
    }
    while (len > 0) {
      unsigned char buf[1024];
      size_t want = len > 1024 ? 1024 : len;
      // readBytes() waits (with delay) until 'want' bytes arrived, error, or timeout
      if (client.readBytes((char *)buf, want) != want) {
        client.stop();
        otaFail("Filesystem download failed");
        return;
      }
      if (f.write(buf, want) != want) {
        client.stop();
        otaFail("Filesystem full or write error");
        return;
      }
      otaFsBytes += (unsigned)want;
      len -= want;
    }
    otaFiles++;
  }
  client.stop();

  // Connect to Update host
  if (!client.connect(updateHost, updatePort)) {
    LOG_E(TAG, "Connection to %s:%d for app update failed\n", updateHost, updatePort);
    otaFail("Firmware host unreachable");
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

  int validType = 0;
  contentLength = fetchHTTPheader( &validType );
  if (validType == 1) isValidContentType = true;

  // Check what is the contentLength and if content type is `application/octet-stream`
  LOG_I(TAG, "contentLength : %d, isValidContentType : %d\n", contentLength, isValidContentType);
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
      LOG_I(TAG, "Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      otaPhase = "flash";
      otaTotal = (unsigned)contentLength;
      otaBytes = 0;
      Update.onProgress([](size_t done, size_t total) {
        otaBytes = (unsigned)done;
        otaTotal = (unsigned)total;
      });
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        LOG_I(TAG, "Written : %u successfully", (unsigned)written);
      } else {
        LOG_W(TAG, "Written only : %u/%d. Retry?", (unsigned)written, contentLength);
      }

      if (Update.end()) {
        LOG_I(TAG, "OTA done!");
        if (Update.isFinished()) {
          LOG_I(TAG, "Update successfully completed. Rebooting.");
          disp.rdis->drawString(0, 7 * dispys, "Rebooting....");
          otaPhase = "reboot";
          delay(1000);
          ESP.restart();
        } else {
          LOG_E(TAG, "Update not finished? Something went wrong!");
          otaFail("Firmware update incomplete");
          return;
        }
      } else {
        LOG_E(TAG, "Error Occurred. Error #: %d", (int)Update.getError());
        otaFail("Firmware write failed");
        return;
      }
    } else {
      LOG_E(TAG, "Not enough space to begin OTA");
      client.clear();
      otaFail("Not enough space for OTA");
      return;
    }
  } else {
    LOG_E(TAG, "There was no content in the response");
    client.clear();
    otaFail("No firmware content");
    return;
  }
  // Back to some normal state
  otaFail("Update aborted");
}

int fetchHTTPheader(int *validType) {
  int contentLength = -1;
  // ms without data before readStringUntil()/readBytes*() give up; also used for the body
  client.setTimeout(5000);
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      LOG_E(TAG, "Client Timeout !");
      client.stop();
      return -1;
    }
    delay(10);  // let other tasks (and the WDT idle task) run while waiting
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
  // not available(): a header split across TCP segments must not end the loop early
  while (client.connected() || client.available()) {
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
    if (line.startsWith("HTTP/1.")) {  // server may answer 1.0 or 1.1
      if (line.indexOf("200") < 0) {
        LOG_E(TAG, "Got a non 200 status code from server. Exiting OTA Update.");
        return -1;
      }
    }

    // extract headers here
    // Start with content length
    static const char *HEADER_CL = "Content-Length: ";
    if (strncasecmp( line.c_str(), HEADER_CL, strlen(HEADER_CL) ) == 0 ) {
      contentLength = atoi( line.c_str() + strlen(HEADER_CL) );
      LOG_I(TAG, "Got %d bytes from server", contentLength);
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
  Serial.println("");  // empty line in the log maybe skip if logging is off?
  LOG_I(TAG, "Running loop in state %d [currentDisp:%d, lastDisp:%d]. free heap: %u (8bit), %u (total), unused stack: %d\n",
                mainState, currentDisplay, lastDisplay,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                (unsigned)ESP.getFreeHeap(),
                uxTaskGetStackHighWaterMark(0));

  Log.handleImprov();

#ifndef REMOVE_ALL_FOR_TESTING
  // Update requested via web: switch modes here, in the main task (works from any mode)
  if (otaRequested.exchange(false, std::memory_order_acquire)) enterMode(ST_UPDATE);
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
  applyNetEdge(netmgr.poll());
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



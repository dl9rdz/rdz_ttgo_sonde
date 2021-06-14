#include <axp20x.h>

#include "features.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
//#include <U8x8lib.h>
//#include <U8g2lib.h>
#include <SPI.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <MicroNMEA.h>
#include <Ticker.h>
#include <SX1278FSK.h>
#include <Sonde.h>
#include <Display.h>
#include <Scanner.h>
#include <aprs.h>
#include "version.h"
#include "geteph.h"
#include "rs92gps.h"
#if FEATURE_MQTT
#include "mqtt.h"
#endif
#include "esp_heap_caps.h"
//#define ESP_MEM_DEBUG 1
int e;

enum MainState { ST_DECODER, ST_SPECTRUM, ST_WIFISCAN, ST_UPDATE, ST_TOUCHCALIB };
static MainState mainState = ST_WIFISCAN; // ST_WIFISCAN;
const char *mainStateStr[5] = {"DECODER", "SPECTRUM", "WIFISCAN", "UPDATE", "TOUCHCALIB" };

AsyncWebServer server(80);

AXP20X_Class axp;
#define PMU_IRQ             35
SemaphoreHandle_t axpSemaphore;
bool pmu_irq = false;

String updateHost = "rdzsonde.mooo.com";
int updatePort = 80;
String updateBinM = "/master/update.ino.bin";
String updateBinD = "/devel/update.ino.bin";
String *updateBin = &updateBinM;

#define LOCALUDPPORT 9002
//Get real UTC time from NTP server
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0; //UTC
const int   daylightOffset_sec = 0; //UTC

boolean connected = false;
WiFiUDP udp;
WiFiClient client;
#if FEATURE_SONDEHUB
#define SONDEHUB_STATION_UPDATE_TIME (60*60*1000) // 60 min
#define SONDEHUB_MOBILE_STATION_UPDATE_TIME (30*1000) // 30 sec
WiFiClient shclient;	// Sondehub v2
unsigned long time_last_update = 0;
#endif

// KISS over TCP for communicating with APRSdroid
WiFiServer tncserver(14580);
WiFiClient tncclient;
// JSON over TCP for communicating with my kotlin andoird test stuff
WiFiServer rdzserver(14570);
WiFiClient rdzclient;

#if FEATURE_MQTT
unsigned long lastMqttUptime = 0;
boolean mqttEnabled;
MQTT mqttclient;
#endif
boolean forceReloadScreenConfig = false;

enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

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
  Serial.println(var);
  if (var == "VERSION_NAME") {
    return String(version_name);
  }
  if (var == "VERSION_ID") {
    return String(version_id);
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
char message[10240 * 4]; //needs to be large enough for all forms (not checked in code)
// QRG form is currently about 24kb with 100 entries

///////////////////////// Functions for Reading / Writing QRG list from/to qrg.txt

void setupChannelList() {
  File file = SPIFFS.open("/qrg.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/qrg.txt' for reading");
    return;
  }
  int i = 0;
  char launchsite[17] = "                ";
  sonde.clearSonde();
  Serial.println("Reading channel config:");
  while (file.available()) {
    String line = readLine(file);   //file.readStringUntil('\n');
    String sitename;
    if (!file.available()) break;
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
    else if (space[1] == '9') {
      type = STYPE_DFM09_OLD;
    }
    else if (space[1] == '6') {
      type = STYPE_DFM06_OLD;
    }
    else if (space[1] == 'D') {
      type = STYPE_DFM;
    }
    else if (space[1] == 'M') {
      type = STYPE_M10;
    }
    else if (space[1] == '2') {
      type = STYPE_M20;
    }
    else if (space[1] == '3') {
      type = STYPE_MP3H;
    }
    else continue;
    int active = space[3] == '+' ? 1 : 0;
    if (space[4] == ' ') {
      memset(launchsite, ' ', 16);
      int str_len = strlen(space + 5);
      strncpy(launchsite, space + 5, str_len > 16 ? 16 : str_len);
      if (sonde.config.debug == 1) {
        Serial.printf("Add %f - sondetype: %d (on/off: %d) - site #%d - name: %s\n ", freq, type, active, i, launchsite);
      }
    }
    sonde.addSonde(freq, type, active, launchsite);
    i++;
  }
  file.close();
}

const char *HTMLHEAD = "<html><head> <meta charset=\"UTF-8\"> <link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\">";
void HTMLBODY(char *ptr, const char *which) {
  strcat(ptr, "<body><form class=\"wrapper\" action=\"");
  strcat(ptr, which);
  strcat(ptr, "\" method=\"post\"><div class=\"content\">");
}
void HTMLBODYEND(char *ptr) {
  strcat(ptr, "</div></form></body></html>");
}
void HTMLSAVEBUTTON(char *ptr) {
  strcat(ptr, "</div><div class=\"footer\"><input type=\"submit\" class=\"save\" value=\"Save changes\"/>");
}

const char *createQRGForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "<script src=\"rdz.js\"/>  <script> window.onload = prep; </script></head>");
  /*
    strcat(ptr, "<script type=\"text/javascript\">"
       "let stypes=new Map();"
       "stypes.set('4', 'RS41');"
       "stypes.set('R', 'RS92');"
       "stypes.set('9', 'DFM9 (old)');"
       "stypes.set('6', 'DFM6 (old)');"
       "stypes.set('D', 'DFM');"
       "stypes.set('M', 'M10');"
       "stypes.set('2', 'M20');"
       "function prep() {"
       " var stlist=document.querySelectorAll(\"input.stype\");"
       " for(txt of stlist){"
       "  var val=txt.getAttribute('value'); var nam=txt.getAttribute('name'); "
       "  var sel=document.createElement('select');"
       "  sel.setAttribute('name',nam);"
       "  for(stype of stypes) { "
       "   var opt=document.createElement('option');"
       "   opt.value=stype[0];"
       "   opt.innerHTML=stype[1];"
       "   if(stype[0]==val) { opt.setAttribute('selected','selected'); }"
       "   sel.appendChild(opt);"
       "  } txt.replaceWith(sel); } } "
       "  window.onload = prep; "
       "</script>");
  */
  HTMLBODY(ptr, "qrg.html");
  //strcat(ptr, "<body><form class=\"wrapper\" action=\"qrg.html\" method=\"post\"><div class=\"content\"><table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Launchsite</th><th>Mode</th></tr>");
  strcat(ptr, "<table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Launchsite</th><th>Mode</th></tr>");
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    //String s = sondeTypeSelect(i >= sonde.nSonde ? 2 : sonde.sondeList[i].type);
    String site = sonde.sondeList[i].launchsite;
    sprintf(ptr + strlen(ptr), "<tr><td>%d</td><td><input name=\"A%d\" type=\"checkbox\" %s/></td>"
            "<td><input name=\"F%d\" type=\"text\" width=12 value=\"%3.3f\"></td>"
            "<td><input name=\"S%d\" type=\"text\" value=\"%s\"></td>"
            //"<td><select name=\"T%d\">%s</select></td>",
            "<td><input class='stype' name='T%d' value='%c'>",
            i + 1,
            i + 1, (i < sonde.nSonde && sonde.sondeList[i].active) ? "checked" : "",
            i + 1, i >= sonde.nSonde ? 400.000 : sonde.sondeList[i].freq,
            i + 1, i >= sonde.nSonde ? "                " : sonde.sondeList[i].launchsite,
            i + 1, i >= sonde.nSonde ? 2 : sondeTypeChar[sonde.sondeList[i].type] );
    //i + 1, s.c_str());
  }
  strcat(ptr, "</table>");
  //</div><div class=\"footer\"><input type=\"submit\" class=\"update\" value=\"Update\"/>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  Serial.printf("QRG form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleQRGPost(AsyncWebServerRequest *request) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File file = SPIFFS.open("/qrg.txt", "w");
  if (!file) {
    Serial.println("Error while opening '/qrg.txt' for writing");
    return "Error while opening '/qrg.txt' for writing";
  }
#endif
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
    Serial.printf("Processing a=%s, f=%s, t=%s, site=%s\n", active ? "YES" : "NO", fstr, tstr, sstr);
    char typech = tstr[0];
    file.printf("%3.3f %c %c %s\n", atof(fstr), typech, active ? '+' : '-', sstr);
  }
  file.close();

  Serial.println("Channel setup finished");
  Serial.println();
  delay(500);
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

// FIXME: For now, we don't uspport wifi networks that contain newline or null characters
// ... would require a more sophisicated file format (currently one line SSID; one line Password
void setupWifiList() {
  File file = SPIFFS.open("/networks.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/networks.txt' for reading");
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
  Serial.print(i); Serial.println(" networks in networks.txt\n");
  for (int j = 0; j < i; j++) {
    Serial.print(networks[j].id);
    Serial.print(": ");
    Serial.println(networks[j].pw);
  }
}


const char *createWIFIForm() {
  char *ptr = message;
  char tmp[4];
  strcpy(ptr, HTMLHEAD); strcat(ptr, "</head>");
  HTMLBODY(ptr, "wifi.html");
  strcat(ptr, "<table><tr><th>Nr</th><th>SSID</th><th>Password</th></tr>");
  for (int i = 0; i < MAX_WIFI; i++) {
    sprintf(tmp, "%d", i);
    sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"S%d\" type=\"text\" value=\"%s\"/></td>"
            "<td><input name=\"P%d\" type=\"text\" value=\"%s\"/></td>",
            i == 0 ? "<b>AP</b>" : tmp,
            i + 1, i < nNetworks ? networks[i].id.c_str() : "",
            i + 1, i < nNetworks ? networks[i].pw.c_str() : "");
  }
  strcat(ptr, "</table>");
  //</div><div class=\"footer\"><input type=\"submit\" class=\"update\" value=\"Update\"/>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  Serial.printf("WIFI form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleWIFIPost(AsyncWebServerRequest *request) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File f = SPIFFS.open("/networks.txt", "w");
  if (!f) {
    Serial.println("Error while opening '/networks.txt' for writing");
    return "Error while opening '/networks.txt' for writing";
  }
#endif
  Serial.println("Handling post request");
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
    Serial.printf("Processing S=%s, P=%s\n", sstr, pstr);
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
  strcat(ptr, "<table>");
  sprintf(ptr + strlen(ptr), "<tr><td id=\"sfreq\">%3.3f MHz, Type: %s</td><tr><td>ID: %s", s->freq, sondeTypeLongStr[s->type],
          s->validID ? s->id : "<?""?>");
  if (s->validID && (TYPE_IS_DFM(s->type) || TYPE_IS_METEO(s->type) || s->type == STYPE_MP3H) ) {
    sprintf(ptr + strlen(ptr), " (ser: %s)", s->ser);
  }
  sprintf(ptr + strlen(ptr), "</td></tr><tr><td>QTH: %.6f,%.6f h=%.0fm</td></tr>\n", s->lat, s->lon, s->alt);
  const time_t t = s->time;
  ts = *gmtime(&t);
  sprintf(ptr + strlen(ptr), "<tr><td>Frame# %d, Sats=%d, %04d-%02d-%02d %02d:%02d:%02d</td></tr>",
          s->frame, s->sats, ts.tm_year + 1900, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec + s->sec);
  if (s->type == STYPE_RS41) {
    sprintf(ptr + strlen(ptr), "<tr><td>Burst-KT=%d Launch-KT=%d Countdown=%d (vor %ds)</td></tr>\n",
            s->burstKT, s->launchKT, s->countKT, ((uint16_t)s->frame - s->crefKT));
  }
  sprintf(ptr + strlen(ptr), "<tr><td><a target=\"_empty\" href=\"geo:%.6f,%.6f\">GEO-App</a> - ", s->lat, s->lon);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://wetterson.de/karte/?%s\">wetterson.de</a> - ", s->id);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://radiosondy.info/sonde_archive.php?sondenumber=%s\">radiosondy.info</a> - ", s->id);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://tracker.sondehub.org/%s\">SondeHub Tracker</a> - ", s->id);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://www.openstreetmap.org/?mlat=%.6f&mlon=%.6f&zoom=14\">OSM</a> - ", s->lat, s->lon);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://www.google.com/maps/search/?api=1&query=%.6f,%.6f\">Google</a></td></tr>", s->lat, s->lon);

  strcat(ptr, "</table><p/>\n");
}

const char *createStatusForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD);
  strcat(ptr, "<meta http-equiv=\"refresh\" content=\"5\"></head><body>");

  for (int i = 0; i < sonde.nSonde; i++) {
    int snum = (i + sonde.currentSonde) % sonde.nSonde;
    if (sonde.sondeList[snum].active) {
      addSondeStatus(ptr, snum);
    }
  }
  strcat(ptr, "</body></html>");
  Serial.printf("Status form: size=%d bytes\n", strlen(message));
  return message;
}

///////////////////// Config form


void setupConfigData() {
  File file = SPIFFS.open("/config.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/config.txt' for reading");
    return;
  }
  while (file.available()) {
    String line = readLine(file);  //file.readStringUntil('\n');
    sonde.setConfig(line.c_str());
  }
}


struct st_configitems {
  const char *name;
  const char *label;
  int type;  // 0: numeric; i>0 string of length i; -1: separator; -2: type selector
  void *data;
};

struct st_configitems config_list[] = {
  /* General config settings */
  {"", "Software configuration", -5, NULL},
  {"wifi", "Wifi mode (0/1/2/3)", 0, &sonde.config.wifi},
  {"debug", "Debug mode (0/1)", 0, &sonde.config.debug},
  {"maxsonde", "Maxsonde", 0, &sonde.config.maxsonde},
  {"screenfile", "Screen config (0=old, 1=OLED, 2=TFT, 3=TFT[port])", 0, &sonde.config.screenfile},
  {"display", "Display screens (scan,default,...)", -6, sonde.config.display},
  /* Spectrum display settings */
  {"spectrum", "Show spectrum (-1=no, 0=forever, >0=seconds)", 0, &sonde.config.spectrum},
  {"startfreq", "Startfreq (MHz)", 0, &sonde.config.startfreq},
  {"channelbw", "Bandwidth (kHz)", 0, &sonde.config.channelbw},
  {"marker", "Spectrum MHz marker", 0, &sonde.config.marker},
  {"noisefloor", "Spectrum noisefloor", 0, &sonde.config.noisefloor},
  /* decoder settings */
  {"", "Receiver configuration", -5, NULL},
  {"showafc", "Show AFC value", 0, &sonde.config.showafc},
  {"freqofs", "RX frequency offset (Hz)", 0, &sonde.config.freqofs},
  {"rs41.agcbw", "RS41 AGC bandwidth", 0, &sonde.config.rs41.agcbw},
  {"rs41.rxbw", "RS41 RX bandwidth", 0, &sonde.config.rs41.rxbw},
  {"rs92.rxbw", "RS92 RX (and AGC) bandwidth", 0, &sonde.config.rs92.rxbw},
  {"rs92.alt2d", "RS92 2D fix default altitude", 0, &sonde.config.rs92.alt2d},
  {"dfm.agcbw", "DFM AGC bandwidth", 0, &sonde.config.dfm.agcbw},
  {"dfm.rxbw", "DFM RX bandwidth", 0, &sonde.config.dfm.rxbw},
  {"m10m20.agcbw", "M10/M20 AGC bandwidth", 0, &sonde.config.m10m20.agcbw},
  {"m10m20.rxbw", "M10/M20 RX bandwidth", 0, &sonde.config.m10m20.rxbw},
  {"mp3h.agcbw", "MP3H AGC bandwidth", 0, &sonde.config.mp3h.agcbw},
  {"mp3h.rxbw", "MP3H RX bandwidth", 0, &sonde.config.mp3h.rxbw},
  {"ephftp", "FTP for eph (RS92)", 39, &sonde.config.ephftp},
  {"", "Data feed configuration", -5, NULL},
  /* APRS settings */
  {"call", "Call", 8, sonde.config.call},
  {"passcode", "Passcode", 8, sonde.config.passcode},
  /* KISS tnc settings */
  {"kisstnc.active", "KISS TNC (port 14590) (needs reboot)", 0, &sonde.config.kisstnc.active},
  {"kisstnc.idformat", "KISS TNC ID Format", -2, &sonde.config.kisstnc.idformat},
  /* AXUDP settings */
  {"axudp.active", "AXUDP active", -3, &sonde.config.udpfeed.active},
  {"axudp.host", "AXUDP Host", 63, sonde.config.udpfeed.host},
  {"axudp.port", "AXUDP Port", 0, &sonde.config.udpfeed.port},
  {"axudp.idformat", "DFM ID Format", -2, &sonde.config.udpfeed.idformat},
  {"axudp.highrate", "Rate limit", 0, &sonde.config.udpfeed.highrate},
  /* APRS TCP settings, current not used */
  {"tcp.active", "APRS TCP active", -3, &sonde.config.tcpfeed.active},
  {"tcp.host", "ARPS TCP Host", 63, sonde.config.tcpfeed.host},
  {"tcp.port", "APRS TCP Port", 0, &sonde.config.tcpfeed.port},
  {"tcp.idformat", "DFM ID Format", -2, &sonde.config.tcpfeed.idformat},
  {"tcp.highrate", "Rate limit", 0, &sonde.config.tcpfeed.highrate},

#if FEATURE_MQTT
  /* MQTT */
  {"mqtt.active", "MQTT Active (needs reboot)", 0, &sonde.config.mqtt.active},
  {"mqtt.id", "MQTT client ID", 63, &sonde.config.mqtt.id},
  {"mqtt.host", "MQTT server hostname", 63, &sonde.config.mqtt.host},
  {"mqtt.port", "MQTT Port", 0, &sonde.config.mqtt.port},
  {"mqtt.username", "MQTT Username", 63, &sonde.config.mqtt.username},
  {"mqtt.password", "MQTT Password", 63, &sonde.config.mqtt.password},
  {"mqtt.prefix", "MQTT Prefix", 63, &sonde.config.mqtt.prefix},
#endif

  /* Hardware dependeing settings */
  {"", "Hardware configuration (requires reboot)", -5, NULL},
  {"disptype", "Display type (0=OLED/SSD1306, 1=TFT/ILI9225, 2=OLED/SH1106)", 0, &sonde.config.disptype},
  {"norx_timeout", "No-RX-Timeout in seconds (-1=disabled)", 0, &sonde.config.norx_timeout},
  {"oled_sda", "OLED SDA/TFT SDA", 0, &sonde.config.oled_sda},
  {"oled_scl", "OLED SCL/TFT CLK", 0, &sonde.config.oled_scl},
  {"oled_rst", "OLED RST/TFT RST (needs reboot)", 0, &sonde.config.oled_rst},
  {"tft_rs", "TFT RS", 0, &sonde.config.tft_rs},
  {"tft_cs", "TFT CS", 0, &sonde.config.tft_cs},
  {"tft_orient", "TFT orientation (0/1/2/3), OLED flip: 3", 0, &sonde.config.tft_orient},
  {"tft_modeflip", "TFT modeflip (usually 0)", 0, &sonde.config.tft_modeflip},
  {"button_pin", "Button input port", -4, &sonde.config.button_pin},
  {"button2_pin", "Button 2 input port", -4, &sonde.config.button2_pin},
  {"button2_axp", "Use AXP192 PWR as Button 2", 0, &sonde.config.button2_axp},
  {"touch_thresh", "Touch button threshold<br>(0 for calib mode)", 0, &sonde.config.touch_thresh},
  {"power_pout", "Power control port", 0, &sonde.config.power_pout},
  {"led_pout", "LED output port", 0, &sonde.config.led_pout},
  {"gps_rxd", "GPS RXD pin (-1 to disable)", 0, &sonde.config.gps_rxd},
  {"gps_txd", "GPS TXD pin (not really needed)", 0, &sonde.config.gps_txd},
  {"mdnsname", "mDNS name", 14, &sonde.config.mdnsname},

#if FEATURE_SONDEHUB
  /* Sondehub v2 settings */
  {"", "Sondehub v2 settings", -5, NULL},
  {"sondehub.active", "Sondehub reporting active", 0, &sonde.config.sondehub.active},
  {"sondehub.chase", "Sondehub chase location active", 0, &sonde.config.sondehub.chase},
  {"sondehub.host", "Sondehub host", 63, &sonde.config.sondehub.host},
  {"sondehub.callsign", "Callsign", 63, &sonde.config.sondehub.callsign},
  {"sondehub.lat", "Latitude", 19, &sonde.config.sondehub.lat},
  {"sondehub.lon", "Longitude", 19, &sonde.config.sondehub.lon},
  {"sondehub.alt", "Altitude", 19, &sonde.config.sondehub.alt},
  {"sondehub.antenna", "Antenna", 63, &sonde.config.sondehub.antenna},
  {"sondehub.email", "Sondehub email", 63, &sonde.config.sondehub.email},
#endif
};
const static int N_CONFIG = (sizeof(config_list) / sizeof(struct st_configitems));

void addConfigStringEntry(char *ptr, int idx, const char *label, int len, char *field) {
  sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" value=\"%s\"/></td></tr>\n",
          label, idx, field);
}
void addConfigNumEntry(char *ptr, int idx, const char *label, int *value) {
  sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" value=\"%d\"/></td></tr>\n",
          label, idx, *value);
}
void addConfigButtonEntry(char *ptr, int idx, const char *label, int *value) {
  int v = *value, ck = 0;
  if (v == 255) v = -1;
  if (v != -1) {
    if (v & 128) ck = 1;
    v = v & 127;
  }

  sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" size=\"3\" value=\"%d\"/>",
          label, idx, v);
  sprintf(ptr + strlen(ptr), "<input type=\"checkbox\" name=\"TO%d\"%s> Touch </td></tr>\n", idx, ck ? " checked" : "");
}
void addConfigTypeEntry(char *ptr, int idx, const char *label, int *value) {
  // TODO
}
void addConfigOnOffEntry(char *ptr, int idx, const char *label, int *value) {
  // TODO
}
void addConfigSeparatorEntry(char *ptr) {
  strcat(ptr, "<tr><td colspan=\"2\" class=\"divider\"><hr /></td></tr>\n");
}
void addConfigHeading(char *ptr, const char *label) {
  strcat(ptr, "<tr><th colspan=\"2\">");
  strcat(ptr, label);
  strcat(ptr, "</th></tr>\n");
}
void addConfigInt8List(char *ptr, int idx, const char *label, int8_t *list) {
  sprintf(ptr + strlen(ptr), "<tr><td>%s", label);
  for (int i = 0; i < disp.nLayouts; i++) {
    sprintf(ptr + strlen(ptr), "<br>%d=%s", i, disp.layouts[i].label);
  }
  sprintf(ptr + strlen(ptr), "</td><td><input name=\"CFG%d\" type=\"text\" value=\"", idx);
  if (*list == -1) {
    strcat(ptr, "0");
  }
  else {
    sprintf(ptr + strlen(ptr), "%d", list[0]);
    list++;
  }
  while (*list != -1) {
    sprintf(ptr + strlen(ptr), ",%d", *list);
    list++;
  }
  strcat(ptr, "\"/></td></tr>\n");
}

const char *createConfigForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD); strcat(ptr, "</head>");
  HTMLBODY(ptr, "config.html");
  strcat(ptr, "<table><tr><th>Option</th><th>Value</th></tr>");
  for (int i = 0; i < N_CONFIG; i++) {
    switch (config_list[i].type) {
      case -5: // Heading
        addConfigHeading(ptr, config_list[i].label);
        break;
      case -6: // List of int8 values
        addConfigInt8List(ptr, i, config_list[i].label, (int8_t *)config_list[i].data);
        break;
      case -3: // in/offt
        addConfigOnOffEntry(ptr, i, config_list[i].label, (int *)config_list[i].data);
        break;
      case -2: // DFM format
        addConfigTypeEntry(ptr, i, config_list[i].label, (int *)config_list[i].data);
        break;
      case -1:
        addConfigSeparatorEntry(ptr);
        break;
      case 0:
        addConfigNumEntry(ptr, i, config_list[i].label, (int *)config_list[i].data);
        break;
      case -4:
        addConfigButtonEntry(ptr, i, config_list[i].label, (int *)config_list[i].data);
        break;
      default:
        addConfigStringEntry(ptr, i, config_list[i].label, config_list[i].type, (char *)config_list[i].data);
        break;
    }
  }
  strcat(ptr, "</table>");
  //</div><div class=\"footer\"><input type=\"submit\" class=\"update\" value=\"Update\"/>");
  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  Serial.printf("Config form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleConfigPost(AsyncWebServerRequest *request) {
  // parameters: a_i, f_1, t_i  (active/frequency/type)
  Serial.println("Handling post request");
#if 1
  File f = SPIFFS.open("/config.txt", "w");
  if (!f) {
    Serial.println("Error while opening '/config.txt' for writing");
    return "Error while opening '/config.txt' for writing";
  }
#endif
  Serial.println("File open for writing.");
#if 1
  int params = request->params();
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
  }
#endif
  for (int i = 0; i < params; i++) {
    String strlabel = request->getParam(i)->name();
    const char *label = strlabel.c_str();
    if (strncmp(label, "CFG", 3) != 0) continue;
    int idx = atoi(label + 3);
    Serial.printf("idx is %d\n", idx);
    if (config_list[idx].type == -1) continue; // skip separator entries, should not happen
    AsyncWebParameter *value = request->getParam(label, true);
    if (!value) continue;
    String strvalue = value->value();
    if (config_list[idx].type == -4) {  // input button port with "touch" checkbox
      char tmp[10];
      snprintf(tmp, 10, "TO%d", idx);
      AsyncWebParameter *touch = request->getParam(tmp, true);
      if (touch) {
        int i = atoi(strvalue.c_str());
        if (i != -1 && i != 255) i += 128;
        strvalue = String(i);
      }
    }
    Serial.printf("Processing  %s=%s\n", config_list[idx].name, strvalue.c_str());
    int wlen = f.printf("%s=%s\n", config_list[idx].name, strvalue.c_str());
    Serial.printf("Written bytes: %d\n", wlen);
  }
  Serial.printf("Flushing file\n");
  f.flush();
  Serial.printf("Closing file\n");
  f.close();
  Serial.printf("Re-reading file file\n");
  setupConfigData();
  return "";
}

const char *ctrlid[] = {"rx", "scan", "spec", "wifi", "rx2", "scan2", "spec2", "wifi2"};

const char *ctrllabel[] = {"Receiver/next freq. (short keypress)", "Scanner (double keypress)", "Spectrum (medium keypress)", "WiFi (long keypress)",
                           "Button 2/next screen (short keypress)", "Button 2 (double keypress)", "Button 2 (medium keypress)", "Button 2 (long keypress)"
                          };

const char *createControlForm() {
  char *ptr = message;
  strcpy(ptr, HTMLHEAD); strcat(ptr, "</head>");
  HTMLBODY(ptr, "control.html");
  for (int i = 0; i < 8; i++) {
    strcat(ptr, "<input class=\"ctlbtn\" type=\"submit\" name=\"");
    strcat(ptr, ctrlid[i]);
    strcat(ptr, "\" value=\"");
    strcat(ptr, ctrllabel[i]);
    strcat(ptr, "\"></input>");
    if (i == 3) {
      strcat(ptr, "<p></p>");
    }
  }
  HTMLBODYEND(ptr);
  Serial.printf("Control form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleControlPost(AsyncWebServerRequest *request) {
  Serial.println("Handling post request");
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
  }
  return "";
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static File file;
  if (!index) {
    Serial.printf("UploadStart: %s\n", filename.c_str());
    file = SPIFFS.open("/" + filename, "w");
    if (!file) {
      Serial.println("There was an error opening the file '/config.txt' for reading");
    }
  }
  if (!file) return;
  for (size_t i = 0; i < len; i++) {
    file.write(data[i]);
  }
  if (final) {
    Serial.printf("UploadEnd: %s, %u B\n", filename.c_str(), index + len);
    file.close();
  }
}


int streamEditForm(int &state, File &file, String filename, char *buffer, size_t maxlen, size_t index) {
  Serial.printf("streamEdit: state=%d  max:%d idx:%d\n", state, maxlen, index);
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
        Serial.printf("Wrote %d bytes. Header finished", i);
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
  File file = SPIFFS.open("/" + filename, "r");
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
  Serial.printf("Edit form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleEditPost(AsyncWebServerRequest *request) {
  Serial.println("Handling post request");
  int params = request->params();
  Serial.printf("Post:, %d params\n", params);
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
      Serial.printf("_FILE[%s]: %s, size: %u\n", name.c_str(), value.c_str(), p->size());
    } else if (p->isPost()) {
      Serial.printf("_POST[%s]: %s\n", name.c_str(), value.c_str());
    } else {
      Serial.printf("_GET[%s]: %s\n", name.c_str(), value.c_str());
    }
  }

  AsyncWebParameter *filep = request->getParam("file");
  if (!filep) return NULL;
  String filename = filep->value();
  Serial.printf("Writing file <%s>\n", filename.c_str());
  AsyncWebParameter *textp = request->getParam("text", true);
  if (!textp) return NULL;
  Serial.printf("Parameter size is %d\n", textp->size());
  Serial.printf("Multipart: %d  contentlen=%d  \n",
                request->multipart(), request->contentLength());
  String content = textp->value();
  if (content.length() == 0) {
    Serial.println("File is empty. Not written.");
    return NULL;
  }
  File file = SPIFFS.open("/" + filename, "w");
  if (!file) {
    Serial.println("There was an error opening the file '/" + filename + "'for writing");
    return "";
  }
  Serial.printf("File is open for writing, content is %d bytes\n", content.length());
  int len = file.print(content);
  file.close();
  Serial.printf("Written: %d bytes\n", len);
  if (strncmp(filename.c_str(), "screens", 7) == 0) {
    // screens update => reload
    forceReloadScreenConfig = true;
  }
  return "";
}

const char *createUpdateForm(boolean run) {
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"update.html\" method=\"post\">");
  if (run) {
    strcat(ptr, "<p>Doing update, wait until reboot</p>");
  } else {
    strcat(ptr, "<input type=\"submit\" name=\"master\" value=\"Master-Update\"></input><br><input type=\"submit\" name=\"devel\" value=\"Devel-Update\">");
  }
  strcat(ptr, "</form></body></html>");
  Serial.printf("Update form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleUpdatePost(AsyncWebServerRequest *request) {
  Serial.println("Handling post request");
  int params = request->params();
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
    if (param.equals("devel")) {
      Serial.println("equals devel");
      updateBin = &updateBinD;
    }
    else if (param.equals("master")) {
      Serial.println("equals master");
      updateBin = &updateBinM;
    }
  }
  Serial.println("Updating: " + *updateBin);
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

  if (!s->validID)
  {
    return;
  }

  sprintf(ptr + strlen(ptr), "<Placemark id=\"%s\"><name>%s</name><Point><coordinates>%.6f,%.6f,%.0f</coordinates></Point><description>%3.3f MHz, Type: %s, h=%.0fm</description></Placemark>",
          s->id, s->id,
          s->lon, s->lat, s->alt,
          s->freq, sondeTypeStr[s->type], s->alt);
}

const char *createKMLDynamic() {
  char *ptr = message;

  strcpy(ptr, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>");

  for (int i = 0; i < sonde.nSonde; i++) {
    int snum = (i + sonde.currentSonde) % sonde.nSonde;
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
  strcpy(si->id, "test");
  si->lat = 48; si->lon = 11; si->alt = 500;
  snprintf(ptr, 10240, "<?xml version='1.0' encoding='UTF-8'?>\n"
           "<gpx version=\"1.1\" creator=\"http://rdzsonde.local\" xmlns=\"http://www.topografix.com/GPX/1/1\" "
           "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
           "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
           "<metadata>"
           "<name>Sonde #%d (%s)</name>\n"
           "<author>rdzTTGOsonde</author>\n"
           "</metadata>\n"
           "<wpt lat=\"%f\" lon=\"%f\">\n  <ele>%f</ele>\n  <name>%s</name>\n  <sym>Radio Beacon</sym><type>Sonde</type>\n"
           "</wpt></gpx>\n", index, si->id, si->lat, si->lon, si->alt, si->id);
  Serial.println(message);
  return message;
}

#if 0
// no longer supported
// tcp socket / json for android app is used now
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Websocket client connection received");
    client->text("Hello from ESP32 Server");
    //globalClient = client;
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
    globalClient = NULL;
  }
}
#endif


const char* PARAM_MESSAGE = "message";
void SetupAsyncServer() {
  Serial.println("SetupAsyncServer()\n");
  server.reset();
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/test.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/test.html", String(), false, processor);
  });

  server.on("/qrg.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createQRGForm());
  });
  server.on("/qrg.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleQRGPost(request);
    request->send(200, "text/html", createQRGForm());
  });

  server.on("/wifi.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createWIFIForm());
  });
  server.on("/wifi.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleWIFIPost(request);
    request->send(200, "text/html", createWIFIForm());
  });

  server.on("/config.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createConfigForm());
  });
  server.on("/config.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleConfigPost(request);
    request->send(200, "text/html", createConfigForm());
  });

  server.on("/status.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createStatusForm());
  });
  server.on("/update.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createUpdateForm(0));
  });
  server.on("/update.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleUpdatePost(request);
    request->send(200, "text/html", createUpdateForm(1));
  });

  server.on("/control.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createControlForm());
  });
  server.on("/control.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleControlPost(request);
    request->send(200, "text/html", createControlForm());
  });

  server.on("/file", HTTP_GET,  [](AsyncWebServerRequest * request) {
    String url = request->url();
    const char *filename = url.c_str() + 5;
    if (*filename == 0) {
      request->send(400, "error");
      return;
    }
    request->send(SPIFFS, filename, "text/plain");
  });
  server.on("/file", HTTP_POST,  [](AsyncWebServerRequest * request) {
    request->send(200);
  }, handleUpload);

  server.on("/edit.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    // new version:
    // Open file
    // store file object in request->_tempObject
    //request->send(200, "text/html", createEditForm(request->getParam(0)->value()));
    const String filename = request->getParam(0)->value();
    File file = SPIFFS.open("/" + filename, "r");
    int state = 0;
    request->send("text/html", 0, [state, file, filename](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t  {
      Serial.printf("******* send callback: %d %d %d\n", state, maxLen, index);
      return streamEditForm(state, file, filename, (char *)buffer, maxLen, index);
    });
  });
  server.on("/edit.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    const char *ret = handleEditPost(request);
    if (ret == NULL)
      request->send(200, "text/html", "<html><head>ERROR</head><body><p>Something went wrong. Uploaded file is empty.</p></body></hhtml>");
    else {
      String f = request->getParam(0)->value();
      request->redirect("/edit.html?file=" + f);
      //request->send(200, "text/html", createEditForm(request->getParam(0)->value()));
    }
  },
  NULL,
  [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    Serial.printf("post data: index=%d len=%d total=%d\n", index, len, total);
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/style.css", "text/css");
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
  });

  // Route to set GPIO to HIGH
  server.on("/test.php", HTTP_POST, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/live.kml", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "application/vnd.google-earth.kml+xml", createKMLLive(sonde.ipaddr.c_str()));
  });

  server.on("/dynamic.kml", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "application/vnd.google-earth.kml+xml", createKMLDynamic());
  });

  server.onNotFound([](AsyncWebServerRequest * request) {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      String url = request->url();
      if (url.endsWith(".gpx"))
        request->send(200, "application/gpx+xml", sendGPX(request));
      else {
        request->send(SPIFFS, url, "text/html");
        Serial.printf("URL is %s\n", url.c_str());
        //request->send(404);
      }
    }
  });

  // Start server
  server.begin();
}

int fetchWifiIndex(const char *id) {
  for (int i = 0; i < nNetworks; i++) {
    if (strcmp(id, networks[i].id.c_str()) == 0) {
      Serial.printf("Match for %s at %d\n", id, i);
      return i;
    }
    //Serial.printf("No match: '%s' vs '%s'\n", id, networks[i].id.c_str());
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
int64_t IRAM_ATTR divs10(int64_t n) {
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

int64_t IRAM_ATTR divs1000(int64_t n) {
  return divs10(divs10(divs10(n)));
}

unsigned long IRAM_ATTR my_millis()
{
  return divs1000(esp_timer_get_time());
}

void checkTouchStatus();
void touchISR();
void touchISR2();

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
    Serial.printf("Initializing touch 1 on pin %d\n", sonde.config.button_pin & 0x7f);
  }
  if ( IS_TOUCH(sonde.config.button2_pin) ) {
    touchAttachInterrupt(sonde.config.button2_pin & 0x7f, touchISR2, sonde.config.touch_thresh);
    Serial.printf("Initializing touch 2 on pin %d\n", sonde.config.button2_pin & 0x7f);
  }
}

char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));



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
        }
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
uint8_t ubx_factorydef[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x09, 13, 0, 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0x17, 0x8A };
uint8_t ubx_hardreset[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x04, 4, 0, 0xff, 0xff, 0, 0, 0x0C, 0x5D };
// GPGST: Class 0xF0 Id 0x07
uint8_t ubx_enable_gpgst[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x01, 3, 0, 0xF0, 0x07, 2, 0x03, 0x1F};

void initGPS() {
  if (sonde.config.gps_rxd < 0) return; // GPS disabled
  if (sonde.config.gps_txd >= 0) {  // TX enable, thus try setting baud to 9600 and do a factory reset
    File testfile = SPIFFS.open("/GPSRESET", FILE_READ);
    if (testfile && !testfile.isDirectory()) {
      testfile.close();
      Serial.println("GPS factory reset...");
      Serial2.begin(115200, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_set9k6, sizeof(ubx_set9k6));
      delay(100);
      Serial2.begin(38400, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_set9k6, sizeof(ubx_set9k6));
      delay(100);
      Serial2.begin(19200, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_set9k6, sizeof(ubx_set9k6));
      delay(100);
      Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
      Serial2.write(ubx_factorydef, sizeof(ubx_factorydef));
      delay(100);
      Serial2.write(ubx_hardreset, sizeof(ubx_hardreset));
      delay(5000);
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
      Serial.printf("RXtask: start DECODER for sonde %d (was %s)\n", rxtask.activate & 0x7f, getStateStr(rxtask.mainState));
      rxtask.mainState = ST_DECODER;
      rxtask.currentSonde = rxtask.activate & 0x7F;
      sonde.setup();
    } else if (rxtask.activate != -1) {
      Serial.printf("RXtask: start %s (was %s)\n", getStateStr(rxtask.activate), getStateStr(rxtask.mainState));
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


void IRAM_ATTR touchISR() {
  if (!button1.isTouched) {
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) button1.doublepress = 1;
    else button1.doublepress = 0;
    button1.keydownTime = now;
    button1.isTouched = true;
  }
}

void IRAM_ATTR touchISR2() {
  if (!button2.isTouched) {
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) button2.doublepress = 1;
    else button2.doublepress = 0;
    button2.keydownTime = now;
    button2.isTouched = true;
  }
}

// touchRead in ISR is also a bad idea. Now moved to Ticker task
void checkTouchButton(Button & button) {
  if (button.isTouched) {
    int tmp = touchRead(button.pin & 0x7f);
    Serial.printf("touch read %d: value is %d\n", button.pin & 0x7f, tmp);
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

void ledOffCallback() {
  digitalWrite(sonde.config.led_pout, LOW);
}
void flashLed(int ms) {
  if (sonde.config.led_pout >= 0) {
    digitalWrite(sonde.config.led_pout, HIGH);
    ledFlasher.once_ms(ms, ledOffCallback);
  }
}

int doTouch = 0;
void checkTouchStatus() {
  checkTouchButton(button1);
  checkTouchButton(button2);
}

unsigned long bdd1, bdd2;
void IRAM_ATTR buttonISR() {
  if (digitalRead(button1.pin) == 0) { // Button down
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

void IRAM_ATTR button2ISR() {
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
  //int x = digitalRead(button1.pin);
  //Serial.printf("Debug: bdd1=%ld, bdd2=%ld\b", bdd1, bdd2);
  //Serial.printf("button1 press (dbl:%d) (now:%d): %d at %ld (%d)\n", button1.doublepress, x, p, button1.keydownTime, button1.numberKeyPresses);
  return p;
}

// called by arduino main loop (from Sonde::waitRXcomplete) as soon as pmu_irq is set
void handlePMUirq() {
  if (sonde.config.button2_axp) {
    // Use AXP power button as second button
    if (pmu_irq) {
      Serial.println("PMU_IRQ is set\n");
      xSemaphoreTake( axpSemaphore, portMAX_DELAY );
      axp.readIRQ();
      if (axp.isPEKShortPressIRQ()) {
        button2.pressed = KP_SHORT;
        button2.keydownTime = my_millis();
      }
      if (axp.isPEKLongtPressIRQ()) {
        button2.pressed = KP_MID;
        button2.keydownTime = my_millis();
      }
      pmu_irq = false;
      axp.clearIRQ();
      xSemaphoreGive( axpSemaphore );
    }
  } else {
    Serial.println("handlePMIirq() called. THIS SHOULD NOT HAPPEN w/o button2_axp set");
    pmu_irq = false;   // prevent main loop blocking
  }
}

int getKey2Press() {
  // TODO: Should be atomic
  KeyPress p = button2.pressed;
  button2.pressed = KP_NONE;
  //Serial.printf("button2 press: %d at %ld (%d)\n", p, button2.keydownTime, button2.numberKeyPresses);
  return p;
}

int getKeyPressEvent() {
  int p = getKeyPress();
  if (p == KP_NONE) {
    p = getKey2Press();
    if (p == KP_NONE)
      return EVT_NONE;
    Serial.printf("Key 2 was pressed [%d]\n", p + 4);
    return p + 4;
  }
  Serial.printf("Key 1 was pressed [%d]\n", p);
  return p;  /* map KP_x to EVT_KEY1_x / EVT_KEY2_x*/
}

#define SSD1306_ADDRESS 0x3c
bool ssd1306_found = false;
bool axp192_found = false;

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
      if (addr == AXP192_SLAVE_ADDRESS) {
        axp192_found = true;
        Serial.println("axp192 PMU found");
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


#ifdef ESP_MEM_DEBUG
typedef void (*esp_alloc_failed_hook_t) (size_t size, uint32_t caps, const char * function_name);
extern esp_err_t heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t callback);

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
  printf("%s was called but failed to allocate %d bytes with 0x%X capabilities. \n", function_name, requested_size, caps);
}
#endif


void setup()
{
  char buf[12];
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  for (int i = 0; i < 39; i++) {
    int v = gpio_get_level((gpio_num_t)i);
    Serial.printf("%d:%d ", i, v);
  }
  Serial.println("");
#ifdef ESP_MEM_DEBUG
  esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);
#endif
  axpSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(axpSemaphore);

#if 0
  delay(2000);
  // temporary test
  volatile uint32_t *ioport = portOutputRegister(digitalPinToPort(4));
  uint32_t portmask = digitalPinToBitMask(4);
  int t = millis();
  for (int i = 0; i < 10000000; i++) {
    digitalWrite(4, LOW);
    digitalWrite(4, HIGH);
  }
  int res = millis() - t;
  Serial.printf("Duration w/ digitalWriteo: %d\n", res);

  t = millis();
  for (int i = 0; i < 10000000; i++) {
    *ioport |=  portmask;
    *ioport &= ~portmask;
  }
  res = millis() - t;
  Serial.printf("Duration w/ fast io: %d\n", res);
#endif
  for (int i = 0; i < 39; i++) {
    Serial.printf("%d:%d ", i, initlevels[i]);
  }
  Serial.println(" (before setup)");
  sonde.defaultConfig();  // including autoconfiguration
  aprs_gencrctab();

  Serial.println("Initializing SPIFFS");
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println("Reading initial configuration");
  setupConfigData();    // configuration must be read first due to OLED ports!!!

  // NOT TTGO v1 (fingerprint 64) or Heltec v1/v2 board (fingerprint 4)
  // and NOT TTGO Lora32 v2.1_1.6 (fingerprint 31/63)
  if ( ( (sonde.fingerprint & (64 + 31)) != 31) && ((sonde.fingerprint & 16) == 16) ) {
    // FOr T-Beam 1.0
    for (int i = 0; i < 10; i++) { // try multiple times
      Wire.begin(21, 22);
      // Make sure the whole thing powers up!?!?!?!?!?
      U8X8 *u8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(0, 22, 21);
      u8x8->initDisplay();
      delay(500);

      scanI2Cdevice();
      if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
      } else {
        Serial.println("AXP192 Begin FAIL");
      }
      axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
      axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
      axp.setDCDC1Voltage(3300);
      axp.adc1Enable(AXP202_VBUS_VOL_ADC1, 1);
      axp.adc1Enable(AXP202_VBUS_CUR_ADC1, 1);
      axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
      if (sonde.config.button2_axp) {
        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
          pmu_irq = true;
        }, FALLING);
        //axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axp.enableIRQ( AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, 1 );
        axp.clearIRQ();
      }
      int ndevices = scanI2Cdevice();
      if (sonde.fingerprint != 17 || ndevices > 0) break; // only retry for fingerprint 17 (startup problems of new t-beam with oled)
      delay(500);
    }
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
    Serial.printf("button1.pin is %d, attaching interrupt\n", button1.pin);
  }
  // Handle button press
  if ( (button2.pin & 0x80) == 0) {
    attachInterrupt( button2.pin, button2ISR, CHANGE);
    Serial.printf("button2.pin is %d, attaching interrupt\n", button2.pin);
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
  Serial.printf("before disp.initFromFile... layouts is %p\n", disp.layouts);

  disp.initFromFile(sonde.config.screenfile);
  Serial.printf("disp.initFromFile... layouts is %p", disp.layouts);


  // == show initial values from config.txt ========================= //
  if (sonde.config.debug == 1) {
    disp.rdis->setFont(FONT_SMALL);
    disp.rdis->drawString(0, 0, "Config:");

    delay(500);
    itoa(sonde.config.oled_sda, buf, 10);
    disp.rdis->drawString(0, 1, " SDA:");
    disp.rdis->drawString(6, 1, buf);

    delay(500);
    itoa(sonde.config.oled_scl, buf, 10);
    disp.rdis->drawString(0, 2, " SCL:");
    disp.rdis->drawString(6, 2, buf);

    delay(500);
    itoa(sonde.config.oled_rst, buf, 10);
    disp.rdis->drawString(0, 3, " RST:");
    disp.rdis->drawString(6, 3, buf);

    delay(1000);
    itoa(sonde.config.led_pout, buf, 10);
    disp.rdis->drawString(0, 4, " LED:");
    disp.rdis->drawString(6, 4, buf);

    delay(500);
    itoa(sonde.config.spectrum, buf, 10);
    disp.rdis->drawString(0, 5, " SPEC:");
    disp.rdis->drawString(6, 5, buf);

    delay(500);
    itoa(sonde.config.maxsonde, buf, 10);
    disp.rdis->drawString(0, 6, " MAX:");
    disp.rdis->drawString(6, 6, buf);

    delay(5000);
    sonde.clearDisplay();
  }
  // == show initial values from config.txt ========================= //

#if 0
  // == check the radio chip by setting default frequency =========== //
  if (rs41.setFrequency(402700000) == 0) {
    Serial.println(F("Setting freq: SUCCESS "));
  } else {
    Serial.println(F("Setting freq: ERROR "));
  }
  float f = sx1278.getFrequency();
  Serial.print("Frequency set to ");
  Serial.println(f);
  // == check the radio chip by setting default frequency =========== //
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
  setupChannelList();
#if 0
  sonde.clearSonde();
  sonde.addSonde(402.700, STYPE_RS41);
  sonde.addSonde(405.700, STYPE_RS41);
  sonde.addSonde(405.900, STYPE_RS41);
  sonde.addSonde(403.450, STYPE_DFM09);
  Serial.println("No channel config file, using defaults!");
  Serial.println();
#endif
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
  initGPS();

  WiFi.onEvent(WiFiEvent);
  getKeyPress();    // clear key buffer
}

void enterMode(int mode) {
  Serial.printf("enterMode(%d)\n", mode);
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
  } else if (mainState == ST_WIFISCAN) {
    sonde.clearDisplay();
  }
  if (mode == ST_DECODER) {
    // trigger activation of background task
    // currentSonde should be set before enterMode()
    rxtask.activate = ACT_SONDE(sonde.currentSonde);
    sonde.clearDisplay();
    sonde.updateDisplay();
  }
}

static char text[40];
static const char *action2text(uint8_t action) {
  if (action == ACT_DISPLAY_DEFAULT) return "Default Display";
  if (action == ACT_DISPLAY_SPECTRUM) return "Spectrum Display";
  if (action == ACT_DISPLAY_WIFI) return "Wifi Scan Display";
  if (action == ACT_NEXTSONDE) return "Go to next sonde";
  if (action == ACT_PREVSONDE) return "presonde (not implemented)";
  if (action == ACT_NONE) return "none";
  if (action >= 128) {
    snprintf(text, 40, "Sonde=%d", action & 127);
  } else {
    snprintf(text, 40, "Display=%d", action);
  }
  return text;
}

#define RDZ_DATA_LEN 128

void parseGpsJson(char *data) {
  char *key = NULL;
  char *value = NULL;
  // very simple json parser: look for ", then key, then ", then :, then number, then , or } or \0
  for (int i = 0; i < RDZ_DATA_LEN; i++) {
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
  Serial.printf("Parse result: lat=%f, lon=%f, alt=%d, valid=%d\n", gpsPos.lat, gpsPos.lon, gpsPos.alt, gpsPos.valid);
}

static char rdzData[RDZ_DATA_LEN];
static int rdzDataPos = 0;

void loopDecoder() {
  // sonde knows the current type and frequency, and delegates to the right decoder
  uint16_t res = sonde.waitRXcomplete();
  int action;
  //Serial.printf("waitRX result is %x\n", (int)res);
  action = (int)(res >> 8);
  // TODO: update displayed sonde?

  if (action != ACT_NONE) {
    int newact = sonde.updateState(action);
    Serial.printf("MAIN: loopDecoder: action %s (%d) => %d  [current: main=%d, rxtask=%d]\n", action2text(action), action, newact, sonde.currentSonde, rxtask.currentSonde);
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
    }
  }

  if (!tncclient.connected()) {
    //Serial.println("TNC client not connected");
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
  if (rdzserver.hasClient()) {
    Serial.println("TCP JSON socket: new connection");
    rdzclient.stop();
    rdzclient = rdzserver.available();
  }
  if (rdzclient.available()) {
    Serial.print("RDZ JSON socket: received ");
    while (rdzclient.available()) {
      char c = (char)rdzclient.read();
      Serial.print(c);
      if (c == '\n' || c == '}' || rdzDataPos >= RDZ_DATA_LEN) {
        // parse GPS position from phone
        rdzData[rdzDataPos] = c;
        if (rdzDataPos > 2) parseGpsJson(rdzData);
        rdzDataPos = 0;
      }
      else {
        rdzData[rdzDataPos++] = c;
      }
    }
    Serial.println("");
  }
  // wifi (axudp) or bluetooth (bttnc) active => send packet
  SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
  if ((res & 0xff) == 0 && (connected || tncclient.connected() )) {
    //Send a packet with position information
    // first check if ID and position lat+lonis ok

    if (s->validID && ((s->validPos & 0x03) == 0x03)) {
      const char *str = aprs_senddata(s, sonde.config.call, sonde.config.udpfeed.symbol);
      if (connected)  {
        char raw[201];
        int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
        Serial.println("Sending AXUDP");
        //Serial.println(raw);
        udp.beginPacket(sonde.config.udpfeed.host, sonde.config.udpfeed.port);
        udp.write((const uint8_t *)raw, rawlen);
        udp.endPacket();
      }
      if (tncclient.connected()) {
        Serial.println("Sending position via TCP");
        char raw[201];
        int rawlen = aprsstr_mon2kiss(str, raw, APRS_MAXLEN);
        Serial.print("sending: "); Serial.println(raw);
        tncclient.write(raw, rawlen);
      }
    }
#if FEATURE_SONDEHUB
    if (sonde.config.sondehub.active) {
      unsigned long time_now = millis();
      // time_delta will be correct, even if time_now overflows
      unsigned long time_delta = time_now - time_last_update;
      if ((sonde.config.sondehub.chase == 0) && (time_delta >= SONDEHUB_STATION_UPDATE_TIME)) {  // 60 min
        sondehub_station_update(&shclient, &sonde.config.sondehub);
        time_last_update = time_now;
      }
      else if ((sonde.config.sondehub.chase == 1) && (time_delta >= SONDEHUB_MOBILE_STATION_UPDATE_TIME)) {  // 30 sec
        sondehub_station_update(&shclient, &sonde.config.sondehub);
        time_last_update = time_now;
      }
      sondehub_send_data(&shclient, s, &sonde.config.sondehub);
    }
#endif


#if FEATURE_MQTT
    // send to MQTT if enabled
    if (connected && mqttEnabled) {
      Serial.println("Sending sonde info via MQTT");
      mqttclient.publishPacket(s);
    }
#endif
  }
  // always send data, even if not valid....
  if (rdzclient.connected()) {
    Serial.println("Sending position via TCP as rdzJSON");
    char raw[1024];
    char gps[128];
    const char *typestr = s->typestr;
    if (*typestr == 0) typestr = sondeTypeStr[s->type];
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
    int len = snprintf(raw, 1024, "{"
                       "\"res\": %d,"
                       "\"type\": \"%s\","
                       "\"active\": %d,"
                       "\"freq\": %.2f,"
                       "\"id\": \"%s\","
                       "\"ser\": \"%s\","
                       "\"validId\": %d,"
                       "\"launchsite\": \"%s\","
                       "\"lat\": %.5f,"
                       "\"lon\": %.5f,"
                       "\"alt\": %.1f,"
                       "\"vs\": %.1f,"
                       "\"hs\": %.1f,"
                       "\"dir\": %.1f,"
                       "\"sats\": %d,"
                       "\"validPos\": %d,"
                       "\"time\": %d,"
                       "\"sec\": %d,"
                       "\"frame\": %d,"
                       "\"validTime\": %d,"
                       "\"rssi\": %d,"
                       "\"afc\": %d,"
                       "\"launchKT\": %d,"
                       "\"burstKT\": %d,"
                       "\"countKT\": %d,"
                       "\"crefKT\": %d"
                       "%s"
                       "}\n",
                       res & 0xff,
                       typestr,
                       (int)s->active,
                       s->freq,
                       s->id,
                       s->ser,
                       (int)s->validID,
                       s->launchsite,
                       s->lat,
                       s->lon,
                       s->alt,
                       s->vs,
                       s->hs,
                       s->dir,
                       s->sats,
                       s->validPos,
                       s->time,
                       s->sec,
                       s->frame,
                       (int)s->validTime,
                       s->rssi,
                       s->afc,
                       s->launchKT,
                       s->burstKT,
                       s->countKT,
                       s->crefKT,
                       gps
                      );
    //Serial.println("Writing rdzclient...");
    if (len > 1024) len = 1024;
    int wlen = rdzclient.write(raw, len);
    if (wlen != len) {
      Serial.println("Writing rdzClient not OK, closing connection");
      rdzclient.stop();
    }
    //Serial.println("Writing rdzclient OK");
  }
  Serial.print("MAIN: updateDisplay started\n");
  if (forceReloadScreenConfig) {
    disp.initFromFile(sonde.config.screenfile);
    sonde.clearDisplay();
    forceReloadScreenConfig = false;
  }
  int t = millis();
  sonde.updateDisplay();
  Serial.printf("MAIN: updateDisplay done (after %d ms)\n", (int)(millis() - t));
}

void setCurrentDisplay(int value) {
  Serial.printf("setCurrentDisplay: setting index %d, entry %d\b", value, sonde.config.display[value]);
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
    Serial.printf("config.spectrum:%d  specTimer:%ld millis:%ld remaining:%d\n", sonde.config.spectrum, specTimer, millis(), remaining);
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

String translateEncryptionType(wifi_auth_mode_t encryptionType) {
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

enum t_wifi_state { WIFI_DISABLED, WIFI_SCAN, WIFI_CONNECT, WIFI_CONNECTED, WIFI_APMODE };

static t_wifi_state wifi_state = WIFI_DISABLED;

void enableNetwork(bool enable) {
  if (enable) {
    MDNS.begin(sonde.config.mdnsname);
    SetupAsyncServer();
    udp.begin(WiFi.localIP(), LOCALUDPPORT);
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("kisstnc", "tcp", 14580);
    MDNS.addService("jsonrdz", "tcp", 14570);
    if (sonde.config.kisstnc.active) {
      tncserver.begin();
      rdzserver.begin();
    }
#if FEATURE_MQTT
    if (sonde.config.mqtt.active && strlen(sonde.config.mqtt.host) > 0) {
      mqttEnabled = true;
      mqttclient.init(sonde.config.mqtt.host, sonde.config.mqtt.port, sonde.config.mqtt.id, sonde.config.mqtt.username, sonde.config.mqtt.password, sonde.config.mqtt.prefix);
    }
#endif
#if FEATURE_SONDEHUB
    if (sonde.config.sondehub.active && wifi_state != WIFI_APMODE) {
      sondehub_station_update(&shclient, &sonde.config.sondehub);
      time_last_update = millis();
    }
#endif
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    connected = true;
  } else {
    MDNS.end();
    connected = false;
  }
}

// Events used only for debug output right now
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case SYSTEM_EVENT_WIFI_READY:
      Serial.println("WiFi interface ready");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi client started");
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi clients stopped");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to access point");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      if (wifi_state == WIFI_CONNECT) {
        // If we get a disconnect event while waiting for connection (as I do sometimes with my FritzBox),
        // just start from scratch with WiFi scan
        wifi_state = WIFI_DISABLED;
        WiFi.disconnect(true);
      }
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println("Authentication mode of access point has changed");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("Lost IP address and IP address is reset to 0");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.println("WiFi access point started");
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("WiFi access point  stopped");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Client connected");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("Client disconnected");
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.println("Assigned IP address to client");
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println("Received probe request");
      break;
    case SYSTEM_EVENT_GOT_IP6:
      Serial.println("IPv6 is preferred");
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("Ethernet started");
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected");
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address");
      break;
    default:
      break;
  }
}


void wifiConnect(int16_t res) {
  Serial.printf("WiFi scan result: found %d networks\n", res);

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
    Serial.printf("WiFi Connecting BSSID: %02X:%02X:%02X:%02X:%02X:%02X SSID: %s PW %s Channel: %d (RSSI %d)\n", bestBSSID[0], bestBSSID[1], bestBSSID[2], bestBSSID[3], bestBSSID[4], bestBSSID[5], fetchWifiSSID(bestEntry), fetchWifiPw(bestEntry), bestChannel, bestRSSI);
    WiFi.begin(fetchWifiSSID(bestEntry), fetchWifiPw(bestEntry), bestChannel, bestBSSID);
    wifi_state = WIFI_CONNECT;
  } else {
    // rescan
    // wifiStart();
    WiFi.disconnect(true);
    wifi_state = WIFI_DISABLED;
  }
}

static int wifi_cto;

void loopWifiBackground() {
  // Serial.printf("WifiBackground: state %d\n", wifi_state);
  // handle Wifi station mode in background
  if (sonde.config.wifi == 0 || sonde.config.wifi == 2) return; // nothing to do if disabled or access point mode

  if (wifi_state == WIFI_DISABLED) {  // stopped => start can
    wifi_state = WIFI_SCAN;
    Serial.println("WiFi start scan");
    WiFi.scanNetworks(true); // scan in async mode
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
    if (WiFi.isConnected()) {
      wifi_state = WIFI_CONNECTED;
      // update IP in display
      String localIPstr = WiFi.localIP().toString();
      sonde.setIP(localIPstr.c_str(), false);
      sonde.updateDisplayIP();
      enableNetwork(true);
    }
    if (wifi_cto > 20) { // failed, restart scanning
      wifi_state = WIFI_DISABLED;
      WiFi.disconnect(true);
    }
  } else if (wifi_state == WIFI_CONNECTED) {
    if (!WiFi.isConnected()) {
      sonde.setIP("", false);
      sonde.updateDisplayIP();

      wifi_state = WIFI_DISABLED;  // restart scan
      enableNetwork(false);
      WiFi.disconnect(true);
    }
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
// 1: station mode in background. directly start initial mode (spectrum or scanner)
// 2: access point mode in background. directly start initial mode (spectrum or scanner)
// 3: traditional sync. WifiScan. Tries to connect to a network, in case of failure activates AP.
//    Mode 3 shows more debug information on serial port and display.
#define MAXWIFIDELAY 20
static const char* _scan[2] = {"/", "\\"};
void loopWifiScan() {
  if (sonde.config.wifi == 0) {   // no Wifi
    wifi_state = WIFI_DISABLED;
    initialMode();
    return;
  }
  if (sonde.config.wifi == 1) { // station mode, setup in background
    wifi_state = WIFI_DISABLED;  // will start scanning in wifiLoopBackgroiund
    initialMode();
    return;
  }
  if (sonde.config.wifi == 2) { // AP mode, setup in background
    startAP();
    initialMode();
    return;
  }
  // wifi==3 => original mode with non-async wifi setup
  disp.rdis->setFont(FONT_SMALL);
  disp.rdis->drawString(0, 0, "WiFi Scan...");
  uint8_t dispw, disph, dispxs, dispys;
  disp.rdis->getDispSize(&disph, &dispw, &dispxs, &dispys);

  int line = 0;
  int cnt = 0;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  int index = -1;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    disp.rdis->drawString(0, dispys * (1 + line), ssid.c_str());
    line = (line + 1) % (disph / dispys);
    String mac = WiFi.BSSIDstr(i);
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.printf("Network %s: RSSI %d, MAC %s, enc: %s\n", ssid.c_str(), WiFi.RSSI(i), mac.c_str(), encryptionTypeDescription.c_str());
    int curidx = fetchWifiIndex(ssid.c_str());
    if (curidx >= 0 && index == -1) {
      index = curidx;
      Serial.printf("Match found at scan entry %d, config network %d\n", i, index);
    }
  }
  int lastl = (disph / dispys - 2) * dispys;
  if (index >= 0) { // some network was found
    Serial.print("Connecting to: "); Serial.print(fetchWifiSSID(index));
    Serial.print(" with password "); Serial.println(fetchWifiPw(index));

    disp.rdis->drawString(0, lastl, "Conn:");
    disp.rdis->drawString(6 * dispxs, lastl, fetchWifiSSID(index));
    WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
    while (WiFi.status() != WL_CONNECTED && cnt < MAXWIFIDELAY)  {
      delay(500);
      Serial.print(".");
      if (cnt == 5) {
        // my FritzBox needs this for reconnecting
        WiFi.disconnect(true);
        delay(500);
        WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
        Serial.print("Reconnecting to: "); Serial.print(fetchWifiSSID(index));
        Serial.print(" with password "); Serial.println(fetchWifiPw(index));
        delay(500);
      }
      disp.rdis->drawString(15 * dispxs, lastl + dispys, _scan[cnt & 1]);
      cnt++;
    }
  }
  if (index < 0 || cnt >= MAXWIFIDELAY) { // no network found, or connect not successful
    WiFi.disconnect(true);
    delay(1000);
    startAP();
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    disp.rdis->drawString(0, lastl, "AP:             ");
    disp.rdis->drawString(6 * dispxs, lastl + 1, networks[0].id.c_str());
    delay(3000);
  } else {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    String localIPstr = WiFi.localIP().toString();
    Serial.println(localIPstr);
    sonde.setIP(localIPstr.c_str(), false);
    sonde.updateDisplayIP();
    wifi_state = WIFI_CONNECTED;
    bool hasRS92 = false;
    for (int i = 0; i < MAXSONDE; i++) {
      if (sonde.sondeList[i].type == STYPE_RS92) hasRS92 = true;
    }
    if (hasRS92) {
      geteph();
      get_eph("/brdc");
    }
    delay(3000);
  }
  enableNetwork(true);
  initialMode();
}


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
  disp.rdis->setFont(FONT_SMALL);
  disp.rdis->drawString(0, 0, "C:");
  String dispHost = updateHost.substring(0, 14);
  disp.rdis->drawString(2, 0, dispHost.c_str());

  Serial.println("Connecting to: " + updateHost);
  // Connect to Update host
  if (client.connect(updateHost.c_str(), updatePort)) {
    // Connection succeeded, fecthing the bin
    Serial.println("Fetching bin: " + String(*updateBin));
    disp.rdis->drawString(0, 1, "Fetching update");

    // Get the contents of the bin file
    client.print(String("GET ") + *updateBin + " HTTP/1.1\r\n" +
                 "Host: " + updateHost + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        client.stop();
        return;
      }
    }
    // Once the response is available,
    // check stuff

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
      // read line till /n
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
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect to updateHost failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(updateHost) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));
  disp.rdis->drawString(0, 2, "Len: ");
  String cls = String(contentLength);
  disp.rdis->drawString(5, 2, cls.c_str());

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);
    disp.rdis->drawString(0, 4, "Starting update");

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      disp.rdis->drawString(0, 5, "Please wait!");
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
          disp.rdis->drawString(0, 7, "Rebooting....");
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
      client.flush();
    }
  } else {
    Serial.println("There was no content in the response");
    client.flush();
  }
  // Back to some normal state
  enterMode(ST_DECODER);
}




void loop() {
  Serial.printf("\nMAIN: Running loop in state %d [currentDisp:%d, lastDisp:%d]. free heap: %d, unused stack: %d\n",
                mainState, currentDisplay, lastDisplay, ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(0));
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

#if FEATURE_MQTT
  int now = millis();
  if (mqttEnabled && (lastMqttUptime == 0 || (lastMqttUptime + 60000 < now) || (lastMqttUptime > now))) {
    mqttclient.publishUptime();
    lastMqttUptime = now;
  }
#endif
}

#if FEATURE_SONDEHUB
// Sondehub v2 DB related codes
/*
 	Update station data to the sondehub v2 DB
*/
void sondehub_station_update(WiFiClient *client, struct st_sondehub *conf) {
  #define STATION_DATA_LEN 300
  char data[STATION_DATA_LEN];
  char *w;

  Serial.println("sondehub_station_update()");

  if (!client->connected()) {
    if (!client->connect(conf->host, 80)) {
      Serial.println("Connection FAILED");
      return;
    }
  }

  w = data;
  memset(w, 0, STATION_DATA_LEN);

  sprintf(w,
          "{"
          "\"software_name\": \"%s\","
          "\"software_version\": \"%s\","
          "\"uploader_callsign\": \"%s\","
          "\"uploader_contact_email\": \"%s\",",
          version_name, version_id, conf->callsign, conf->email);
  w += strlen(w);
  if (conf->chase == 0) {
    sprintf(w,
            "\"uploader_position\": [%s,%s,%s],"
            "\"uploader_antenna\": \"%s\""
            "}",
            conf->lat, conf->lon, conf->alt, conf->antenna);
  }
  else if (gpsPos.valid && gpsPos.lat != 0 && gpsPos.lon != 0) {
    sprintf(w,
            "\"uploader_position\": [%.6f, %.6f, %d],"
            "\"uploader_antenna\": \"%s\""
            "\"mobile\": \"true\""
            "}",
            gpsPos.lat, gpsPos.lon, gpsPos.alt, conf->antenna);
  }
  else {
    return;
  }

  client->println("PUT /listeners HTTP/1.1");
  client->print("Host: ");
  client->println(conf->host);
  client->println("accept: text/plain");
  client->println("Content-Type: application/json");
  client->print("Content-Length: ");
  client->println(strlen(data));
  client->println();
  client->println(data);
  Serial.println(strlen(data));
  Serial.println(data);
  Serial.println("Waiting for response");
  String response = client->readString();
  Serial.println(response);
  Serial.println("Response done...");
  //client->stop();
}

/*
 	Update sonde data to the sondehub v2 DB
*/
enum SHState { SH_DISCONNECTED, SH_CONNECTING, SH_CONN_IDLE, SH_CONN_WAITACK };

SHState shState = SH_DISCONNECTED;

/* Sonde.h: enum SondeType { STYPE_DFM, STYPE_DFM09_OLD, STYPE_RS41, STYPE_RS92, STYPE_M10, STYPE_M20, STYPE_DFM06_OLD, STYPE_MP3H }; */
const char *sondeTypeStrSH[NSondeTypes] = { "DFM", "DFM", "RS41", "RS92", "M10", "M20", "DFM", "MRZ" };
const char *dfmSubtypeStrSH[16] = { NULL, NULL, NULL, NULL, NULL, NULL,
                                    "DFM06",  // 0x06
                                    "PS15",   // 0x07
                                    NULL, NULL,
                                    "DFM09",  // 0x0A
                                    "DFM17",  // 0x0B
                                    "DFM09P", // 0x0C
                                    "DFM17",  // 0x0D
                                    NULL, NULL
                                  };

// in hours.... max allowed diff UTC <-> sonde time
#define SONDEHUB_TIME_THRESHOLD (3)
void sondehub_send_data(WiFiClient *client, SondeInfo *s, struct st_sondehub *conf) {
  Serial.println("sondehub_send_data()");

#define MSG_SIZE 550
  char rs_msg[MSG_SIZE];
  char *w;
  struct tm ts;
  // For DFM, s->time is data from subframe DAT8 (gps date/hh/mm), and sec is from DAT1 (gps sec/usec)
  // For all others, sec should always be 0 and time the exact time in seconds
  time_t t = s->time + s->sec;

  while (client->available() > 0) {
    // data is available from remote server, process it...
    int cnt = client->readBytesUntil('\n', rs_msg, MSG_SIZE);
    rs_msg[cnt] = 0;
    Serial.println(rs_msg);
    // If something that looks like a valid HTTP response is received, we are ready to send the next data item
    if (shState == SH_CONN_WAITACK && cnt > 11 && strncmp(rs_msg, "HTTP/1", 6) == 0) {
      shState = SH_CONN_IDLE;
    }
  }

  // Check if current sonde data is valid. If not, don't do anything....
  if (*s->ser == 0) return;	// Don't send anything without serial number
  if (((int)s->lat == 0) && ((int)s->lon == 0)) return;	// Sometimes these values are zeroes. Don't send those to the sondehub
  if ((int)s->alt > 50000) return;	// If alt is too high don't send to SondeHub
  if ((int)s->sats < 4) return;	// If not enough sats don't send to SondeHub

  // If not connected to sondehub, try reconnecting.
  // TODO: do this outside of main loop
  if (!client->connected()) {
    Serial.println("NO CONNECTION");
    shState = SH_DISCONNECTED;
    if (!client->connect(conf->host, 80)) {
      Serial.println("Connection FAILED");
      return;
    }
    client->Client::setTimeout(0);  // does this work?
    shState = SH_CONN_IDLE;
  }

  if ( shState == SH_CONN_WAITACK ) {
    Serial.println("Previous SH-frame not yet ack'ed, not sending new data");
    return;
  }

  struct tm timeinfo;
  time_t now;
  time(&now);
  gmtime_r(&now, &timeinfo);
  if (timeinfo.tm_year <= (2016 - 1900)) {
    Serial.println("Failed to obtain time");
    return;
  }
  if ( abs(now - (time_t)s->time) > (3600 * SONDEHUB_TIME_THRESHOLD) ) {
    Serial.printf("Sonde time %d too far from current UTC time %ld", s->time, now);
    return;
  }

  //  DFM uses UTC. Most of the other radiosondes use GPS time
  // SondeHub expect datetime to be the same time sytem as the sonde transmits as time stamp
  if ( s->type == STYPE_RS41 || s->type == STYPE_RS92 || s->type == STYPE_M10 || s->type == STYPE_M20 ) {
    t += 18;	// convert back to GPS time from UTC time +18s
  }

  gmtime_r(&t, &ts);

  memset(rs_msg, 0, MSG_SIZE);
  w = rs_msg;

  sprintf(w,
          "[ {"
          "\"software_name\": \"%s\","
          "\"software_version\": \"%s\","
          "\"uploader_callsign\": \"%s\","
          "\"time_received\": \"%04d-%02d-%02dT%02d:%02d:%02d.000Z\","
          "\"manufacturer\": \"%s\","
          "\"serial\": \"%s\","
          "\"datetime\": \"%04d-%02d-%02dT%02d:%02d:%02d.000Z\","
          "\"lat\": %.6f,"
          "\"lon\": %.6f,"
          "\"alt\": %.2f,"
          "\"frequency\": %.3f,"
          "\"vel_h\": %.1f,"
          "\"vel_v\": %.1f,"
          "\"heading\": %.1f,"
          "\"sats\": %d,"
          "\"rssi\": %.1f,",
          version_name, version_id, conf->callsign,
          timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
          manufacturer_string[s->type], s->ser,
          ts.tm_year + 1900, ts.tm_mon + 1, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec + s->sec,
          (float)s->lat, (float)s->lon, (float)s->alt, (float)s->freq, (float)s->hs, (float)s->vs,
          (float)s->dir, (int)s->sats, -((float)s->rssi / 2)
         );
  w += strlen(w);

  if ( TYPE_IS_DFM(s->type) || TYPE_IS_METEO(s->type) || s->type == STYPE_MP3H ) {
    // send frame as gps timestamp for these sonde, identical to autorx
    // For M10, this is real GPS time (seconds since Jqn 6 1980, without adjusting for leap seconds)
    // DFM and MP3H send real UTC (with leap seconds considered), so for them the frame number actually
    // is gps time plus number of leap seconds since the beginning of GPS time.
    sprintf(w, "\"frame\": %d,", int(t - 315964800));
  } else {
    sprintf(w, "\"frame\": %d,", s->frame);
  }
  w += strlen(w);

  sprintf(w, "\"type\": \"%s\",", sondeTypeStrSH[s->type]);
  w += strlen(w);

  /* if there is a subtype (DFM only) */
  if ( TYPE_IS_DFM(s->type) && s->subtype > 0 && s->subtype < 16 ) {
    const char *t = dfmSubtypeStrSH[s->subtype];
    // as in https://github.com/projecthorus/radiosonde_auto_rx/blob/e680221f69a568e1fdb24e76db679233f32cb027/auto_rx/autorx/sonde_specific.py#L84
    if (t) sprintf(w, "\"subtype\": \"%s\",", t);
    else sprintf(w, "\"subtype\": \"DFMx%X\",", s->subtype); // Unknown subtype
    w += strlen(w);
  }

  if (((int)s->temperature != 0) && ((int)s->relativeHumidity != 0)) {
    sprintf(w,
            "\"temp\": %.2f,"
            "\"humidity\": %.2f,",
            float(s->temperature), float(s->relativeHumidity)
           );
    w += strlen(w);
  }

  if (conf->chase == 0) {
    sprintf(w,
            "\"uploader_position\": [ %s, %s, %s ],"
            "\"uploader_antenna\": \"%s\""
            "}]",
            conf->lat, conf->lon, conf->alt, conf->antenna
          );
  }
  else if (gpsPos.valid && gpsPos.lat != 0 && gpsPos.lon != 0) {
    sprintf(w,
            "\"uploader_position\": [ %.6f, %.6f, %d ],"
            "\"uploader_antenna\": \"%s\""
            "}]",
            gpsPos.lat, gpsPos.lon, gpsPos.alt, conf->antenna
          );
  }
  else {
    sprintf(w, 
            "\"uploader_antenna\": \"%s\""
            "}]",
            conf->antenna
           );
  }

  client->println("PUT /sondes/telemetry HTTP/1.1");
  client->print("Host: ");
  client->println(conf->host);
  client->println("accept: text/plain");
  client->println("Content-Type: application/json");
  client->print("Content-Length: ");
  client->println(strlen(rs_msg));
  client->println();
  client->println(rs_msg);
  Serial.println(rs_msg);
  shState = SH_CONN_WAITACK;
  //String response = client->readString();
  //Serial.println(response);
}
// End of sondehub v2 related codes
#endif

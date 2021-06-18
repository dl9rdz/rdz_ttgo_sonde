# 1 "C:\\Users\\Uskompuf\\AppData\\Local\\Temp\\tmpj3dac7od"
#include <Arduino.h>
# 1 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
#include <axp20x.h>

#include "features.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>


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

int e;

enum MainState { ST_DECODER, ST_SPECTRUM, ST_WIFISCAN, ST_UPDATE, ST_TOUCHCALIB };
static MainState mainState = ST_WIFISCAN;
const char *mainStateStr[5] = {"DECODER", "SPECTRUM", "WIFISCAN", "UPDATE", "TOUCHCALIB" };

AsyncWebServer server(80);

AXP20X_Class axp;
#define PMU_IRQ 35
SemaphoreHandle_t axpSemaphore;
bool pmu_irq = false;

String updateHost = "rdzsonde.mooo.com";
int updatePort = 80;
String updateBinM = "/master/update.ino.bin";
String updateBinD = "/devel/update.ino.bin";
String *updateBin = &updateBinM;

#define LOCALUDPPORT 9002

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

boolean connected = false;
WiFiUDP udp;
WiFiClient client;
#if FEATURE_SONDEHUB
#define SONDEHUB_STATION_UPDATE_TIME (60*60*1000)
#define SONDEHUB_MOBILE_STATION_UPDATE_TIME (30*1000)
WiFiClient shclient;
unsigned long time_last_update = 0;
#endif


WiFiServer tncserver(14580);
WiFiClient tncclient;

WiFiServer rdzserver(14570);
WiFiClient rdzclient;

#if FEATURE_MQTT
unsigned long lastMqttUptime = 0;
boolean mqttEnabled;
MQTT mqttclient;
#endif
boolean forceReloadScreenConfig = false;

enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };


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


static unsigned long specTimer;

void enterMode(int mode);
void WiFiEvent(WiFiEvent_t event);
String readLine(Stream &stream);
int readLine(Stream &stream, char *buffer, int maxlen);
String processor(const String& var);
const String sondeTypeSelect(int activeType);
void setupChannelList();
void HTMLBODY(char *ptr, const char *which);
void HTMLBODYEND(char *ptr);
void HTMLSAVEBUTTON(char *ptr);
void setupWifiList();
void addSondeStatus(char *ptr, int i);
void setupConfigData();
void addConfigStringEntry(char *ptr, int idx, const char *label, int len, char *field);
void addConfigNumEntry(char *ptr, int idx, const char *label, int *value);
void addConfigButtonEntry(char *ptr, int idx, const char *label, int *value);
void addConfigTypeEntry(char *ptr, int idx, const char *label, int *value);
void addConfigOnOffEntry(char *ptr, int idx, const char *label, int *value);
void addConfigSeparatorEntry(char *ptr);
void addConfigHeading(char *ptr, const char *label);
void addConfigInt8List(char *ptr, int idx, const char *label, int8_t *list);
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
int streamEditForm(int &state, File &file, String filename, char *buffer, size_t maxlen, size_t index);
void addSondeStatusKML(char *ptr, int i);
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void SetupAsyncServer();
int fetchWifiIndex(const char *id);
int64_t IRAM_ATTR divs10(int64_t n);
int64_t IRAM_ATTR divs1000(int64_t n);
void initTouch();
template<typename T>

void unkHandler(T nmea);
void gpsTask(void *parameter);
void initGPS();
void sx1278Task(void *parameter);
void IRAM_ATTR touchISR();
void IRAM_ATTR touchISR2();
void checkTouchButton(Button & button);
void ledOffCallback();
void flashLed(int ms);
void IRAM_ATTR buttonISR();
void IRAM_ATTR button2ISR();
int getKeyPress();
void handlePMUirq();
int getKey2Press();
int getKeyPressEvent();
int scanI2Cdevice(void);
void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name);
void setup();
void parseGpsJson(char *data);
void loopDecoder();
void setCurrentDisplay(int value);
void loopSpectrum();
void startSpectrumDisplay();
String translateEncryptionType(wifi_auth_mode_t encryptionType);
void enableNetwork(bool enable);
void wifiConnect(int16_t res);
void loopWifiBackground();
void startAP();
void initialMode();
void loopTouchCalib();
void loopWifiScan();
String getHeaderValue(String header, String headerName);
void execOTA();
void loop();
void sondehub_station_update(WiFiClient *client, struct st_sondehub *conf);
void sondehub_send_data(WiFiClient *client, SondeInfo *s, struct st_sondehub *conf);
#line 104 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
String readLine(Stream &stream) {
  String s = stream.readStringUntil('\n');
  int len = s.length();
  if (len == 0) return s;
  if (s.charAt(len - 1) == '\r') s.remove(len - 1);
  return s;
}



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





char message[10240 * 4];




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
    String line = readLine(file);
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
# 278 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
  HTMLBODY(ptr, "qrg.html");

  strcat(ptr, "<table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Launchsite</th><th>Mode</th></tr>");
  for (int i = 0; i < sonde.config.maxsonde; i++) {

    String site = sonde.sondeList[i].launchsite;
    sprintf(ptr + strlen(ptr), "<tr><td>%d</td><td><input name=\"A%d\" type=\"checkbox\" %s/></td>"
            "<td><input name=\"F%d\" type=\"text\" width=12 value=\"%3.3f\"></td>"
            "<td><input name=\"S%d\" type=\"text\" value=\"%s\"></td>"

            "<td><input class='stype' name='T%d' value='%c'>",
            i + 1,
            i + 1, (i < sonde.nSonde && sonde.sondeList[i].active) ? "checked" : "",
            i + 1, i >= sonde.nSonde ? 400.000 : sonde.sondeList[i].freq,
            i + 1, i >= sonde.nSonde ? "                " : sonde.sondeList[i].launchsite,
            i + 1, i >= sonde.nSonde ? 2 : sondeTypeChar[sonde.sondeList[i].type] );

  }
  strcat(ptr, "</table>");

  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  Serial.printf("QRG form: size=%d bytes\n", strlen(message));
  return message;
}

const char *handleQRGPost(AsyncWebServerRequest *request) {
  char label[10];

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




#define MAX_WIFI 10
int nNetworks;
struct {
  String id;
  String pw;
} networks[MAX_WIFI];



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
    String line = readLine(file);
    if (!file.available()) break;
    networks[i].id = line;
    networks[i].pw = readLine(file);
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

  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  Serial.printf("WIFI form: size=%d bytes\n", strlen(message));
  return message;
}

const char *createSondeHubMap() {
  SondeInfo *s = &sonde.sondeList[0];
  char *ptr = message;
  strcpy(ptr, HTMLHEAD); strcat(ptr, "</head>");
  HTMLBODY(ptr, "map.html");
  if (!sonde.config.sondehub.active) {
    strcat(ptr, "<div>NOTE: SondeHub uploading is not enabled, detected sonde will not be visable on map</div>");
    sprintf(ptr + strlen(ptr), "<iframe src=\"https://tracker.sondehub.org/?sondehub=1#!mz=11&f=%s&q=%s\" allow=\"geolocation\" style=\"border:1px solid #00A3D3;border-radius:20px;height:95vh\"></iframe>", s-> ser, s-> ser);
  } else {
    sprintf(ptr, "<iframe src=\"https://tracker.sondehub.org/?sondehub=1#!mz=11&f=%s&q=%s\" allow=\"geolocation\" style=\"border:1px solid #00A3D3;border-radius:20px;height:98vh;width:100%%\"></iframe>", s-> ser, s-> ser);
  }
  HTMLBODYEND(ptr);
  return message;
}

const char *handleWIFIPost(AsyncWebServerRequest *request) {
  char label[10];

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




void setupConfigData() {
  File file = SPIFFS.open("/config.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/config.txt' for reading");
    return;
  }
  while (file.available()) {
    String line = readLine(file);
    sonde.setConfig(line.c_str());
  }
}


struct st_configitems {
  const char *name;
  const char *label;
  int type;
  void *data;
};

struct st_configitems config_list[] = {

  {"", "Software configuration", -5, NULL},
  {"wifi", "Wifi mode (0/1/2/3)", 0, &sonde.config.wifi},
  {"debug", "Debug mode (0/1)", 0, &sonde.config.debug},
  {"maxsonde", "Maxsonde", 0, &sonde.config.maxsonde},
  {"screenfile", "Screen config (0=old, 1=OLED, 2=TFT, 3=TFT[port])", 0, &sonde.config.screenfile},
  {"display", "Display screens (scan,default,...)", -6, sonde.config.display},

  {"spectrum", "Show spectrum (-1=no, 0=forever, >0=seconds)", 0, &sonde.config.spectrum},
  {"startfreq", "Startfreq (MHz)", 0, &sonde.config.startfreq},
  {"channelbw", "Bandwidth (kHz)", 0, &sonde.config.channelbw},
  {"marker", "Spectrum MHz marker", 0, &sonde.config.marker},
  {"noisefloor", "Spectrum noisefloor", 0, &sonde.config.noisefloor},

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

  {"call", "Call", 8, sonde.config.call},
  {"passcode", "Passcode", 8, sonde.config.passcode},

  {"kisstnc.active", "KISS TNC (port 14590) (needs reboot)", 0, &sonde.config.kisstnc.active},
  {"kisstnc.idformat", "KISS TNC ID Format", -2, &sonde.config.kisstnc.idformat},

  {"axudp.active", "AXUDP active", -3, &sonde.config.udpfeed.active},
  {"axudp.host", "AXUDP Host", 63, sonde.config.udpfeed.host},
  {"axudp.port", "AXUDP Port", 0, &sonde.config.udpfeed.port},
  {"axudp.idformat", "DFM ID Format", -2, &sonde.config.udpfeed.idformat},
  {"axudp.highrate", "Rate limit", 0, &sonde.config.udpfeed.highrate},

  {"tcp.active", "APRS TCP active", -3, &sonde.config.tcpfeed.active},
  {"tcp.host", "ARPS TCP Host", 63, sonde.config.tcpfeed.host},
  {"tcp.port", "APRS TCP Port", 0, &sonde.config.tcpfeed.port},
  {"tcp.idformat", "DFM ID Format", -2, &sonde.config.tcpfeed.idformat},
  {"tcp.highrate", "Rate limit", 0, &sonde.config.tcpfeed.highrate},

#if FEATURE_MQTT

  {"mqtt.active", "MQTT Active (needs reboot)", 0, &sonde.config.mqtt.active},
  {"mqtt.id", "MQTT client ID", 63, &sonde.config.mqtt.id},
  {"mqtt.host", "MQTT server hostname", 63, &sonde.config.mqtt.host},
  {"mqtt.port", "MQTT Port", 0, &sonde.config.mqtt.port},
  {"mqtt.username", "MQTT Username", 63, &sonde.config.mqtt.username},
  {"mqtt.password", "MQTT Password", 63, &sonde.config.mqtt.password},
  {"mqtt.prefix", "MQTT Prefix", 63, &sonde.config.mqtt.prefix},
#endif


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

}
void addConfigOnOffEntry(char *ptr, int idx, const char *label, int *value) {

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
      case -5:
        addConfigHeading(ptr, config_list[i].label);
        break;
      case -6:
        addConfigInt8List(ptr, i, config_list[i].label, (int8_t *)config_list[i].data);
        break;
      case -3:
        addConfigOnOffEntry(ptr, i, config_list[i].label, (int *)config_list[i].data);
        break;
      case -2:
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

  HTMLSAVEBUTTON(ptr);
  HTMLBODYEND(ptr);
  Serial.printf("Config form: size=%d bytes\n", strlen(message));
  return message;
}


const char *handleConfigPost(AsyncWebServerRequest *request) {

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
    if (config_list[idx].type == -1) continue;
    AsyncWebParameter *value = request->getParam(label, true);
    if (!value) continue;
    String strvalue = value->value();
    if (config_list[idx].type == -4) {
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
    case 0:
      {

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
    case 1:
      while (file.available()) {
        int cnt = readLine(file, buffer + i, maxlen - i - 1);
        i += cnt;
        buffer[i++] = '\n';
        buffer[i] = 0;
        if (i + 256 > maxlen) break;
      }
      if (i > 0) return i;
      file.close();
      state++;
    case 2:
      Serial.println("Appending footer\n");
      strncpy(buffer, "</textarea><input type=\"submit\" value=\"Save\"></input></form></body></html>", maxlen);
      state++;
      return strlen(buffer);
    case 3:
      return 0;
  }
  return 0;
}


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
    String line = readLine(file);
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


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Websocket client connection received");
    client->text("Hello from ESP32 Server");

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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/test.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/test.html", String(), false, processor);
  });

  server.on("/qrg.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createQRGForm());
  });
  server.on("/qrg.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleQRGPost(request);
    request->send(200, "text/html", createQRGForm());
  });

  server.on("/wifi.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createWIFIForm());
  });
  server.on("/wifi.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleWIFIPost(request);
    request->send(200, "text/html", createWIFIForm());
  });

  server.on("/map.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createSondeHubMap());
  });
  server.on("/map.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleWIFIPost(request);
    request->send(200, "text/html", createSondeHubMap());
  });

  server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createConfigForm());
  });
  server.on("/config.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleConfigPost(request);
    request->send(200, "text/html", createConfigForm());
  });

  server.on("/status.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createStatusForm());
  });
  server.on("/update.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createUpdateForm(0));
  });
  server.on("/update.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleUpdatePost(request);
    request->send(200, "text/html", createUpdateForm(1));
  });

  server.on("/control.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createControlForm());
  });
  server.on("/control.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleControlPost(request);
    request->send(200, "text/html", createControlForm());
  });

  server.on("/file", HTTP_GET, [](AsyncWebServerRequest * request) {
    String url = request->url();
    const char *filename = url.c_str() + 5;
    if (*filename == 0) {
      request->send(400, "error");
      return;
    }
    request->send(SPIFFS, filename, "text/plain");
  });
  server.on("/file", HTTP_POST, [](AsyncWebServerRequest * request) {
    request->send(200);
  }, handleUpload);

  server.on("/edit.html", HTTP_GET, [](AsyncWebServerRequest * request) {




    const String filename = request->getParam(0)->value();
    File file = SPIFFS.open("/" + filename, "r");
    int state = 0;
    request->send("text/html", 0, [state, file, filename](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
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

    }
  },
  NULL,
  [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    Serial.printf("post data: index=%d len=%d total=%d\n", index, len, total);
  });


  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/style.css", "text/css");
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
  });


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

      }
    }
  });


  server.begin();
}

int fetchWifiIndex(const char *id) {
  for (int i = 0; i < nNetworks; i++) {
    if (strcmp(id, networks[i].id.c_str()) == 0) {
      Serial.printf("Match for %s at %d\n", id, i);
      return i;
    }

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





    if (strcmp(id, networks[i].id.c_str()) == 0) return networks[i].pw.c_str();
  }
  return NULL;
}






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
# 1332 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
Ticker ticker;
Ticker ledFlasher;

#define IS_TOUCH(x) (((x)!=255)&&((x)!=-1)&&((x)&128))
void initTouch() {

  ticker.attach_ms(300, checkTouchStatus);

  if ( !(IS_TOUCH(sonde.config.button_pin) || IS_TOUCH(sonde.config.button2_pin)) ) return;
# 1349 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
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




template<typename T>

void unkHandler(T nmea) {
  if (strcmp(nmea.getMessageID(), "VTG") == 0) {
    const char *s = nmea.getSentence();
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    if (*s == ',') return;
    int lastCourse = nmea.parseFloat(s, 0, NULL);
    Serial.printf("Course update: %d\n", lastCourse);
  } else if (strcmp(nmea.getMessageID(), "GST") == 0) {


    const char *s = nmea.getSentence();
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;

    int stdlat = nmea.parseFloat(s, 1, NULL);
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;

    int stdlon = nmea.parseFloat(s, 1, NULL);


    int poserr = 0;
    if (stdlat < 10000 && stdlon < 10000) {
      poserr = (int)(sqrt(0.5 * (stdlat * stdlat + stdlon * stdlon)));
    }

    gpsPos.accuracy = poserr;
  }
}


static bool gpsCourseOld;
static int lastCourse;
void gpsTask(void *parameter) {
  nmea.setUnknownSentenceHandler(unkHandler);

  while (1) {
    while (Serial2.available()) {
      char c = Serial2.read();

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

            if (lastCourse != 0)
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

uint8_t ubx_enable_gpgst[] = {UBX_SYNCH_1, UBX_SYNCH_2, 0x06, 0x01, 3, 0, 0xF0, 0x07, 2, 0x03, 0x1F};

void initGPS() {
  if (sonde.config.gps_rxd < 0) return;
  if (sonde.config.gps_txd >= 0) {
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

    Serial2.write(ubx_enable_gpgst, sizeof(ubx_enable_gpgst));
  } else {
    Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);
  }
  xTaskCreate( gpsTask, "gpsTask",
               5000,
               NULL,
               1,
               NULL);
}

const char *getStateStr(int what) {
  if (what < 0 || what >= (sizeof(mainStateStr) / sizeof(const char *)))
    return "--";
  else
    return mainStateStr[what];
}

void sx1278Task(void *parameter) {
# 1512 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
  while (1) {
    if (rxtask.activate >= 128) {

      Serial.printf("RXtask: start DECODER for sonde %d (was %s)\n", rxtask.activate & 0x7f, getStateStr(rxtask.mainState));
      rxtask.mainState = ST_DECODER;
      rxtask.currentSonde = rxtask.activate & 0x7F;
      sonde.setup();
    } else if (rxtask.activate != -1) {
      Serial.printf("RXtask: start %s (was %s)\n", getStateStr(rxtask.activate), getStateStr(rxtask.mainState));
      rxtask.mainState = rxtask.activate;
    }
    rxtask.activate = -1;

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
  if (digitalRead(button1.pin) == 0) {
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) {

      if (now - button1.keydownTime > 100)
        button1.doublepress = 1;
      bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button1.doublepress = 0;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  } else {
    unsigned long now = my_millis();
    if (button1.doublepress == -1) return;
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
  if (digitalRead(button2.pin) == 0) {
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) {

      if (now - button2.keydownTime > 100)
        button2.doublepress = 1;

    } else {
      button2.doublepress = 0;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  } else {
    unsigned long now = my_millis();
    if (button2.doublepress == -1) return;
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



  return p;
}


void handlePMUirq() {
  if (sonde.config.button2_axp) {

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
    pmu_irq = false;
  }
}

int getKey2Press() {

  KeyPress p = button2.pressed;
  button2.pressed = KP_NONE;

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
  return p;
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
    *ioport |= portmask;
    *ioport &= ~portmask;
  }
  res = millis() - t;
  Serial.printf("Duration w/ fast io: %d\n", res);
#endif
  for (int i = 0; i < 39; i++) {
    Serial.printf("%d:%d ", i, initlevels[i]);
  }
  Serial.println(" (before setup)");
  sonde.defaultConfig();
  aprs_gencrctab();

  Serial.println("Initializing SPIFFS");

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println("Reading initial configuration");
  setupConfigData();



  if ( ( (sonde.fingerprint & (64 + 31)) != 31) && ((sonde.fingerprint & 16) == 16) ) {

    for (int i = 0; i < 10; i++) {
      Wire.begin(21, 22);

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

        axp.enableIRQ( AXP202_PEK_LONGPRESS_IRQ | AXP202_PEK_SHORTPRESS_IRQ, 1 );
        axp.clearIRQ();
      }
      int ndevices = scanI2Cdevice();
      if (sonde.fingerprint != 17 || ndevices > 0) break;
      delay(500);
    }
  }
  if (sonde.config.power_pout >= 0) {
    pinMode(sonde.config.power_pout & 127, OUTPUT);
    digitalWrite(sonde.config.power_pout & 127, sonde.config.power_pout & 128 ? 1 : 0);
  }

  if (sonde.config.led_pout >= 0) {
    pinMode(sonde.config.led_pout, OUTPUT);
    flashLed(1000);
  }

  button1.pin = sonde.config.button_pin;
  button2.pin = sonde.config.button2_pin;
  if (button1.pin != 0xff) {
    if ( (button1.pin & 0x80) == 0 && button1.pin < 34 ) {
      Serial.println("Button 1 configured as input with pullup");
      pinMode(button1.pin, INPUT_PULLUP);
    } else
      pinMode(button1.pin, INPUT);
  }
  if (button2.pin != 0xff) {
    if ( (button2.pin & 0x80) == 0 && button2.pin < 34 ) {
      Serial.println("Button 2 configured as input with pullup");
      pinMode(button2.pin, INPUT_PULLUP);
    } else
      pinMode(button2.pin, INPUT);
  }

  if ( (button1.pin & 0x80) == 0) {
    attachInterrupt( button1.pin, buttonISR, CHANGE);
    Serial.printf("button1.pin is %d, attaching interrupt\n", button1.pin);
  }

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


#if 0

  if (rs41.setFrequency(402700000) == 0) {
    Serial.println(F("Setting freq: SUCCESS "));
  } else {
    Serial.println(F("Setting freq: ERROR "));
  }
  float f = sx1278.getFrequency();
  Serial.print("Frequency set to ");
  Serial.println(f);

#endif


  sx1278.setLNAGain(0);

  int gain = sx1278.getLNAGain();
  Serial.print("RX LNA Gain is ");
  Serial.println(gain);


  Serial.println(F("SX1278 configuration finished"));

  Serial.println("Setup finished");
  Serial.println();
# 1994 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
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


#ifndef DISABLE_SX1278
  xTaskCreate( sx1278Task, "sx1278Task",
               10000,
               NULL,
               1,
               NULL);
#endif
  sonde.setup();
  initGPS();

  WiFi.onEvent(WiFiEvent);
  getKeyPress();
}

void enterMode(int mode) {
  Serial.printf("enterMode(%d)\n", mode);


  if (mode != ST_DECODER) {
    rxtask.activate = mode;
    while (rxtask.activate == mode) {
      delay(10);
    }
  }
  mainState = (MainState)mode;
  if (mainState == ST_SPECTRUM) {
    Serial.println("Entering ST_SPECTRUM mode");
    sonde.clearDisplay();
    disp.rdis->setFont(FONT_SMALL);
    specTimer = millis();

  } else if (mainState == ST_WIFISCAN) {
    sonde.clearDisplay();
  }
  if (mode == ST_DECODER) {


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

      double val = strtod(value, NULL);

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

  uint16_t res = sonde.waitRXcomplete();
  int action;

  action = (int)(res >> 8);


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

    tncclient = tncserver.available();
    if (tncclient.connected()) {
      Serial.println("new TCP KISS connection");
    }
  }
  if (tncclient.available()) {
    Serial.print("TCP KISS socket: recevied ");
    while (tncclient.available()) {
      Serial.print(tncclient.read());
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

  SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
  if ((res & 0xff) == 0 && (connected || tncclient.connected() )) {



    if (s->validID && ((s->validPos & 0x03) == 0x03)) {
      const char *str = aprs_senddata(s, sonde.config.call, sonde.config.udpfeed.symbol);
      if (connected) {
        char raw[201];
        int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
        Serial.println("Sending AXUDP");

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

      unsigned long time_delta = time_now - time_last_update;
      if ((sonde.config.sondehub.chase == 0) && (time_delta >= SONDEHUB_STATION_UPDATE_TIME)) {
        sondehub_station_update(&shclient, &sonde.config.sondehub);
        time_last_update = time_now;
      }
      else if ((sonde.config.sondehub.chase == 1) && (time_delta >= SONDEHUB_MOBILE_STATION_UPDATE_TIME)) {
        sondehub_station_update(&shclient, &sonde.config.sondehub);
        time_last_update = time_now;
      }
      sondehub_send_data(&shclient, s, &sonde.config.sondehub);
    }
#endif


#if FEATURE_MQTT

    if (connected && mqttEnabled) {
      Serial.println("Sending sonde info via MQTT");
      mqttclient.publishPacket(s);
    }
#endif
  }

  if (rdzclient.connected()) {
    Serial.println("Sending position via TCP as rdzJSON");
    char raw[1024];
    char gps[128];
    const char *typestr = s->typestr;
    if (*typestr == 0) typestr = sondeTypeStr[s->type];

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

    if (len > 1024) len = 1024;
    int wlen = rdzclient.write(raw, len);
    if (wlen != len) {
      Serial.println("Writing rdzClient not OK, closing connection");
      rdzclient.stop();
    }

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
    case KP_SHORT:
      sonde.nextConfig();
      enterMode(ST_DECODER);
      return;
    case KP_MID: break;
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
# 2359 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
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


    WiFi.disconnect(true);
    wifi_state = WIFI_DISABLED;
  }
}

static int wifi_cto;

void loopWifiBackground() {


  if (sonde.config.wifi == 0 || sonde.config.wifi == 2) return;

  if (wifi_state == WIFI_DISABLED) {
    wifi_state = WIFI_SCAN;
    Serial.println("WiFi start scan");
    WiFi.scanNetworks(true);
  } else if (wifi_state == WIFI_SCAN) {
    int16_t res = WiFi.scanComplete();
    if (res == 0 || res == WIFI_SCAN_FAILED) {

      Serial.println("WiFi restart scan");
      WiFi.disconnect(true);
      wifi_state = WIFI_DISABLED;
      return;
    }
    if (res == WIFI_SCAN_RUNNING) {
      return;
    }

    wifiConnect(res);
    wifi_cto = 0;
  } else if (wifi_state == WIFI_CONNECT) {
    wifi_cto++;
    if (WiFi.isConnected()) {
      wifi_state = WIFI_CONNECTED;

      String localIPstr = WiFi.localIP().toString();
      sonde.setIP(localIPstr.c_str(), false);
      sonde.updateDisplayIP();
      enableNetwork(true);
    }
    if (wifi_cto > 20) {
      wifi_state = WIFI_DISABLED;
      WiFi.disconnect(true);
    }
  } else if (wifi_state == WIFI_CONNECTED) {
    if (!WiFi.isConnected()) {
      sonde.setIP("", false);
      sonde.updateDisplayIP();

      wifi_state = WIFI_DISABLED;
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







#define MAXWIFIDELAY 20
static const char* _scan[2] = {"/", "\\"};
void loopWifiScan() {
  if (sonde.config.wifi == 0) {
    wifi_state = WIFI_DISABLED;
    initialMode();
    return;
  }
  if (sonde.config.wifi == 1) {
    wifi_state = WIFI_DISABLED;
    initialMode();
    return;
  }
  if (sonde.config.wifi == 2) {
    startAP();
    initialMode();
    return;
  }

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
  if (index >= 0) {
    Serial.print("Connecting to: "); Serial.print(fetchWifiSSID(index));
    Serial.print(" with password "); Serial.println(fetchWifiPw(index));

    disp.rdis->drawString(0, lastl, "Conn:");
    disp.rdis->drawString(6 * dispxs, lastl, fetchWifiSSID(index));
    WiFi.begin(fetchWifiSSID(index), fetchWifiPw(index));
    while (WiFi.status() != WL_CONNECTED && cnt < MAXWIFIDELAY) {
      delay(500);
      Serial.print(".");
      if (cnt == 5) {

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
  if (index < 0 || cnt >= MAXWIFIDELAY) {
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





String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}


void execOTA() {
  int contentLength = 0;
  bool isValidContentType = false;
  sonde.clearDisplay();
  disp.rdis->setFont(FONT_SMALL);
  disp.rdis->drawString(0, 0, "C:");
  String dispHost = updateHost.substring(0, 14);
  disp.rdis->drawString(2, 0, dispHost.c_str());

  Serial.println("Connecting to: " + updateHost);

  if (client.connect(updateHost.c_str(), updatePort)) {

    Serial.println("Fetching bin: " + String(*updateBin));
    disp.rdis->drawString(0, 1, "Fetching update");


    client.print(String("GET ") + *updateBin + " HTTP/1.1\r\n" +
                 "Host: " + updateHost + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");







    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        client.stop();
        return;
      }
    }
# 2841 "D:/Uskompuf/Documents/coding/esp32 sonde tracker test/gps/rdz_ttgo_sonde/RX_FSK/RX_FSK.ino"
    while (client.available()) {

      String line = client.readStringUntil('\n');

      line.trim();






      if (!line.length()) {

        break;
      }



      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }



      if (line.startsWith("Content-Length: ")) {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }


      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {



    Serial.println("Connection to " + String(updateHost) + " failed. Please check your setup");


  }


  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));
  disp.rdis->drawString(0, 2, "Len: ");
  String cls = String(contentLength);
  disp.rdis->drawString(5, 2, cls.c_str());


  if (contentLength && isValidContentType) {

    bool canBegin = Update.begin(contentLength);
    disp.rdis->drawString(0, 4, "Starting update");


    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");


      disp.rdis->drawString(0, 5, "Please wait!");
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );


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



      Serial.println("Not enough space to begin OTA");
      client.flush();
    }
  } else {
    Serial.println("There was no content in the response");
    client.flush();
  }

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

}




enum SHState { SH_DISCONNECTED, SH_CONNECTING, SH_CONN_IDLE, SH_CONN_WAITACK };

SHState shState = SH_DISCONNECTED;


const char *sondeTypeStrSH[NSondeTypes] = { "DFM", "DFM", "RS41", "RS92", "M10", "M20", "DFM", "MRZ" };
const char *dfmSubtypeStrSH[16] = { NULL, NULL, NULL, NULL, NULL, NULL,
                                    "DFM06",
                                    "PS15",
                                    NULL, NULL,
                                    "DFM09",
                                    "DFM17",
                                    "DFM09P",
                                    "DFM17",
                                    NULL, NULL
                                  };


#define SONDEHUB_TIME_THRESHOLD (3)
void sondehub_send_data(WiFiClient *client, SondeInfo *s, struct st_sondehub *conf) {
  Serial.println("sondehub_send_data()");

#define MSG_SIZE 550
  char rs_msg[MSG_SIZE];
  char *w;
  struct tm ts;


  time_t t = s->time + s->sec;

  while (client->available() > 0) {

    int cnt = client->readBytesUntil('\n', rs_msg, MSG_SIZE);
    rs_msg[cnt] = 0;
    Serial.println(rs_msg);

    if (shState == SH_CONN_WAITACK && cnt > 11 && strncmp(rs_msg, "HTTP/1", 6) == 0) {
      shState = SH_CONN_IDLE;
    }
  }


  if (*s->ser == 0) return;
  if (((int)s->lat == 0) && ((int)s->lon == 0)) return;
  if ((int)s->alt > 50000) return;
  if ((int)s->sats < 4) return;



  if (!client->connected()) {
    Serial.println("NO CONNECTION");
    shState = SH_DISCONNECTED;
    if (!client->connect(conf->host, 80)) {
      Serial.println("Connection FAILED");
      return;
    }
    client->Client::setTimeout(0);
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



  if ( s->type == STYPE_RS41 || s->type == STYPE_RS92 || s->type == STYPE_M10 || s->type == STYPE_M20 ) {
    t += 18;
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




    sprintf(w, "\"frame\": %d,", int(t - 315964800));
  } else {
    sprintf(w, "\"frame\": %d,", s->frame);
  }
  w += strlen(w);

  sprintf(w, "\"type\": \"%s\",", sondeTypeStrSH[s->type]);
  w += strlen(w);


  if ( TYPE_IS_DFM(s->type) && s->subtype > 0 && s->subtype < 16 ) {
    const char *t = dfmSubtypeStrSH[s->subtype];

    if (t) sprintf(w, "\"subtype\": \"%s\",", t);
    else sprintf(w, "\"subtype\": \"DFMx%X\",", s->subtype);
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


}

#endif
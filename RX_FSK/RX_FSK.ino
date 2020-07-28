#include <axp20x.h>

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

int LORA_LED = 9;                             // default POUT for LORA LED used as serial monitor
int e;

enum MainState { ST_DECODER, ST_SPECTRUM, ST_WIFISCAN, ST_UPDATE };
static MainState mainState = ST_WIFISCAN; // ST_WIFISCAN;

AsyncWebServer server(80);
AXP20X_Class axp;
#define PMU_IRQ             35


String updateHost = "rdzsonde.mooo.com";
int updatePort = 80;
String updateBinM = "/master/update.ino.bin";
String updateBinD = "/devel/update.ino.bin";
String *updateBin = &updateBinM;

#define LOCALUDPPORT 9002

boolean connected = false;
WiFiUDP udp;
WiFiClient client;

// KISS over TCP für communicating with APRSdroid
WiFiServer tncserver(14580);
WiFiClient tncclient;

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

// Set LED GPIO
int ledPin = 1;
// Stores LED state
String ledState;

// timestamp when spectrum display was activated
static unsigned long specTimer;

// Replaces placeholder with LED state value
String processor(const String& var) {
  Serial.println(var);
  if (var == "STATE") {
    if (digitalRead(ledPin)) {
      ledState = "ON";
    }
    else {
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  if (var == "VERSION_NAME") {
    return String(version_name);
  }
  if (var == "VERSION_ID") {
    return String(version_id);
  }
  return String();
}

const String sondeTypeSelect(int activeType) {
  String sts = "";
  for (int i = 0; i < 4; i++) {
    sts += "<option value=\"";
    sts += sondeTypeStr[i];
    sts += "\"";
    if (activeType == i) {
      sts += " selected";
    }
    sts += ">";
    sts += sondeTypeStr[i];
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
    String line = file.readStringUntil('\n');
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
      type = STYPE_DFM09;
    }
    else if (space[1] == '6') {
      type = STYPE_DFM06;
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

const char *createQRGForm() {
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"qrg.html\" method=\"post\"><table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Launchsite</th><th>Mode</th></tr>");
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    String s = sondeTypeSelect(i >= sonde.nSonde ? 2 : sonde.sondeList[i].type);
    String site = sonde.sondeList[i].launchsite;
    sprintf(ptr + strlen(ptr), "<tr><td>%d</td><td><input name=\"A%d\" type=\"checkbox\" %s/></td>"
            "<td><input name=\"F%d\" type=\"text\" value=\"%3.3f\"></td>"
            "<td><input name=\"S%d\" type=\"text\" value=\"%s\"></td>"
            "<td><select name=\"T%d\">%s</select></td>",
            i + 1,
            i + 1, (i < sonde.nSonde && sonde.sondeList[i].active) ? "checked" : "",
            i + 1, i >= sonde.nSonde ? 400.000 : sonde.sondeList[i].freq,
            i + 1, i >= sonde.nSonde ? "                " : sonde.sondeList[i].launchsite,
            i + 1, s.c_str());
  }
  strcat(ptr, "</table><input type=\"submit\" value=\"Update\"/></form></body></html>");
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
    char typech = (tstr[2] == '4' ? '4' : tstr[2] == '9' ? 'R' : tstr[3]);   // a bit ugly
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
    String line = file.readStringUntil('\n');
    if (!file.available()) break;
    networks[i].id = line;
    networks[i].pw = file.readStringUntil('\n');
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
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"wifi.html\" method=\"post\"><table><tr><th>Nr</th><th>SSID</th><th>Password</th></tr>");
  for (int i = 0; i < MAX_WIFI; i++) {
    sprintf(tmp, "%d", i);
    sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"S%d\" type=\"text\" value=\"%s\"/></td>"
            "<td><input name=\"P%d\" type=\"text\" value=\"%s\"/></td>",
            i == 0 ? "<b>AP</b>" : tmp,
            i + 1, i < nNetworks ? networks[i].id.c_str() : "",
            i + 1, i < nNetworks ? networks[i].pw.c_str() : "");
  }
  strcat(ptr, "</table><input type=\"submit\" value=\"Update\"></input></form></body></html>");
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
  SondeInfo *s = &sonde.sondeList[i];
  strcat(ptr, "<table>");
  sprintf(ptr + strlen(ptr), "<tr><td id=\"sfreq\">%3.3f MHz, Type: %s</td><tr><td>ID: %s</td></tr><tr><td>QTH: %.6f,%.6f h=%.0fm</td></tr>\n",
          s->freq, sondeTypeStr[s->type],
          s->validID ? s->id : "<?""?>",
          s->lat, s->lon, s->alt);
  sprintf(ptr + strlen(ptr), "<tr><td><a target=\"_empty\" href=\"geo:%.6f,%.6f\">GEO-App</a> - ", s->lat, s->lon);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://wx.dl2mf.de/?%s\">WX.DL2MF.de</a> - ", s->id);
  sprintf(ptr + strlen(ptr), "<a target=\"_empty\" href=\"https://www.openstreetmap.org/?mlat=%.6f&mlon=%.6f&zoom=14\">OSM</a></td></tr>", s->lat, s->lon);
  strcat(ptr, "</table><p/>\n");
}

const char *createStatusForm() {
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"><meta http-equiv=\"refresh\" content=\"5\"></head><body>");

  for (int i = 0; i < sonde.nSonde; i++) {
    int snum = (i + sonde.currentSonde) % sonde.nSonde;
    if (sonde.sondeList[snum].active) {
      addSondeStatus(ptr, snum);
    }
  }
  strcat(ptr, "</body></html>");
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
    String line = file.readStringUntil('\n');
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
  {"wifi", "Wifi mode (0/1/2/3)", 0, &sonde.config.wifi},
  {"debug", "Debug mode (0/1)", 0, &sonde.config.debug},
  {"maxsonde", "Maxsonde", 0, &sonde.config.maxsonde},
  {"display", "Display mode (1/2/3)", 0, &sonde.config.display},
  {"---", "---", -1, NULL},
  /* Spectrum display settings */
  {"spectrum", "ShowSpectrum (s)", 0, &sonde.config.spectrum},
  {"startfreq", "Startfreq (MHz)", 0, &sonde.config.startfreq},
  {"channelbw", "Bandwidth (kHz)", 0, &sonde.config.channelbw},
  {"timer", "Spectrum Timer", 0, &sonde.config.timer},
  {"marker", "Spectrum MHz marker", 0, &sonde.config.marker},
  {"noisefloor", "Sepctrum noisefloor", 0, &sonde.config.noisefloor},
  {"showafc", "Show AFC value", 0, &sonde.config.showafc},
  {"freqofs", "RX frequency offset (Hz)", 0, &sonde.config.freqofs},
  {"---", "---", -1, NULL},
  /* APRS settings */
  {"call", "Call", 8, sonde.config.call},
  {"passcode", "Passcode", 8, sonde.config.passcode},
  {"---", "---", -1, NULL},
  /* KISS tnc settings */
  {"kisstnc", "KISS TNC (port 14590) (needs reboot)", 0, &sonde.config.kisstnc.active},
  {"kisstnc.idformat", "DFM ID Format", -2, &sonde.config.kisstnc.idformat},
  /* AXUDP settings */
  {"axudp.active", "AXUDP active", -3, &sonde.config.udpfeed.active},
  {"axudp.host", "AXUDP Host", 63, sonde.config.udpfeed.host},
  {"axudp.port", "AXUDP Port", 0, &sonde.config.udpfeed.port},
  {"axudp.idformat", "DFM ID Format", -2, &sonde.config.udpfeed.idformat},
  {"axudp.highrate", "Rate limit", 0, &sonde.config.udpfeed.highrate},
  {"---", "---", -1, NULL},
  /* APRS TCP settings, current not used */
  {"tcp.active", "APRS TCP active", -3, &sonde.config.tcpfeed.active},
  {"tcp.host", "ARPS TCP Host", 63, sonde.config.tcpfeed.host},
  {"tcp.port", "APRS TCP Port", 0, &sonde.config.tcpfeed.port},
  {"tcp.idformat", "DFM ID Format", -2, &sonde.config.tcpfeed.idformat},
  {"tcp.highrate", "Rate limit", 0, &sonde.config.tcpfeed.highrate},
  {"---", "---", -1, NULL},
  /* decoder settings */
  {"rs41.agcbw", "RS41 AGC bandwidth", 0, &sonde.config.rs41.agcbw},
  {"rs41.rxbw", "RS41 RX bandwidth", 0, &sonde.config.rs41.rxbw},
  {"rs92.rxbw", "RS92 RX (and AGC) bandwidth", 0, &sonde.config.rs92.rxbw},
  {"rs92.alt2d", "RS92 2D fix default altitude", 0, &sonde.config.rs92.alt2d},
  {"dfm.agcbw", "DFM6/9 AGC bandwidth", 0, &sonde.config.dfm.agcbw},
  {"dfm.rxbw", "DFM6/9 RX bandwidth", 0, &sonde.config.dfm.rxbw},
  {"---", "---", -1, NULL},
  /* Hardware dependeing settings */
  {"disptype", "Display type (0=OLED/SSD1306, 1=TFT/ILI9225, 2=OLED/SH1106)", 0, &sonde.config.disptype},
  {"oled_sda", "OLED/TFT SDA (needs reboot)", 0, &sonde.config.oled_sda},
  {"oled_scl", "OLED SCL/TFT CLK (needs reboot)", 0, &sonde.config.oled_scl},
  {"oled_rst", "OLED/TFT RST (needs reboot)", 0, &sonde.config.oled_rst},
  {"tft_rs", "TFT RS (needs reboot)", 0, &sonde.config.tft_rs},
  {"tft_cs", "TFT CS (needs reboot)", 0, &sonde.config.tft_cs},
  {"button_pin", "Button input port (needs reboot)", -4, &sonde.config.button_pin},
  {"button2_pin", "Button 2 input port (needs reboot)", -4, &sonde.config.button2_pin},
  {"touch_thresh", "Touch button threshold (needs reboot)", 0, &sonde.config.touch_thresh},
  {"led_pout", "LED output port (needs reboot)", 0, &sonde.config.led_pout},
  {"gps_rxd", "GPS RXD pin (-1 to disable)", 0, &sonde.config.gps_rxd},
  {"gps_txd", "GPS TXD pin (not really needed)", 0, &sonde.config.gps_txd},
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
  sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" size=\"3\" value=\"%d\"/>",
          label, idx, 127 & *value);
  sprintf(ptr + strlen(ptr), "<input type=\"checkbox\" name=\"TO%d\"%s> Touch </td></tr>\n", idx, 128 & *value ? " checked" : "");
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

const char *createConfigForm() {
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"config.html\" method=\"post\"><table><tr><th>Option</th><th>Value</th></tr>");
  for (int i = 0; i < N_CONFIG; i++) {
    switch (config_list[i].type) {
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
  strcat(ptr, "</table><input type=\"submit\" value=\"Update\"></input></form></body></html>");
  return message;
}


const char *handleConfigPost(AsyncWebServerRequest *request) {
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File f = SPIFFS.open("/config.txt", "w");
  if (!f) {
    Serial.println("Error while opening '/config.txt' for writing");
    return "Error while opening '/config.txt' for writing";
  }
#endif
  Serial.println("Handling post request");
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
        int i = atoi(strvalue.c_str()) + 128;
        strvalue = String(i);
      }
    }
    Serial.printf("Processing  %s=%s\n", config_list[idx].name, strvalue.c_str());
    f.printf("%s=%s\n", config_list[idx].name, strvalue.c_str());
  }
  f.close();
  setupConfigData();
  return "";
}

const char *ctrlid[] = {"rx", "scan", "spec", "wifi", "rx2", "scan2", "spec2", "wifi2"};

const char *ctrllabel[] = {"Receiver (short keypress)", "Scanner (double keypress)", "Spectrum (medium keypress)", "WiFi (long keypress)",
                           "Button 2 (short keypress)", "Button 2 (double keypress)", "Button 2 (medium keypress)", "Button 2 (long keypress)"
                          };

const char *createControlForm() {
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"control.html\" method=\"post\">");
  for (int i = 0; i < 8; i++) {
    strcat(ptr, "<input type=\"submit\" name=\"");
    strcat(ptr, ctrlid[i]);
    strcat(ptr, "\" value=\"");
    strcat(ptr, ctrllabel[i]);
    strcat(ptr, "\"></input><br>");
  }
  strcat(ptr, "</form></body></html>");
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

// bad idea. prone to buffer overflow. use at your own risk...
const char *createEditForm(String filename) {
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
  strcat(ptr, "\" method=\"post\">");
  strcat(ptr, "<textarea name=\"text\" cols=\"80\" rows=\"40\">");
  while (file.available()) {
    String line = file.readStringUntil('\n');
    strcat(ptr, line.c_str()); strcat(ptr, "\n");
  }
  strcat(ptr, "</textarea><input type=\"submit\" value=\"Save\"></input></form></body></html>");
  return message;
}


const char *handleEditPost(AsyncWebServerRequest *request) {
  Serial.println("Handling post request");
  AsyncWebParameter *filep = request->getParam("file");
  if (!filep) return NULL;
  String filename = filep->value();
  AsyncWebParameter *textp = request->getParam("text", true);
  if (!textp) return NULL;
  String content = textp->value();
  File file = SPIFFS.open("/" + filename, "w");
  if (!file) {
    Serial.println("There was an error opening the file '/" + filename + "'for writing");
    return "";
  }
  file.print(content);
  file.close();
  if (strcmp(filename.c_str(), "screens.txt") == 0) {
    // screens update => reload
    disp.initFromFile();
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


const char* PARAM_MESSAGE = "message";
void SetupAsyncServer() {
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

  server.on("/edit.html", HTTP_GET,  [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", createEditForm(request->getParam(0)->value()));
  });
  server.on("/edit.html", HTTP_POST, [](AsyncWebServerRequest * request) {
    handleEditPost(request);
    request->send(200, "text/html", createEditForm(request->getParam(0)->value()));
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  server.on("/test.php", HTTP_POST, [](AsyncWebServerRequest * request) {
    //digitalWrite(ledPin, HIGH);
    request->send(SPIFFS, "/index.html", String(), false, processor);
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
    Serial.printf("No match: '%s' vs '%s'\n", id, networks[i].id.c_str());
    const char *cfgid = networks[i].id.c_str();
    int len = strlen(cfgid);
    if (strlen(id) > len) len = strlen(id);
    Serial.print("SSID: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02x ", id[i]);
    } Serial.println("");
    Serial.print("Conf: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02x ", cfgid[i]);
    } Serial.println("");
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

#define IS_TOUCH(x) (((x)!=255)&&((x)!=-1)&&((x)&128))
void initTouch() {
  if ( !(IS_TOUCH(sonde.config.button_pin) || IS_TOUCH(sonde.config.button2_pin)) ) return; // no touch buttons configured


  /*
   *  ** no. readTouch is not safe to use in ISR!
      so now using Ticker
    hw_timer_t *timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, checkTouchStatus, true);
    timerAlarmWrite(timer, 300000, true);
    timerAlarmEnable(timer);
  */
  ticker.attach_ms(300, checkTouchStatus);

  if ( IS_TOUCH(sonde.config.button_pin) ) {
    touchAttachInterrupt(sonde.config.button_pin & 0x7f, touchISR, 60);
    Serial.printf("Initializing touch 1 on pin %d\n", sonde.config.button_pin & 0x7f);
  }
  if ( IS_TOUCH(sonde.config.button2_pin) ) {
    touchAttachInterrupt(sonde.config.button2_pin & 0x7f, touchISR2, 60);
    Serial.printf("Initializing touch 2 on pin %d\n", sonde.config.button2_pin & 0x7f);
  }
}

char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));

void gpsTask(void *parameter) {
  while (1) {
    while (Serial2.available()) {
      char c = Serial2.read();
      //Serial.print(c);
      if (nmea.process(c)) {
        long lat = nmea.getLatitude();
        long lon = nmea.getLongitude();
        long alt = -1;
        bool b = nmea.getAltitude(alt);
        bool valid = nmea.isValid();
        uint8_t hdop = nmea.getHDOP();
        Serial.printf("\nDecode: valid: %d  N %ld  E %ld  alt %ld (%d) dop:%d", valid ? 1 : 0, lat, lon, alt, b, hdop);
      }
    }
    delay(50);
  }
}

void initGPS() {
  if (sonde.config.gps_rxd < 0) return; // GPS disabled
  Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);

  xTaskCreate( gpsTask, "gpsTask",
               5000, /* stack size */
               NULL, /* paramter */
               1, /* priority */
               NULL);  /* task handle*/
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
      Serial.printf("rx task: activate=%d  mainstate=%d\n", rxtask.activate, rxtask.mainState);
      rxtask.mainState = ST_DECODER;
      rxtask.currentSonde = rxtask.activate & 0x7F;
      Serial.println("rx task: calling sonde.setup()");
      sonde.setup();
    } else if (rxtask.activate != -1) {
      Serial.printf("rx task: activate=%d  mainstate=%d\n", rxtask.activate, rxtask.mainState);
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
    Serial.printf("touch read %d: value is %d\n", button.pin,tmp);
    if (tmp > sonde.config.touch_thresh) {
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

int getKeyPress() {
  KeyPress p = button1.pressed;
  button1.pressed = KP_NONE;
  int x = digitalRead(button1.pin);
  Serial.printf("Debug: bdd1=%ld, bdd2=%ld\b", bdd1, bdd2);

  Serial.printf("button1 press (dbl:%d) (now:%d): %d at %ld (%d)\n", button1.doublepress, x, p, button1.keydownTime, button1.numberKeyPresses);
  return p;
}

int getKey2Press() {
  KeyPress p = button2.pressed;
  button2.pressed = KP_NONE;
  Serial.printf("button2 press: %d at %ld (%d)\n", p, button2.keydownTime, button2.numberKeyPresses);
  return p;
}
int hasKeyPress() {
  return button1.pressed || button2.pressed;
}
int getKeyPressEvent() {
  int p = getKeyPress();
  if (p == KP_NONE) {
    p = getKey2Press();
    if (p == KP_NONE)
      return EVT_NONE;
    return p + 4;
  }
  return p;  /* map KP_x to EVT_KEY1_x / EVT_KEY2_x*/
}

#define SSD1306_ADDRESS 0x3c
#define AXP192_SLAVE_ADDRESS    0x34
bool ssd1306_found = false;
bool axp192_found = false;

void scanI2Cdevice(void)
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
}

extern int initlevels[40];
extern DispInfo *layouts;
bool pmu_irq = false;


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
  delay(2000);

  for (int i = 0; i < 39; i++) {
    Serial.printf("%d:%d ", i, initlevels[i]);
  }
  Serial.println(" (before setup)");

  aprs_gencrctab();

  Serial.println("Initializing SPIFFS");
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println("Reading initial configuration");
  setupConfigData();    // configuration must be read first due to OLED ports!!!

  // FOr T-Beam 1.0
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

  pinMode(PMU_IRQ, INPUT_PULLUP);
  attachInterrupt(PMU_IRQ, [] {
    pmu_irq = true;
  }, FALLING);

  axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
  axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
  axp.clearIRQ();

  delay(500);
  scanI2Cdevice();


  LORA_LED = sonde.config.led_pout;
  pinMode(LORA_LED, OUTPUT);

  button1.pin = sonde.config.button_pin;
  button2.pin = sonde.config.button2_pin;
  if (button1.pin != 0xff)
    pinMode(button1.pin, INPUT);  // configure as input if not disabled
  if (button2.pin != 0xff)
    pinMode(button2.pin, INPUT);  // configure as input if not disabled

  // Handle button press
  if ( (button1.pin & 0x80) == 0) {
    attachInterrupt( button1.pin, buttonISR, CHANGE);
    Serial.printf("button1.pin is %d, attaching interrupt\n", button1.pin);
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
  Serial.printf("before disp.initFromFile... layouts is %p", layouts);

  disp.initFromFile();
  Serial.printf("disp.initFromFile... layouts is %p", layouts);


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

  xTaskCreate( sx1278Task, "sx1278Task",
               10000, /* stack size */
               NULL, /* paramter */
               1, /* priority */
               NULL);  /* task handle*/
  sonde.setup();
  initGPS();

  if (sonde.config.kisstnc.active) {
    tncserver.begin();
  }
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
void loopDecoder() {
  // sonde knows the current type and frequency, and delegates to the right decoder
  uint16_t res = sonde.waitRXcomplete();
  int action, event = 0;
  action = (int)(res >> 8);
  // TODO: update displayed sonde?

  if (action != ACT_NONE) {
    Serial.printf("Loop: triggering action %s (%d)\n", action2text(action), action);
    action = sonde.updateState(action);
    Serial.printf("Loop: action is %d, sonde index is %d\n", action, sonde.currentSonde);
    if (action != 255) {
      if (action == ACT_DISPLAY_SPECTRUM) {
        enterMode(ST_SPECTRUM);
        return;
      }
      else if (action == ACT_DISPLAY_WIFI) {
        enterMode(ST_WIFISCAN);
        return;
      }
      // no... we are already in DECODER mode, so no need to do anything!?
      //else if (action == ACT_NEXTSONDE) enterMode(ST_DECODER); // update rx background task
    }
    Serial.printf("current main is %d, current rxtask is %d\n", sonde.currentSonde, rxtask.currentSonde);
  }

  if (!tncclient.connected()) {
    Serial.println("TNC client not connected");
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
  // wifi (axudp) or bluetooth (bttnc) active => send packet
  if ((res & 0xff) == 0 && (connected || tncclient.connected() )) {
    //Send a packet with position information
    // first check if ID and position lat+lonis ok
    SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
    if (s->validID && ((s->validPos & 0x03) == 0x03)) {
      const char *str = aprs_senddata(s->lat, s->lon, s->alt, s->hs, s->dir, s->vs, sondeTypeStr[s->type], s->id, "TE0ST",
                                      sonde.config.udpfeed.symbol);
      if (connected)  {
        char raw[201];
        int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
        Serial.println("Sending position via UDP");
        Serial.print("Sending: "); Serial.println(raw);
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
  }
  Serial.println("updateDisplay started");
  sonde.updateDisplay();
  Serial.println("updateDisplay done");
}


void loopSpectrum() {
  int marker = 0;
  char buf[10];

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
      currentDisplay = 0;
      enterMode(ST_DECODER);
      return;
    default: break;
  }

  scanner.scan();
  scanner.plotResult();
  if (sonde.config.marker != 0) {
    itoa((sonde.config.startfreq), buf, 10);
    disp.rdis->drawString(0, 1, buf);
    disp.rdis->drawString(7, 1, "MHz");
    itoa((sonde.config.startfreq + 6), buf, 10);
    disp.rdis->drawString(13, 1, buf);
  }
  if (sonde.config.timer) {
    int remaining = sonde.config.spectrum - (millis() - specTimer) / 1000;
    itoa(remaining, buf, 10);
    Serial.printf("timer:%d config.spectrum:%d  specTimer:%ld millis:%ld remaining:%d\n", sonde.config.timer, sonde.config.spectrum, specTimer, millis(), remaining);
    if (sonde.config.marker != 0) {
      marker = 1;
    }
    disp.rdis->drawString(0, 1 + marker, buf);
    disp.rdis->drawString(2, 1 + marker, "Sec.");
    if (remaining <= 0) {
      currentDisplay = 0;
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

void enableNetwork(bool enable) {
  if (enable) {
    MDNS.begin("rdzsonde");
    SetupAsyncServer();
    udp.begin(WiFi.localIP(), LOCALUDPPORT);
    MDNS.addService("http", "tcp", 80);
    tncserver.begin();
    connected = true;
  } else {
    MDNS.end();
    connected = false;
  }
}


enum t_wifi_state { WIFI_DISABLED, WIFI_SCAN, WIFI_CONNECT, WIFI_CONNECTED, WIFI_APMODE };

static t_wifi_state wifi_state = WIFI_DISABLED;

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
  Serial.printf("WLAN scan result: found %d networks\n", res);

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
    Serial.println("WLAN start scan");
    WiFi.scanNetworks(true); // scan in async mode
  } else if (wifi_state == WIFI_SCAN) {
    int16_t res = WiFi.scanComplete();
    if (res == 0 || res == WIFI_SCAN_FAILED) {
      // retry
      Serial.println("WLAN restart scan");
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
      sonde.clearIP();
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
  Serial.println(WiFi.softAPConfig(IPAddress (192,168,4,1), IPAddress (0,0,0,0), IPAddress (255,255,255,0)) ? "Ready" : "Failed!");
  
  IPAddress myIP = WiFi.softAPIP();
  String myIPstr = myIP.toString();
  sonde.setIP(myIPstr.c_str(), true);
  sonde.updateDisplayIP();
  enableNetwork(true);
}

void initialMode() {
  if (sonde.config.spectrum != 0) {    // enable Spectrum in config.txt: spectrum=number_of_seconds
    startSpectrumDisplay();
    //done in startSpectrumScan(): enterMode(ST_SPECTRUM);
  } else {
    currentDisplay = 0;
    enterMode(ST_DECODER);
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

  int line = 0;
  int cnt = 0;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  int index = -1;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.print("Network name: ");
    String ssid = WiFi.SSID(i);
    Serial.println(ssid);
    disp.rdis->drawString(0, 1 + line, ssid.c_str());
    line = (line + 1) % 5;
    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));
    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");
    int curidx = fetchWifiIndex(ssid.c_str());
    if (curidx >= 0 && index == -1) {
      index = curidx;
      Serial.printf("Match found at scan entry %d, config network %d\n", i, index);
    }
  }
  if (index >= 0) { // some network was found
    Serial.print("Connecting to: "); Serial.print(fetchWifiSSID(index));
    Serial.print(" with password "); Serial.println(fetchWifiPw(index));

    disp.rdis->drawString(0, 6, "Conn:");
    disp.rdis->drawString(6, 6, fetchWifiSSID(index));
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
      disp.rdis->drawString(15, 7, _scan[cnt & 1]);
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
    disp.rdis->drawString(0, 6, "AP:             ");
    disp.rdis->drawString(6, 6, networks[0].id.c_str());
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

  if (sonde.config.spectrum != 0) {     // enable Spectrum in config.txt: spectrum=number_of_seconds
    //startSpectrumDisplay();
    enterMode(ST_SPECTRUM);
  } else {
    currentDisplay = 0;
    enterMode(ST_DECODER);
  }
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
  Serial.printf("\nRunning main loop in state %d. free heap: %d;\n", mainState, ESP.getFreeHeap());
  Serial.printf("currentDisp:%d lastDisp:%d\n", currentDisplay, lastDisplay);
  switch (mainState) {
    case ST_DECODER: loopDecoder(); break;
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_WIFISCAN: loopWifiScan(); break;
    case ST_UPDATE: execOTA(); break;
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
}

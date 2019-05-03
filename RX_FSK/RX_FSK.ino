#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <U8x8lib.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Update.h>
#include <ESPmDNS.h>

#include <SX1278FSK.h>
#include <Sonde.h>
#include <Scanner.h>
#include <aprs.h>
#include "version.h"

// UNCOMMENT one of the constructor lines below
U8X8_SSD1306_128X64_NONAME_SW_I2C *u8x8 = NULL; // initialize later after reading config file
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

int LORA_LED = 9;                             // default POUT for LORA LED used as serial monitor
int e;

enum MainState { ST_DECODER, ST_SCANNER, ST_SPECTRUM, ST_WIFISCAN, ST_UPDATE };
static MainState mainState = ST_WIFISCAN; // ST_WIFISCAN;

AsyncWebServer server(80);

String updateHost = "rdzsonde.my.to";
int updatePort = 80;
String updateBinM = "/master/update.ino.bin";
String updateBinD = "/devel/update.ino.bin";
String *updateBin = &updateBinM;

#define LOCALUDPPORT 9002

boolean connected = false;
WiFiUDP udp;
WiFiClient client;

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
  if(var == "VERSION_NAME") {
    return String(version_name);
  }
  if(var == "VERSION_ID") {
    return String(version_id);
  }
  return String();
}

const String sondeTypeSelect(int activeType) {
  String sts = "";
  for (int i = 0; i < 3; i++) {
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
char message[10240*4];  //needs to be large enough for all forms (not checked in code)
// QRG form is currently about 24kb with 100 entries

///////////////////////// Functions for Reading / Writing QRG list from/to qrg.txt

void setupChannelList() {
  File file = SPIFFS.open("/qrg.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/qrg.txt' for reading");
    return;
  }
  int i = 0;
  sonde.clearSonde();
  Serial.println("Reading channel config:");
  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (!file.available()) break;
    if (line[0] == '#') continue;
    char *space = strchr(line.c_str(), ' ');
    if (!space) continue;
    *space = 0;
    float freq = atof(line.c_str());
    SondeType type;
    if (space[1] == '4') {
      type = STYPE_RS41;
    }
    else if (space[1] == '9') {
      type = STYPE_DFM09;
    }
    else if (space[1] == '6') {
      type = STYPE_DFM06;
    }
    else continue;
    int active = space[3] == '+' ? 1 : 0;
    char *launchsite = strchr(line.c_str(), ' ');
    Serial.printf("Add %f - type %d (on/off: %d)- Site: \n", freq, type, active, launchsite);
    sonde.addSonde(freq, type, active, launchsite);
    i++;
  }
}

const char *createQRGForm() {
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"qrg.html\" method=\"post\"><table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Mode</th></tr>");
  for (int i = 0; i < sonde.config.maxsonde; i++) {
    String s = sondeTypeSelect(i >= sonde.nSonde ? 2 : sonde.sondeList[i].type);
    sprintf(ptr + strlen(ptr), "<tr><td>%d</td><td><input name=\"A%d\" type=\"checkbox\" %s/></td>"
            "<td><input name=\"F%d\" type=\"text\" value=\"%3.3f\"></td>"
            "<td><select name=\"T%d\">%s</select></td>",
            i + 1,
            i + 1, (i < sonde.nSonde && sonde.sondeList[i].active) ? "checked" : "",
            i + 1, i >= sonde.nSonde ? 400.000 : sonde.sondeList[i].freq,
            i + 1, s.c_str());
  }
  strcat(ptr, "</table><input type=\"submit\" value=\"Update\"/></form></body></html>");
  return message;
}

const char *handleQRGPost(AsyncWebServerRequest *request) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File f = SPIFFS.open("/qrg.txt", "w");
  if (!f) {
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
    if (!freq) continue;
    snprintf(label, 10, "T%d", i);
    AsyncWebParameter *type = request->getParam(label, true);
    if (!type) continue;
    String fstring = freq->value();
    String tstring = type->value();
    const char *fstr = fstring.c_str();
    const char *tstr = tstring.c_str();
    Serial.printf("Processing a=%s, f=%s, t=%s\n", active ? "YES" : "NO", fstr, tstr);
    char typech = (tstr[2] == '4' ? '4' : tstr[3]); // Ugly TODO
    f.printf("%3.3f %c %c\n", atof(fstr), typech, active ? '+' : '-');
  }
  f.close();
  Serial.println("Channel setup finished");
  Serial.println();

  setupChannelList();
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
}

// Show current status
void addSondeStatus(char *ptr, int i)
{
  SondeInfo *s = &sonde.sondeList[i];
  strcat(ptr, "<table>");
  sprintf(ptr + strlen(ptr), "<tr><td id=\"sfreq\">%3.3f MHz, Type: %s</td><tr><td>ID: %s</td></tr><tr><td>QTH: %.6f,%.6f h=%.0fm</td></tr>\n",
          s->freq, sondeTypeStr[s->type],
          s->validID ? s->id : "<??>",
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
    addSondeStatus(ptr, (i + sonde.currentSonde) % sonde.nSonde);
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
  {"wifi","Wifi mode (0/1/2/3)", 0, &sonde.config.wifi},
  {"debug","Debug mode (0/1)", 0, &sonde.config.debug},
  {"maxsonde","Maxsonde (requires reboot?)", 0, &sonde.config.maxsonde},
  /* Spectrum display settings */
  {"spectrum","ShowSpectrum (s)", 0, &sonde.config.spectrum},
  {"startfreq","Startfreq (MHz)", 0, &sonde.config.startfreq},
  {"channelbw","Bandwidth (kHz)", 0, &sonde.config.channelbw},
  {"timer","Spectrum Timer", 0, &sonde.config.timer},
  {"marker","Spectrum MHz marker", 0, &sonde.config.marker},
  {"noisefloor","Sepctrum noisefloor", 0, &sonde.config.noisefloor},
  {"---", "---", -1, NULL},
  /* APRS settings */
  {"call","Call", 8, sonde.config.call},
  {"passcode","Passcode", 8, sonde.config.passcode},
  {"---", "---", -1, NULL},
  /* AXUDP settings */
  {"axudp.active","AXUDP active", -3, &sonde.config.udpfeed.active},
  {"axudp.host","AXUDP Host", 63, sonde.config.udpfeed.host},
  {"axudp.port","AXUDP Port", 0, &sonde.config.udpfeed.port},
  {"axudp.idformat","DFM ID Format", -2, &sonde.config.udpfeed.idformat},
  {"axudp.highrate","Rate limit", 0, &sonde.config.udpfeed.highrate},
  {"---", "---", -1, NULL},
  /* APRS TCP settings, current not used */
  {"tcp.active","APRS TCP active", -3, &sonde.config.tcpfeed.active},
  {"tcp.host","ARPS TCP Host", 63, sonde.config.tcpfeed.host},
  {"tcp.port","APRS TCP Port", 0, &sonde.config.tcpfeed.port},
  {"tcp.idformat","DFM ID Format", -2, &sonde.config.tcpfeed.idformat},
  {"tcp.highrate","Rate limit", 0, &sonde.config.tcpfeed.highrate},
  {"---", "---", -1, NULL},
  /* Hardware dependeing settings */
  {"oled_sda","OLED SDA (needs reboot)", 0, &sonde.config.oled_sda},
  {"oled_scl","OLED SCL (needs reboot)", 0, &sonde.config.oled_scl},
  {"oled_rst","OLED RST (needs reboot)", 0, &sonde.config.oled_rst},
  {"button_pin","Button input port (needs reboot)", 0, &sonde.config.button_pin},
  {"led_pout","LED output port (needs reboot)", 0, &sonde.config.led_pout},
};
const static int N_CONFIG=(sizeof(config_list)/sizeof(struct st_configitems));

void addConfigStringEntry(char *ptr, int idx, const char *label, int len, char *field) {
  sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" value=\"%s\"/></td></tr>\n",
          label, idx, field);
}
void addConfigNumEntry(char *ptr, int idx, const char *label, int *value) {
  sprintf(ptr + strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" value=\"%d\"/></td></tr>\n",
          label, idx, *value);
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
  char tmp[4];
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
      default:
        addConfigStringEntry(ptr, i, config_list[i].label, config_list[i].type, (char *)config_list[i].data);
        break;
    }
  }
  strcat(ptr, "</table><input type=\"submit\" value=\"Update\"></input></form></body></html>");
  return message;
}


const char *handleConfigPost(AsyncWebServerRequest *request) {
  char label[10];
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
    if(strncmp(label, "CFG", 3)!=0) continue;
    int idx = atoi(label+3);
    Serial.printf("idx is %d\n", idx);
    if(config_list[idx].type == -1) continue;  // skip separator entries, should not happen
    AsyncWebParameter *value = request->getParam(label, true);
    if(!value) continue;
    String strvalue = value->value();
    Serial.printf("Processing  %s=%s\n", config_list[idx].name, strvalue.c_str());
    f.printf("%s=%s\n", config_list[idx].name, strvalue.c_str());
  }
  f.close();
  setupConfigData();
}

const char *createUpdateForm(boolean run) {
  char *ptr = message;
  char tmp[4];
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"update.html\" method=\"post\">");
  if(run) {
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
    if(param.equals("devel")) {
      Serial.println("equals devel");
      updateBin = &updateBinD;
    }
    else if(param.equals("master")) {
      Serial.println("equals master");
      updateBin = &updateBinM;  
    }
  }
  Serial.println("Updating: "+*updateBin);
  enterMode(ST_UPDATE);
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
    int len=strlen(cfgid);
    if(strlen(id)>len) len=strlen(id);
    Serial.print("SSID: ");
    for(int i = 0; i < len; i++) { Serial.printf("%02x ", id[i]); } Serial.println("");
    Serial.print("Conf: ");
    for(int i = 0; i < len; i++) { Serial.printf("%02x ", cfgid[i]); } Serial.println("");   
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


enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

struct Button {
  uint8_t pin;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long press_ts;
  boolean doublepress;
};
Button button1 = {0, 0, KP_NONE, 0, false};

void IRAM_ATTR buttonISR() {
  if (digitalRead(button1.pin) == 0) { // Button down
    if (millis() - button1.press_ts < 500) {
      // Double press
      button1.doublepress = true;
    } else {
      button1.doublepress = false;
    }
    button1.press_ts = millis();
  } else { //Button up
    unsigned int elapsed = millis() - button1.press_ts;
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
    button1.press_ts = millis();
  }
}

int getKeyPress() {
  KeyPress p = button1.pressed;
  button1.pressed = KP_NONE;
  return p;
}
int hasKeyPress() {
  return button1.pressed;
}

void setup()
{
  char buf[12];
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  for(int i=0; i<39; i++) {
    int v = gpio_get_level((gpio_num_t)i);
    Serial.printf("%d:%d ",i,v);
  }
  Serial.println("");
  pinMode(LORA_LED, OUTPUT);

  aprs_gencrctab();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  setupConfigData();    // configuration must be read first due to OLED ports!!!
  LORA_LED = sonde.config.led_pout;

  u8x8 = new U8X8_SSD1306_128X64_NONAME_SW_I2C(/* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda, /* reset=*/ sonde.config.oled_rst); // Unbuffered, basic graphics, software I2C
  u8x8->begin();
  delay(100);

  u8x8->clear();

  u8x8->setFont(u8x8_font_7x14_1x2_r);
  u8x8->drawString(8 - strlen(version_name) / 2, 1, version_name);
  u8x8->drawString(8 - strlen(version_id) / 2, 3, version_id);
  u8x8->setFont(u8x8_font_chroma48medium8_r);
  u8x8->drawString(0, 5, "by Hansi, DL9RDZ");
  u8x8->drawString(1, 6, "Mods by DL2MF");
  delay(3000);

  sonde.clearDisplay();

  setupWifiList();
  button1.pin = sonde.config.button_pin;
  pinMode(button1.pin, INPUT);

  // == show initial values from config.txt ========================= //
  if (sonde.config.debug == 1) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->drawString(0, 0, "Config:");

    delay(500);
    itoa(sonde.config.oled_sda, buf, 10);
    u8x8->drawString(0, 1, " SDA:");
    u8x8->drawString(6, 1, buf);

    delay(500);
    itoa(sonde.config.oled_scl, buf, 10);
    u8x8->drawString(0, 2, " SCL:");
    u8x8->drawString(6, 2, buf);

    delay(500);
    itoa(sonde.config.oled_rst, buf, 10);
    u8x8->drawString(0, 3, " RST:");
    u8x8->drawString(6, 3, buf);

    delay(1000);
    itoa(sonde.config.led_pout, buf, 10);
    u8x8->drawString(0, 4, " LED:");
    u8x8->drawString(6, 4, buf);

    delay(500);
    itoa(sonde.config.spectrum, buf, 10);
    u8x8->drawString(0, 5, " SPEC:");
    u8x8->drawString(6, 5, buf);

    delay(500);
    itoa(sonde.config.maxsonde, buf, 10);
    u8x8->drawString(0, 6, " MAX:");
    u8x8->drawString(6, 6, buf);

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

  // Handle button press
  attachInterrupt(button1.pin, buttonISR, CHANGE);

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

  sonde.setup();

  WiFi.onEvent(WiFiEvent);
}

void enterMode(int mode) {
  mainState = (MainState)mode;
  if(mainState == ST_SPECTRUM) {
    sonde.clearDisplay();
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    specTimer = millis(); 
  }
  sonde.clearDisplay();
}

void loopDecoder() {
  switch (getKeyPress()) {
    case KP_SHORT:
      sonde.nextConfig();
      sonde.updateDisplayRXConfig();
      sonde.updateDisplay();
      break;
    case KP_DOUBLE:
      enterMode(ST_SCANNER);
      return;
    case KP_MID:
      enterMode(ST_SPECTRUM);
      return;
    case KP_LONG:
      enterMode(ST_WIFISCAN);
      return;
  }
  // sonde knows the current type and frequency, and delegates to the right decoder
  int res = sonde.receiveFrame();

  if (res == 0 && connected) {
    //Send a packet with position information
    // first check if ID and position lat+lonis ok
    if (sonde.si()->validID && (sonde.si()->validPos & 0x03 == 0x03)) {
      Serial.println("Sending position via UDP");
      SondeInfo *s = sonde.si();
      char raw[201];
      const char *str = aprs_senddata(s->lat, s->lon, s->alt, s->hs, s->dir, s->vs, sondeTypeStr[s->type], s->id, "TE0ST",
                                      sonde.config.udpfeed.symbol);
      int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
      Serial.print("Sending: "); Serial.println(raw);
      udp.beginPacket(sonde.config.udpfeed.host, sonde.config.udpfeed.port);
      udp.write((const uint8_t *)raw, rawlen);
      udp.endPacket();
    }
  }
  sonde.updateDisplay();
}

#define SCAN_MAXTRIES 1
void loopScanner() {
  sonde.updateDisplayScanner();
  static int tries = 0;
  switch (getKeyPress()) {
    case KP_SHORT:
      enterMode(ST_DECODER);
      return;
    case KP_DOUBLE: break; /* ignored */
    case KP_MID:
      enterMode(ST_SPECTRUM);
      return;
    case KP_LONG:
      enterMode(ST_WIFISCAN);
      return;
  }
  // receiveFrame returns 0 on success, 1 on timeout
  int res = sonde.receiveFrame();   // Maybe instead of receiveFrame, just detect if right type is present? TODO
  Serial.print("Scanner: receiveFrame returned: ");
  Serial.println(res);
  if (res == 0) {
    enterMode(ST_DECODER);
    return;
  }
  if (++tries >= SCAN_MAXTRIES && !hasKeyPress()) {
    sonde.nextConfig();
    tries = 0;
  }
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
      enterMode(ST_WIFISCAN);
      return;
    case KP_DOUBLE: /* ignore */ break;
    default: break;
  }

  scanner.scan();
  scanner.plotResult();
  if (sonde.config.marker != 0) {
    itoa((sonde.config.startfreq), buf, 10);
    u8x8->drawString(0, 1, buf);
    u8x8->drawString(7, 1, "MHz");
    itoa((sonde.config.startfreq + 6), buf, 10);
    u8x8->drawString(13, 1, buf);
  }
  if (sonde.config.timer) {
    int remaining = sonde.config.spectrum - (millis() - specTimer)/1000;
    itoa(remaining, buf, 10);
    Serial.printf("timer:%d config.spectrum:%d  specTimer:%ld millis:%ld remaining:%d\n",sonde.config.timer, sonde.config.spectrum, specTimer, millis(), remaining);
    if (sonde.config.marker != 0) {
      marker = 1;
    }
    u8x8->drawString(0, 1 + marker, buf);
    u8x8->drawString(2, 1 + marker, "Sec.");
    if (remaining <= 0) {
      enterMode(ST_SCANNER);
    }
  }
}

void startSpectrumDisplay() {
  sonde.clearDisplay();
  u8x8->setFont(u8x8_font_chroma48medium8_r);
  u8x8->drawString(0, 0, "Spectrum Scan...");
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
  }
}

void enableNetwork(bool enable) {
  if (enable) {
    MDNS.begin("rdzsonde");
    SetupAsyncServer();
    udp.begin(WiFi.localIP(), LOCALUDPPORT);
    MDNS.addService("http", "tcp", 80);
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
  }
}


int wifiConnect(int16_t res) {
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
  Serial.printf("WifiBackground: state %d\n", wifi_state);
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
  IPAddress myIP = WiFi.softAPIP();
  String myIPstr = myIP.toString();
  sonde.setIP(myIPstr.c_str(), true);
  sonde.updateDisplayIP();
  SetupAsyncServer();
}

void initialMode() {
  if (sonde.config.spectrum != 0) {    // enable Spectrum in config.txt: spectrum=number_of_seconds
    startSpectrumDisplay();
    //done in startSpectrumScan(): enterMode(ST_SPECTRUM);
  } else {
    enterMode(ST_SCANNER);
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
  u8x8->setFont(u8x8_font_chroma48medium8_r);
  u8x8->drawString(0, 0, "WiFi Scan...");

  int line = 0;
  int cnt = 0;
  int marker = 0;
  char buf[5];

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  int index = -1;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.print("Network name: ");
    String ssid = WiFi.SSID(i);
    Serial.println(ssid);
    u8x8->drawString(0, 1 + line, ssid.c_str());
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
    
    u8x8->drawString(0, 6, "Conn:");
    u8x8->drawString(6, 6, fetchWifiSSID(index));
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
      u8x8->drawString(15, 7, _scan[cnt & 1]);
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
    u8x8->drawString(0, 6, "AP:             ");
    u8x8->drawString(6, 6, networks[0].id.c_str());
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
    delay(3000);
  }
  SetupAsyncServer();
  initialMode();

  if (sonde.config.spectrum != 0) {     // enable Spectrum in config.txt: spectrum=number_of_seconds
    //startSpectrumDisplay();
    enterMode(ST_SPECTRUM);
  } else {
    enterMode(ST_SCANNER);
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
  u8x8->setFont(u8x8_font_chroma48medium8_r);
  u8x8->drawString(0, 0, "C:");
  String dispHost = updateHost.substring(0,14);
  u8x8->drawString(2, 0, dispHost.c_str());
  
  Serial.println("Connecting to: " + updateHost);
  // Connect to Update host
  if (client.connect(updateHost.c_str(), updatePort)) {
    // Connection succeeded, fecthing the bin
    Serial.println("Fetching bin: " + String(*updateBin));
    u8x8->drawString(0, 1, "Fetching update");

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
  u8x8->drawString(0, 2, "Len: ");
  String cls = String(contentLength);
  u8x8->drawString(5, 2, cls.c_str());

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);
    u8x8->drawString(0, 4, "Starting update");

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      u8x8->drawString(0, 5, "Please wait!");
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
          u8x8->drawString(0, 7, "Rebooting....");
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
  Serial.print("Running main loop. free heap:");
  Serial.println(ESP.getFreeHeap());
  switch (mainState) {
    case ST_DECODER: loopDecoder(); break;
    case ST_SCANNER: loopScanner(); break;
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_WIFISCAN: loopWifiScan(); break;
    case ST_UPDATE: execOTA(); break;
  }
#if 1
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);

  int gain = sx1278.getLNAGain();
  Serial.print(" LNA Gain: "),
               Serial.println(gain);
#endif
  loopWifiBackground();
}

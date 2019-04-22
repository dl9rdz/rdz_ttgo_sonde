#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <U8x8lib.h>
#include <U8g2lib.h>
#include <SPI.h>

#include <SX1278FSK.h>
#include <Sonde.h>
#include <Scanner.h>
#include <aprs.h>

#define LORA_LED  9

// UNCOMMENT one of the constructor lines below
U8X8_SSD1306_128X64_NONAME_SW_I2C *u8x8=NULL;  // initialize later after reading config file
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

int e;

AsyncWebServer server(80);

#define LOCALUDPPORT 9002

boolean connected = false;
WiFiUDP udp;

// Set LED GPIO
int ledPin = 1;
// Stores LED state
String ledState;

// Replaces placeholder with LED state value
String processor(const String& var){
  Serial.println(var);
  if(var == "STATE"){
    if(digitalRead(ledPin)){
      ledState = "ON";
    }
    else{
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  return String();
}

#define MAX_QRG 10

const String sondeTypeSelect(int activeType) {
  String sts = "";
  for(int i=0; i<3; i++) {
     sts += "<option value=\"";
     sts += sondeTypeStr[i];
     sts += "\"";
     if(activeType==i) { sts += " selected"; }
     sts += ">";
     sts += sondeTypeStr[i];
     sts += "</option>";
  }
  return sts;
}


//trying to work around
//"assertion "heap != NULL && "free() target pointer is outside heap areas"" failed:"
// which happens if request->send is called in createQRGForm!?!??
char message[10240];

///////////////////////// Functions for Reading / Writing QRG list from/to qrg.txt

void setupChannelList() {
  File file = SPIFFS.open("/qrg.txt", "r");
  if(!file) {
    Serial.println("There was an error opening the file '/qrg.txt' for reading");
    return;
  }
  int i=0;
  sonde.clearSonde();
  Serial.println("Reading channel config:");  
  while(file.available()) {
    String line = file.readStringUntil('\n');
    if(!file.available()) break;
    if(line[0] == '#') continue;
    char *space = strchr(line.c_str(), ' ');
    if(!space) continue;
    *space = 0;
    float freq = atof(line.c_str());
    SondeType type;
    if(space[1]=='4') { type=STYPE_RS41; }
    else if (space[1]=='9') { type=STYPE_DFM09; }
    else if (space[1]=='6') { type=STYPE_DFM06; }
    else continue;
    int active = space[3]=='+'?1:0;
    char *launchsite = strchr(line.c_str(), ' ');
    Serial.printf("Add %f - type %d (on/off: %d)- Site: \n",freq,type,active,launchsite);
    sonde.addSonde(freq, type, active, launchsite);
    i++;
  }
}

const char *createQRGForm() {
  char *ptr = message;
  strcpy(ptr,"<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"qrg.html\" method=\"post\"><table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Mode</th></tr>");
   for(int i=0; i<sonde.config.maxsonde; i++) {
    String s = sondeTypeSelect(i>=sonde.nSonde?2:sonde.sondeList[i].type);
     sprintf(ptr+strlen(ptr), "<tr><td>%d</td><td><input name=\"A%d\" type=\"checkbox\" %s/></td>"
        "<td><input name=\"F%d\" type=\"text\" value=\"%3.3f\"></td>"       
        "<td><select name=\"T%d\">%s</select></td>",
        i+1,
        i+1, (i<sonde.nSonde&&sonde.sondeList[i].active)?"checked":"",
        i+1, i>=sonde.nSonde?400.000:sonde.sondeList[i].freq,
        i+1, s.c_str());
   }
   strcat(ptr,"</table><input type=\"submit\" value=\"Update\"/></form></body></html>");
   return message;
}

const char *handleQRGPost(AsyncWebServerRequest *request) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File f = SPIFFS.open("/qrg.txt", "w");
  if(!f) {
    Serial.println("Error while opening '/qrg.txt' for writing");
    return "Error while opening '/qrg.txt' for writing";
  }
#endif
  Serial.println("Handling post request");
#if 0
  int params = request->params();
  for(int i=0; i<params; i++) {
    Serial.println(request->getParam(i)->name().c_str());
  }
#endif
  for(int i=1; i<=MAX_QRG; i++) {  
    snprintf(label, 10, "A%d", i);
    AsyncWebParameter *active = request->getParam(label, true);
    snprintf(label, 10, "F%d", i);
    AsyncWebParameter *freq = request->getParam(label, true);
    if(!freq) continue;
    snprintf(label, 10, "T%d", i);
    AsyncWebParameter *type = request->getParam(label, true);
    if(!type) continue;
    const char *fstr = freq->value().c_str();
    const char *tstr = type->value().c_str();
    Serial.printf("Processing a=%s, f=%s, t=%s\n", active?"YES":"NO", fstr, tstr);
    char typech = (tstr[2]=='4'?'4':tstr[3]);  // Ugly TODO 
    f.printf("%3.3f %c %c\n", atof(fstr), typech, active?'+':'-');
  }
  f.close();
  Serial.println("Channel setup finished");
  Serial.println();
  
  setupChannelList();
}


/////////////////// Functions for reading/writing Wifi networks from networks.txt

#define MAX_WIFI 10
int nNetworks;
struct { String id; String pw; } networks[MAX_WIFI];

// FIXME: For now, we don't uspport wifi networks that contain newline or null characters
// ... would require a more sophisicated file format (currently one line SSID; one line Password
void setupWifiList() {
  File file = SPIFFS.open("/networks.txt", "r");
  if(!file){
    Serial.println("There was an error opening the file '/networks.txt' for reading");
    return;
  }
  int i=0;
  
  while(file.available()) {
    String line = file.readStringUntil('\n');
    if(!file.available()) break;
     networks[i].id = line;
     networks[i].pw = file.readStringUntil('\n');
     i++;
  }
  nNetworks = i;
  Serial.print(i); Serial.println(" networks in networks.txt\n");
  for(int j=0; j<i; j++) { Serial.print(networks[j].id); Serial.print(": "); Serial.println(networks[j].pw); }
}


const char *createWIFIForm() {
  char *ptr = message;
  char tmp[4];
  strcpy(ptr,"<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"wifi.html\" method=\"post\"><table><tr><th>Nr</th><th>SSID</th><th>Password</th></tr>");
  for(int i=0; i<MAX_WIFI; i++) {
     sprintf(tmp,"%d",i);
     sprintf(ptr+strlen(ptr), "<tr><td>%s</td><td><input name=\"S%d\" type=\"text\" value=\"%s\"/></td>"
        "<td><input name=\"P%d\" type=\"text\" value=\"%s\"/></td>",   
        i==0?"<b>AP</b>":tmp,
        i+1, i<nNetworks?networks[i].id.c_str():"",
        i+1, i<nNetworks?networks[i].pw.c_str():"");
   }
   strcat(ptr,"</table><input type=\"submit\" value=\"Update\"></input></form></body></html>");
   return message;
}

const char *handleWIFIPost(AsyncWebServerRequest *request) {
  char label[10];
  // parameters: a_i, f_1, t_i  (active/frequency/type)
#if 1
  File f = SPIFFS.open("/networks.txt", "w");
  if(!f) {
    Serial.println("Error while opening '/networks.txt' for writing");
    return "Error while opening '/networks.txt' for writing";
  }
#endif
  Serial.println("Handling post request");
#if 0
  int params = request->params();
  for(int i=0; i<params; i++) {
    Serial.println(request->getParam(i)->name().c_str());
  }
#endif
  for(int i=1; i<=MAX_WIFI; i++) {  
    snprintf(label, 10, "S%d", i);
    AsyncWebParameter *ssid = request->getParam(label, true);
    if(!ssid) continue;
    snprintf(label, 10, "P%d", i);
    AsyncWebParameter *pw = request->getParam(label, true);
    if(!pw) continue;
    const char *sstr = ssid->value().c_str();
    const char *pstr = pw->value().c_str();
    if(strlen(sstr)==0) continue;
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
  sprintf(ptr+strlen(ptr),"<tr><td id=\"sfreq\">%3.3f MHz, Type: %s</td><tr><td>ID: %s</td></tr><tr><td>QTH: %.6f,%.6f h=%.0fm</td></tr>\n",
    s->freq, sondeTypeStr[s->type],
    s->validID?s->id:"<??>",
    s->lat, s->lon, s->alt);
  sprintf(ptr+strlen(ptr), "<tr><td><a target=\"_empty\" href=\"geo:%.6f,%.6f\">GEO-App</a> - ", s->lat, s->lon);
  sprintf(ptr+strlen(ptr), "<a target=\"_empty\" href=\"https://wx.dl2mf.de/?%s\">WX.DL2MF.de</a> - ", s->id);
  sprintf(ptr+strlen(ptr), "<a target=\"_empty\" href=\"https://www.openstreetmap.org/?mlat=%.6f&mlon=%.6f&zoom=14\">OSM</a></td></tr>", s->lat, s->lon);
  strcat(ptr, "</table><p/>\n");
}

const char *createStatusForm() {
  char *ptr = message;
  strcpy(ptr,"<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"><meta http-equiv=\"refresh\" content=\"5\"></head><body>");
  
  for(int i=0; i<sonde.nSonde; i++) {
    addSondeStatus(ptr, (i+sonde.currentSonde)%sonde.nSonde);
  }
  strcat(ptr,"</body></html>");
  return message;
}

///////////////////// Config form


void setupConfigData() {
  File file = SPIFFS.open("/config.txt", "r");
  if(!file) {
    Serial.println("There was an error opening the file '/config.txt' for reading");
    return;
  }
  while(file.available()) {
    String line = file.readStringUntil('\n');
    sonde.setConfig(line.c_str());
  }
}


struct st_configitems {
  const char *label;
  int type;  // 0: numeric; i>0 string of length i; -1: separator; -2: type selector
  void *data;
}; 
#define N_CONFIG 20
struct st_configitems config_list[N_CONFIG] = {
  {"ShowSpectrum (s)", 0, &sonde.config.spectrum},
  {"Startfreq (MHz)", 0, &sonde.config.startfreq},
  {"Bandwidth (kHz)", 0, &sonde.config.channelbw},  
  {"---", -1, NULL},
  {"Call", 8, sonde.config.call},
  {"Passcode", 8, sonde.config.passcode},
  {"---", -1, NULL},
  {"AXUDP active", -3, &sonde.config.udpfeed.active},
  {"AXUDP Host", 63, sonde.config.udpfeed.host},
  {"AXUDP Port", 0, &sonde.config.udpfeed.port},
  {"DFM ID Format", -2, &sonde.config.udpfeed.idformat},
  {"Rate limit", 0, &sonde.config.udpfeed.highrate},
  {"---", -1, NULL},
  {"APRS TCP active", -3, &sonde.config.tcpfeed.active},
  {"ARPS TCP Host", 63, sonde.config.tcpfeed.host},
  {"APRS TCP Port", 0, &sonde.config.tcpfeed.port},
  {"DFM ID Format", -2, &sonde.config.tcpfeed.idformat},
  {"Rate limit", 0, &sonde.config.tcpfeed.highrate},
  {"---", -1, NULL},
  {"Spectrum noise floor", 0, &sonde.config.noisefloor}
};

void addConfigStringEntry(char *ptr, int idx, const char *label, int len, char *field) {
   sprintf(ptr+strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" value=\"%s\"/></td></tr>\n",
      label, idx, field);
}
void addConfigNumEntry(char *ptr, int idx, const char *label, int *value) {
   sprintf(ptr+strlen(ptr), "<tr><td>%s</td><td><input name=\"CFG%d\" type=\"text\" value=\"%d\"/></td></tr>\n",
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
  strcpy(ptr,"<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"config.html\" method=\"post\"><table><tr><th>Option</th><th>Value</th></tr>");
  for(int i=0; i<N_CONFIG; i++) {
    switch(config_list[i].type) {
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
  strcat(ptr,"</table><input type=\"submit\" value=\"Update not yet implemented\"></input></form></body></html>");
  return message;
}



const char* PARAM_MESSAGE = "message";
void SetupAsyncServer() {
// Route for root / web page
 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
    });
    
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/test.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/test.html", String(), false, processor);
  });

  server.on("/qrg.html", HTTP_GET,  [](AsyncWebServerRequest *request){
     request->send(200, "text/html", createQRGForm());
  });
  server.on("/qrg.html", HTTP_POST, [](AsyncWebServerRequest *request){
    handleQRGPost(request);
    request->send(200, "text/html", createQRGForm());
  });

  server.on("/wifi.html", HTTP_GET,  [](AsyncWebServerRequest *request){
     request->send(200, "text/html", createWIFIForm());
  });
  server.on("/wifi.html", HTTP_POST, [](AsyncWebServerRequest *request){
    handleWIFIPost(request);
    request->send(200, "text/html", createWIFIForm());
  });

  server.on("/config.html", HTTP_GET,  [](AsyncWebServerRequest *request){
     request->send(200, "text/html", createConfigForm());
  });  

  server.on("/status.html", HTTP_GET,  [](AsyncWebServerRequest *request){
     request->send(200, "text/html", createStatusForm());
  });
  
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

   // Route to set GPIO to HIGH
  server.on("/test.php", HTTP_POST, [](AsyncWebServerRequest *request){
    //digitalWrite(ledPin, HIGH);    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Start server
  server.begin();
}


const char *fetchWifiPw(const char *id) {
  for(int i=0; i<nNetworks; i++) {
    //Serial.print("Comparing '");
    //Serial.print(id);
    //Serial.print("' and '");
    //Serial.print(networks[i].id.c_str());
    //Serial.println("'");
    if(strcmp(id,networks[i].id.c_str())==0) return networks[i].pw.c_str();
  }
  return NULL;
}
  

enum KeyPress { KP_NONE=0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

struct Button {
  uint8_t pin;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long press_ts;
  boolean doublepress;
};
Button button1 = {0, 0, KP_NONE, 0, false};

void IRAM_ATTR buttonISR() {
  if(digitalRead(button1.pin)==0) { // Button down
    if(millis()-button1.press_ts<500) {
      // Double press
      button1.doublepress = true;
    } else {
      button1.doublepress = false;
    }
    button1.press_ts = millis();
  } else { //Button up
    unsigned int elapsed = millis()-button1.press_ts;
    if(elapsed>1500) {
      if(elapsed<4000) { button1.pressed=KP_MID; }
      else { button1.pressed=KP_LONG; }
    } else {
      if(button1.doublepress) button1.pressed=KP_DOUBLE;
      else button1.pressed=KP_SHORT;
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
  pinMode(LORA_LED, OUTPUT);
  
  aprs_gencrctab();

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  setupConfigData();    // configuration must be read first due to OLED ports!!!  

  u8x8 = new U8X8_SSD1306_128X64_NONAME_SW_I2C(/* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda, /* reset=*/ sonde.config.oled_rst); // Unbuffered, basic graphics, software I2C
  u8x8->begin();
  delay(100);

  u8x8->clear();

  u8x8->setFont(u8x8_font_7x14_1x2_r);
  u8x8->drawString(1, 1, "RDZ_TTGO_SONDE"); 
  u8x8->drawString(2, 3, "    V0.1e");   
  u8x8->drawString(1, 5, "Mods by DL2MF");     
  delay(3000);
 
  sonde.clearDisplay();
  
  setupWifiList();
  button1.pin = sonde.config.button_pin;

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
  itoa(sonde.config.led_pin, buf, 10);
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
  if(rs41.setFrequency(402700000)==0) {
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
}

enum MainState { ST_DECODER, ST_SCANNER, ST_SPECTRUM, ST_WIFISCAN };

static MainState mainState = ST_WIFISCAN;

void enterMode(int mode) {
  mainState = (MainState)mode;
  sonde.clearDisplay();
}

void loopDecoder() {
  switch(getKeyPress()) {
    case KP_SHORT:
      sonde.nextConfig();
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

  if(res==0 && connected){
    //Send a packet with position information
    // first check if ID and position lat+lonis ok
    if(sonde.si()->validID && (sonde.si()->validPos&0x03==0x03)) {
      Serial.println("Sending position via UDP");
      SondeInfo *s = sonde.si();
      char raw[201];
      const char *str = aprs_senddata(s->lat, s->lon, s->alt, s->hs, s->dir, s->vs, sondeTypeStr[s->type], s->id, "TE0ST", 
        sonde.config.udpfeed.symbol);
      int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
      Serial.print("Sending: "); Serial.println(raw);
      udp.beginPacket(sonde.config.udpfeed.host,sonde.config.udpfeed.port);
      udp.write((const uint8_t *)raw,rawlen);
      udp.endPacket();
    }
  }  
  sonde.updateDisplay();
}

#define SCAN_MAXTRIES 1
void loopScanner() {
  sonde.updateDisplayScanner();
  static int tries=0;
  switch(getKeyPress()) {
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
  if(res==0) {
      enterMode(ST_DECODER);
      return;
  }
  if(++tries>=SCAN_MAXTRIES && !hasKeyPress()) {
    sonde.nextConfig();
    tries = 0;
  }
}

void loopSpectrum() {
  switch(getKeyPress()) {
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

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),LOCALUDPPORT);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}

static char* _scan[2]={"/","\\"};
void loopWifiScan() {
  u8x8->setFont(u8x8_font_chroma48medium8_r);
  if (sonde.config.wifi != 0) {
    u8x8->drawString(0,0,"WiFi Scan...");
  }
  else if (sonde.config.wifiap != 0) {
    u8x8->drawString(0,0,"WiFi AP-Mode:");
  }
  
  int line=0;
  int cnt=0;
  int marker=0;
  char buf[5];

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  const char *id, *pw;
  char idstr[64]="test";

  if (sonde.config.wifi != 0) {
    int n = WiFi.scanNetworks();
     for (int i = 0; i < n; i++) {
        Serial.print("Network name: ");
        Serial.println(WiFi.SSID(i));
        u8x8->drawString(0,1+line,WiFi.SSID(i).c_str());
        line = (line+1)%5;
        Serial.print("Signal strength: ");
        Serial.println(WiFi.RSSI(i));
        Serial.print("MAC address: ");
        Serial.println(WiFi.BSSIDstr(i));
        Serial.print("Encryption type: ");
        String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
        Serial.println(encryptionTypeDescription);
        Serial.println("-----------------------");
        id=WiFi.SSID(i).c_str();
        pw=fetchWifiPw(id);
       if(pw) { strncpy(idstr, id, 63); }
      }
      if(!pw) { pw="test"; }
      Serial.print("Connecting to: "); Serial.println(idstr);
      u8x8->drawString(0,6, "Conn:");
      u8x8->drawString(6,6, idstr);
      //register event handler
      WiFi.onEvent(WiFiEvent);
  
      WiFi.begin(idstr, pw);
  }
  
  while(WiFi.status() != WL_CONNECTED)  {
        delay(500);
        Serial.print(".");
        u8x8->drawString(15,7,_scan[cnt&1]);
        cnt++;
        #if 0
        if(cnt==4) {
            WiFi.disconnect(true);  // retry, for my buggy FritzBox
            WiFi.onEvent(WiFiEvent);
            WiFi.begin(idstr, pw);
        }
        #endif
        if(cnt==15) {
            WiFi.disconnect(true);
            
            if (sonde.config.wifiap != 0) {         // enable WiFi AP mode in config.txt: wifi=1
              delay(1000);
              WiFi.softAP(networks[0].id.c_str(),networks[0].pw.c_str());
              IPAddress myIP = WiFi.softAPIP();
              Serial.print("AP IP address: ");
              Serial.println(myIP);
              u8x8->drawString(0,6, "AP:             ");
              u8x8->drawString(6,6, networks[0].id.c_str());
              sonde.setIP(myIP.toString().c_str(), true);
              sonde.updateDisplayIP();
              SetupAsyncServer();
              delay(3000);
            }

            if (sonde.config.spectrum != 0) {     // enable Spectrum in config.txt: spectrum=number_of_seconds
              sonde.clearDisplay();
              u8x8->setFont(u8x8_font_chroma48medium8_r);
              u8x8->drawString(0, 0, "Spectrum Scan...");
              delay(500);
            
              enterMode(ST_SPECTRUM);
             
              for (int i = 0; i < sonde.config.spectrum; i++) {                         
                scanner.scan();
                scanner.plotResult();

                if (sonde.config.marker != 0) {
                  itoa((sonde.config.startfreq), buf, 10);
                  u8x8->drawString(0, 1, buf);
                  u8x8->drawString(7, 1, "MHz");                 
                  itoa((sonde.config.startfreq + 6), buf, 10);                  
                  u8x8->drawString(13, 1, buf);
                }                

                if (sonde.config.timer != 0) {
                  itoa((sonde.config.spectrum - i), buf, 10);
				  if (sonde.config.marker != 0) {
				   marker = 1;
				  }
					u8x8->drawString(0, 1+marker, buf);
					u8x8->drawString(2, 1+marker, "Sec.");
                }                          
              }

              delay(1000);
            }           

            enterMode(ST_SCANNER);
            return;
        }

  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  sonde.setIP(WiFi.localIP().toString().c_str(), false);
  sonde.updateDisplayIP();
  SetupAsyncServer();
  delay(2000);

  // enterMode(ST_DECODER);     ### 2019-04-20 - changed DL2MF
  enterMode(ST_SCANNER);  
}

void loop() {
  Serial.println("Running main loop");
  switch(mainState) {
    case ST_DECODER: loopDecoder(); break;
    case ST_SCANNER: loopScanner(); break;
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_WIFISCAN: loopWifiScan(); break;
  }
#if 1                  
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);
  
  int gain = sx1278.getLNAGain();
  Serial.print(" LNA Gain: "),
  Serial.println(gain);
#endif
}

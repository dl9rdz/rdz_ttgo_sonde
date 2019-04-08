#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <U8x8lib.h>
#include <SPI.h>

#include <SX1278FSK.h>
#include <Sonde.h>
#include <Scanner.h>
//#include <RS41.h>
//#include <DFM.h>

#define LORA_LED  9

// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

// UNCOMMENT one of the constructor lines below
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

int e;

AsyncWebServer server(80);

// Set LED GPIO
const int ledPin = 2;
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

void SetupAsyncServer() {
// Route for root / web page
 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });
    
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to set GPIO to LOW
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, LOW);    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Start server
  server.begin();
}

int nNetworks;
struct { String id; String pw; } networks[20];

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

void setupChannelList() {
  File file = SPIFFS.open("/qrg.txt", "r");
  if(!file) {
    Serial.println("There was an error opening the file '/qrg.txt' for reading");
    return;
  }
  int i=0;
  sonde.clearSonde();
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
    Serial.printf("Adding %f with type %d\b",freq,type);
    sonde.addSonde(freq, type);
    i++;
  }
  nNetworks = i;
  Serial.print(i); Serial.println(" networks in networks.txt\n");
  for(int j=0; j<i; j++) { Serial.print(networks[j].id); Serial.print(": "); Serial.println(networks[j].pw); }

}

const char *fetchWifiPw(const char *id) {
  for(int i=0; i<nNetworks; i++) {
    Serial.print("Comparing '");
    Serial.print(id);
    Serial.print("' and '");
    Serial.print(networks[i].id.c_str());
    Serial.println("'");
    if(strcmp(id,networks[i].id.c_str())==0) return networks[i].pw.c_str();
  }
  return NULL;
}
  

enum KeyPress { KP_NONE=0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long press_ts;
  boolean doublepress;
};
Button button1 = {0, 0, KP_NONE, 0, false};

void IRAM_ATTR buttonISR() {
  if(digitalRead(0)==0) { // Button down
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
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  
  u8x8.begin();

  pinMode(LORA_LED, OUTPUT);

    // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  setupWifiList();

#if 0 
  if(rs41.setFrequency(402700000)==0) {
    Serial.println(F("Setting freq: SUCCESS "));
  } else {
    Serial.println(F("Setting freq: ERROR "));
  }
  float f = sx1278.getFrequency();
  Serial.print("Frequency set to ");
  Serial.println(f);
#endif

  //sx1278.setLNAGain(-48);
  sx1278.setLNAGain(0);
  int gain = sx1278.getLNAGain();
  Serial.print("RX LNA Gain is ");
  Serial.println(gain);

  // Print a success message
  Serial.println(F("sx1278 configured finished"));
  Serial.println();


  Serial.println("Setup finished");
  // int returnValue = pthread_create(&wifithread, NULL, wifiloop, (void *)0);
 
  //  if (returnValue) {
  //     Serial.println("An error has occurred");
  //  }
  //   xTaskCreate(mainloop, "MainServer", 10240, NULL, 10, NULL);

  // Handle button press
  attachInterrupt(0, buttonISR, CHANGE);

  setupChannelList();
 #if 0
  sonde.clearSonde();
  sonde.addSonde(402.300, STYPE_RS41);
  sonde.addSonde(402.700, STYPE_RS41);
  sonde.addSonde(403.450, STYPE_DFM09);
 #endif
  /// not here, done by sonde.setup(): rs41.setup();
  sonde.setup();
}

enum MainState { ST_DECODER, ST_SCANNER, ST_SPECTRUM, ST_WIFISCAN };

static MainState mainState = ST_DECODER;

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
  sonde.receiveFrame();
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
  Serial.print("Scanner: receiveFrame returned");
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

static char* _scan[2]={"/","\\"};
void loopWifiScan() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0,0,"WiFi Scan...");
  int line=0;
  int cnt=0;

  WiFi.mode(WIFI_STA);
  const char *id, *pw;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
    u8x8.drawString(0,1+line,WiFi.SSID(i).c_str());
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
    if(pw) break;
  }
  if(1||!pw) { id="test"; pw="test"; }
  Serial.print("Connecting to: "); Serial.println(id);
  u8x8.drawString(0,6, "Conn:");
  u8x8.drawString(6,6, id);
  WiFi.begin(id, pw);
  while(WiFi.status() != WL_CONNECTED)  {
        delay(500);
        Serial.print(".");
        u8x8.drawString(15,7,_scan[cnt&1]);
        cnt++;
        if(cnt==4) {
            WiFi.disconnect();  // retry, for my buggy FritzBox
            WiFi.begin(id, pw);
        }
        if(cnt==10) {
            WiFi.disconnect();
            delay(1000);
            WiFi.softAP("sonde","sondesonde");
            IPAddress myIP = WiFi.softAPIP();
            Serial.print("AP IP address: ");
            Serial.println(myIP);
            sonde.setIP(myIP.toString().c_str());
            sonde.updateDisplayIP();
            SetupAsyncServer();
            delay(5000);
            enterMode(ST_DECODER);
            return;
        }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  sonde.setIP(WiFi.localIP().toString().c_str());
  sonde.updateDisplayIP();
  SetupAsyncServer();
  delay(5000);
  enterMode(ST_DECODER);
}


void loop() {
  Serial.println("Running main loop");
  switch(mainState) {
    case ST_DECODER: loopDecoder(); break;
    case ST_SCANNER: loopScanner(); break;
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_WIFISCAN: loopWifiScan(); break;
  }
  //wifiloop(NULL);
  //e = dfm.receiveFrame();
#if 1                  
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);
  
  int gain = sx1278.getLNAGain();
  Serial.print(" LNA Gain: "),
  Serial.println(gain);
#endif
}

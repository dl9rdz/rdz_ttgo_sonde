#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#include <U8x8lib.h>
#include <Sonde.h>
#include <Scanner.h>


#include <RS41.h>
#include <SX1278FSK.h>
#include <rsc.h>

#include <SPI.h>

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
char my_packet[100];

//const char* ssid     = "DinoGast";
//const char* password = "Schokolade";
const char *ssid="AndroidDD";
const char *password="dl9rdzhr";

AsyncWebServer server(80);

//pthread_t wifithread;


int conn = 0;
String currentLine;
WiFiClient client;
unsigned long lastdu;


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
  
#if 0
void wifiloop(void *arg){
  lastdu=millis();
  while(true) {
    if(millis()-lastdu>500) {
  // This is too slow to do in main loop
  //u8x8.setFont(u8x8_font_chroma48medium8_r);
  //u8x8.clearDisplay();
    sonde.updateDisplay();
    lastdu=millis();
    }

  
    delay(1);
  if(!conn) {
    client = server.available();   // listen for incoming clients
    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      currentLine = "";                // make a String to hold incoming data from the client
      conn = 1;
    }
  } else {
    if(!client.connected()) {            // loop while the client's connected
       conn = 0;
       Serial.println("Client no longer connected");
       continue;
    }
    while (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            // close the connection:
            client.stop();
            Serial.println("Client Disconnected.");
            continue;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(5, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(5, LOW);                // GET /L turns the LED off
        }
      
    }
  }
  }
}
#endif

enum KeyPress { KP_NONE, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long press;
};
Button button1 = {0, 0, KP_NONE, 0};

void IRAM_ATTR buttonISR() {
  if(digitalRead(0)==0) { // Button down
    button1.press = millis();
  } else { //Button up
    unsigned int elapsed = millis()-button1.press;
    if(elapsed>1500) { if(elapsed<4000) { button1.pressed=KP_MID; } else { button1.pressed=KP_LONG; } }
    else { button1.pressed=KP_SHORT; }
    button1.numberKeyPresses += 1;
  }
}

int getKeyPress() {
  KeyPress p = button1.pressed;
  button1.pressed = KP_NONE;
  return p;
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

  rs41.setup();
  
  if(rs41.setFrequency(402700000)==0) {
    Serial.println(F("Setting freq: SUCCESS "));
  } else {
    Serial.println(F("Setting freq: ERROR "));
  }
  float f = sx1278.getFrequency();
  Serial.print("Frequency set to ");
  Serial.println(f);

  sx1278.setLNAGain(-48);
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
}

enum MainState { ST_DECODER, ST_SCANNER, ST_SPECTRUM, ST_WIFISCAN };

static MainState mainState = ST_SPECTRUM;

void loopDecoder() {
  
}

void loopScanner() {
  
}

void loopSpectrum() {
  switch(getKeyPress()) {
    case KP_SHORT: /* move selection of peak, TODO */ break;
    case KP_MID: /* restart, TODO */ break;
    case KP_LONG: mainState = ST_WIFISCAN; return;
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

void loopWifiScan() {
  WiFi.mode(WIFI_STA);
  const char *id, *pw;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
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
  if(!pw) { id="test"; pw="test"; }
  Serial.print("Connecting to: "); Serial.println(id);
  WiFi.begin(id, pw);
  while(WiFi.status() != WL_CONNECTED)  {
        delay(500);
        Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  SetupAsyncServer();
  delay(5000);
  mainState=ST_SPECTRUM;
}


void loop() {
  Serial.println("Running main loop");
  switch(mainState) {
    case ST_DECODER: loopDecoder(); break;
    case ST_SCANNER: loopScanner(); break;
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_WIFISCAN: loopWifiScan(); break;
  }
#if 0
  if (button1.pressed) {
      Serial.print( ((const char *[]){"SHORT PRESS -","LONG PRESS -","VERYLONG PRESS -"})[button1.pressed-1]);
      Serial.printf("Button 1 has been pressed %u times\n", button1.numberKeyPresses);
      button1.pressed = false;
  }
  
  //wifiloop(NULL);
  //e = dfm.receiveFrame();
  //e = rs41.receiveFrame();
  scanner.scan();
  scanner.plotResult();
  delay(1000);
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);
  
  int gain = sx1278.getLNAGain();
  Serial.print(" LNA Gain: "),
  Serial.println(gain);
  #endif
  
}

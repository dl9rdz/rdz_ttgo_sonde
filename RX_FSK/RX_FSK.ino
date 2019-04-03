#include <U8x8lib.h>
#include <Sonde.h>

#include <WiFi.h>

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

const char* ssid     = "DinoGast";
const char* password = "Schokolade";

WiFiServer server(80);

pthread_t wifithread;


int conn = 0;
String currentLine;
WiFiClient client;
unsigned long lastdu;

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

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  
  u8x8.begin();

  
  pinMode(LORA_LED, OUTPUT);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED)  {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

     server.begin();

     xTaskCreatePinnedToCore(wifiloop, "WifiServer", 10240, NULL, 10, NULL, 0);


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

}

void loop() {
  Serial.println("Running main loop");

  
  //wifiloop(NULL);
  //e = dfm.receiveFrame();
  e = rs41.receiveFrame();
  #if 0                       
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);
  
  int gain = sx1278.getLNAGain();
  Serial.print(" LNA Gain: "),
  Serial.println(gain);
  #endif
  
}

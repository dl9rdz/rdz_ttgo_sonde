#define TAG "Startup"
#include "logger.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8x8lib.h>

#include "Startup.h"
#include "Sonde.h"
#include "pmu.h"

extern int initlevels[40];
extern PMU *pmu;
extern void flashLed(int ms);

int fingerprintValue[]={ 17, 31, 64, 4, 55, 48, 23, 128+23, 119, 128+119, 95, 79, 91, -1 };
const char *fingerprintText[]={
  "TTGO T-Beam (new version 1.0),  I2C not working after powerup, assuming 0.9\" OLED@21,22",
  "TTGO LORA32 v2.1_1.6 (0.9\" OLED@21,22)",
  "TTGO LORA v1.0 (0.9\" OLED@4,15)",
  "Heltec v1/v2 (0.9\"OLED@4,15)",
  "TTGO T-Beam (V0.7), 0.9\" OLED@21,22",
  "TTGO T-Beam (V0.7), SPI TFT@4,21,22",
  "TTGO T-Beam (V1.0), 0.9\" OLED@21,22",
  "TTGO T-Beam (V1.0), SPI TFT@4,13,14",
  "TTGO T-Beam (V1.1), 0.9\" OLED@21,22",
  "TTGO T-Beam (V1.1), SPI TFT@4,13,14",
  "M5 Core Gray",
  "M5 Core Gray?",
  "CYB (USB-C) w/ SPI display + Ra02",
};

static void applyBoardDefaults(BoardTypes board) {
	switch(board) {
	case BOARD_M5_CORE_GRAY:
		LOG_I(TAG, "Autoconfig: M5stack Core Gray board detected\n");
		sonde.config.type = TYPE_M5_CORE;
		sonde.config.button_pin = 39;
		sonde.config.button2_pin = 38;
		sonde.config.button2_axp = 0;
		sonde.config.disptype = 4;  // ILI9342
		sonde.config.oled_sda = 23;
		sonde.config.oled_scl = 18;
		sonde.config.oled_rst = 33;
		sonde.config.tft_rs = 27;
		sonde.config.tft_cs = 14;
		sonde.config.power_pout = 32+128;
		// gpio32 is backlight of tft
		sonde.config.screenfile = 4;
		sonde.config.gps_rxd = 16;
		sonde.config.gps_txd = -1;  // 17
		sonde.config.sx1278_ss = 5;
		sonde.config.sx1278_miso = 19;
		sonde.config.sx1278_mosi = 23; //MOSI;
		sonde.config.sx1278_sck = 18; // SCK;
		break;
	case BOARD_TTGOv1_HELTEC:
		LOG_I(TAG, "Autoconfig: looks like TTGO v1 / Heltec v1/V2 board\n");
		sonde.config.oled_sda = 4;
		sonde.config.oled_scl = 15;
		sonde.config.oled_rst = 16;
		sonde.config.button_pin = 0;
		sonde.config.button2_pin = T4 + 128;	 // T4 == GPIO13
		sonde.config.power_pout = 21;		   // for Heltec v2
		sonde.config.led_pout = 2;	
		LOG_W(TAG, "WARNING: v1 boards with 26 MHz crystal are NOT supported any more!\n");
		// Note: binary image compiled for other boards (40 MHz crystal) will no longer work for v1 board (26 MHz crystal)
		// -- serial speed is wrong (staring arduino-idf 3.0.4), and WiFi also is partially broken.
		// So if using that old board, you need to recompile specifically for the 26 MHz.
		break;
	case BOARD_CYD_E32R28T:
	       {
		SPIClass *hspi = new SPIClass(HSPI);
		LOG_I(TAG, "Autoconfig: looks like a CYD (new) E32R28T");
		// SPI.begin(SCK, MISO, MOSI, SS);
		hspi->begin(14, 12, 13, -1);
	 	pinMode(2, OUTPUT);
		digitalWrite(15, 1);  // CS
		pinMode(15, OUTPUT);
		char buf[4];

		digitalWrite(2, 0); // cmd/data -> cmd
                hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
                digitalWrite(15, 0);  // CS
                hspi->transfer(0xD3); // cmd query identification (ID4)
                digitalWrite(2, 1);
		buf[0]=buf[1]=buf[2]=buf[3]=0;
                hspi->transfer(buf, 4);
                digitalWrite(15, 1);
                hspi->endTransaction();
		Serial.printf("Device ID4:  %x %x %x\n",  buf[1], buf[2], buf[3]);
		// ST7789 identifes as 00 00 00
		// ILI9341 should identify as 00 93 41 but also returns 00 00 00
		//
		digitalWrite(2, 0); // cmd/data -> cmd
  		hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		digitalWrite(15, 0);  // CS
		hspi->transfer(0x04); // cmd query identification (ID1-3)
		digitalWrite(2, 1);
		buf[0] = buf[1] = buf[2] = buf[3] = {0};
		hspi->transfer(buf, 4);
		digitalWrite(15, 1);
                hspi->endTransaction();
                hspi->end();
		Serial.printf("Device ID: %x %x %x\n", buf[1], buf[2], buf[3]);
		// my ST7789 identifies as c0 d9 ff
		// my ILI9341 identifies as 00 00 00
		
		sonde.config.disptype = buf[2]==0xd9? 5 : 3; // could be 3 or 5, both versions exist...
		sonde.config.oled_sda = 13;
		sonde.config.oled_scl = 14;
		sonde.config.oled_rst = -1;
		sonde.config.tft_rs = 2;
		sonde.config.tft_cs = 15;
		sonde.config.power_pout = 21 + 128;
		sonde.config.gps_rxd = 35;
		sonde.config.gps_txd = -1;
		sonde.config.sx1278_ss = 27;
		sonde.config.sx1278_miso = 19;
		sonde.config.sx1278_mosi= 23;
		sonde.config.sx1278_sck = 18;
		sonde.config.button_pin = 0;
		sonde.config.button2_pin = -1;
	       }
	       break;
	}
}


void startupBoardDetect() {
	sonde.fingerprint = initlevels[4];
	sonde.fingerprint = (sonde.fingerprint<<1) | initlevels[12];
	sonde.fingerprint = (sonde.fingerprint<<1) | initlevels[16];
	sonde.fingerprint = (sonde.fingerprint<<1) | initlevels[17];
	sonde.fingerprint = (sonde.fingerprint<<1) | initlevels[21];
	sonde.fingerprint = (sonde.fingerprint<<1) | initlevels[22];
	sonde.fingerprint = (sonde.fingerprint<<1) | initlevels[23];
	LOG_I(TAG, "Board fingerprint is %d\n", sonde.fingerprint);

	if(initlevels[16]==0) {
		// Plain M5 Core Gray sometimes ends up here (without Lora/GPS board attached, or after reboot)
		// 71 w/o GPS, 79 after reboot via flash utility
		if(sonde.fingerprint == 79 || sonde.fingerprint==71)      
			applyBoardDefaults(BOARD_M5_CORE_GRAY);
		else
			applyBoardDefaults(BOARD_TTGOv1_HELTEC);

	} else {
		sonde.config.oled_sda = 21;
		sonde.config.oled_scl = 22;
		if(initlevels[17]==0) { // T-Beam or M5Stack Core2?
			int tbeam=7;
			if(initlevels[12]==0) {
				tbeam = 10;
				LOG_I(TAG, "Autoconfig: looks like T-Beam 1.0 or M5Stack Core2 board\n");
			} else if ( initlevels[4]==1 && initlevels[12]==1 ) {
				tbeam = 11;
				LOG_I(TAG, "Autoconfig: looks like T-Beam 1.1 board\n");
			}
			if(tbeam == 10  || tbeam == 11) {  // T-Beam v1.0  or T-Beam v1.1
				Wire.begin(21, 22);
#define BM8563_ADDRESS 0x51
				Wire.beginTransmission(BM8563_ADDRESS);
				byte err = Wire.endTransmission();
				if(err) { // try again
				  delay(400);
				  Wire.beginTransmission(BM8563_ADDRESS);
				  err = Wire.endTransmission();
				}
				if(err==0) {
					LOG_I(TAG, "M5stack Core2 board detected\n");
					sonde.config.type = TYPE_M5_CORE2;
					sonde.config.button_pin = 255;
					sonde.config.button2_pin = 255;
					sonde.config.button2_axp = 1;
					sonde.config.disptype = 4;  // ILI9342
					sonde.config.oled_sda = 23;
					sonde.config.oled_scl = 18;
					sonde.config.oled_rst = -1;
					sonde.config.tft_rs = 15;
					sonde.config.tft_cs = 5;
					sonde.config.screenfile = 4;
					sonde.config.gps_rxd = 13;
					sonde.config.gps_txd = -1;  // 14
					sonde.config.sx1278_ss = 33;
					sonde.config.sx1278_miso = 38;
					sonde.config.sx1278_mosi = 23; //MOSI;
					sonde.config.sx1278_sck = 18; // SCK;
				} else { // some t-beam...
					sonde.config.button_pin = 38;
					sonde.config.button2_pin = 15 + 128; //T4 + 128;  // T4 = GPIO13
					// Maybe in future use as default only PWR as button2?
					//config.button2_pin = 255;
					sonde.config.button2_axp = 1;
					sonde.config.gps_rxd = 34;
					sonde.config.gps_txd = 12;
					// Check for I2C-Display@21,22
#define SSD1306_ADDRESS 0x3c
						Wire.beginTransmission(SSD1306_ADDRESS);
						err = Wire.endTransmission();
					delay(100);  // otherwise its too fast?!
						Wire.beginTransmission(SSD1306_ADDRESS);
						err = Wire.endTransmission();
					if(err!=0 && sonde.fingerprint!=17) {  // hmm. 17 after powerup with oled commected and no i2c answer!?!?
					sonde.fingerprint |= 128;
					LOG_I(TAG, "T-Beam, no I2C display found, assuming large TFT display\n");
					// CS=0, RST=14, RS=2, SDA=4, CLK=13
					sonde.config.disptype = 1;
					sonde.config.oled_sda = 4;
					sonde.config.oled_scl = 13;
					sonde.config.oled_rst = 14;
					sonde.config.tft_rs = 2;
					sonde.config.tft_cs = 0;
					sonde.config.spectrum = -1; // no spectrum for now on large display
					sonde.config.screenfile = 2;
					} else {
					// OLED display, pins 21,22 ok...
					sonde.config.disptype = 0;
					LOG_I(TAG, "T_BEAM with I2C OLED display\n");
					}
				}
			} else {
				LOG_I(TAG, "Autoconfig: looks like T-Beam v0.7 board");
				sonde.config.button_pin = 39;
				sonde.config.button2_pin = T4 + 128;  // T4 == GPIO13
				sonde.config.gps_rxd = 12;
				// Check if we possibly have a large display
				if(initlevels[21]==0) {
					LOG_I(TAG, "Autoconfig: looks like T-Beam v0.7 board with large TFT display");
					sonde.config.disptype = 1;
					sonde.config.oled_sda = 4;
					sonde.config.oled_scl = 21;
					sonde.config.oled_rst = 22;
					sonde.config.tft_rs = 2;
					sonde.config.tft_cs = 0;
					sonde.config.spectrum = -1; // no spectrum for now on large display
					sonde.config.screenfile = 2;
				}
			}
		} else {
			Serial.println("Looks like a TTGO V2.1_1.6, could also be a M5 Core, or CYD");
			// M5 core has I2C devices 0x16 0x68 0x75
			Wire.begin(21, 22);
#define BMM150 0x10
			Wire.beginTransmission(BMM150);
			byte err = Wire.endTransmission();
			if(err) { // try again
				 delay(400);
				 Wire.beginTransmission(BMM150);
				 err = Wire.endTransmission();
			}
			if(err==0) {
				applyBoardDefaults(BOARD_M5_CORE_GRAY);
			} else {
				// On CYD (new boards with USB-C, E32R28T&E32N28T)
				// GPIO4 is pulled high (pull up on audio enable) and GPIO21 is pulled low (polldown on TFT backlight control)
				if(initlevels[4]==1 && initlevels[21]==0) {
					applyBoardDefaults(BOARD_CYD_E32R28T);
				} else { 
					// Likely a TTGO V2.1_1.6
					// TODO: Maybe find some other default values for touch buttons?
					sonde.config.button_pin = -1;  // not good with SD-Card: 2 + 128;	 // GPIO2 / T2
					sonde.config.button2_pin = -1;  // not good with SD-Card: 14 + 128;   // GPIO14 / T6
					sonde.config.led_pout = 25;
					sonde.config.batt_adc = 35; 
					sonde.config.sd.cs = 13;
					sonde.config.sd.miso = 2;
					sonde.config.sd.mosi = 15;
			sonde.config.sd.clk = 14;
				}
			}
		}
	}
}

#ifndef SSD1306_ADDRESS
#define SSD1306_ADDRESS 0x3c
#endif
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

void startupPower() {
  // NOT TTGO v1 (fingerprint 64) or Heltec v1/v2 board (fingerprint 4)
  // and NOT TTGO Lora32 v2.1_1.6 (fingerprint 31/63)
  if ( ( (sonde.fingerprint & (64 + 31)) != 31) && ((sonde.fingerprint & 16) == 16) ) {
    // FOr T-Beam 1.0
    for (int i = 0; i < 2; i++) { // try multiple times
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
}

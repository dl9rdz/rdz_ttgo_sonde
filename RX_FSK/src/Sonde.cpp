#include <U8x8lib.h>
#include <U8g2lib.h>

#include "../features.h"

#define TAG "Sonde.cpp"
#include "src/logger.h"

#include "Sonde.h"
#include "RS41.h"
#if FEATURE_RS92
#include "RS92.h"
#endif
#include "DFM.h"
#include "M10M20.h"
#include "MP3H.h"
#include "SX1278FSK.h"
#include "Display.h"
#include <Wire.h>
#include "conn-mqtt.h"
#include "conn-sondeseeker.h"

RXTask rxtask = { -1, -1, -1, 0xFFFF, 0 };

const char *evstring[]={"NONE", "KEY1S", "KEY1D", "KEY1M", "KEY1L", "KEY2S", "KEY2D", "KEY2M", "KEY2L",
				   "VIEWTO", "RXTO", "NORXTO", "(max)"};

const char *RXstr[]={"RX_OK", "RX_TIMEOUT", "RX_ERROR", "RX_UNKNOWN"};

// Dependency to enum SondeType
const char *sondeTypeStr[NSondeTypes] = { "DFM ", "RS41", "RS92", "Mxx ", "M10 ", "M20 ", "MP3H" };
const char *sondeTypeLongStr[NSondeTypes] = { "DFM (all)", "RS41", "RS92", "M10/M20", "M10 ", "M20 ", "MP3-H1" };
const char sondeTypeChar[NSondeTypes] = { 'D', '4', 'R', 'M', 'M', '2', '3' };
const char *manufacturer_string[]={"Graw", "Vaisala", "Vaisala", "Meteomodem", "Meteomodem", "Meteomodem", "Meteo-Radiy"};
// tiny library printf does not support $ parameters, so remove....
// for now, only urls with the right order of parameters are supported, this maybe will change in the future again.
//const char *DEFEPH="gssc.esa.int/gnss/data/daily/%1$04d/brdc/brdc%2$03d0.%3$02dn.gz";
const char *DEFEPH="gssc.esa.int/cddis/gnss/data/daily/%04d/brdc/brdc%03d0.%02dn.gz";

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

/* global variables from RX_FSK.ino */
int getKeyPressEvent(); 
int handlePMUirq();
extern uint8_t pmu_irq;
extern SX1278FSK sx1278;

/* Task model:
 * There is a background task for all SX1278 interaction.
 *  - On startup and on each mode/frequency change (requested by setting requestNextSonde
 *    to an sonde index >=0) it calls Sonde::setup(), which will call the new decoder's
 *    setup function.  Setup will update the value currentSonde.
 *  - Periodically it calls Sonde::receive(), which calls the current decoder's receive()
 *    function. It should return control to the SX1278 main loop at least once per second.
 *    It will also set the internal variable receiveResult.  The decoder's receive function
 *    must make sure that there are no FIFO overflows in the SX1278.
 *  - the Arduino main loop will call the waitRXcomplete function, which should return as
 *    soon as there is some new data to display, or no later than after 1s, returning the
 *    value of receiveResult (or timeout, if receiveResult was not set within 1s). It
 *    should also return immediately if there is some keyboard input.
 */   
int initlevels[40];

Sonde::Sonde() {
	for (int i = 0; i < 39; i++) {
		initlevels[i] = gpio_get_level((gpio_num_t)i);
  	}
}

void Sonde::setDefaultConfig(BoardTypes board) {
	switch(board) {
	case BOARD_M5_CORE_GRAY:
		LOG_I(TAG, "Autoconfig: M5stack Core Gray board detected\n");
		config.type = TYPE_M5_CORE;
		config.button_pin = 39;
		config.button2_pin = 38;
		config.button2_axp = 0;
		config.disptype = 4;  // ILI9342
		config.oled_sda = 23;
		config.oled_scl = 18;
		config.oled_rst = 33;
		config.tft_rs = 27;
		config.tft_cs = 14;
		config.power_pout = 32+128;
		// gpio32 is backlight of tft
		config.screenfile = 4;
		config.gps_rxd = 16;
		config.gps_txd = -1;  // 17
		config.sx1278_ss = 5;
		config.sx1278_miso = 19;
		config.sx1278_mosi = 23; //MOSI;
		config.sx1278_sck = 18; // SCK;
		break;
	case BOARD_TTGOv1_HELTEC:
		LOG_I(TAG, "Autoconfig: looks like TTGO v1 / Heltec v1/V2 board\n");
		config.oled_sda = 4;
		config.oled_scl = 15;
		config.oled_rst = 16;
		config.button_pin = 0;
		config.button2_pin = T4 + 128;	 // T4 == GPIO13
		config.power_pout = 21;		   // for Heltec v2
		config.led_pout = 2;	
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
		
		config.disptype = buf[2]==0xd9? 5 : 3; // could be 3 or 5, both versions exist...
		config.oled_sda = 13;
		config.oled_scl = 14;
		config.oled_rst = -1;
		config.tft_rs = 2;
		config.tft_cs = 15;
		config.power_pout = 21 + 128;
		config.gps_rxd = 35;
		config.gps_txd = -1;
		config.sx1278_ss = 27;
		config.sx1278_miso = 19;
		config.sx1278_mosi= 23;
		config.sx1278_sck = 18;
		config.button_pin = 0;
		config.button2_pin = -1;
	       }
	       break;
	}
}


void Sonde::defaultConfig() {
	fingerprint = initlevels[4];
	fingerprint = (fingerprint<<1) | initlevels[12];
	fingerprint = (fingerprint<<1) | initlevels[16];
	fingerprint = (fingerprint<<1) | initlevels[17];
	fingerprint = (fingerprint<<1) | initlevels[21];
	fingerprint = (fingerprint<<1) | initlevels[22];
	fingerprint = (fingerprint<<1) | initlevels[23];
	LOG_I(TAG, "Board fingerprint is %d\n", fingerprint);

	sondeList = (SondeInfo *)malloc((MAXSONDE+1)*sizeof(SondeInfo));
	// addSonde should initialize everything anyway, so this should not strictly be necessary, but does no harm either
	memset(sondeList, 0, (MAXSONDE+1)*sizeof(SondeInfo));
	for(int i=0; i<(MAXSONDE+1); i++) {
		sondeList[i].freq=400;
		sondeList[i].type=STYPE_RS41;
		clearAllData(&sondeList[i]);
	}
	config.touch_thresh = 70;
	config.led_pout = -1;
	config.power_pout = -1;
	config.spectrum=10;
	config.b2mute = 360;
	// Try autodetecting board type
	config.type = TYPE_TTGO;
  	// Seems like on startup, GPIO4 is 1 on v1 boards, 0 on v2.1 boards?
	config.gps_rxd = -1;
	config.gps_txd = -1;
	config.batt_adc = -1;
	config.sx1278_ss = SS; // default SS pin, on all TTGOs
	config.sx1278_miso = MISO;
	config.sx1278_mosi = MOSI;
	config.sx1278_sck = SCK;
	// config.oled_rst = 16;
	config.oled_rst = -1;   // GPIO16 is Flash CS on Lora32 v1.6
	config.sd.cs = -1;
	config.sd.miso = -1;
	config.sd.mosi = -1;
	config.sd.clk = -1;
	config.sd.name = 0;
	config.sd.speed = 4000000;
	config.disptype = 0;
	config.dispcontrast = -1;
	config.tft_orient = 1;
	config.button2_axp = 0;
	config.norx_timeout = 20;
	config.screenfile = 0;
	config.tft_spifreq = SPI_DEFAULT_FREQ;
	// TFT RS and CS not used for LCD, if TFT detected this will be changed below
	config.tft_rs = -1;
	config.tft_cs = -1;
	if(initlevels[16]==0) {
		// Plain M5 Core Gray sometimes ends up here (without Lora/GPS board attached, or after reboot)
		// 71 w/o GPS, 79 after reboot via flash utility
		if(fingerprint == 79 || fingerprint==71)      
			setDefaultConfig(BOARD_M5_CORE_GRAY);
		else
			setDefaultConfig(BOARD_TTGOv1_HELTEC);

	} else {
		config.oled_sda = 21;
		config.oled_scl = 22;
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
					config.type = TYPE_M5_CORE2;
					config.button_pin = 255;
					config.button2_pin = 255;
					config.button2_axp = 1;
					config.disptype = 4;  // ILI9342
					config.oled_sda = 23;
					config.oled_scl = 18;
					config.oled_rst = -1;
					config.tft_rs = 15;
					config.tft_cs = 5;
					config.screenfile = 4;
					config.gps_rxd = 13;
					config.gps_txd = -1;  // 14
					config.sx1278_ss = 33;
					config.sx1278_miso = 38;
					config.sx1278_mosi = 23; //MOSI;
					config.sx1278_sck = 18; // SCK;
				} else { // some t-beam...
					config.button_pin = 38;
					config.button2_pin = 15 + 128; //T4 + 128;  // T4 = GPIO13
					// Maybe in future use as default only PWR as button2?
					//config.button2_pin = 255;
					config.button2_axp = 1;
					config.gps_rxd = 34;
					config.gps_txd = 12;
					// Check for I2C-Display@21,22
#define SSD1306_ADDRESS 0x3c
						Wire.beginTransmission(SSD1306_ADDRESS);
						err = Wire.endTransmission();
					delay(100);  // otherwise its too fast?!
						Wire.beginTransmission(SSD1306_ADDRESS);
						err = Wire.endTransmission();
					if(err!=0 && fingerprint!=17) {  // hmm. 17 after powerup with oled commected and no i2c answer!?!?
					fingerprint |= 128;
					LOG_I(TAG, "T-Beam, no I2C display found, assuming large TFT display\n");
					// CS=0, RST=14, RS=2, SDA=4, CLK=13
					config.disptype = 1;
					config.oled_sda = 4;
					config.oled_scl = 13;
					config.oled_rst = 14;
					config.tft_rs = 2;
					config.tft_cs = 0;
					config.spectrum = -1; // no spectrum for now on large display
					config.screenfile = 2;
					} else {
					// OLED display, pins 21,22 ok...
					config.disptype = 0;
					LOG_I(TAG, "T_BEAM with I2C OLED display\n");
					}
				}
			} else {
				LOG_I(TAG, "Autoconfig: looks like T-Beam v0.7 board");
				config.button_pin = 39;
				config.button2_pin = T4 + 128;  // T4 == GPIO13
				config.gps_rxd = 12;
				// Check if we possibly have a large display
				if(initlevels[21]==0) {
					LOG_I(TAG, "Autoconfig: looks like T-Beam v0.7 board with large TFT display");
					config.disptype = 1;
					config.oled_sda = 4;
					config.oled_scl = 21;
					config.oled_rst = 22;
					config.tft_rs = 2;
					config.tft_cs = 0;
					config.spectrum = -1; // no spectrum for now on large display
					config.screenfile = 2;
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
				setDefaultConfig(BOARD_M5_CORE_GRAY);
			} else {
				// On CYD (new boards with USB-C, E32R28T&E32N28T)
				// GPIO4 is pulled high (pull up on audio enable) and GPIO21 is pulled low (polldown on TFT backlight control)
				if(initlevels[4]==1 && initlevels[21]==0) {
					setDefaultConfig(BOARD_CYD_E32R28T);
				} else { 
					// Likely a TTGO V2.1_1.6
					// TODO: Maybe find some other default values for touch buttons?
					config.button_pin = -1;  // not good with SD-Card: 2 + 128;	 // GPIO2 / T2
					config.button2_pin = -1;  // not good with SD-Card: 14 + 128;   // GPIO14 / T6
					config.led_pout = 25;
					config.batt_adc = 35; 
					config.sd.cs = 13;
					config.sd.miso = 2;
					config.sd.mosi = 15;
			config.sd.clk = 14;
				}
			}
		}
	}
	//
	config.noisefloor = -125;
	strcpy(config.call,"NOCALL");
	config.passcode = -1;
	strcpy(config.mdnsname, "rdzsonde");
	config.maxsonde=15;
	config.debug=0;
	config.wifi=1;
	config.display[0]=0;
	config.display[1]=1;
	config.display[2]=-1;
	config.startfreq=400;
	config.channelbw=10;
	config.marker=0;
	config.freqofs=0;
	config.rs41.agcbw=12500;
	config.rs41.rxbw=6300;
	config.rs92.rxbw=12500;
	config.rs92.alt2d=480;
	config.dfm.agcbw=20800;
	config.dfm.rxbw=10400;
	config.m10m20.agcbw=20800;
	config.m10m20.rxbw=12500;
	config.mp3h.agcbw=12500;
	config.mp3h.rxbw=12500;
	config.udpfeed.active = 1;
	strcpy(config.udpfeed.host, "192.168.42.20:9002");
	config.udpfeed.ratelimit= 1;
	config.tcpfeed.active = 0;
	strcpy(config.tcpfeed.host, "radiosondy.info:14580");
	// default config consistent with default config.txt: set only first aprs host
	// strcpy(config.tcpfeed.host2, "wettersonde.net:14580");
	strcpy(config.tcpfeed.symbol, "/O");
	config.tcpfeed.highrate = 10;
	config.kisstnc.active = 0;
	strcpy(config.ephftp,DEFEPH);

	config.mqtt.active = 0;
	strcpy(config.mqtt.id, "rdz_sonde_server");
	config.mqtt.port = 1883;
	strcpy(config.mqtt.username, "/0");
	strcpy(config.mqtt.password, "/0");
	strcpy(config.mqtt.prefix, "rdz_sonde_server/");
	config.mqtt.report_interval = 60000;

	config.ss.active = 1;
 	config.ss.port = 62655;
 	strcpy(config.ss.host, "239.255.0.1");
}

extern struct st_configitems config_list[];
extern const int N_CONFIG;

void Sonde::checkConfig() {
	if(config.maxsonde > MAXSONDE) config.maxsonde = MAXSONDE;
	if(config.sondehub.fiinterval<5) config.sondehub.fiinterval = 5;
	if(config.sondehub.fimaxdist>700) config.sondehub.fimaxdist = 700;
	if(config.sondehub.fimaxage>48) config.sondehub.fimaxage = 48;
	if(config.sondehub.fimaxdist==0) config.sondehub.fimaxdist = 150;
	if(config.sondehub.fimaxage==0) config.sondehub.fimaxage = 2;
	// auto upgrade config to new version with format arguments in string
	if(!strchr(sonde.config.ephftp,'%')) strcpy(sonde.config.ephftp,DEFEPH);
	// $ not supported for now...
	if(strchr(sonde.config.ephftp,'$')) strcpy(sonde.config.ephftp,DEFEPH);
	switch(strlen(config.beaconsym)) {
		case 0: case 1:
			strcpy(config.beaconsym,"/`/("); // default: dish antenne, mobile sat station
			break;
		case 2: case 3:
			strcpy(config.beaconsym+2, "/(");  // default sym2: mobile sat station
	}
}
void Sonde::setConfig(const char *cfg) {
	while(*cfg==' '||*cfg=='\t') cfg++;
	if(*cfg=='#') return;
	char *s = strchr(cfg,'=');
	if(!s) return;
	char *val = s+1;
	*s=0; s--;
	while(s>cfg && (*s==' '||*s=='\t')) { *s=0; s--; }
	//LOG_D(TAG, "configuration option '%s'=%s \n", cfg, val);

	// new code: use config_list to find config entry...
	int i;
	for(i=0; i<N_CONFIG; i++) {
		if(strcmp(cfg, config_list[i].name)!=0) continue;

		if(config_list[i].type>0) {  // string with that length
			strlcpy((char *)config_list[i].data, val, config_list[i].type+1);
			break;
		} 
		switch(config_list[i].type) {
		case 0:   // integer
		case -4:  // integer (with "touch button" checkbox in web form)
		case -3:  // integer (boolean on/off swith in web form)
		case -2:  // integer (ID type)
			*(int *)config_list[i].data = atoi(val);
			break;
		case -7:  // double
		{
			double d = atof(val);
			if(*val == 0 || d==0) d = NAN;
			*(double *)config_list[i].data = d;
			break;
		}
		case -6:  // display list
		{
			int idx = 0;
			char *ptr;
			while(val) {
				ptr = strchr(val,',');
				if(ptr) *ptr = 0;
				config.display[idx++] = atoi(val);
				val = ptr?ptr+1:NULL;
				LOG_D(TAG, "appending value %d  next is %s\n", config.display[idx-1], val?val:"");
			}
			config.display[idx] = -1;
			break;
		}
		default:
			// skipping non-supported types
			break;
		}

		break;
	}
	if(i==N_CONFIG) {
		LOG_E(TAG, "Invalid config option '%s'=%s \n", cfg, val);
	}
}

void Sonde::setIP(String ip, bool AP) {
	ipaddr = ip;
	isAP = AP;
}

void Sonde::clearSonde() {
	nSonde = 0;
}
void Sonde::addSonde(float frequency, SondeType type, int active, char *launchsite)  {
	if(nSonde>=config.maxsonde) {
		LOG_E(TAG, "Cannot add another sonde, MAXSONDE reached");
		return;
	}
	LOG_I(TAG, "Adding %f - %d - %d - %s\n", frequency, type, active, launchsite);
	// reset all data if type or frequency has changed
	if(type != sondeList[nSonde].type || frequency != sondeList[nSonde].freq) {
		//TODO: Check for potential race condition with decoders
		// do not clear extra while decoder is potentiall still accessing it!
		if(sondeList[nSonde].extra) free(sondeList[nSonde].extra);
			memset(&sondeList[nSonde], 0, sizeof(SondeInfo));
		sondeList[nSonde].type = type;
		sondeList[nSonde].d.typestr[0] = 0;
		sondeList[nSonde].freq = frequency;
		memcpy(sondeList[nSonde].rxStat, "\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3\x3", 18); // unknown/undefined
		clearAllData(sondeList+nSonde);
	}
	sondeList[nSonde].active = active;
	strncpy(sondeList[nSonde].launchsite, launchsite, 17);	
	nSonde++;
}

// called by updateState (only)
void Sonde::nextConfig() {
	currentSonde++;
	if(currentSonde>=config.maxsonde) {
		currentSonde=0;
	}
	// Skip non-active entries (but don't loop forever if there are no active ones)
	for(int i=0; i<config.maxsonde - 1; i++) {
		if(!sondeList[currentSonde].active) {
			currentSonde++;
			if(currentSonde>=config.maxsonde) currentSonde=0;
		}
	}
}
void Sonde::nextRxSonde() {
	rxtask.currentSonde++;
	if(rxtask.currentSonde>=config.maxsonde) {
		rxtask.currentSonde=0;
	}
	for(int i=0; i<config.maxsonde - 1; i++) {
		if(!sondeList[rxtask.currentSonde].active) {
			rxtask.currentSonde++;
			if(rxtask.currentSonde>=config.maxsonde) rxtask.currentSonde=0;
		}
	}
	LOG_D(TAG, "nextRxSonde: %d\n", rxtask.currentSonde);
}
void Sonde::nextRxFreq(int addkhz) {
	// last entry is for the variable frequency
	rxtask.currentSonde = nSonde - 1;
	sondeList[rxtask.currentSonde].active = 1;
	sondeList[rxtask.currentSonde].freq += addkhz*0.001;
	if(sondeList[rxtask.currentSonde].freq>406)
		sondeList[rxtask.currentSonde].freq = 400;
	LOG_D(TAG, "nextRxFreq: %d\n", rxtask.currentSonde);
}
SondeInfo *Sonde::si() {
	return &sondeList[currentSonde];
}

void Sonde::setup() {
	if(rxtask.currentSonde<0 || rxtask.currentSonde>=config.maxsonde) {
		LOG_E(TAG, "Invalid rxtask.currentSonde: %d\n", rxtask.currentSonde);
		rxtask.currentSonde = 0;
		for(int i=0; i<config.maxsonde - 1; i++) {
			if(!sondeList[rxtask.currentSonde].active) {
				rxtask.currentSonde++;
				if(rxtask.currentSonde>=config.maxsonde) rxtask.currentSonde=0;
			}
		}
		sonde.currentSonde = rxtask.currentSonde;
	}

	// update receiver config
	LOG_I(TAG, "Sonde::setup() start on index %d\n", rxtask.currentSonde);
	switch(sondeList[rxtask.currentSonde].type) {
	case STYPE_RS41:
		rs41.setup(sondeList[rxtask.currentSonde].freq * 1000000);
		break;
	case STYPE_DFM:
		dfm.setup( sondeList[rxtask.currentSonde].freq * 1000000, sondeList[rxtask.currentSonde].type );
		break;
	case STYPE_RS92:
#if FEATURE_RS92
		rs92.setup( sondeList[rxtask.currentSonde].freq * 1000000);
#endif
		break;
	case STYPE_M10:
	case STYPE_M20:
	case STYPE_M10M20:
		m10m20.setup( sondeList[rxtask.currentSonde].freq * 1000000);
		break;
	case STYPE_MP3H:
		mp3h.setup( sondeList[rxtask.currentSonde].freq * 1000000);
		break;
	}
	// debug
	int freq = (int)sx1278.getFrequency();
	int afcbw = (int)sx1278.getAFCBandwidth();
	int rxbw = (int)sx1278.getRxBandwidth();
	LOG_I(TAG, "Sonde::setup() done: Type %s Freq %f, AFC BW: %d, RX BW: %d\n", sondeTypeStr[sondeList[rxtask.currentSonde].type], 0.000001*freq, afcbw, rxbw);

	// reset rxtimer / norxtimer state
	sonde.sondeList[sonde.currentSonde].lastState = -1;
}

extern void flashLed(int ms);

void Sonde::receive() {
	uint16_t res = 0;
	SondeInfo *si = &sondeList[rxtask.currentSonde];
	switch(si->type) {
	case STYPE_RS41:
		res = rs41.receive();
		break;
	case STYPE_RS92:
#if FEATURE_RS92
		res = rs92.receive();
#endif
		break;
	case STYPE_M10:
	case STYPE_M20:
	case STYPE_M10M20:
		res = m10m20.receive();
		break;
	case STYPE_DFM:
		res = dfm.receive();
		break;
	case STYPE_MP3H:
		res = mp3h.receive();
		break;
	}

	// state information for RX_TIMER / NORX_TIMER events
	if(res==RX_OK || res==RX_ERROR) {  // something was received...
		flashLed( (res==RX_OK)?700:100);
		if(si->lastState != 1) {
			si->rxStart = millis();
			si->lastState = 1;
			sonde.dispsavectlON();
		}
	} else { // RX Timeout
		//LOG_I(TAG, "Sonde::receive(): result %d (%s), laststate was %d\n", res, (res<=3)?RXstr[res]:"?", si->lastState);
		if(si->lastState != 0) {
			si->norxStart = millis();
			si->lastState = 0;
		}
	}
	// LOG_I(TAG, "debug: res was %d, now lastState is %d\n", res, si->lastState);


	// we should handle timer events here, because after returning from receive,
	// we'll directly enter setup
	rxtask.receiveSonde = rxtask.currentSonde; // pass info about decoded sonde to main loop

	int event = getKeyPressEvent();
	if (!event) event = timeoutEvent(si);
	else sonde.dispsavectlON();
	int action = (event==EVT_NONE) ? ACT_NONE : 
                     (event==EVT_RINEX) ? ACT_RINEX_UPDATE : 
		     (event==EVT_FORMAT) ? ACT_FORMAT_SD : disp.layout->actions[event];
	if(action!=ACT_NONE) { LOG_I(TAG, "event %x: action is %x\n", event, action); }
	// If action is to move to a different sonde index, we do update things here, set activate
	// to force the sx1278 task to call sonde.setup(), and pass information about sonde to
	// main loop (display update...)
	if(action == ACT_DISPLAY_SCANNER || action == ACT_NEXTSONDE || action==ACT_PREVSONDE || (action>64&&action<128) ) {
		// handled here...
		if(action==ACT_DISPLAY_SCANNER) {
			// nothing to do here, be re-call setup() for M10/M20 for repeating AFC
		}
		else {
			if(action==ACT_NEXTSONDE||action==ACT_PREVSONDE)
				nextRxSonde();
			else
				nextRxFreq( action-64 );
			action = ACT_SONDE(rxtask.currentSonde);
		}
		if(rxtask.activate==-1) {
			// race condition here. maybe better use mutex. TODO
			rxtask.activate = ACT_SONDE(rxtask.currentSonde);
		}
	}
	LOG_I(TAG, "Sonde:receive(): result %d (%s), event %02x => action %02x\n", res, (res<=3)?RXstr[res]:"?", event, action);
	res = (action<<8) | (res&0xff);
	// let waitRXcomplete resume...
	rxtask.receiveResult = res;
}

// return (action<<8) | (rxresult)
uint16_t Sonde::waitRXcomplete() {
	uint16_t res=0;
	uint32_t t0 = millis();
rxloop:
	while( (pmu_irq!=1) && rxtask.receiveResult==0xFFFF && millis()-t0 < 3000) { delay(50); }
	if( pmu_irq ) {
		handlePMUirq();
		if(pmu_irq!=2) goto rxloop;
	}
	if( rxtask.receiveResult == RX_UPDATERSSI ) {
		rxtask.receiveResult = 0xFFFF;
		LOG_I(TAG, "RSSI update: %d/2\n", sonde.si()->rssi);
		disp.updateDisplayRSSI();
		goto rxloop;
	}

	if( rxtask.receiveResult==0xFFFF) {
		LOG_I(TAG, "TIMEOUT in waitRXcomplete. Should never happen!\n");
		res = RX_TIMEOUT;
	} else {
		res = rxtask.receiveResult;
	}
	rxtask.receiveResult = 0xFFFF;
	/// TODO: THis has caused an exception when swithcing back to spectrumm...
	LOG_I(TAG, "waitRXcomplete returning %04x (%s)\n", res, (res&0xff)<4?RXstr[res&0xff]:"");
	// currently used only by RS92
	switch(sondeList[rxtask.receiveSonde].type) {
	case STYPE_RS41:
		rs41.waitRXcomplete();
		break;
	case STYPE_RS92:
#if FEATURE_RS92
		rs92.waitRXcomplete();
#endif
		break;
	case STYPE_M10:
	case STYPE_M20:
	case STYPE_M10M20:
		m10m20.waitRXcomplete();
		break;
	case STYPE_DFM:
		dfm.waitRXcomplete();
		break;
	case STYPE_MP3H:
		mp3h.waitRXcomplete();
		break;
	}
	memmove(sonde.si()->rxStat+1, sonde.si()->rxStat, 17);
	sonde.si()->rxStat[0] = res;
	return res;
}

uint8_t Sonde::timeoutEvent(SondeInfo *si) {
	uint32_t now = millis();
#if 0
	LOG_I(TAG, "Timeout check: %d - %d vs %d; %d - %d vs %d; %d - %d vs %d; lastState: %d\n",
		now, si->viewStart, disp.layout->timeouts[0],
		now, si->rxStart, disp.layout->timeouts[1],
		now, si->norxStart, disp.layout->timeouts[2], si->lastState);
#endif
	if(disp.layout->timeouts[0]>=0 && now - si->viewStart >= disp.layout->timeouts[0]) {
		LOG_I(TAG, "Sonde::timeoutEvent: View\n");
		return EVT_VIEWTO;
	}
	if(si->lastState==1 && disp.layout->timeouts[1]>=0 && now - si->rxStart >= disp.layout->timeouts[1]) {
		LOG_I(TAG, "Sonde::timeoutEvent: RX\n");
		return EVT_RXTO;
	}
	if(si->lastState==0 && disp.layout->timeouts[2]>=0 && now - si->norxStart >= disp.layout->timeouts[2]) {
		LOG_I(TAG, "Sonde::timeoutEvent: NORX\n");
		return EVT_NORXTO;
	}
	return 0;
}

uint8_t Sonde::updateState(uint8_t event) {
	//LOG_I(TAG, "Sonde::updateState for event %02x\n", event);
	// No change
	if(event==ACT_NONE) return 0xFF;

	// In all cases (new display mode, new sonde) we reset the mode change timers
	sonde.sondeList[sonde.currentSonde].viewStart = millis();
	sonde.sondeList[sonde.currentSonde].lastState = -1;

	// Moving to a different display mode
	if (event==ACT_DISPLAY_SPECTRUM || event==ACT_DISPLAY_WIFI) {
		// main loop will call setMode() and disable sx1278 background task
		return event;
	}
	int n = event;
	if(event==ACT_DISPLAY_DEFAULT) {
		n = config.display[1];
	} else if(event==ACT_DISPLAY_SCANNER) {
		n= config.display[0];
	} else if(event==ACT_DISPLAY_NEXT) {
		int i;
		for(i=0; config.display[i]!=-1; i++) {
			if(config.display[i] == disp.layoutIdx) break;
		}
		if(config.display[i]==-1 || config.display[i+1]==-1) {
			//unknown index, or end of list => loop to start
			n = config.display[1];
		} else {
			n = config.display[i+1];
		}
	}
	if(n>=0 && n<ACT_MAXDISPLAY) {
		if(n>=disp.nLayouts) {
			LOG_I(TAG, "WARNNG: next layout out of range");
			n = config.display[1];
		}
		LOG_I(TAG, "Setting display mode %d\n", n);
		disp.setLayout(n);
		sonde.clearDisplay();
		return 0xFF;
	}

	// Moving to a different value for currentSonde
	// TODO: THis should be done in sx1278 task, not in main loop!!!!!
	if(event==ACT_NEXTSONDE) {
		sonde.nextConfig();
		LOG_I(TAG, "advancing to next sonde %d\n", sonde.currentSonde);
		return event;
	}
	if (event==ACT_PREVSONDE) {
		// TODO
		LOG_I(TAG, "previous not supported, advancing to next sonde\n");
		sonde.nextConfig();
		return ACT_NEXTSONDE;
	}
        if (event==ACT_RINEX_UPDATE) {
                return ACT_RINEX_UPDATE;
        }
	if (event==ACT_FORMAT_SD) {
		return ACT_FORMAT_SD;
	}
	if(event&0x80) {
		sonde.currentSonde = (event&0x7F);
		return ACT_NEXTSONDE;
	}
	return 0xFF;
}

void Sonde::clearAllData(SondeInfo *si) {
	// set everything to 0
	memset(&(si->d), 0, sizeof(SondeData));
	// set floats to NaN
	si->d.lat = si->d.lon = si->d.alt = si->d.vs = si->d.hs = si->d.dir = NAN;
	si->d.temperature = si->d.tempRHSensor = si->d.relativeHumidity = si->d.pressure = si->d.batteryVoltage = NAN;
}

void Sonde::updateDisplayPos() {
	disp.updateDisplayPos();
}

void Sonde::updateDisplayPos2() {
	disp.updateDisplayPos2();
}

void Sonde::updateDisplayID() {
	disp.updateDisplayID();
}
	
void Sonde::updateDisplayRSSI() {
	disp.updateDisplayRSSI();
}

void Sonde::updateStat() {
	disp.updateStat();
}

void Sonde::updateDisplayRXConfig() {
	disp.updateDisplayRXConfig();
}

void Sonde::updateDisplayIP() {
	disp.updateDisplayIP();
}

void Sonde::updateDisplay()
{
	disp.updateDisplay();
}

void Sonde::clearDisplay() {
	disp.rdis->clear();
}

void Sonde::dispsavectlON() {
	disp.dispsavectlON();
}

void Sonde::dispsavectlOFF(int rxactive) {
	disp.dispsavectlOFF(rxactive);
}


SondeType Sonde::realType(SondeInfo *si) {
	if(TYPE_IS_METEO(si->type) && si->d.subtype>0 ) { return si->d.subtype==1 ? STYPE_M10:STYPE_M20; }
	else return si->type;
}

Sonde sonde = Sonde();

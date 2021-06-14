
#ifndef Sonde_h
#define Sonde_h

enum DbgLevel { DEBUG_OFF=0, DEBUG_INFO=1, DEBUG_SPARSER=16, DEBUG_DISPLAY=8 };  // to be extended for configuring serial debug output
extern uint8_t debug;

#define DebugPrint(l,x) if(debug&l) { Serial.print(x); }
#define DebugPrintln(l,x) if(debug&l) { Serial.println(x); }
#define DebugPrintf(l,...) if(debug&l) { Serial.printf(__VA_ARGS__); }

// RX_TIMEOUT: no header detected
// RX_ERROR: header detected, but data not decoded (crc error, etc.)
// RX_OK: header and data ok
enum RxResult { RX_OK, RX_TIMEOUT, RX_ERROR, RX_UNKNOWN, RX_NOPOS };
#define RX_UPDATERSSI 0xFFFE

// Events that change what is displayed (mode, sondenr)
// Keys:
// 1  Button (short)  or Touch (short)
// 2  Button (double) or Touch (double)
// 3  Button (mid)    or Touch (mid)
// 4  Button (long)   or Touch (long)
// 5  Touch1/2 (short)
// 6  Touch1/2 (double)
// 7  Touch1/2 (mid)
// 8  Touch1/2 (long)

/* Keypress => Sonde++ / Sonde-- / Display:=N*/
enum Events { EVT_NONE, EVT_KEY1SHORT, EVT_KEY1DOUBLE, EVT_KEY1MID, EVT_KEY1LONG,
                        EVT_KEY2SHORT, EVT_KEY2DOUBLE, EVT_KEY2MID, EVT_KEY2LONG,
                        EVT_VIEWTO, EVT_RXTO, EVT_NORXTO,
              EVT_MAX };
extern const char *evstring[];
extern const char *RXstr[];
#define EVENTNAME(s) evstring[s]

//int8_t actions[EVT_MAX];
#define ACT_NONE 255
#define ACT_DISPLAY(n) (n)
#define ACT_MAXDISPLAY 50
#define ACT_DISPLAY_SCANNER 0
#define ACT_DISPLAY_NEXT 64
#define ACT_DISPLAY_DEFAULT 63
#define ACT_DISPLAY_SPECTRUM 62
#define ACT_DISPLAY_WIFI 61
#define ACT_NEXTSONDE 65
#define ACT_PREVSONDE 66
#define ACT_ADDFREQ(n) ((n)+64)
#define ACT_SONDE(n) ((n)+128)

// 0000nnnn => goto display nnnn
// 01000000 => goto sonde -1
// 01000001 => goto sonde +1

#define NSondeTypes 8
enum SondeType { STYPE_DFM, STYPE_DFM09_OLD, STYPE_RS41, STYPE_RS92, STYPE_M10, STYPE_M20, STYPE_DFM06_OLD, STYPE_MP3H };
extern const char *sondeTypeStr[NSondeTypes];
extern const char *sondeTypeLongStr[NSondeTypes];
extern const char sondeTypeChar[NSondeTypes];
extern const char *manufacturer_string[NSondeTypes];

#define TYPE_IS_DFM(t) ( (t)==STYPE_DFM || (t)==STYPE_DFM09_OLD || (t)==STYPE_DFM06_OLD )
#define TYPE_IS_METEO(t) ( (t)==STYPE_M10 || (t)==STYPE_M20 )

typedef struct st_sondeinfo {
        // receiver configuration
	bool active;
        SondeType type;
	int8_t subtype;   /* 0 for none/unknown, hex type for dfm, maybe add 1/2 for M10/M20 as well?*/
        float freq;
        // decoded ID
	char typestr[5];			// decoded type (use type if *typestr==0)
        char id[10];
	char ser[12];
        bool validID;
	char launchsite[18];		
        // decoded position
        float lat;			// latitude
        float lon;			// longitude
        float alt;			// altitude
        float vs;			// vertical speed in m/s
        float hs;			// horizontal speed in m/s
	float dir; 			// 0..360
	uint8_t sats;			// number of sats
        uint8_t validPos;   // bit pattern for validity of above 7 fields; 0x80: position is old
	// decoded GPS time
	uint32_t time;
	uint16_t sec;
	uint32_t frame;
	bool validTime;
        // RSSI from receiver
        int rssi;			// signal strength
	int32_t afc;			// afc correction value
	// statistics
	uint8_t rxStat[20];
	uint32_t rxStart;    		// millis() timestamp of continuous rx start
	uint32_t norxStart;		// millis() timestamp of continuous no rx start
	uint32_t viewStart;		// millis() timestamp of viewinf this sonde with current display
	int8_t lastState;		// -1: disabled; 0: norx; 1: rx
	// shut down timers, currently only for RS41; -1=disabled
	int16_t launchKT, burstKT, countKT;
	uint16_t crefKT; // frame number in which countKT was last sent
	// sonde specific extra data, NULL if unused or not yet initialized, currently used for RS41 subframe data (calibration)
        void *extra;
	float temperature = -300.0; // platinum resistor temperature
	float tempRHSensor = -300.0; // temperature of relative humidity sensor
	float relativeHumidity = -1.0; // relative humidity
} SondeInfo;
// rxStat: 3=undef[empty] 1=timeout[.] 2=errro[E] 0=ok[|] 4=no valid position[°]

// Used for interacting with the RX background task
typedef struct st_RXTask {
	// Variables set by Arduino main loop to value >=0 for requesting
	// mode change to sonde reception for sonde <value) in RXTask.
	// Will be reset to -1 by RXTask
	int activate;
	// Variables set by RXTask, corresponding to mode ST_DECODER (if active) or something else,
	// and currently received sonde
	int mainState;
	int currentSonde;
	// Variable set by RXTask to communicate status to Arduino task
	// via waitRXcomplete function
	uint16_t receiveResult;
	uint16_t receiveSonde;  // sonde inde corresponding to receiveResult
	// status variabe set by decoder to indicate something is broken
	// int fifoOverflow;
} RXTask;

extern RXTask rxtask;

struct st_rs41config {
	int agcbw;
	int rxbw;
};
struct st_rs92config {
	int rxbw;
	int alt2d;
};
struct st_dfmconfig {
	int agcbw;
	int rxbw;
};
struct st_m10m20config {
	int agcbw;
	int rxbw;
};
struct st_mp3hconfig {
	int agcbw;
	int rxbw;
};


enum IDTYPE { ID_DFMDXL, ID_DFMGRAW, ID_DFMAUTO };

struct st_feedinfo {
        bool active;
        int type;       // 0:UDP(axudp), 1:TCP(aprs.fi)
        char host[64];
        int port;
        char symbol[3];
        int lowrate;
        int highrate;
        int lowlimit;
        int idformat;   // 0: dxl  1: real  2: auto
};

// maybe extend for external Bluetooth interface?
// internal bluetooth consumes too much memory
struct st_kisstnc {
        bool active;
        int idformat;
};

struct st_mqtt {
	int active;
	char id[64];
	char host[64];
	int port;
	char username[64];
	char password[64];
	char prefix[64];
};

struct st_sondehub {
	int active;
	int chase;
	char host[64];
	char callsign[64];
	char lat[20];
	char lon[20];
	char alt[20];
	char antenna[64];
	char email[64];
};

typedef struct st_rdzconfig {
	// hardware configuration
	int button_pin;			// PIN port number menu button (+128 for touch mode)
	int button2_pin;		// PIN port number menu button (+128 for touch mode)
	int button2_axp;		// Use AXP192 power button as button2
	int touch_thresh;		// Threshold value (0..100) for touch input button
	int led_pout;			// POUT port number of LED (used as serial monitor)
	int power_pout;			// Power control pin (for Heltec v2)
	int disptype;			// 0=OLED; 1=ILI9225
	int oled_sda;			// OLED/TFT data pin 
	int oled_scl;			// OLED/TFT clock pin
	int oled_rst;			// OLED/TFT reset pin
	int tft_rs;			// TFT RS pin
	int tft_cs;			// TFT CS pin
	int tft_orient;			// TFT orientation (default: 1)
	int tft_modeflip;		// Hack for Joerg's strange display
	int gps_rxd;			// GPS module RXD pin. We expect 9600 baud NMEA data.
	int gps_txd;			// GPS module TXD pin
	// software configuration
	int debug;				// show port and config options after reboot
	int wifi;				// connect to known WLAN 0=skip
	int wifiap;				// enable/disable WiFi AccessPoint mode 0=disable
	int screenfile;
	int8_t display[30];			// list of display mode (0:scanner, 1:default, 2,... additional modes)
	int startfreq;			// spectrum display start freq (400, 401, ...)
	int channelbw;			// spectrum channel bandwidth (valid: 5, 10, 20, 25, 50, 100 kHz)	
	int spectrum;			// show freq spectrum for n seconds -1=disable; 0=forever
	int marker;				// show freq marker in spectrum  0=disable
	int maxsonde;			// number of max sonde in scan (range=1-99)
	int norx_timeout;		// Time after which rx mode switches to scan mode (without rx signal)
	int noisefloor;			// for spectrum display
	char mdnsname[15];		// mDNS-Name, defaults to rdzsonde
	// receiver configuration
	int showafc;			// show afc value in rx screen
	int freqofs;			// frequency offset (tuner config = rx frequency + freqofs) in Hz
	struct st_rs41config rs41;	// configuration options specific for RS41 receiver
	struct st_rs92config rs92;
	struct st_dfmconfig dfm;
	struct st_m10m20config m10m20;
	struct st_mp3hconfig mp3h;
	char ephftp[40];
	// data feed configuration
	// for now, one feed for each type is enough, but might get extended to more?
	char call[10];			// APRS callsign
	char passcode[9];		// APRS passcode
	struct st_feedinfo udpfeed;	// target for AXUDP messages
	struct st_feedinfo tcpfeed;	// target for APRS-IS TCP connections
	struct st_kisstnc kisstnc;	// target for KISS TNC (via TCP, mainly for APRSdroid)
	struct st_mqtt mqtt;
	struct st_sondehub sondehub;
} RDZConfig;


#define MAXSONDE 99

extern int fingerprintValue[];
extern const char *fingerprintText[];

class Sonde
{
private:
public:
	RDZConfig config;
	int fingerprint = 0;
	int currentSonde = 0;
	int nSonde;
	String ipaddr;
	bool isAP;
	// moved to heap, saving space in .bss
	//SondeInfo sondeList[MAXSONDE+1];
	SondeInfo *sondeList;

	Sonde();
	void defaultConfig();
	void setConfig(const char *str);

	void clearSonde();
	void addSonde(float frequency, SondeType type, int active, char *launchsite);
	void nextConfig();
	void nextRxSonde();
	void nextRxFreq(int addkhz);

	/* new interface */
	void setup();
	void receive();
	uint16_t waitRXcomplete();
	/* old and temp interface */
#if 0
	void processRXbyte(uint8_t data);
	int  receiveFrame();
#endif

	SondeInfo *si();

	uint8_t timeoutEvent(SondeInfo *si);
	uint8_t updateState(uint8_t event);

	void updateDisplayPos();
	void updateDisplayPos2();
	void updateDisplayID();
	void updateDisplayRSSI();
	void updateDisplayRXConfig();
	void updateStat();
	void updateDisplayIP();
	void updateDisplay();
	void clearDisplay();

	void setIP(String ip, bool isAP);
};

extern Sonde sonde;

#endif



#ifndef Sonde_h
#define Sonde_h

#include "aprs.h"

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
#define ACT_NONE -1
#define ACT_DISPLAY(n) (n)
#define ACT_DISPLAY_DEFAULT 15
#define ACT_DISPLAY_SPECTRUM 14
#define ACT_DISPLAY_WIFI 13
#define ACT_NEXTSONDE 65
#define ACT_PREVSONDE 64

// 0000nnnn => goto display nnnn
// 01000000 => goto sonde -1
// 01000001 => goto sonde +1

enum SondeType { STYPE_DFM06, STYPE_DFM09, STYPE_RS41, STYPE_RS92 };
extern const char *sondeTypeStr[5];

// Used for interacting with the RX background task
typedef struct st_RXTask {
	// Variables set by Arduino main loop to value >=0 for requesting
	// mode change in RXTask. Will be reset to -1 by RXTask
	int activate;
	int requestSonde;
	// Variables set by RXTask, corresponding to activate
	// and requestSonde
	int mainState;
	int currentSonde;
	int lastSonde;
	// Variable set by RXTask to communicate status to Arduino task
	// via waitRXcomplete function
	uint16_t receiveResult;
	// status variabe set by decoder to indicate something is broken
	// int fifoOverflo;
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

typedef struct st_rdzconfig {
	int button_pin;			// PIN port number menu button (+128 for touch mode)
	int button2_pin;		// PIN port number menu button (+128 for touch mode)
	int touch_thresh;		// Threshold value (0..100) for touch input button
	int led_pout;			// POUT port number of LED (used as serial monitor)
	int oled_sda;			// OLED data pin
	int oled_scl;			// OLED clock pin
	int oled_rst;			// OLED reset pin
	int debug;				// show port and config options after reboot
	int wifi;				// connect to known WLAN 0=skip
	int wifiap;				// enable/disable WiFi AccessPoint mode 0=disable
	int display;			// select display mode (0=default, 1=default, 2=fieldmode)
	int startfreq;			// spectrum display start freq (400, 401, ...)
	int channelbw;			// spectrum channel bandwidth (valid: 5, 10, 20, 25, 50, 100 kHz)	
	int spectrum;			// show freq spectrum for n seconds 0=disable
	int timer;				// show remaining time in spectrum  0=disable
	int marker;				// show freq marker in spectrum  0=disable
	int maxsonde;			// number of max sonde in scan (range=1-99)
	int norx_timeout;		// Time after which rx mode switches to scan mode (without rx signal)
	int noisefloor;			// for spectrum display
	int showafc;			// show afc value in rx screen
	int freqofs;			// frequency offset (tuner config = rx frequency + freqofs) in Hz
	char call[9];			// APRS callsign
	char passcode[9];		// APRS passcode
	struct st_rs41config rs41;	// configuration options specific for RS41 receiver
	struct st_rs92config rs92;
	struct st_dfmconfig dfm;
	// for now, one feed for each type is enough, but might get extended to more?
	struct st_feedinfo udpfeed;	// target for AXUDP messages
	struct st_feedinfo tcpfeed;	// target for APRS-IS TCP connections
} RDZConfig;

typedef struct st_sondeinfo {
        // receiver configuration
	bool active;
        SondeType type;
        float freq;
        // decoded ID
        char id[10];
        bool validID;
	char launchsite[18];		
        // decoded position
        float lat;			// latitude
        float lon;			// longitude
        float alt;			// altitude
        float vs;			// vertical speed
        float hs;			// horizontal speed
	float dir; 			// 0..360
        uint8_t validPos;   // bit pattern for validity of above 6 fields
        // RSSI from receiver
        int rssi;			// signal strength
	int32_t afc;			// afc correction value
	// statistics
	uint8_t rxStat[20];
	uint32_t rxStart;    		// millis() timestamp of continuous rx start
	uint32_t norxStart;		// millis() timestamp of continuous no rx start
	uint32_t viewStart;		// millis() timestamp of viewinf this sonde with current display
	int8_t lastState;		// -1: disabled; 0: norx; 1: rx
} SondeInfo;
// rxStat: 3=undef[empty] 1=timeout[.] 2=errro[E] 3=ok[1] 5=no valid position[°]


#define MAXSONDE 99

class Sonde
{
private:
public:
	RDZConfig config;
	int currentSonde = 0;
	int nSonde;
	// moved to heap, saving space in .bss
	//SondeInfo sondeList[MAXSONDE+1];
	SondeInfo *sondeList;

	Sonde();
	void setConfig(const char *str);

	void clearSonde();
	void addSonde(float frequency, SondeType type, int active, char *launchsite);
	void nextConfig();
	void nextRxSonde();

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

	uint8_t timeoutEvent();
	int updateState(int8_t event);

	void updateDisplayPos();
	void updateDisplayPos2();
	void updateDisplayID();
	void updateDisplayRSSI();
	void updateDisplayRXConfig();
	void updateStat();
	void updateDisplayIP();
	void updateDisplay();
	void updateDisplayScanner();
	void clearDisplay();
	void setIP(const char *ip, bool isAP);
	void clearIP();
};

extern Sonde sonde;

#endif


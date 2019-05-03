
#ifndef Sonde_h
#define Sonde_H

#include "aprs.h"

// RX_TIMEOUT: no header detected
// RX_ERROR: header detected, but data not decoded (crc error, etc.)
// RX_OK: header and data ok
enum RxResult { RX_OK, RX_TIMEOUT, RX_ERROR, RX_UNKNOWN };

enum SondeType { STYPE_DFM06, STYPE_DFM09, STYPE_RS41 };
extern const char *sondeTypeStr[5];

typedef struct st_rdzconfig {
	int button_pin;			// PIN port number menu button (for some boards)
	int led_pout;			// POUT port number of LED (used as serial monitor)
	int oled_sda;			// OLED data pin
	int oled_scl;			// OLED clock pin
	int oled_rst;			// OLED reset pin
	int debug;				// show port and config options after reboot
	int wifi;				// connect to known WLAN 0=skip
	int wifiap;				// enable/disable WiFi AccessPoint mode 0=disable
	int startfreq;			// spectrum display start freq (400, 401, ...)
	int channelbw;			// spectrum channel bandwidth (valid: 5, 10, 20, 25, 50, 100 kHz)	
	int spectrum;			// show freq spectrum for n seconds 0=disable
	int timer;				// show remaining time in spectrum  0=disable
	int marker;				// show freq marker in spectrum  0=disable
	int maxsonde;			// number of max sonde in scan (range=1-99)
	int noisefloor;			// for spectrum display
	char call[9];			// APRS callsign
	char passcode[9];		// APRS passcode
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
	uint8_t rxStat[20];
} SondeInfo;
// rxState: 0=undef[empty] 1=timeout[.] 2=errro[E] 3=ok[1]


#define MAXSONDE 99

class Sonde
{
private:
public:
	RDZConfig config;
	int currentSonde = 0;
	int nSonde;
	SondeInfo sondeList[MAXSONDE+1];

	Sonde();
	void setConfig(const char *str);

	void clearSonde();
	void addSonde(float frequency, SondeType type, int active, char *launchsite);
	void nextConfig();
	void setup();

	int  receiveFrame();
	SondeInfo *si();

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


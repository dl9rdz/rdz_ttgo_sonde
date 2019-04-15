
#ifndef Sonde_h
#define Sonde_H

#include "aprs.h"

// RX_TIMEOUT: no header detected
// RX_ERROR: header detected, but data not decoded (crc error, etc.)
// RX_OK: header and data ok
enum RxResult { RX_OK, RX_TIMEOUT, RX_ERROR };

enum SondeType { STYPE_DFM06, STYPE_DFM09, STYPE_RS41 };
extern const char *sondeTypeStr[5];

typedef struct st_rdzconfig {
	int noisefloor;			// for spectrum display
	char call[9];
	char passcode[9];
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
        // decoded position
        float lat;
        float lon;
        float hei;
        float vs;
        float hs;
	float dir; // 0..360
        uint8_t validPos;   // bit pattern for validity of above 6 fields
        // RSSI from receiver
        int rssi;
	uint8_t rxStat[20];
} SondeInfo;
// rxState: 0=undef[empty] 1=timeout[.] 2=errro[E] 3=ok[1]


#define MAXSONDE 10

class Sonde
{
private:
public:
	RDZConfig config;
	int currentSonde = 0;
	int nSonde;
	SondeInfo sondeList[MAXSONDE+1];

	void clearSonde();
	void addSonde(float frequency, SondeType type, int active);
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
};

extern Sonde sonde;

#endif


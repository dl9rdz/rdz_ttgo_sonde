
#ifndef Sonde_h
#define Sonde_H

// RX_TIMEOUT: no header detected
// RX_ERROR: header detected, but data not decoded (crc error, etc.)
// RX_OK: header and data ok
enum RxResult { RX_OK, RX_TIMEOUT, RX_ERROR };

enum SondeType { STYPE_DFM06, STYPE_DFM09, STYPE_RS41 };
extern const char *sondeTypeStr[5];

typedef struct st_sondeinfo {
        // receiver configuration
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
        bool validPos;
        // RSSI from receiver
        int rssi;
	uint8_t rxStat[20];
} SondeInfo;
// rxState: 0=undef[empty] 1=timeout[.] 2=errro[E] 3=ok[1]


#define MAXSONDE 10

class Sonde
{
private:
	int nSonde;
	int currentSonde = 0;
	SondeInfo sondeList[MAXSONDE+1];
public:
	void clearSonde();
	void addSonde(float frequency, SondeType type);
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
	void setIP(const char *ip);
};

extern Sonde sonde;

#endif


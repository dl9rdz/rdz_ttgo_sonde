

#ifndef Sonde_h
#define Sonde_H

typedef struct st_sondeinfo {
        // receiver configuration
        char type[5];
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
} SondeInfo;

extern SondeInfo si;

class Sonde
{
private:
public:
	void updateDisplayPos();
	void updateDisplayPos2();
	void updateDisplayID();
	void updateDisplayRSSI();
	void updateDisplayRXConfig();
	void updateDisplay();
};

extern Sonde sonde;

#endif


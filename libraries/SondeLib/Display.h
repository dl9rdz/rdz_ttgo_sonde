
#ifndef Display_h
#define Display_h

#define FONT_LARGE 1
#define FONT_SMALL 0


struct DispEntry {
	int8_t y;
	int8_t x;
	int16_t fmt;
	void (*func)(DispEntry *de);
	const char *extra;
};

struct DispInfo {
        DispEntry *de;
        int8_t *actions;
        int16_t *timeouts;
};


class Display {
private:
public:
	void setLayout(DispInfo *layout);
	DispInfo *layout;

	Display();
	static char buf[17];
	static void drawLat(DispEntry *de);
	static void drawLon(DispEntry *de);
	static void drawAlt(DispEntry *de);
	static void drawHS(DispEntry *de);
	static void drawVS(DispEntry *de);
	static void drawID(DispEntry *de);
	static void drawRSSI(DispEntry *de);
	static void drawQS(DispEntry *de);
	static void drawType(DispEntry *de);
	static void drawFreq(DispEntry *de);
	static void drawAFC(DispEntry *de);
	static void drawIP(DispEntry *de);
	static void drawSite(DispEntry *de);
	static void drawTelemetry(DispEntry *de);
	static void drawGPSdist(DispEntry *de);
	static void drawText(DispEntry *de);
	void clearIP();
	void setIP(const char *ip, bool AP);
	void updateDisplayPos();
	void updateDisplayPos2();
	void updateDisplayID();
	void updateDisplayRSSI();
	void updateStat();
	void updateDisplayRXConfig();
	void updateDisplayIP();
	void updateDisplay();

	void setLayout(int layout);
};

extern Display disp;

#endif 


#ifndef Display_h
#define Display_h

#define FONT_LARGE 1
#define FONT_SMALL 0


struct DispEntry {
	uint8_t y;
	uint8_t x;
	uint16_t fmt;
	void (*func)(DispEntry *de);
	char *extra;
};


class Display {
private:
	DispEntry *layout;
	void setLayout(DispEntry *layout);
public:
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


#ifndef Display_h
#define Display_h

#define FONT_LARGE 1
#define FONT_SMALL 0

#include <SPI.h>
#include <TFT_22_ILI9225.h>
#include <U8x8lib.h>


struct DispEntry {
	int8_t y;
	int8_t x;
	int16_t fmt;
	void (*func)(DispEntry *de);
	const char *extra;
};

struct DispInfo {
        DispEntry *de;
        uint8_t *actions;
        int16_t *timeouts;
};

// Now starting towards supporting different Display types / libraries
class RawDisplay {
public:
	virtual void begin() = 0;
	virtual void clear() = 0;
	virtual void setFont(int nr) = 0;
	virtual void drawString(uint8_t x, uint8_t y, const char *s) = 0;
	virtual void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr) = 0;
};

class U8x8Display : public RawDisplay {
private:
	U8X8_SSD1306_128X64_NONAME_SW_I2C *u8x8 = NULL; // initialize later after reading config file

public:
	void begin();
	void clear();
	void setFont(int nr);
        void drawString(uint8_t x, uint8_t y, const char *s);
        void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr);
};

class ILI9225Display : public RawDisplay {
private:
	TFT_22_ILI9225 *tft = NULL; // initialize later after reading config file
	uint8_t yofs=0;

public:
	void begin();
	void clear();
	void setFont(int nr);
        void drawString(uint8_t x, uint8_t y, const char *s);
        void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr);
};

class Display {
private:
	void freeLayouts();
	int allocDispInfo(int entries, DispInfo *d);
	void parseDispElement(char *text, DispEntry *de);

public:
	void initFromFile();

	void setLayout(DispInfo *layout);
	DispInfo *layout;
	static RawDisplay *rdis;

	Display();
	void init();
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
	static void drawGPS(DispEntry *de);
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

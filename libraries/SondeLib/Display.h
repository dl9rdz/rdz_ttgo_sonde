
#ifndef Display_h
#define Display_h

#define FONT_LARGE 1
#define FONT_SMALL 0

#include <SPI.h>
#include <TFT22_ILI9225.h>
#include <U8x8lib.h>


struct DispEntry {
	int8_t y;
	int8_t x;
	int16_t fmt, width;
	uint16_t fg,bg;
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
	virtual void setFont(uint8_t fontindex) = 0;
	virtual void drawString(uint8_t x, uint8_t y, const char *s, int16_t width=-1, uint16_t fg=0xffff, uint16_t bg=0 ) = 0;
	virtual void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr) = 0;
	virtual void welcome() = 0;
	virtual void drawIP(uint8_t x, uint8_t y, int16_t width=-1, uint16_t fg=0xffff, uint16_t bg=0 ) = 0;
};

class U8x8Display : public RawDisplay {
private:
	U8X8 *u8x8 = NULL; // initialize later after reading config file
	int _type;
	const uint8_t **fontlist;
	int nfonts;

public:
	U8x8Display(int type = 0) { _type = type; }
	void begin();
	void clear();
	void setFont(uint8_t fontindex);
        void drawString(uint8_t x, uint8_t y, const char *s, int16_t width=-1, uint16_t fg=0xffff, uint16_t bg=0);
        void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr);
	void welcome();
	void drawIP(uint8_t x, uint8_t y, int16_t width=-1, uint16_t fg=0xffff, uint16_t bg=0);
};

class MY_ILI9225 : public TFT22_ILI9225 {
	using TFT22_ILI9225::TFT22_ILI9225;
public:
	uint16_t drawGFXChar(int16_t x, int16_t y, unsigned char c, uint16_t color);
	void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr);
};

class ILI9225Display : public RawDisplay {
private:
	MY_ILI9225 *tft = NULL; // initialize later after reading config file
	uint8_t yofs=0;
	uint8_t findex=0;


public:
	void begin();
	void clear();
	void setFont(uint8_t fontindex);
        void drawString(uint8_t x, uint8_t y, const char *s, int16_t width=-1, uint16_t fg=0xffff, uint16_t bg=0);
        void drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr);
	void welcome();
	void drawIP(uint8_t x, uint8_t y, int16_t width=-1, uint16_t fg=0xffff, uint16_t bg=0);
};

class Display {
private:
	void freeLayouts();
	int allocDispInfo(int entries, DispInfo *d);
	void parseDispElement(char *text, DispEntry *de);
	int xscale=13, yscale=22;
	int fontsma=0, fontlar=1;
	uint16_t colfg, colbg;
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
	static void drawString(DispEntry *de, const char *str);
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

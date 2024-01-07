#ifndef Display_h
#define Display_h

#define FONT_LARGE 1
#define FONT_SMALL 0

#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <U8x8lib.h>
#include <SPIFFS.h>

#include "posinfo.h"

#define WIDTH_AUTO 9999
struct DispEntry {
	int16_t y;
	int16_t x;
	int16_t fmt, width;
	uint16_t fg,bg;
	void (*func)(DispEntry *de);
	const char *extra;
};

#define GPSUSE_BASE 1
#define GPSUSE_DIST 2
#define GPSUSE_BEARING 4
struct DispInfo {
        DispEntry *de;
        uint8_t *actions;
        int16_t *timeouts;
	const char *label;
	uint8_t usegps;
};

struct StatInfo {
	uint8_t len;
	uint8_t size;
};

struct CircleInfo {  // 3,5=g0NCS,50,ff0000,000033,5,ffff00,4,ffffff
	char type;
	char top,bul,arr; // what to point to with top, bullet, array
	uint16_t fgcol, bgcol;
	uint8_t radius;
	uint8_t brad;
	uint16_t bcol;
	uint8_t awidth;
	uint16_t acol;
};

// Now starting towards supporting different Display types / libraries
class RawDisplay {
public:
	virtual void begin() = 0;
	virtual void clear() = 0;
	virtual void setContrast(uint8_t contrast) = 0;
	virtual void setFont(uint8_t fontindex) = 0;
	virtual void getDispSize(uint8_t *height, uint8_t *width, uint8_t *lineskip, uint8_t *colskip) = 0;
	virtual void drawString(uint16_t x, uint16_t y, const char *s, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0 ) = 0;
	virtual void drawTile(uint16_t x, uint16_t y, uint8_t cnt, uint8_t *tile_ptr) = 0;
	virtual void drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, bool fill) = 0;
	virtual void drawBitmap(uint16_t x1, uint16_t y1, const uint16_t* bitmap, int16_t w, int16_t h) = 0;
	virtual void welcome() = 0;
	virtual void drawIP(uint16_t x, uint16_t y, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0 ) = 0;
	virtual void drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t size, uint8_t *stat, uint16_t fg=0xffff, uint16_t bg=0) = 0;
};

class U8x8Display : public RawDisplay {
private:
	U8X8 *u8x8 = NULL; // initialize later after reading config file
	uint8_t _type;
	const uint8_t **fontlist;
	int nfonts;

public:
	U8x8Display(uint8_t  type = 0) { _type = type; }
	void begin();
	void clear();
	void setContrast(uint8_t contrast);
	void setFont(uint8_t fontindex);
	void getDispSize(uint8_t *height, uint8_t *width, uint8_t *lineskip, uint8_t *colskip);
        void drawString(uint16_t x, uint16_t y, const char *s, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0);
        void drawTile(uint16_t x, uint16_t y, uint8_t cnt, uint8_t *tile_ptr);
	void drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, bool fill);
        void drawBitmap(uint16_t x1, uint16_t y1, const uint16_t* bitmap, int16_t w, int16_t h);
	void welcome();
	void drawIP(uint16_t x, uint16_t y, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0);
        void drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t size, uint8_t *stat, uint16_t fg=0xffff, uint16_t bg=0);
};

typedef Arduino_GFX MY_ILI9225;

class ILI9225Display : public RawDisplay {
private:
	uint8_t yofs=0;
	uint8_t findex=0;
	uint8_t _type;

public:
	MY_ILI9225 *tft = NULL; // initialize later after reading config file
	ILI9225Display(int type = 1) { _type = type; }
	void begin();
	void clear();
	void setContrast(uint8_t contrast);
	void setFont(uint8_t fontindex);
	void getDispSize(uint8_t *height, uint8_t *width, uint8_t *lineskip, uint8_t *colskip);
        void drawString(uint16_t x, uint16_t y, const char *s, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0);
        void drawTile(uint16_t x, uint16_t y, uint8_t cnt, uint8_t *tile_ptr);
	void drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, bool fill);
        void drawBitmap(uint16_t x1, uint16_t y1, const uint16_t* bitmap, int16_t w, int16_t h);
	void welcome();
	void drawIP(uint16_t x, uint16_t y, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0);
        void drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t size, uint8_t *stat, uint16_t fg=0xffff, uint16_t bg=0);
};

class Display {
private:
	void replaceLayouts(DispInfo *newlayout, int nnew);
	int allocDispInfo(int entries, DispInfo *d, char *label);
	void parseDispElement(char *text, DispEntry *de);
	int xscale=13, yscale=22;
	int fontsma=0, fontlar=1;
	uint16_t colfg, colbg;
	static void circ(uint16_t *bm, int16_t w, int16_t x0, int16_t y0, int16_t r, uint16_t fg, boolean fill, uint16_t bg);
	static int countEntries(File f);
	void calcGPS();
	int gpsDist; // -1: invalid
	int gpsDir, gpsBear;   // 0..360; -1: invalid
	boolean gpsCourseOld;
	static const int LINEBUFLEN{ 255 };
	static char lineBuf[LINEBUFLEN];
	static const char *trim(char *s) {
		char *ret = s;
		while(*ret && isspace(*ret)) { ret++; }
		while(1) {
			int lastidx;
			lastidx = strlen(ret)-1;
			if(lastidx>=0 && isspace(ret[lastidx]))
				ret[lastidx] = 0;
			else
				break;
		}
		return ret;
	}
public:
	static int getScreenIndex(int index);
	void initFromFile(int index);

	int layoutIdx;
	DispInfo *layout;

	DispInfo *layouts;
	int nLayouts;
	static RawDisplay *rdis;
	uint16_t dispstate;

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
	static void drawKilltimer(DispEntry *de);
	static void drawGPS(DispEntry *de);
	static void drawText(DispEntry *de);
	static void drawBatt(DispEntry *de);
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
	void dispsavectlON();
	void dispsavectlOFF(int rxactive);
	void setLayout(int layout);
	void setContrast();
};

extern Display disp;

#endif 

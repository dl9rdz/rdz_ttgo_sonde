#ifndef DisplayU8x8_h
#define DisplayU8x8_h

#include "Display.h"
#include <U8x8lib.h>

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
	void drawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
	void welcome();
	void drawIP(uint16_t x, uint16_t y, int16_t width=WIDTH_AUTO, uint16_t fg=0xffff, uint16_t bg=0);
        void drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t size, uint8_t *stat, uint16_t fg=0xffff, uint16_t bg=0);
};

#endif

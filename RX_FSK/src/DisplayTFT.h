#ifndef DisplayTFT_h
#define DisplayTFT_h

#include "Display.h"

class TFTDisplay : public RawDisplay {
private:
	uint8_t yofs=0;
	uint8_t findex=0;
	uint8_t _type;

public:
	Arduino_GFX *tft = NULL; // initialize later after reading config file
	TFTDisplay(int type = 1) { _type = type; }
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

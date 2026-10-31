#include "../features.h"

#define TAG "Display"
#include "logger.h"

#include <U8x8lib.h>
#include "DisplayU8x8.h"
#include "Sonde.h"

extern const char *version_name;
extern const char *version_id;

extern Sonde sonde;

byte myIP_tiles[8*11];
static uint8_t ap_tile[8]={0x00,0x04,0x22,0x92, 0x92, 0x22, 0x04, 0x00};

static const uint8_t font[10][5]={
	0x3E, 0x51, 0x49, 0x45, 0x3E,   // 0
	0x00, 0x42, 0x7F, 0x40, 0x00,   // 1
	0x42, 0x61, 0x51, 0x49, 0x46,   // 2
	0x21, 0x41, 0x45, 0x4B, 0x31,   // 3
	0x18, 0x14, 0x12, 0x7F, 0x10,   // 4
	0x27, 0x45, 0x45, 0x45, 0x39,   // 5
	0x3C, 0x4A, 0x49, 0x49, 0x30,   // 6
	0x01, 0x01, 0x79, 0x05, 0x03,   // 7
	0x36, 0x49, 0x49, 0x49, 0x36,   // 8
	0x06, 0x49, 0x49, 0x29, 0x1E }; // 9;  .=0x40


// ALLFONTS requires 30k extra flash memory... for now there is still enough space :)
//#define ALLFONTS 1
static const uint8_t *fl[] = { 
	u8x8_font_chroma48medium8_r,        // 0 ** default small
	u8x8_font_7x14_1x2_f,               // 1 ** default large
#ifdef ALLFONTS
	u8x8_font_amstrad_cpc_extended_f,   // 2
	u8x8_font_5x7_f,                    // 3
	u8x8_font_5x8_f,                    // 4
	u8x8_font_8x13_1x2_f,               // 5
	u8x8_font_8x13B_1x2_f,              // 6
	u8x8_font_7x14B_1x2_f,              // 7
	u8x8_font_artossans8_r,             // 8
	u8x8_font_artosserif8_r,            // 9
	u8x8_font_torussansbold8_r,         // 10 
	u8x8_font_victoriabold8_r,          // 11
	u8x8_font_victoriamedium8_r,        // 12
	u8x8_font_pressstart2p_f,           // 13
	u8x8_font_pcsenior_f,               // 14
	u8x8_font_pxplusibmcgathin_f,       // 15
	u8x8_font_pxplusibmcga_f,           // 16
	u8x8_font_pxplustandynewtv_f,       // 17
#endif
};


void U8x8Display::begin() {
	LOG_I(TAG, "Init SSD1306 display scl=%d sda=%d rst=%d\n", sonde.config.oled_scl, sonde.config.oled_sda, sonde.config.oled_rst);
	if (_type==2) {
		u8x8 = new U8X8_SH1106_128X64_NONAME_HW_I2C(/* reset=*/ sonde.config.oled_rst, /* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda);
	} else { //__type==0 or anything else
		u8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(/* reset=*/ sonde.config.oled_rst, /* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda);
	} 
	LOG_D(TAG, "calling begin...\n");
	u8x8->begin();
	if(sonde.config.tft_orient==3) u8x8->setFlipMode(true);
	if(sonde.config.dispcontrast>=0) u8x8->setContrast(sonde.config.dispcontrast);
	LOG_D(TAG, "setup finishing...\n");

	fontlist = fl;
	nfonts = sizeof(fl)/sizeof(uint8_t *);
	LOG_I(TAG, "Size of font list is %d\n", nfonts);
}

void U8x8Display::clear() {
	u8x8->clear();
}

void U8x8Display::setContrast(uint8_t contrast) {
	u8x8->setContrast(contrast);
}


// For u8x8 oled display: 0=small font, 1=large font 7x14
void U8x8Display::setFont(uint8_t fontindex) {
	if(fontindex>=nfonts) fontindex=0; // prevent overflow
	u8x8->setFont( fontlist[fontindex] );
}

void U8x8Display::getDispSize(uint8_t *height, uint8_t *width, uint8_t *lineskip, uint8_t *colskip) {
	// TODO: maybe we should decided depending on font size (single/double?)
	if(height) *height = 8;
	if(width) *width = 16;
	if(lineskip) *lineskip = 1;
	if(colskip) *colskip = 1;
}

void U8x8Display::drawString(uint16_t x, uint16_t y, const char *s, int16_t width, uint16_t fg, uint16_t bg) {
	char buf[50];
	utf2latin15(s, buf, 50);
	if(width!=WIDTH_AUTO && width>0) {
		for(int l = strlen(buf); l<width; l++) {
			buf[l] = ' ';
		}
		buf[width] = 0;
	}
	if(width<0) {
		int l = strlen(buf);
		memset(buf, ' ', -width-l);
		utf2latin15(s, buf+l, 50-l);
	}
	u8x8->drawString(x, y, buf);
}

void U8x8Display::drawTile(uint16_t x, uint16_t y, uint8_t cnt, uint8_t *tile_ptr) {
	u8x8->drawTile(x, y, cnt, tile_ptr);
}

void U8x8Display::drawBitmap(uint16_t x1, uint16_t y1, const uint16_t* bitmap, int16_t w, int16_t h) {
	// not supported
}
void U8x8Display::drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, bool fill) {
	// not supported (yet)
}

void U8x8Display::drawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
	// not supported
}

void U8x8Display::welcome() {
	u8x8->clear();
	setFont(FONT_LARGE);
	drawString(8 - strlen(version_name) / 2, 0, version_name);
	drawString(8 - strlen(version_id) / 2, 2, version_id);
	setFont(FONT_SMALL);
	drawString(0, 4, "RS41/92,DFM,Mx0");
	drawString(0, 6, "by Hansi, DL9RDZ");
}

static String previp;
void U8x8Display::drawIP(uint16_t x, uint16_t y, int16_t width, uint16_t fg, uint16_t bg) {
	if(!previp.equals(sonde.ipaddr)) {
		// ip address has changed
		// create tiles
		memset(myIP_tiles, 0, 11*8);
		int len = sonde.ipaddr.length();
		const char *ip = sonde.ipaddr.c_str();
		int pix = (len-3)*6+6;
		int tp = 80-pix+8;
		if(sonde.isAP) memcpy(myIP_tiles+(tp<16?0:8), ap_tile, 8);
		for(int i=0; i<len; i++) {
			if(ip[i]=='.') { myIP_tiles[tp++]=0x40; myIP_tiles[tp++]=0x00; }
			else {
				int idx = ip[i]-'0';
				memcpy(myIP_tiles+tp, &font[idx], 5);
				myIP_tiles[tp+5] = 0;
				tp+=6;
			}
		}
		while(tp<8*10) { myIP_tiles[tp++]=0; }
		previp = sonde.ipaddr;
	}
	// draw tiles
	u8x8->drawTile(x, y, 11, myIP_tiles);
}

// len must be multiple of 2, size is fixed for u8x8 display
void U8x8Display::drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t /*size*/, uint8_t *stat, uint16_t fg, uint16_t bg) {
	for(int i=0; i<len; i+=2) {
		uint8_t tile[8];
		*(uint32_t *)(&tile[0]) = *(uint32_t *)(&(stattiles[stat[i]]));
		*(uint32_t *)(&tile[4]) = *(uint32_t *)(&(stattiles[stat[i+1]]));
		drawTile(x+i/2, y, 1, tile);
	}
}


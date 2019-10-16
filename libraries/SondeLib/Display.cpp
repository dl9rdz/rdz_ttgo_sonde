#include <U8x8lib.h>
#include <U8g2lib.h>
#include <SPIFFS.h>

#include <MicroNMEA.h>
#include "Display.h"
#include "Sonde.h"

extern const char *version_name;
extern const char *version_id;

#include <../fonts/FreeMono9pt7b.h>
#include <../fonts/FreeMono12pt7b.h>
#include <../fonts/FreeSans9pt7b.h>
#include <../fonts/FreeSans12pt7b.h>
#include <../fonts/Picopixel.h>

extern Sonde sonde;

extern MicroNMEA nmea;

SPIClass spiDisp(HSPI);

const char *sondeTypeStr[5] = { "DFM6", "DFM9", "RS41", "RS92" };

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
  0x06, 0x49, 0x39, 0x29, 0x1E }; // 9;  .=0x40


static unsigned char kmh_tiles[] U8X8_PROGMEM = {
   0x1F, 0x04, 0x0A, 0x11, 0x00, 0x1F, 0x02, 0x04, 0x42, 0x3F, 0x10, 0x08, 0xFC, 0x22, 0x20, 0xF8
   };
static unsigned char ms_tiles[] U8X8_PROGMEM = {
   0x1F, 0x02, 0x04, 0x02, 0x1F, 0x40, 0x20, 0x10, 0x08, 0x04, 0x12, 0xA8, 0xA8, 0xA4, 0x40, 0x00
   };
static unsigned char stattiles[5][4] =  {
   0x00, 0x1F, 0x00, 0x00 ,   // | == ok
   0x00, 0x10, 0x10, 0x00 ,   // . == no header found
   0x1F, 0x15, 0x15, 0x00 ,   // E == decode error
   0x00, 0x00, 0x00, 0x00 ,   // ' ' == unknown/unassigned
   0x07, 0x05, 0x07, 0x00 };  // ° = rx ok, but no valid position


//static uint8_t halfdb_tile[8]={0x80, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00};

static uint8_t halfdb_tile1[8]={0x00, 0x38, 0x28, 0x28, 0x28, 0xC8, 0x00, 0x00};
static uint8_t halfdb_tile2[8]={0x00, 0x11, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00};

//static uint8_t empty_tile[8]={0x80, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00};

static uint8_t empty_tile1[8]={0x00, 0xF0, 0x88, 0x48, 0x28, 0xF0, 0x00, 0x00};
static uint8_t empty_tile2[8]={0x00, 0x11, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00};

//static uint8_t gps_tile[8]={0x3E, 0x77, 0x63, 0x77, 0x3E, 0x1C, 0x08, 0x00};
static uint8_t gps_tile[8]={0x00, 0x0E, 0x1F, 0x3B, 0x71, 0x3B, 0x1F, 0x0E};
static uint8_t nogps_tile[8]={0x41, 0x22, 0x14, 0x08, 0x14, 0x22, 0x41, 0x00};

static uint8_t deg_tile[8]={0x00, 0x06,0x09, 0x09, 0x06, 0x00, 0x00, 0x00};


/* Description of display layouts.
 * for each display, the content is described by a DispEntry structure
 * timeout values are in milliseconds, for view activ, rx signal present, no rx signal present
 * for each displey, actions (switching to different sonde or different view) can be defined
 * based on key presses or on expired timeouts
 */
DispEntry searchLayout[] = {
	{0, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawText, "Scan:"},
	{0, 8, FONT_LARGE, -1, 0xFFFF, 0, disp.drawType, NULL},
	{3, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawFreq, " MHz"},
	{5, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawSite, NULL},
	{7, 5, 0, -1, 0xFFFF, 0, disp.drawIP, NULL},	
	{-1, -1, -1, 0, 0, 0, NULL, NULL},
};
int16_t searchTimeouts[] = { -1, 0, 0 };
uint8_t searchActions[] = {
	ACT_NONE,
	ACT_DISPLAY_DEFAULT, ACT_NONE, ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_NONE, ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_DISPLAY_DEFAULT, ACT_NEXTSONDE};
DispEntry legacyLayout[] = {
	{0, 5, FONT_SMALL, -1, 0xFFFF, 0, disp.drawFreq, " MHz"},
	{1, 8, FONT_SMALL, -1, 0xFFFF, 0, disp.drawAFC, NULL},
	{0, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawType, NULL},
	{1, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawID, NULL},
	{2, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawLat, NULL},
	{4, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawLon, NULL},
	{2, 10, FONT_SMALL, -1, 0xFFFF, 0, disp.drawAlt, NULL},
	{3, 10, FONT_SMALL, -1, 0xFFFF, 0, disp.drawHS, NULL},
	{4, 9, FONT_SMALL, -1, 0xFFFF, 0, disp.drawVS, NULL},
	{6, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawRSSI, NULL},
	{6, 7, 0, -1, 0xFFFF, 0, disp.drawQS, NULL},
	{7, 5, 0, -1, 0xFFFF, 0, disp.drawIP, NULL},	
	{-1, -1, -1, 0, 0, 0, NULL, NULL},
};
int16_t legacyTimeouts[] = { -1, -1, 20000 };
uint8_t legacyActions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(2), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_DISPLAY(0)};
DispEntry fieldLayout[] = {
	{2, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawLat, NULL},
	{4, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawLon, NULL},
	{3, 10, FONT_SMALL, -1, 0xFFFF, 0, disp.drawHS, NULL},
	{4, 9, FONT_SMALL, -1, 0xFFFF, 0, disp.drawVS, NULL},
	{0, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawID, NULL},
	{6, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawAlt, NULL},
	{6, 7, 0, -1, 0xFFFF, 0, disp.drawQS, NULL},
	{-1, -1, -1, 0, 0, 0, NULL, NULL},
};
int16_t fieldTimeouts[] = { -1, -1, -1 };
uint8_t fieldActions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(4), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_NONE};
DispEntry field2Layout[] = {
	{2, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawLat, NULL},
	{4, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawLon, NULL},
	{1, 12, FONT_SMALL, -1, 0xFFFF, 0, disp.drawType, NULL},
	{0, 9, FONT_SMALL, -1, 0xFFFF, 0, disp.drawFreq, ""},
	{3, 10, FONT_SMALL, -1, 0xFFFF, 0, disp.drawHS, NULL},
	{4, 9, FONT_SMALL, -1, 0xFFFF, 0, disp.drawVS, NULL},
	{0, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawID, NULL},
	{6, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawAlt, NULL},
	{6, 7, 0, -1, 0xFFFF, 0, disp.drawQS, NULL},
	{-1, -1, -1, 0, 0, 0, NULL, NULL},
};
uint8_t field2Actions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(1), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_NONE};
DispEntry gpsLayout[] = {
	{0, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawID, NULL},
	{2, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawLat, NULL},
	{3, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawLon, NULL},
	{4, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawAlt, NULL},
	{6, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawGPS, "V"},
	//{6, 1, FONT_SMALL, disp.drawGPS, "A"},
	//{6, 8, FONT_SMALL, disp.drawGPS, "O"},
	{7, 0, FONT_SMALL, -1, 0xFFFF, 0, disp.drawGPS, "D"},
	{7, 8, FONT_SMALL, -1, 0xFFFF, 0, disp.drawGPS, "I"},
	{-1, -1, -1, 0, 0, 0, NULL, NULL},
};
uint8_t gpsActions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(1), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_NONE};

DispInfo staticLayouts[5] = {
  { searchLayout, searchActions, searchTimeouts },
  { legacyLayout, legacyActions, legacyTimeouts },
  { fieldLayout, fieldActions, fieldTimeouts },
  { field2Layout, field2Actions, fieldTimeouts },
  { gpsLayout, gpsActions, fieldTimeouts } };

DispInfo *layouts = staticLayouts;

/////////////// Wrapper code for various display

// ALLFONTS requires 30k extra flash memory... for now there is still enough space :)
#define ALLFONTS 1
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
	Serial.printf("Init SSD1306 display %d %d\n", sonde.config.oled_scl, sonde.config.oled_sda);
	//u8x8 = new U8X8_SSD1306_128X64_NONAME_SW_I2C(/* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda, /* reset=*/ sonde.config.oled_rst); // Unbuffered, basic graphics, software I2C
	if (_type==2) {
               	u8x8 = new U8X8_SH1106_128X64_NONAME_HW_I2C(/* reset=*/ sonde.config.oled_rst, /* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda); // Unbuffered, basic graphics, software I2C
	} else { //__type==0 or anything else
		u8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(/* reset=*/ sonde.config.oled_rst, /* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda); // Unbuffered, basic graphics, software I2C
	} 
	u8x8->begin();

	fontlist = fl;
	nfonts = sizeof(fl)/sizeof(uint8_t *);
	Serial.printf("Size of font list is %d\n", nfonts);
}

void U8x8Display::clear() {
	u8x8->clear();
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

void U8x8Display::drawString(uint8_t x, uint8_t y, const char *s, int16_t width, uint16_t fg, uint16_t bg) {
	u8x8->drawString(x, y, s);
}

void U8x8Display::drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr) {
	u8x8->drawTile(x, y, cnt, tile_ptr);
}
void U8x8Display::welcome() {
	u8x8->clear();
  	setFont(FONT_LARGE);
  	drawString(8 - strlen(version_name) / 2, 0, version_name);
  	drawString(8 - strlen(version_id) / 2, 2, version_id);
  	setFont(FONT_SMALL);
	drawString(0, 4, "RS41,RS92,DFM6/9");
  	drawString(0, 6, "by Hansi, DL9RDZ");
}

static String previp;
void U8x8Display::drawIP(uint8_t x, uint8_t y, int16_t width, uint16_t fg, uint16_t bg) {
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


const GFXfont *gfl[] = {
	&FreeMono9pt7b,		// 3
	&FreeMono12pt7b,	// 4
	&FreeSans9pt7b,		// 5
	&FreeSans12pt7b,	// 6
	&Picopixel,             // 7
};
struct gfxoffset_t {
	uint8_t yofs, yclear;
};
// obtained as max offset from font (last column) and maximum height (3rd column) in glyphs
// first value: offset: max offset from font glyphs (last column * (-1))   (check /, \, `, $)`
// yclear:max height: max of (height in 3rd column) + (yofs + 6th column)  (check j)
const struct gfxoffset_t gfxoffsets[]={
	{ 11, 15 },  // 13+11-9 "j"
	{ 15, 20 },  // 19+15-14
        { 13, 18 },  // 17+13-12 "j" 
        { 17, 23 }, // 23+17-17
        {  4, 6},       // 6+4-4
};
static int ngfx = sizeof(gfl)/sizeof(GFXfont *);

	
#define TFT_LED 0 // 0 if wired to +5V directly
#define TFT_BRIGHTNESS 100 // Initial brightness of TFT backlight (optional)

void ILI9225Display::begin() {
	tft = new MY_ILI9225(sonde.config.oled_rst, sonde.config.tft_rs, sonde.config.tft_cs,
			sonde.config.oled_sda, sonde.config.oled_scl, TFT_LED, TFT_BRIGHTNESS);
        tft->begin(spiDisp);
	tft->setOrientation(1);
}

void ILI9225Display::clear() {
        tft->clear();
}

// for now, 0=small=FreeSans9pt7b, 1=large=FreeSans18pt7b
void ILI9225Display::setFont(uint8_t fontindex) {
	findex = fontindex;
	switch(fontindex) {
	case 0: tft->setFont(Terminal6x8); break;
	case 1: tft->setFont(Terminal11x16); break;
	case 2: tft->setFont(Terminal12x16); break;
	default: tft->setGFXFont(gfl[fontindex-3]);
	}
}

void ILI9225Display::getDispSize(uint8_t *height, uint8_t *width, uint8_t *lineskip, uint8_t *colskip) {
	if(height) *height = 176;
	if(width) *width = 220;
	switch(findex) {
	case 0:
		if(lineskip) *lineskip=10;
		if(colskip) *colskip=8;
		break;
	case 1:
		if(lineskip) *lineskip=18;
		if(colskip) *colskip=13;
		break;
	case 2:
		if(lineskip) *lineskip=18;
		if(colskip) *colskip=14;
		break;
	default: // get size from GFX Font
	{
		int16_t w,h,a;
		tft->getGFXCharExtent('|',&w,&h,&a);
		if(lineskip) *lineskip = h+2;
		tft->getGFXCharExtent('A',&w,&h,&a);
		if(colskip) *colskip = w+2; // just an approximation
	}
	}
}

void ILI9225Display::drawString(uint8_t x, uint8_t y, const char *s, int16_t width, uint16_t fg, uint16_t bg) {
	int16_t w,h;
	boolean alignright=false;
	Serial.printf("drawString: width=%d\n", width);
	if(findex<3) {  // standard font
		//////////////tft->drawText(x...);
		Serial.printf("Simple Text %s at %d,%d [%d]\n", s, x, y, width);
		tft->drawText(x, y, s, fg);
		return;
	}
	// GFX font
	if(width<0) {
		width = -width;
		alignright = true;
	}
	if(width==WIDTH_AUTO || alignright) {
		tft->getGFXTextExtent(s, x, y + gfxoffsets[findex-3].yofs, &w, &h);
		if(width==WIDTH_AUTO) width=w;
	}

	if(findex-3>=ngfx) findex=3;
	tft->fillRectangle(x, y, x + width, y + gfxoffsets[findex-3].yclear, bg);
	Serial.printf("GFX Text %s at %d,%d+%d in color %x, width=%d\n", s, x, y, gfxoffsets[findex-3].yofs, fg, width);
	if(alignright) {
        	tft->drawGFXText(x + width - w, y + gfxoffsets[findex-3].yofs, s, fg);
	} else {
        	tft->drawGFXText(x, y + gfxoffsets[findex-3].yofs, s, fg);
	}
}

void ILI9225Display::drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr) {
	tft->drawTile(x, 2*y, cnt, tile_ptr);
#if 0
	int i,j;
	tft->startWrite();
	for(int i=0; i<cnt*8; i++) {
		uint8_t v = tile_ptr[i];
		for(j=0; j<8; j++) {
			tft->drawPixel(8*x+i, 8*y+j, (v&0x01) ? COLOR_GREEN:COLOR_BLUE);
			v >>= 1;
		}
	}
	tft->endWrite();
	//tft->drawBitmap(x*8, y*8, tile_ptr, cnt*8, 8, COLOR_RED, COLOR_BLUE);
        //???u8x8->drawTile(x, y, cnt, tile_ptr);
#endif
}

void ILI9225Display::welcome() {
	tft->clear();
        setFont(6);
        drawString(0, 0*22, version_name, WIDTH_AUTO, 0xff77);
        setFont(5);
        drawString(0, 1*22, "RS41,RS92,DFM6/9");
        drawString(0, 3*22, version_id);
        drawString(0, 5*22, "by Hansi, DL9RDZ");

}

void ILI9225Display::drawIP(uint8_t x, uint8_t y, int16_t width, uint16_t fg, uint16_t bg) {
	setFont(0);
	char buf[20];
	snprintf(buf, 20, "%c %s", sonde.isAP?'A':' ', sonde.ipaddr.c_str());
	drawString(x, y, buf, width, fg, bg);		
}

#include <pgmspace.h>
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))

void MY_ILI9225::drawTile(uint8_t x, uint8_t y, uint8_t cnt, uint8_t *tile_ptr) {
        int i,j;
        startWrite();
        for(i=0; i<cnt*8; i++) {
                uint8_t v = tile_ptr[i];
                for(j=0; j<8; j++) {
                        drawPixel(8*x+i, 8*y+j, (v&0x01) ? COLOR_GREEN:COLOR_BLUE);
                        v >>= 1;
                }
        }
        endWrite();
}


uint16_t MY_ILI9225::drawGFXChar(int16_t x, int16_t y, unsigned char c, uint16_t color) {

    c -= (uint8_t)pgm_read_byte(&gfxFont->first);
    GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
    uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t  w  = pgm_read_byte(&glyph->width),
             h  = pgm_read_byte(&glyph->height),
             xa = pgm_read_byte(&glyph->xAdvance);
    int8_t   xo = pgm_read_byte(&glyph->xOffset),
             yo = pgm_read_byte(&glyph->yOffset);
    uint8_t  xx, yy, bits = 0, bit = 0;

    // Add character clipping here one day

    startWrite();
    for(yy=0; yy<h; yy++) {
        for(xx=0; xx<w; xx++) {
            if(!(bit++ & 7)) {
                bits = pgm_read_byte(&bitmap[bo++]);
            }
            if(bits & 0x80) {
                drawPixel(x+xo+xx, y+yo+yy, color);
            } else {
                drawPixel(x+xo+xx, y+yo+yy, COLOR_YELLOW); //color);
	    }
            bits <<= 1;
        }
    }
    endWrite();

    return (uint16_t)xa;
}
///////////////


char Display::buf[17];

RawDisplay *Display::rdis = NULL;

//TODO: maybe merge with initFromFile later?
void Display::init() {
	Serial.printf("disptype is %d\n",sonde.config.disptype);
	if(sonde.config.disptype==1) {
		rdis = new ILI9225Display();
	} else {
		rdis = new U8x8Display(sonde.config.disptype);
	}
	Serial.println("Display created");
	rdis->begin();
	delay(100);
	Serial.println("Display initialized");
	rdis->clear();
}


Display::Display() {
	setLayout(0);
}

#define MAXSCREENS 10
#define DISP_ACTIONS_N 12
#define DISP_TIMEOUTS_N 3

void Display::freeLayouts() {
	if(layouts==staticLayouts) return;
	DispInfo *old = layouts;
	layouts=staticLayouts;
	setLayout(0);
	delay(500);  // Make it unlikely that anyone else is still using previous layouts
	for(int i=0; i<MAXSCREENS; i++) {
		if(old[i].de) free(old[i].de);
	}
	free(old);
}

int Display::allocDispInfo(int entries, DispInfo *d)
{
	int totalsize = (entries+1)*sizeof(DispEntry) + DISP_ACTIONS_N*sizeof(uint8_t) + DISP_TIMEOUTS_N * sizeof(int16_t);
	char *mem = (char *)malloc(totalsize);
	if(!mem) return -1;
	memset (mem, 0, totalsize);

	d->de = (DispEntry *)mem;
	mem += (entries+1) * sizeof(DispEntry);

	d->actions = (uint8_t *)mem;
	mem += DISP_ACTIONS_N * sizeof(uint8_t);
	d->actions[0] = ACT_NONE;

	d->timeouts = (int16_t *)mem;
	Serial.printf("allocated %d bytes (%d entries) for %p (addr=%p)\n", totalsize, entries, d, d->de);
	return 0;
}

void Display::parseDispElement(char *text, DispEntry *de)
{
	char type = *text;
	if(type>='A'&&type<='Z') {
		type += 32;  // lc
		de->fmt = fontlar;
	} else {
		de->fmt = fontsma;
	}
	de->fg = colfg;
	de->bg = colbg;
	switch(type) {
	case 'l':
		de->func = disp.drawLat; break;
	case 'o':
		de->func = disp.drawLon; break;
	case 'a':
		de->func = disp.drawAlt; break;
	case 'h':
		de->func = disp.drawHS; 
		de->extra = text[1]?strdup(text+1):NULL; break;
	case 'v':
		de->func = disp.drawVS; 
		de->extra = text[1]?strdup(text+1):NULL; break;
	case 'i':
		de->func = disp.drawID;
		de->extra = strdup(text+1);
		break;
	case 'q':
		de->func = disp.drawQS; break;
	case 't':
		de->func = disp.drawType; break;
	case 'c':
		de->func = disp.drawAFC; break;
	case 'f':
		de->func = disp.drawFreq;
		de->extra = strdup(text+1);
		Serial.printf("parsing 'f' entry: extra is '%s'\n", de->extra);
		break;
	case 'n':
		de->func = disp.drawIP; break;
	case 's':
		de->func = disp.drawSite; break;
	case 'g':
		de->func = disp.drawGPS;
		de->extra = strdup(text+1);
		Serial.printf("parsing 'g' entry: extra is '%s'\n", de->extra);
		break;
	case 'r':
		de->func = disp.drawRSSI; break;
	case 'x':
		de->func = disp.drawText;
		de->extra = strdup(text+1);
		break;
	default:
		Serial.printf("Unknown element: %c\n", type);
		break;
	}
}

static uint8_t ACTION(char c) {
	switch(c) {
	case 'D': 
		return ACT_DISPLAY_DEFAULT;
	case 'F':
		return ACT_DISPLAY_SPECTRUM;
	case 'W':
		return ACT_DISPLAY_WIFI;
	case '+':
		return ACT_NEXTSONDE;
	case '#':	
		return ACT_NONE;
	default:
		if(c>='0'&&c<='9')
		return ACT_DISPLAY(c-'0');
	}
	return ACT_NONE;
}

void Display::initFromFile() {
	File d = SPIFFS.open("/screens.txt", "r");
	if(!d) return;

	freeLayouts();
	DispInfo *layouts = (DispInfo *)malloc(MAXSCREENS * sizeof(DispInfo));
	if(!layouts) {
		layouts = staticLayouts;
		return;
	}
	memset(layouts, 0, MAXSCREENS * sizeof(DispInfo));

	// default color
	colfg = 0xffff; // white; only used for ILI9225
	colbg = 0;  // black; only used for ILI9225
	int idx = -1;
	int what = -1;
	int entrysize;
	Serial.printf("Reading from /screens.txt. available=%d\n",d.available());
	while(d.available()) {
		const char *ptr;
		String line = d.readStringUntil('\n');
		line.trim();
		const char *s = line.c_str();
		Serial.printf("Line: '%s'\n", s);
		if(*s == '#') continue;  // ignore comments
		switch(what) {
		case -1:	// wait for start of screen (@)
			{
			if(*s != '@') {
				Serial.printf("Illegal start of screen: %s\n", s);
				continue;
			}
			char *num = strchr(s, ':');
			if(!num) {
				Serial.println("Line missing size length indication");
				continue;
			}
			entrysize = atoi(num+1);
			Serial.printf("Reading entry with %d elements\n", entrysize);
			idx++;
			int res = allocDispInfo(entrysize, &layouts[idx]);
			if(res<0) {
				Serial.println("Error allocating memory for disp info");
				continue;
			}
			what = 0;
			}
			break;
		default:	// parse content... (additional data or line `what`)
			if(strncmp(s,"timer=",6)==0) {  // timer values
				sscanf(s+6, "%hd,%hd,%hd", layouts[idx].timeouts, layouts[idx].timeouts+1, layouts[idx].timeouts+2);
				Serial.printf("timer values: %d, %d, %d\n", layouts[idx].timeouts[0], layouts[idx].timeouts[1], layouts[idx].timeouts[2]);
			} else if(strncmp(s, "key1action=",11)==0) { // key 1 actions
				char c1,c2,c3,c4;
				sscanf(s+11, "%c,%c,%c,%c", &c1, &c2, &c3, &c4);
				layouts[idx].actions[1] = ACTION(c1);
				layouts[idx].actions[2] = ACTION(c2);
				layouts[idx].actions[3] = ACTION(c3);
				layouts[idx].actions[4] = ACTION(c4);
			} else if(strncmp(s, "key2action=",11)==0) { // key 2 actions
				char c1,c2,c3,c4;
				sscanf(s+11, "%c,%c,%c,%c", &c1, &c2, &c3, &c4);
				layouts[idx].actions[5] = ACTION(c1);
				layouts[idx].actions[6] = ACTION(c2);
				layouts[idx].actions[7] = ACTION(c3);
				layouts[idx].actions[8] = ACTION(c4);
				Serial.printf("parsing key2action: %c %c %c %c\n", c1, c2, c3, c4);
			} else if(strncmp(s, "timeaction=",11)==0) { // timer actions
				char c1,c2,c3;
				sscanf(s+11, "%c,%c,%c", &c1, &c2, &c3);
				layouts[idx].actions[9] = ACTION(c1);
				layouts[idx].actions[10] = ACTION(c2);
				layouts[idx].actions[11] = ACTION(c3);
			} else if(strncmp(s, "fonts=",6)==0) { // change font
				sscanf(s+6, "%d,%d", &fontsma, &fontlar);
			} else if(strncmp(s, "scale=",6)==0) { // change line->pixel scaling for ILI9225 display
				sscanf(s+6, "%d,%d", &yscale, &xscale);
			} else if(strncmp(s, "color=",6)==0) { //
				int res;
				uint32_t fg,bg;
				res=sscanf(s+6, "%" SCNx32 ",%" SCNx32, &fg, &bg);
				colfg = (fg>>19) << 11 | ((fg>>10)&0x3F) << 5 | ((fg>>3)&0x1F);
				if(res==2) {
					colbg = (bg>>19) << 11 | ((bg>>10)&0x3F) << 5 | ((bg>>3)&0x1F);
				}
			} else if( (ptr=strchr(s, '=')) ) {  // one line with some data...
				int x,y,w,n;
				char text[30];
				n=sscanf(s, "%d,%d,%d", &y, &x, &w);
				sscanf(ptr+1, "%30[^\r\n]", text);
				if(sonde.config.disptype==1) { x*=xscale; y*=yscale; w*=xscale; }
				layouts[idx].de[what].x = x;
				layouts[idx].de[what].y = y;
				layouts[idx].de[what].width = n>2 ? w : WIDTH_AUTO;
				parseDispElement(text, layouts[idx].de+what);
				Serial.printf("entry at %d,%d width=%d font %d, color=%x,%x\n", x, y, layouts[idx].de[what].width, layouts[idx].de[what].fmt,
					layouts[idx].de[what].fg, layouts[idx].de[what].bg);
				what++;
				layouts[idx].de[what].func = NULL;
			} else {
				for(int i=0; i<12; i++) {
					Serial.printf("action %d: %d\n", i, (int)layouts[idx].actions[i]);
				}
 				what=-1;
			}
			break;
		}
	}
	::layouts = layouts;
	setLayout(0);
}

void Display::setLayout(int layoutIdx) {
	layout = &layouts[layoutIdx];
}

void Display::drawString(DispEntry *de, const char *str) {
	rdis->drawString(de->x, de->y, str, de->width, de->fg, de->bg);
}

void Display::drawLat(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->validPos) {
	   drawString(de,"<?""?>      ");
	   return;
	}
	snprintf(buf, 16, "%2.5f", sonde.si()->lat);
	drawString(de,buf);
}
void Display::drawLon(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->validPos) {
	   drawString(de,"<?""?>      ");
	   return;
	}
	snprintf(buf, 16, "%2.5f", sonde.si()->lon);
	drawString(de,buf);
}
void Display::drawAlt(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->validPos) {
	   drawString(de,"     ");
	   return;
	}
	snprintf(buf, 16, sonde.si()->alt>=1000?"   %5.0fm":"   %3.1fm", sonde.si()->alt);
	drawString(de,buf+strlen(buf)-6);
}
void Display::drawHS(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->validPos) {
	   drawString(de,"     ");
	   return;
	}
	snprintf(buf, 16, sonde.si()->hs>99?" %3.0f":" %2.1f", sonde.si()->hs);
	if(de->extra) { strcat(buf, de->extra); }
	drawString(de,buf+strlen(buf)-4- (de->extra?strlen(de->extra):0) );
	if(!de->extra) rdis->drawTile(de->x+4,de->y,2,kmh_tiles);
}
void Display::drawVS(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->validPos) {
	   drawString(de,"     ");
	   return;
	}
	snprintf(buf, 16, "  %+2.1f", sonde.si()->vs);
	Serial.printf("drawVS: extra is %s width=%d\n", de->extra?de->extra:"<null>", de->width);
	if(de->extra) { strcat(buf, de->extra); }
	drawString(de, buf+strlen(buf)-5- (de->extra?strlen(de->extra):0) );
	if(!de->extra) rdis->drawTile(de->x+5,de->y,2,ms_tiles);
}
void Display::drawID(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->validID) {
		drawString(de, "nnnnnnnn ");
		return;
	}
	// TODO: handle DFM6 IDs

	if(!de->extra || de->extra[0]=='s') {
		// real serial number, as printed on sonde
		drawString(de, sonde.si()->id);
	} else if (de->extra[0]=='a') {
		// autorx sonde number ("DF9" and last 6 digits of real serial number
		strcpy(buf, sonde.si()->id);
		memcpy(buf, "DF9", 3);
		drawString(de, buf);
	} else {
		// dxlAPRS sonde number (DF6 (why??) and 5 last digits of serial number as hex number
		uint32_t id = atoi(sonde.si()->id);
		id = id&0xfffff;
		snprintf(buf, 16, "DF6%05X", id);
		drawString(de, buf);
	}
}
void Display::drawRSSI(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(sonde.config.disptype!=1) {
		snprintf(buf, 16, "-%d   ", sonde.si()->rssi/2);
		int len=strlen(buf)-3;
		Serial.printf("drawRSSI: %d %d %d (%d)[%d]\n", de->y, de->x, sonde.si()->rssi/2, sonde.currentSonde, len);
		buf[5]=0;
		drawString(de,buf);
		rdis->drawTile(de->x+len, de->y, 1, (sonde.si()->rssi&1)?halfdb_tile1:empty_tile1);
		rdis->drawTile(de->x+len, de->y+1, 1, (sonde.si()->rssi&1)?halfdb_tile2:empty_tile2);
	} else { // special for 2" display
		snprintf(buf, 16, "-%d.%c  ", sonde.si()->rssi/2, (sonde.si()->rssi&1)?'5':'0');
		drawString(de,buf);
	}
}
void Display::drawQS(DispEntry *de) {
	uint8_t *stat = sonde.si()->rxStat;
	for(int i=0; i<18; i+=2) {
	        uint8_t tile[8];
	        *(uint32_t *)(&tile[0]) = *(uint32_t *)(&(stattiles[stat[i]]));
	        *(uint32_t *)(&tile[4]) = *(uint32_t *)(&(stattiles[stat[i+1]]));
	        rdis->drawTile(de->x+i/2, de->y, 1, tile);
	}
}
void Display::drawType(DispEntry *de) {
	rdis->setFont(de->fmt);
        drawString(de, sondeTypeStr[sonde.si()->type]);
}
void Display::drawFreq(DispEntry *de) {
	rdis->setFont(de->fmt);
        snprintf(buf, 16, "%3.3f%s", sonde.si()->freq, de->extra?de->extra:"");
        drawString(de, buf);
}
void Display::drawAFC(DispEntry *de) {
 	if(!sonde.config.showafc) return;
	rdis->setFont(de->fmt);
	if(sonde.si()->afc==0) { strcpy(buf, "        "); }
	else { snprintf(buf, 15, "     %+3.2fk", sonde.si()->afc*0.001); }
        drawString(de, buf+strlen(buf)-8);
}
void Display::drawIP(DispEntry *de) {
	rdis->drawIP(de->x, de->y, de->width, de->fg, de->bg);
}
void Display::drawSite(DispEntry *de) {
        rdis->setFont(de->fmt);
	drawString(de, sonde.si()->launchsite);
}
void Display::drawTelemetry(DispEntry *de) {
}

#define EARTH_RADIUS (6371000.0F)
#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
// defined by Arduino.h   #define radians(x) ( (x)*180.0F/PI )

void Display::drawGPS(DispEntry *de) {
	if(sonde.config.gps_rxd<0) return;
	rdis->setFont(de->fmt);
	switch(de->extra[0]) {
	case 'V':
		{
		// show if GPS location is valid
		uint8_t *tile = nmea.isValid()?gps_tile:nogps_tile;
		rdis->drawTile(de->x, de->y, 1, tile);
		}
		break;
	case 'O':
		// GPS long
		{
		float lon = nmea.getLongitude()*0.000001;
		Serial.print("lon: "); Serial.println(lon);
		snprintf(buf, 16, "%2.5f", lon);
		drawString(de,buf);
		}
		break;
	case 'A':
		// GPS lat
		{
		float lat = nmea.getLatitude()*0.000001;
		Serial.print("lat: "); Serial.println(lat);
		snprintf(buf, 16, "%2.5f", lat);
		drawString(de,buf);
		}
		break;
	case 'H':
		// GPS alt
		{
		long alt = -1;
		nmea.getAltitude(alt);
		snprintf(buf, 16, "%4.0fm", alt*0.001);
		drawString(de,buf);
		}
		break;
	case 'C':
		// GPS Course over ground
		snprintf(buf, 4, "%3d", (int)(nmea.getCourse()/1000));
		drawString(de, buf);
		break;
	case 'D':
		{
		// distance
		// equirectangular approximation is good enough
		if( (sonde.si()->validPos&0x03)!=0x03 ) {
			snprintf(buf, 16, "no pos ");
			if(de->extra && *de->extra=='5') buf[5]=0;
		} else if(!nmea.isValid()) {
			snprintf(buf, 16, "no gps ");
			if(de->extra && *de->extra=='5') buf[5]=0;
		} else {
			float lat1 = nmea.getLatitude()*0.000001;
			float lat2 = sonde.si()->lat;
			float x = radians(nmea.getLongitude()*0.000001-sonde.si()->lon) * cos( radians((lat1+lat2)/2) );
			float y = radians(lat2-lat1);
			float d = sqrt(x*x+y*y)*EARTH_RADIUS;
			if(de->extra && *de->extra=='5') { // 5-character version: ****m / ***km / **e6m
				if(d>999999) snprintf(buf, 16, "%de6m  ", (int)(d/1000000));
				if(d>9999) snprintf(buf, 16, "%dkm   ", (int)(d/1000));
				else snprintf(buf, 16, "%dm    ", (int)d);
				buf[5]=0;
			} else { // 6-character version: *****m / ****km)
				if(d>99999) snprintf(buf, 16, "%dkm    ", (int)(d/1000));
				else snprintf(buf, 16, "%dm     ", (int)d);
				buf[6]=0;
			}
		}
		drawString(de, buf);
		}
		break;
	case 'I':
		// dIrection
		if( (!nmea.isValid()) || ((sonde.si()->validPos&0x03)!=0x03 ) ) {
			drawString(de, "---");
			break;
		}
		{
		float lat1 = radians(nmea.getLatitude()*0.000001);
                float lat2 = radians(sonde.si()->lat);
		float lon1 = radians(nmea.getLongitude()*0.000001);
                float lon2 = radians(sonde.si()->lon);
		float y = sin(lon2-lon1)*cos(lat2);
		float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
		float dir = atan2(y, x)/PI*180;
		if(dir<0) dir+=360;
		Serial.printf("direction is %.2f\n", dir);
		snprintf(buf, 16, "%3d", (int)dir);
		buf[3]=0;
		drawString(de, buf);
		if(de->extra[1]==(char)176)
			rdis->drawTile(de->x+3, de->y, 1, deg_tile);
		}
		break;
	case 'B':
		// relative bearing
		if( (!nmea.isValid()) || ((sonde.si()->validPos&0x03)!=0x03 ) ) {
			drawString(de, "---");
			break;
		}
		{
		float lat1 = radians(nmea.getLatitude()*0.000001);
                float lat2 = radians(sonde.si()->lat);
		float lon1 = radians(nmea.getLongitude()*0.000001);
                float lon2 = radians(sonde.si()->lon);
		float y = sin(lon2-lon1)*cos(lat2);
		float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
		float dir = atan2(y, x)/PI*180;
		if(dir<0) dir+=360;
		Serial.printf("direction is %.2f\n", dir);
		float course = nmea.getCourse()*0.001;
		float bearing = dir - course;
		if(bearing<0) bearing += 360;
		if(bearing>=360) bearing -= 360;
		snprintf(buf, 16, "%3d", (int)bearing);
		buf[3]=0;
		drawString(de, buf);
		if(de->extra[1]==(char)176)
			rdis->drawTile(de->x+3, de->y, 1, deg_tile);
		}
		break;
	case 'E':
		// elevation
		break;
	}
}

void Display::drawText(DispEntry *de) {
        rdis->setFont(de->fmt);
	drawString(de, de->extra);
}

void Display::updateDisplayPos() {
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawLat && di->func != disp.drawLon) continue;
		di->func(di);
	}
}
void Display::updateDisplayPos2() {
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawAlt && di->func != disp.drawHS && di->func != disp.drawVS) continue;
		di->func(di);
	}
}
void Display::updateDisplayID() {
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawID) continue;
		di->func(di);
	}
}
void Display::updateDisplayRSSI() {
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawRSSI) continue;
		di->func(di);
	}
}
void Display::updateStat() {
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawQS) continue;
		di->func(di);
	}
}

void Display::updateDisplayRXConfig() {
       for(DispEntry *di=layout->de; di->func != NULL; di++) {
                if(di->func != disp.drawQS && di->func != disp.drawAFC) continue;
                di->func(di);
        }
}

void Display::updateDisplayIP() {
       for(DispEntry *di=layout->de; di->func != NULL; di++) {
                if(di->func != disp.drawIP) continue;
		Serial.printf("updateDisplayIP: %d %d\n",di->x, di->y);
                di->func(di);
        }
}

void Display::updateDisplay() {
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		di->func(di);
	}
}

Display disp = Display();

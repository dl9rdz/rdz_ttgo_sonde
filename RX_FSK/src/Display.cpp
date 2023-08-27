#include "../features.h"
#include <U8x8lib.h>
#include <U8g2lib.h>
#include <SPIFFS.h>
#include <axp20x.h>
#include <MicroNMEA.h>
#include "Display.h"
#include "Sonde.h"

int readLine(Stream &stream, char *buffer, int maxlen);

extern const char *version_name;
extern const char *version_id;

#include "fonts/FreeMono9pt7b.h"
#include "fonts/FreeMono12pt7b.h"
#include "fonts/FreeSans9pt7b.h"
#include "fonts/FreeSans12pt7b.h"
#include "fonts/FreeSans18pt7b.h"
#include "fonts/Picopixel.h"
#include "fonts/Terminal11x16.h"

extern Sonde sonde;

extern AXP20X_Class axp;
extern bool axp192_found;
extern SemaphoreHandle_t axpSemaphore;

extern xSemaphoreHandle globalLock;
#define SPI_MUTEX_LOCK() \
  do                     \
  {                      \
  } while (xSemaphoreTake(globalLock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK() xSemaphoreGive(globalLock)

//SPIClass spiDisp(HSPI);

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
static unsigned char stattilesXL[5][5] =  {
   0x00, 0x7F, 0x00, 0x00, 0x00,  // | == ok
   0x00, 0x40, 0x40, 0x00, 0x00,  // . == no header found
   0x7F, 0x49, 0x49, 0x49, 0x00,  // E == decode error
   0x00, 0x00, 0x00, 0x00, 0x00,   // ' ' == unknown/unassigned
   0x07, 0x05, 0x07, 0x00, 0x00 };  // ° = rx ok, but no valid position (not yet used?)


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
	{5, 0, FONT_LARGE, -1, 0xFFFF, 0, disp.drawSite, "l"},
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
  { searchLayout, searchActions, searchTimeouts, "StaticSearch" },
  { legacyLayout, legacyActions, legacyTimeouts, "StaticLegacy" },
  { fieldLayout, fieldActions, fieldTimeouts, "StaticField1" },
  { field2Layout, field2Actions, fieldTimeouts, "StaticFiel2" },
  { gpsLayout, gpsActions, fieldTimeouts, "StaticGPS" } };


/////////////// Wrapper code for various display

static void utf2latin15(const char *src, char *dst, int dstlen) {
	static uint8_t decoderState = 0;
	static uint16_t decoderBuffer;

	while(uint8_t c = *src++) {
		if( (c&0x80)==0 || c==0xB0) { // 7bit code point (or special case: old °sign)
			decoderState = 0;
			*dst++ = c;
			continue;
		}
		if(decoderState==0) {
			if((c&0xE0)==0xC0) { // 11bit code point
				decoderBuffer = ((c&0x1F)<<6);	// first 5 bit
				decoderState = 1;
			} else if ((c&0xF0)==0xE0) { // 16 bit code point
				decoderBuffer = ((c&0x0f)<<12); // first 4 bit
				decoderState = 2;
			}
		} else {
			decoderState --;
			if(decoderState == 1)
				decoderBuffer |= ((c&0x3f)<<6); // next 6 bit of 16 bit code point
			else if (decoderState==0) {
				decoderBuffer |= (c&0x3f);   // last 6 bit of code point$a
			}
		}
		if(decoderState==0) {
			// emit encoded byte if possible
			switch(decoderBuffer) {
			case 0x0152: decoderBuffer=0xbc; break;
			case 0x0153: decoderBuffer=0xbd; break;
			case 0x0160: decoderBuffer=0xa6; break;
			case 0x0161: decoderBuffer=0xa8; break;
			case 0x0178: decoderBuffer=0xbe; break;
			case 0x017D: decoderBuffer=0xb4; break;
			case 0x017E: decoderBuffer=0xb8; break;
			case 0x20AC: decoderBuffer=0xa4; break;
			}

			if(decoderBuffer>0xff) decoderBuffer=0x7f;
			else if(decoderBuffer>0x7f) decoderBuffer-=0x20;
			*dst++ = (uint8_t)decoderBuffer;
		}

	}
	*dst = 0;
}


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
	Serial.printf("Init SSD1306 display %d %d\n", sonde.config.oled_scl, sonde.config.oled_sda);
	//u8x8 = new U8X8_SSD1306_128X64_NONAME_SW_I2C(/* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda, /* reset=*/ sonde.config.oled_rst); // Unbuffered, basic graphics, software I2C
	if (_type==2) {
               	u8x8 = new U8X8_SH1106_128X64_NONAME_HW_I2C(/* reset=*/ sonde.config.oled_rst, /* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda); // Unbuffered, basic graphics, software I2C
	} else { //__type==0 or anything else
		u8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(/* reset=*/ sonde.config.oled_rst, /* clock=*/ sonde.config.oled_scl, /* data=*/ sonde.config.oled_sda); // Unbuffered, basic graphics, software I2C
	} 
	u8x8->begin();
	if(sonde.config.tft_orient==3) u8x8->setFlipMode(true);
	if(sonde.config.dispcontrast>=0) u8x8->setContrast(sonde.config.dispcontrast);

	fontlist = fl;
	nfonts = sizeof(fl)/sizeof(uint8_t *);
	Serial.printf("Size of font list is %d\n", nfonts);
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


#ifdef LEGACY_FONTS_IN_CODEBIN
const GFXfont *legacygfl[] = {
	&Terminal11x16Font,		// 1 (replacement for 1 with old library)
	&Terminal11x16Font,		// 2 (replacement for 2 with old library)
	&Terminal11x16Font,		// 3 (replacement for 1 or 2 with old library)
	&Terminal11x16Font,		// 4 (replacement for 1 or 2 with old library)
	&FreeSans9pt7b,		// 5
	&FreeSans12pt7b,	// 6
	&Picopixel,             // 7
	&FreeSans18pt7b,        // 8
};

const GFXfont **gfl = legacygfl;
static int ngfx = sizeof(legacygfl)/sizeof(GFXfont *);
#else
// will crash if no font partition exists........
const GFXdont **gfl = NULL;
#endif


#define MAXFONT 20

static void init_gfx_fonts() {
	// if we have a fonts partition, relocate and use that...
	const esp_partition_t *part;
	spi_flash_mmap_handle_t handle;
	void *data;
	esp_err_t err;

	part = esp_partition_find_first((esp_partition_type_t)0x40, ESP_PARTITION_SUBTYPE_APP_FACTORY, "fonts");
	if(part) {
		Serial.println("FONT partition found!");
		err = esp_partition_mmap(part, 0, part->size, SPI_FLASH_MMAP_DATA, (const void **)&data, &handle);
		if( err != ESP_OK ) {
			Serial.println("mmap not OK\n");
			return;
		}
		Serial.println("font partition successfully mmaped");
		// do relocation stuff....
		const GFXfont **fptr = (const GFXfont **)data;
		if((uint32_t)*fptr != 0x544E4F46) { // FONT
			Serial.println("No font data in font partition");
			return; 
		}
		int n=0; 
		for(const GFXfont **g=fptr; *g!=NULL; g++) n++;
		Serial.printf("There a %d fonts in the font partition\n", n);
		if(n>MAXFONT) n=MAXFONT;
		GFXfont **newgfl = (GFXfont **)malloc( n*sizeof(GFXfont) + MAXFONT*sizeof(GFXfont *)  );
		if(!newgfl) {
			Serial.println("no memory for gfx fonts");
			return;
		}
		GFXfont *fonts = (GFXfont *)(((char *)newgfl) + MAXFONT*sizeof(GFXfont *));
		// create GFXfont list
		for(int i=0; i<n; i++) {
			newgfl[i] = fonts+i;
			GFXfont *orig = (GFXfont *)(((char *)data) + (uint32_t)fptr[i]);
			memcpy(newgfl[i], orig, sizeof(GFXfont));
			newgfl[i]->bitmap = newgfl[i]->bitmap + (uint32_t)data;  // relocate bitmap pointer to mmap partition
			newgfl[i]->glyph = (GFXglyph *)( ((char *)newgfl[i]->glyph) + (uint32_t)data );
			Serial.printf("font i: gfl[i] is %p,  gfl[i]->bitmap is %p, gfl[i]->glyph is %p\n", newgfl[i], newgfl[i]->bitmap, newgfl[i]->glyph);
		}
		gfl = (const GFXfont **)newgfl;
		ngfx = n;
	}
}

struct gfxoffset_t {
	uint8_t yofs, yclear;
};
#if 0
///// **** this is now calculated automatically
//
// obtained as max offset from font (last column) and maximum height (3rd column) in glyphs
// first value: offset: max offset from font glyphs (last column * (-1))   (check /, \, `, $)`
// yclear:max height: max of (height in 3rd column) + (yofs + 6th column)  (check j)
const struct gfxoffset_t gfxoffsets[]={
        { 16, 18},
        { 16, 18},
        { 13, 18 },  // 17+13-12 "j" 
        { 17, 23 }, // 23+17-17
        {  4, 6},       // 6+4-4
        { 25, 34 },   // 34 25 -25
};
#endif

struct gfxoffset_t gfxoffsets[MAXFONT];

void calc_gfx_offsets() {
	for(int i=0; i<ngfx; i++) {
		// find offset from top to baseline
		int bofs = 0;
		GFXglyph *g = gfl[i]->glyph;
		for(int j=0; j<=gfl[i]->last-gfl[i]->first; j++) {
			if(g[j].yOffset < bofs) bofs = g[j].yOffset;  // yOffset are negative values
		}
		gfxoffsets[i].yofs = -bofs;
		// find max. height (yofs + yOffset + height) -- note that yOffset is negative
		int hgt = 0;
		for(int j=0; j<=gfl[i]->last-gfl[i]->first; j++) {
			int h = gfxoffsets[i].yofs + g[j].yOffset + g[j].height;
			if(h>hgt) hgt=h;
		}
		gfxoffsets[i].yclear = hgt;
		printf("Font %d: yofs=%d, yclear=%d\n", i, gfxoffsets[i].yofs, gfxoffsets[i].yclear);
	}
}
	
#define TFT_LED 0 // 0 if wired to +5V directly
#define TFT_BRIGHTNESS 100 // Initial brightness of TFT backlight (optional)

Arduino_DataBus *bus;
		
void ILI9225Display::begin() {
	Serial.println("ILI9225/ILI9341 init");
	init_gfx_fonts();
	calc_gfx_offsets();
	// On the M5, the display and the Lora chip are on the same SPI interface (VSPI default pins),
	// we must use the same SPI bus with correct locking 
	if(sonde.config.type == TYPE_M5_CORE2) {
		bus = new Arduino_ESP32SPI( sonde.config.tft_rs, sonde.config.tft_cs,
			sonde.config.oled_scl, sonde.config.oled_sda, 38, VSPI);
	} else {
		bus = new Arduino_ESP32SPI( sonde.config.tft_rs, sonde.config.tft_cs,
			sonde.config.oled_scl, sonde.config.oled_sda, -1, HSPI);
	}
	if(_type == 3) 
	  tft = new Arduino_ILI9341(bus, sonde.config.oled_rst);
	else if(_type == 4) 
	  tft = new Arduino_ILI9342(bus, sonde.config.oled_rst);
	else if(_type == 5) 
	  tft = new Arduino_ST7789(bus, sonde.config.oled_rst);
	else 
	  tft = new Arduino_ILI9225(bus, sonde.config.oled_rst);
	Serial.println("ILI9225/ILI9341/ST7789 init: done");
	tft->begin(sonde.config.tft_spifreq);
        tft->fillScreen(BLACK);
	tft->setRotation(sonde.config.tft_orient);
	tft->setTextWrap(false);
	if(sonde.config.type == TYPE_M5_CORE2) 
		tft->invertDisplay(true);
}

void ILI9225Display::clear() {
	SPI_MUTEX_LOCK();
	tft->fillScreen(BLACK);
	SPI_MUTEX_UNLOCK();
}

void ILI9225Display::setContrast(uint8_t /*contrast*/) {
}

// for now, 0=small=FreeSans9pt7b, 1=large=FreeSans18pt7b
void ILI9225Display::setFont(uint8_t fontindex) {
	//if(fontindex==1 || fontindex==2) { fontindex=3; }
	findex = fontindex;
	switch(fontindex) {
	case 0: tft->setFont(); tft->setTextSize(1); break;
	//case 1: tft->setFont(NULL); tft->setTextSize(2); break;
	//case 2: tft->setFont(NULL); tft->setTextSize(2); break;
	default: tft->setFont(gfl[fontindex-1]);
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
		int16_t x, y;
		uint16_t w, h;
		tft->getTextBounds("|", 0, 0, &x, &y, &w, &h);
		if(lineskip) *lineskip = h+2;
		tft->getTextBounds("A", 0, 0, &x, &y, &w, &h);
		if(colskip) *colskip = w+2;
		if(lineskip&&colskip) { Serial.printf("skip size from bounds: %d, %d\n", *lineskip, *colskip); }
	}
	}
}



// Note: alignright means that there is a box from x to x+(-width), with text right-justified
//       x is the *left* corner! not the right...
void ILI9225Display::drawString(uint16_t x, uint16_t y, const char *s, int16_t width, uint16_t fg, uint16_t bg) {
	int16_t w,h;
	boolean alignright=false;
	char buf[50];
	utf2latin15(s, buf, 50);	

	if(width<0) {
		width = -width;
		alignright = true;
	}
	// Standard font
	if(findex==0) {
		SPI_MUTEX_LOCK();
		DebugPrintf(DEBUG_DISPLAY, "Simple Text %s at %d,%d [%d]\n", s, x, y, width); 
		// for gpx fonts and new library, cursor is at baseline!!
		int h = 6;
		if( alignright ) {
			//w = tft->getTextWidth(s);
			/// TODO
			if( width==WIDTH_AUTO ) { width = w; }
			if( width > w ) {
				tft->writeFillRect(x, y, width - w, h - 1, bg);
			}
			tft->setCursor(x + width - w, y);
			tft->setTextColor(fg, bg);
			tft->print(buf);
		} else {
			tft->setCursor(x, y);
			tft->setTextColor(fg, bg);
			tft->print(buf);
		}
		SPI_MUTEX_UNLOCK();
		return;
	}
	// GFX font
	SPI_MUTEX_LOCK();
	int16_t x1, y1;
	if(1||width==WIDTH_AUTO || alignright) {
		tft->getTextBounds(buf, x, y + gfxoffsets[findex-1].yofs, &x1, &y1, (uint16_t *)&w, (uint16_t *)&h);
		w += x1 - x + 1;
		if(width==WIDTH_AUTO) { width=w; }
		if(alignright) {
			if(w > width) {
				// fast drawBitmap does bad things if not within viewport
				// Maybe better truncate on the right? (TODO)
				x = x - w + width; if(x<0) x=0;
				width = w;
			}
			//x -= width;
			//DebugPrintf(DEBUG_DISPLAY, "Reducing x by width %d, its now %d\n", width, x); 
		}
	}

	if(findex-1>=ngfx) findex=1;
	DebugPrintf(DEBUG_DISPLAY,"GFX Text %s at %d,%d+%d in color %x, width=%d (w=%d)\n", s, x, y, gfxoffsets[findex-1].yofs, fg, width, w);
#if 0
	// Text by clear rectangle and refill, causes some flicker
	tft->fillRectangle(x, y, x + width, y + gfxoffsets[findex-1].yclear, bg);
	if(alignright) {
        	tft->drawGFXText(x + width - w, y + gfxoffsets[findex-1].yofs, s, fg);
	} else {
        	tft->drawGFXText(x, y + gfxoffsets[findex-1].yofs, s, fg);
	}
#else 
	// Text by drawing bitmap.... => less "flicker"
#if 1
	//TODO
	tft->setCursor( alignright? x+width-w : x, y + gfxoffsets[findex-1].yofs);
	tft->setTextColor( fg, bg );
	tft->print(buf);
	uint16_t height = gfxoffsets[findex-1].yclear;
	if(alignright) {
		// fill with bg from x+w to width
		if(width>w) tft->fillRect( x, y, width-w, height, bg);
		DebugPrintf(DEBUG_DISPLAY,"rtext fill %d %d %d %d -- %d %d\n", x, y, width-w, height, x1, y1);
	} else {
		// fill with bg from x+w to width
		if(width>w) tft->fillRect( x+w, y, width-w, height, bg);
		DebugPrintf(DEBUG_DISPLAY,"ltext fill %d %d %d %d -- %d %d\n", x+w, y, width-w, height, x1, y1);
	}
#else
	uint16_t height = gfxoffsets[findex-1].yclear;
        uint16_t *bitmap = (uint16_t *)malloc(sizeof(uint16_t) * width * height);
	if(!bitmap) {
		Serial.println("FATAL: OUT OF MEMORY when allocating bitmap");
		Serial.printf("w=%d, h=%d, s==%d\n",width, height, 2*width*height);
		heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
		return;
	}
        for(int i=0; i<width*height; i++) { bitmap[i] = bg; }   // fill with background
	int x0 = 0;
	if(alignright) { x0 = width - w; }
	int y0 = gfxoffsets[findex-1].yofs;
	DebugPrintf(DEBUG_DISPLAY,"GFX: w=%d h=%d\n", width, height);
	for (uint8_t k = 0; k < strlen(s); k++) {	
            x0 += tft->drawGFXcharBM(x0, y0, s[k], fg, bitmap, width, height) + 1;
	    DebugPrintf(DEBUG_DISPLAY,"[%c->%d]",s[k],x0);
	}
	// TODO: if x+width exceeds display width, garbage is generated....
        drawBitmap(x, y, bitmap, width, height);
	free(bitmap);
#endif
#endif
	SPI_MUTEX_UNLOCK();
}

void ILI9225Display::drawTile(uint16_t x, uint16_t y, uint8_t cnt, uint8_t *tile_ptr) {
        int i,j;
	SPI_MUTEX_LOCK();
        tft->startWrite();
        for(i=0; i<cnt*8; i++) {
                uint8_t v = tile_ptr[i];
                for(j=0; j<8; j++) {
                        tft->writePixel(8*x+i, 8*y+j, (v&0x01) ? GREEN:BLUE);
                        v >>= 1;
                }
        }
        tft->endWrite();
	SPI_MUTEX_UNLOCK();
}

void ILI9225Display::drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, boolean fill) {
	SPI_MUTEX_LOCK();
	if(fill)
		tft->fillTriangle(x1, y1, x2, y2, x3, y3, color);
	else
		tft->drawTriangle(x1, y1, x2, y2, x3, y3, color);
	SPI_MUTEX_UNLOCK();
}

void ILI9225Display::drawBitmap(uint16_t x1, uint16_t y1, const uint16_t* bitmap, int16_t w, int16_t h) {
	SPI_MUTEX_LOCK();
	tft->draw16bitRGBBitmap(x1, y1, bitmap, w, h);
	SPI_MUTEX_UNLOCK();
}

void ILI9225Display::welcome() {
	SPI_MUTEX_LOCK();
	tft->fillScreen(0);
	SPI_MUTEX_UNLOCK();
        setFont(6);
        drawString(0, 0*22, version_name, WIDTH_AUTO, 0xff00);
        setFont(5);
	int l=3*22;
	if(sonde.config.tft_orient&1) {
        	drawString(0, 1*22, "RS41/92,DFM,M10/20");
	} else {
        	drawString(0, 1*22, "RS41,RS92,");
        	drawString(0, 2*22, "DFM,M10/20");
		l+=22;
	}
       	drawString(0, l, version_id);
       	drawString(0, l+2*22, "by Hansi, DL9RDZ");
}

void ILI9225Display::drawIP(uint16_t x, uint16_t y, int16_t width, uint16_t fg, uint16_t bg) {
	char buf[20];
	if(sonde.isAP) strcpy(buf, "A "); else *buf=0;   
	strncat(buf, sonde.ipaddr.c_str(), 16);
	drawString(x, y, buf, width, fg, bg);		
}

// size: 3=> 3x5 symbols; 4=> 4x7 symbols
void ILI9225Display::drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t size, uint8_t *stat, uint16_t fg, uint16_t bg) {
	if(size<3) size=3;
	if(size>4) size=4;
	const uint16_t width = len*(size+1);
	const uint16_t height = (size+3);
	uint16_t bitmap[width*height];
	for(int i=0; i<len; i++) {
		for(int xx=0; xx<size+1; xx++) {
			for(int yy=0; yy<size+3; yy++) {
				if(size==3) {
					bitmap[ yy*width + i*(size+1) + xx ] = ((stattiles[stat[i]][xx]>>yy)&0x01) ?fg:bg;
				} else {
					bitmap[ yy*width + i*(size+1) + xx ] = ((stattilesXL[stat[i]][xx]>>yy)&0x01) ?fg:bg;
				}
			}
		}
	}
	drawBitmap(x, y, bitmap, len*(size+1), (size+3));
}

#include <pgmspace.h>
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))


///////////////


char Display::buf[17];
char Display::lineBuf[Display::LINEBUFLEN];

RawDisplay *Display::rdis = NULL;

//TODO: maybe merge with initFromFile later?
void Display::init() {
	Serial.printf("disptype is %d\n",sonde.config.disptype);
	if(sonde.config.disptype==0 || sonde.config.disptype==2) {
		rdis = new U8x8Display(sonde.config.disptype);
	} else {
		rdis = new ILI9225Display(sonde.config.disptype);
	}
	Serial.println("Display created");
	rdis->begin();
	delay(100);
	Serial.println("Display initialized");
	rdis->clear();
	dispstate = 1;  // display active by default
}


Display::Display() {
	layouts = staticLayouts;
	setLayout(0);
}

#define MAXSCREENS 20
#define DISP_ACTIONS_N 12
#define DISP_TIMEOUTS_N 3

void Display::replaceLayouts(DispInfo *newlayouts, int nnew) {
	if(nnew<1) return;  // no new layouts => ignore

	// remember old layouts
	DispInfo *old = layouts;

	// assign new layouts and current layout
	Serial.printf("replaceLayouts: idx=%d n(new)=%d\n", layoutIdx, nLayouts);
	layouts = newlayouts;
	nLayouts = nnew;
	if(layoutIdx >= nLayouts) layoutIdx = 0;
	layout = layouts+layoutIdx;

	// Make it unlikely that anyone else is still using previous layouts
	delay(500);

	// and release memory not used any more
	if(old==staticLayouts) return;
	for(int i=0; i<MAXSCREENS; i++) {
		if(old[i].de) free(old[i].de);
	}
	free(old);
}

int Display::allocDispInfo(int entries, DispInfo *d, char *label)
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

	d->label = label;
	Serial.printf("%s: alloc %d bytes (%d entries) for %p (addr=%p)\n", label, totalsize, entries, d, d->de);
	return 0;
}

uint16_t encodeColor(uint32_t col) {
	return (col>>19) << 11 | ((col>>10)&0x3F) << 5 | ((col>>3)&0x1F);
}
uint16_t encodeColor(char *colstr) {
	uint32_t col;
	int res=sscanf(colstr, "%" SCNx32, &col);
	if(res!=1) return 0xffff;
	return encodeColor(col);
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
		{
		struct StatInfo *statinfo = (struct StatInfo *)malloc(sizeof(struct StatInfo));
		// maybe enable more flexible configuration?
		statinfo->size=3;
		statinfo->len=18;
		if(text[1]=='4') statinfo->size = 4;

		de->extra = (const char *)statinfo;
		de->func = disp.drawQS;
		}
		break;
	case 't':
		de->func = disp.drawType; break;
	case 'c':
		de->func = disp.drawAFC; break;
	case 'f':
		de->func = disp.drawFreq;
		de->extra = strdup(text+1);
		//Serial.printf("parsing 'f' entry: extra is '%s'\n", de->extra);
		break;
	case 'm':
		de->func = disp.drawTelemetry;
		de->extra = strdup(text+1);
		break;
	case 'n':
		// IP address / small always uses tiny font on TFT for backward compatibility
		// Large font can be used arbitrarily
		if(de->fmt==fontsma) de->fmt=0;
		de->func = disp.drawIP;
		de->extra = strdup(text+1);
		break;
	case 's':
		de->func = disp.drawSite;
		de->extra = strdup(text+1);
		break;
	case 'k':
		de->func = disp.drawKilltimer;
		de->extra = strdup(text+1);
		break;
	case 'g':
		de->func = disp.drawGPS;
		if(text[1]=='0') {  
			// extended configuration for arrow...
			struct CircleInfo *circinfo = (struct CircleInfo *)malloc(sizeof(struct CircleInfo));
#if 1
			circinfo->type = '0';
			circinfo->top = text[2];
			circinfo->arr = text[3];
			circinfo->bul = text[4];
			char *ptr=text+5;
			while(*ptr && *ptr!=',') ptr++; ptr++;
			// next: radius
			circinfo->radius = atoi(ptr);
			while(*ptr && *ptr!=',') ptr++; ptr++;
			circinfo->fgcol = encodeColor(ptr);
			while(*ptr && *ptr!=',') ptr++; ptr++;
			circinfo->bgcol = encodeColor(ptr);
#else
			circinfo->type = '0';
			circinfo->top = 'N';
			circinfo->bul = 'S';
			circinfo->arr = 'C';
			circinfo->radius = 50;
			circinfo->fgcol = 0xfe80;
			circinfo->bgcol = 0x0033;
#endif
			while(*ptr && *ptr!=',') ptr++; ptr++;
			circinfo->awidth = atoi(ptr);
			while(*ptr && *ptr!=',') ptr++; ptr++;
			circinfo->acol = encodeColor(ptr);
			while(*ptr && *ptr!=',') ptr++; ptr++;
			circinfo->brad = atoi(ptr);
			while(*ptr && *ptr!=',') ptr++; ptr++;
			circinfo->bcol = encodeColor(ptr);
			de->extra = (char *)circinfo;
		} else {
			de->extra = strdup(text+1);
			//Serial.printf("parsing 'g' entry: extra is '%s'\n", de->extra);
		}
		break;
	case 'r':
		de->func = disp.drawRSSI; break;
	case 'x':
		de->func = disp.drawText;
		de->extra = strdup(text+1);
		break;
	case 'b':
		de->func = disp.drawBatt;
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
	case '>':
		return ACT_DISPLAY_NEXT;
	default:
		if(c>='0'&&c<='9')
			return ACT_DISPLAY(c-'0');
		// Hack, will change later to better syntax
		if(c>='a'&&c<='z')
			return ACT_ADDFREQ(c-'a'+2);
	}
	return ACT_NONE;
}


int Display::countEntries(File f) {
	int pos = f.position();
	int n = 0;
	while(1) {
		//String line = readLine(f);  //f.readStringUntil('\n');
		//line.trim();
		//const char *c=line.c_str();
		readLine(f, lineBuf, LINEBUFLEN);
		const char *c = trim(lineBuf);
		if(*c=='#') continue;
		if(*c>='0'&&*c<='9') n++;
		if(strchr(c,'=')) continue;
		break;
	}	
	f.seek(pos, SeekSet);
	//Serial.printf("Counted %d entries\n", n);
	return n;
}

int Display::getScreenIndex(int index) {
	if(index!=0) return index;
	switch(sonde.config.disptype) {
	case 1:		// ILI9225
		index = 2;      // landscape mode (orient=1/3)
		if( (sonde.config.tft_orient&0x01)==0 ) index++;   // portrait mode (0/2)
		break;
	case 3:		// ILI9341
	case 4:		// ILI9342
	case 5:
		index = 4;      // landscape mode (orient=1/3)
		if( (sonde.config.tft_orient&0x01)==0 ) index++;   // portrait mode (0/2)
		break;
	case 0: case 2: 	// small OLED display (SD1306/SH1106)
	default:
		index = 1; break;
	}
	return index;
}
void Display::initFromFile(int index) {
	File d;
	char file[20];

	index = getScreenIndex(index);  // auto selection for index==0
	snprintf(file, 20, "/screens%d.txt", index);
	Serial.printf("Reading %s\n", file);
	d = SPIFFS.open(file, "r");
	if(!d || d.available()==0 ) { Serial.printf("%s not found\n", file); return; }

	DispInfo *newlayouts = (DispInfo *)malloc(MAXSCREENS * sizeof(DispInfo));
	if(!newlayouts) {
		Serial.println("Init from file: FAILED, not updating layouts");
		return;
	}
	memset(newlayouts, 0, MAXSCREENS * sizeof(DispInfo));

	// default values
        xscale=13;
	yscale=22;
        fontsma=0;
	fontlar=1;
	// default color
	colfg = 0xffff; // white; only used for ILI9225
	colbg = 0;  // black; only used for ILI9225
	int idx = -1;
	int what = -1;
	int entrysize;
	Serial.printf("Reading from screen config: available=%d\n",d.available());
	while(d.available()) {
		//Serial.printf("Unused stack: %d\n", uxTaskGetStackHighWaterMark(0));
		const char *ptr;
		readLine(d, lineBuf, LINEBUFLEN);
		const char *s = trim(lineBuf);
		// String line = readLine(d);  
		// line.trim();
		// const char *s = line.c_str();
		DebugPrintf(DEBUG_SPARSER, "Line: '%s'\n", s);
		if(*s == '#') continue;  // ignore comments
		switch(what) {
		case -1:	// wait for start of screen (@)
			{
			if(*s != '@') {
				if(*s==0 || *s==10 || *s==13) continue;
				Serial.printf("Illegal start of screen: %s\n", s);
				continue;
			}
			char *label = strdup(s+1);
			entrysize = countEntries(d);
			DebugPrintf(DEBUG_SPARSER,"Reading entry with %d elements\n", entrysize);
			idx++;
			int res = allocDispInfo(entrysize, &newlayouts[idx], label);
			if(res<0) {
				Serial.println("Error allocating memory for disp info");
				continue;
			}
			what = 0;
			}
			break;
		default:	// parse content... (additional data or line `what`)
			if(strncmp(s,"timer=",6)==0) {  // timer values
				char t1[10],t2[10],t3[10];
				sscanf(s+6, "%5[0-9a-zA-Z-] , %5[0-9a-zA-Z-] , %5[0-9a-zA-Z-]", t1, t2, t3);
				DebugPrintf(DEBUG_SPARSER,"timers are %s, %s, %s\n", t1, t2, t3);
				newlayouts[idx].timeouts[0] = (*t1=='n'||*t1=='N')?sonde.config.norx_timeout:atoi(t1);
				newlayouts[idx].timeouts[1] = (*t2=='n'||*t2=='N')?sonde.config.norx_timeout:atoi(t2);
				newlayouts[idx].timeouts[2] = (*t3=='n'||*t3=='N')?sonde.config.norx_timeout:atoi(t3);
				// Code later assumes milliseconds, but config.txt and screens.txt use values in seconds
				if(newlayouts[idx].timeouts[0]>0) newlayouts[idx].timeouts[0]*=1000;
				if(newlayouts[idx].timeouts[1]>0) newlayouts[idx].timeouts[1]*=1000;
				if(newlayouts[idx].timeouts[2]>0) newlayouts[idx].timeouts[2]*=1000;
				//sscanf(s+6, "%hd,%hd,%hd", newlayouts[idx].timeouts, newlayouts[idx].timeouts+1, newlayouts[idx].timeouts+2);
				//Serial.printf("timer values: %d, %d, %d\n", newlayouts[idx].timeouts[0], newlayouts[idx].timeouts[1], newlayouts[idx].timeouts[2]);
			} else if(strncmp(s, "key1action=",11)==0) { // key 1 actions
				char c1,c2,c3,c4;
				sscanf(s+11, "%c,%c,%c,%c", &c1, &c2, &c3, &c4);
				newlayouts[idx].actions[1] = ACTION(c1);
				newlayouts[idx].actions[2] = ACTION(c2);
				newlayouts[idx].actions[3] = ACTION(c3);
				newlayouts[idx].actions[4] = ACTION(c4);
			} else if(strncmp(s, "key2action=",11)==0) { // key 2 actions
				char c1,c2,c3,c4;
				sscanf(s+11, "%c,%c,%c,%c", &c1, &c2, &c3, &c4);
				newlayouts[idx].actions[5] = ACTION(c1);
				newlayouts[idx].actions[6] = ACTION(c2);
				newlayouts[idx].actions[7] = ACTION(c3);
				newlayouts[idx].actions[8] = ACTION(c4);
				//Serial.printf("parsing key2action: %c %c %c %c\n", c1, c2, c3, c4);
			} else if(strncmp(s, "timeaction=",11)==0) { // timer actions
				char c1,c2,c3;
				sscanf(s+11, "%c,%c,%c", &c1, &c2, &c3);
				newlayouts[idx].actions[9] = ACTION(c1);
				newlayouts[idx].actions[10] = ACTION(c2);
				newlayouts[idx].actions[11] = ACTION(c3);
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
				float x,y,w;
				int n;
				char text[61];
				n=sscanf(s, "%f,%f,%f", &y, &x, &w);
				sscanf(ptr+1, "%60[^\r\n]", text);
				if(sonde.config.disptype!=0 && sonde.config.disptype!=2) {
					x*=xscale; y*=yscale; w*=xscale;
				}
				newlayouts[idx].de[what].x = x;
				newlayouts[idx].de[what].y = y;
				newlayouts[idx].de[what].width = n>2 ? w : WIDTH_AUTO;
				parseDispElement(text, newlayouts[idx].de+what);
				DebugPrintf(DEBUG_SPARSER,"entry at %d,%d width=%d font %d, color=%x,%x\n", (int)x, (int)y, newlayouts[idx].de[what].width, newlayouts[idx].de[what].fmt,
					newlayouts[idx].de[what].fg, newlayouts[idx].de[what].bg);
				if(newlayouts[idx].de[what].func == disp.drawGPS) {
					newlayouts[idx].usegps = GPSUSE_BASE|GPSUSE_DIST|GPSUSE_BEARING; // just all for now
				}
				what++;
				newlayouts[idx].de[what].func = NULL;
			} else {
 				what=-1;
			}
			break;
		}
	}
	replaceLayouts(newlayouts, idx+1);
}

void Display::circ(uint16_t *bm, int16_t size, int16_t x0, int16_t y0, int16_t r, uint16_t fg, boolean fill, uint16_t bg) {
	// draw circle
        int x = 0;
        int y = r;
        int ddF_x = 1;
        int ddF_y = -2 * r;
        int f = 1-r;
        bm[x0 + (y0+r)*size] = fg;
        bm[x0 + (y0-r)*size] = fg;
        bm[x0+r + y0*size] = fg;
        bm[x0-r + y0*size] = fg;
	if(fill) { for(int yy=-y+1; yy<y-1; yy++) { bm[ (x0+yy) + y0*size ] = bg; } }
        while(x<y) {
		boolean newy = false;
                if(f>=0) { y--; ddF_y += 2; f += ddF_y; newy = true; }
                x++; ddF_x += 2; f += ddF_x;
                bm[ (x0+x) + (y0+y)*size ] = fg;
                bm[ (x0-x) + (y0+y)*size ] = fg;
                bm[ (x0+x) + (y0-y)*size ] = fg;
                bm[ (x0-x) + (y0-y)*size ] = fg;
                bm[ (x0+y) + (y0+x)*size ] = fg;
                bm[ (x0-y) + (y0+x)*size ] = fg;
                bm[ (x0+y) + (y0-x)*size ] = fg;
                bm[ (x0-y) + (y0-x)*size ] = fg;
		if(fill) {
			if(newy) {
				for(int xx = -x+1; xx<x-1; xx++) bm[ (x0+xx) + (y0+y)*size ] = bg;
				for(int xx = -x+1; xx<x-1; xx++) bm[ (x0+xx) + (y0-y)*size ] = bg;
			}
			for(int yy = -y+1; yy<y-1; yy++) bm[ (x0+yy) + (y0+x)*size ] = bg;
			for(int yy = -y+1; yy<y-1; yy++) bm[ (x0+yy) + (y0-x)*size ] = bg;
		}
        }
}


void Display::setLayout(int newidx) {
	Serial.printf("setLayout: %d (max is %d)\n", newidx, nLayouts);
	if(newidx>=nLayouts) newidx = 0; 
	layout = &layouts[newidx];
	layoutIdx = newidx;
}

void Display::drawString(DispEntry *de, const char *str) {
	rdis->drawString(de->x, de->y, str, de->width, de->fg, de->bg);
}

void Display::drawLat(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!VALIDPOS(sonde.si()->d.validPos)) {
	   drawString(de,"<?""?>      ");
	   return;
	}
	snprintf(buf, 16, "%2.5f", sonde.si()->d.lat);
	drawString(de,buf);
}
void Display::drawLon(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!VALIDPOS(sonde.si()->d.validPos)) {
	   drawString(de,"<?""?>      ");
	   return;
	}
	snprintf(buf, 16, "%2.5f", sonde.si()->d.lon);
	drawString(de,buf);
}
void Display::drawAlt(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!VALIDALT(sonde.si()->d.validPos)) {
	   drawString(de,"     ");
	   return;
	}
	float alt = sonde.si()->d.alt;
	//testing only....   alt += 30000-454;
	snprintf(buf, 16, alt>=1000?"   %5.0fm":"   %3.1fm", alt);
	drawString(de,buf+strlen(buf)-6);
}
void Display::drawHS(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!VALIDHS(sonde.si()->d.validPos)) {
	   drawString(de,"     ");
	   return;
	}
	boolean is_ms = (de->extra && de->extra[0]=='m')?true:false;  // m/s or km/h
	float hs = sonde.si()->d.hs;
	if(!is_ms) hs = hs * 3.6;
	boolean has_extra = (de->extra && de->extra[1]!=0)? true: false;
	snprintf(buf, 16, hs>99?" %3.0f":" %2.1f", hs);
	if(has_extra) { strcat(buf, de->extra+1); }
	drawString(de,buf+strlen(buf)-4- (has_extra?strlen(de->extra+1):0) );
	if(!has_extra) rdis->drawTile(de->x+4,de->y,2,is_ms?ms_tiles:kmh_tiles);
}
void Display::drawVS(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!VALIDVS(sonde.si()->d.validPos)) {
	   drawString(de,"     ");
	   return;
	}
	snprintf(buf, 16, "  %+2.1f", sonde.si()->d.vs);
	DebugPrintf(DEBUG_DISPLAY, "drawVS: extra is %s width=%d\n", de->extra?de->extra:"<null>", de->width);
	if(de->extra) { strcat(buf, de->extra); }
	drawString(de, buf+strlen(buf)-5- (de->extra?strlen(de->extra):0) );
	if(!de->extra) rdis->drawTile(de->x+5,de->y,2,ms_tiles);
}
void Display::drawID(DispEntry *de) {
	rdis->setFont(de->fmt);
	if(!sonde.si()->d.validID) {
		drawString(de, "nnnnnnnn ");
		return;
	}
	if(de->extra && de->extra[0]=='n') {
		// real serial number, as printed on sonde, can be up to 11 digits long
		drawString(de, sonde.si()->d.ser);
	} else if (de->extra && de->extra[0]=='s') {
		// short ID, max 8 digits (no initial "D" for DFM, "M" instead of "ME" for M10)
		if( TYPE_IS_DFM(sonde.si()->type) ) {
			drawString(de, sonde.si()->d.id+1);
		} else if (TYPE_IS_METEO(sonde.si()->type)) {
			char sid[9];
			sid[0]='M';
			memcpy(sid+1, sonde.si()->d.id+2, 8);
			sid[8] = 0;
			drawString(de, sid);
		} else {
			drawString(de, sonde.si()->d.id);
		}
	} else {
		// dxlAPRS sonde number, max 9 digits, as used on aprs.fi and radiosondy.info
		drawString(de, sonde.si()->d.id);
	}
}
void Display::drawRSSI(DispEntry *de) {
	rdis->setFont(de->fmt);
	// TODO.... 3/4!!!!!
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
	struct StatInfo *statinfo = (struct StatInfo *)de->extra;
	rdis->drawQS(de->x, de->y, statinfo->len, statinfo->size, stat, de->fg, de->bg);
}

void Display::drawType(DispEntry *de) {
	rdis->setFont(de->fmt);
	const char *typestr = sonde.si()->d.typestr;
	if(*typestr==0) typestr = sondeTypeStr[sonde.si()->type];
        drawString(de, typestr);
}
void Display::drawFreq(DispEntry *de) {
	rdis->setFont(de->fmt);
        snprintf(buf, 16, "%3.3f%s", sonde.si()->freq, de->extra?de->extra:"");
        drawString(de, buf);
}
void Display::drawAFC(DispEntry *de) {
	rdis->setFont(de->fmt);
	{ snprintf(buf, 15, "     %+3.2fk", sonde.si()->afc*0.001); }
        drawString(de, buf+strlen(buf)-8);
}
void Display::drawIP(DispEntry *de) {
	rdis->setFont(de->fmt);
	rdis->drawIP(de->x, de->y, de->width, de->fg, de->bg);
}
void Display::drawSite(DispEntry *de) {
        rdis->setFont(de->fmt);
	switch(de->extra[0]) {
	case '#':
		// currentSonde is index in array starting with 0;
		// but we draw "1" for the first entry and so on...
		snprintf(buf, 3, "%d ", sonde.currentSonde+1);
		buf[2]=0;
		break;
	case 't':
		snprintf(buf, 3, "%d", sonde.config.maxsonde);
		buf[2]=0;
		break;
	case 'a':
		{
		uint8_t active = 0;
 		for(int i=0; i<sonde.config.maxsonde; i++) {
	                if(sonde.sondeList[i].active) active++;
		}
 		snprintf(buf, 3, "%d", active);
		buf[2]=0;
		}
		break;
	case '/':
		snprintf(buf, 6, "%d/%d  ", sonde.currentSonde+1, sonde.config.maxsonde);
		buf[5]=0;
		break;
	case 0: case 'l': default: // launch site
		// TODO: This is a workaround to be compatible with older screens1.txt
		// This does not work correctly with non-ascii utf8 characters.
		// For this reason, this workaround will likely be removed in the future
		// Instead, all screens?.txt should always indicate the max. length of the displayed string.
		snprintf(buf, 17, "%-16s", sonde.si()->launchsite);
		//drawString(de, sonde.si()->launchsite);
		//return;
	}
	if(de->extra[0]) strcat(buf, de->extra+1);
	drawString(de, buf);
}
void Display::drawTelemetry(DispEntry *de) {
	rdis->setFont(de->fmt);
	float value=0;
	memset(buf, ' ', 16);
	switch(de->extra[0]) {
	case 't':
		value = sonde.si()->d.temperature;
		if(!isnan(value)) {
			sprintf(buf, "%5.1f", value);
			strcat(buf, de->extra+1);
		}
		buf[5+strlen(de->extra+1)] = 0;
		break;
	case 'p':
		value = sonde.si()->d.pressure;
		if(!isnan(value)) {
			if(value>=1000) sprintf(buf, "%6.1f", value);
			else sprintf(buf, "%6.2f", value);
			strcat(buf, de->extra+1);
		}
		buf[6+strlen(de->extra+1)] = 0;
		break;
	case 'h':
		value = sonde.si()->d.relativeHumidity;
		if(!isnan(value)) {
			sprintf(buf, "%4.1f", value);
			strcat(buf, de->extra+1);
		}
		buf[4+strlen(de->extra+1)] = 0;
		break;
	case 'b':
		value = sonde.si()->d.batteryVoltage;
		if(!isnan(value)) {
			snprintf(buf, 5, "%4.2f", value);
			strcat(buf, de->extra+1);
		}
		buf[5+strlen(de->extra+1)] = 0;
		break;
	}
	drawString(de,buf);
}

void Display::drawKilltimer(DispEntry *de) {
	rdis->setFont(de->fmt);
	uint16_t value=0;
	switch(de->extra[0]) {
	case 'l': value = sonde.si()->d.launchKT; break;
	case 'b': value = sonde.si()->d.burstKT; break;
	case 'c': value = sonde.si()->d.countKT; break;
	}
	// format: 4=h:mm; 6=h:mm:ss; s=sssss
	uint16_t h=value/3600;
	uint16_t m=(value-h*3600)/60;
	uint16_t s=value%60;
	switch(de->extra[1]) {
	case '4':
		if(value!=0xffff) snprintf(buf, 5, "%d:%02d", h, m);
		else strcpy(buf, "    ");
		break;
	case '6':
		if(value!=0xffff) snprintf(buf, 7, "%d:%02d:%02d", h, m, s);
		else strcpy(buf, "       ");
		break;
	default:
		if(value!=0xffff) snprintf(buf, 6, "%5d", value);
		else strcpy(buf, "     ");
		break;
	}
	if(de->extra[1])
		strcat(buf, de->extra+2);
	drawString(de, buf);
}
#define EARTH_RADIUS (6371000.0F)
#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
// defined by Arduino.h   #define radians(x) ( (x)*180.0F/PI )

extern int lastCourse; // from RX_FSK.ino


float calcLatLonDist(float lat1, float lon1, float lat2, float lon2) {
        float x = radians(lon1-lon2) * cos( radians((lat1+lat2)/2) );
        float y = radians(lat2-lat1);
        float d = sqrt(x*x+y*y)*EARTH_RADIUS;
	return d;
}

void Display::calcGPS() {
	float mylat = sonde.config.rxlat;
	float mylon = sonde.config.rxlon;
	bool valid = !(isnan(mylat)||isnan(mylon));
	if( gpsPos.valid) {
		mylat = gpsPos.lat;
		mylon = gpsPos.lon;
		valid = true;
	}
	// distance
	if( valid && VALIDPOS(sonde.si()->d.validPos) && (layout->usegps&GPSUSE_DIST)) {
		gpsDist = (int)calcLatLonDist(mylat, mylon, sonde.si()->d.lat, sonde.si()->d.lon);
	} else {
		gpsDist = -1;
	}
	// bearing
	if( valid && VALIDPOS(sonde.si()->d.validPos&0x03) && (layout->usegps&GPSUSE_BEARING)) {
                float lat1 = radians(mylat);
                float lat2 = radians(sonde.si()->d.lat);
                float lon1 = radians(mylon);
                float lon2 = radians(sonde.si()->d.lon);
                float y = sin(lon2-lon1)*cos(lat2);
                float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
                float dir = atan2(y, x)/PI*180;
                if(dir<0) dir+=360;
		gpsDir = (int)dir;
		gpsBear = gpsDir - gpsPos.course;
		if(gpsBear < 0) gpsBear += 360;
		if(gpsBear >= 360) gpsBear -= 360;
	} else {
		gpsDir = -1;
		gpsBear = -1;
	}
	
	DebugPrintf(DEBUG_DISPLAY, "GPS data: valid%d  GPS at %f,%f (alt=%d,cog=%d);  sonde at dist=%d, dir=%d rel.bear=%d\n",gpsPos.valid?1:0,
		gpsPos.lat, gpsPos.lon, gpsPos.alt, gpsPos.course, gpsDist, gpsDir, gpsBear);
}

void Display::drawGPS(DispEntry *de) {
	// TODO: FIXME: ??? if(sonde.config.gps_rxd<0) return;
	rdis->setFont(de->fmt);
	switch(de->extra[0]) {
	case 'V':
		{
		// show if GPS location is valid
		uint8_t *tile = gpsPos.valid?gps_tile:nogps_tile;
		rdis->drawTile(de->x, de->y, 1, tile);
		}
		break;
	case 'O':
		// GPS long
		snprintf(buf, 16, "%2.5f", gpsPos.lon);
		drawString(de,buf);
		break;
	case 'A':
		// GPS lat
		snprintf(buf, 16, "%2.5f", gpsPos.lat);
		drawString(de,buf);
		break;
	case 'H':
		// GPS alt
		snprintf(buf, 16, "%4dm", gpsPos.alt);
		drawString(de,buf);
		break;
	case 'C':
		// GPS Course over ground
		snprintf(buf, 4, "%3d", gpsPos.course);
		drawString(de, buf);
		break;
	case 'D':
		{
		// distance
		// equirectangular approximation is good enough
		if( !VALIDPOS(sonde.si()->d.validPos) ) {
			snprintf(buf, 16, "no pos ");
			if( de->extra[1]=='5') buf[5]=0;
		} else if( disp.gpsDist < 0 ) {
			snprintf(buf, 16, "no gps ");
			if( de->extra[1]=='5') buf[5]=0;
		} else {
			if( de->extra[1]=='5') { // 5-character version: ****m / ***km / **e6m
				if(disp.gpsDist>999999) snprintf(buf, 16, "%de6m  ", (int)(disp.gpsDist/1000000));
				if(disp.gpsDist>9999) snprintf(buf, 16, "%dkm   ", (int)(disp.gpsDist/1000));
				else snprintf(buf, 16, "%dm    ", (int)disp.gpsDist);
				buf[5]=0;
			} else { // 6-character version: *****m / ****km)
				if(disp.gpsDist>99999) snprintf(buf, 16, "%dkm    ", (int)(disp.gpsDist/1000));
				else snprintf(buf, 16, "%dm     ", (int)disp.gpsDist);
				buf[6]=0;
			}
		}
		drawString(de, buf);
		}
		break;
	case 'I':
		// dIrection
		if( disp.gpsDir<0 ) {  // 0..360 valid, -1 invalid
			drawString(de, "---");
			break;
		}
		snprintf(buf, 16, "%3d", disp.gpsDir);
		buf[3]=0;
		drawString(de, buf);
		if(de->extra[1]==(char)176)
			rdis->drawTile(de->x+3, de->y, 1, deg_tile);
		break;
	case 'B':
		// relative bearing
		if( disp.gpsBear < 0 ) {  // 0..360 valid, -1 invalid
			drawString(de, "---");
			break;
		}
		snprintf(buf, 16, "%3d", disp.gpsBear);
		buf[3]=0;
		drawString(de, buf);
		if(de->extra[1]==(char)176)
			rdis->drawTile(de->x+3, de->y, 1, deg_tile);
		break;
	case '0':
		// diagram
		{
		static int alpha = 0;
		alpha = (alpha+5)%360;
		struct CircleInfo *circinfo = (struct CircleInfo *)de->extra;
		int border = circinfo->brad;
		if(border<7) border=7; // space for "N" label
		int size = 1 + 2*circinfo->radius + 2*border;
		uint16_t *bitmap = (uint16_t *)malloc(sizeof(uint16_t) * size * size);
		Serial.printf("Drawing circle with size %d at %d,%d\n",size,de->x, de->y);
		for(int i=0; i<size*size; i++) { bitmap[i] = 0; }
		// draw circle
		int x0=size/2;
		int y0=x0;
		circ(bitmap, size, x0, y0, circinfo->radius, de->fg, true, de->bg);
		// 
		bool rxgood = (sonde.si()->rxStat[0]==0);
		int angN, angA, angB;   // angle of north, array, bullet
		int validA, validB;     // 0: no, 1: yes, -1: old
		if(circinfo->arr=='C') {  angA=gpsPos.course; validA=disp.gpsCourseOld?-1:1; }
		else { angA=disp.gpsDir; validA=sonde.si()->d.validPos?(rxgood?1:-1):0; }
		if(circinfo->bul=='C') {  angB=gpsPos.course; validB=disp.gpsCourseOld?-1:1; }
		else { angB=disp.gpsDir; validB=sonde.si()->d.validPos?(rxgood?1:-1):0; }
		if(circinfo->top=='N') {
			angN = 0;
		} else {
			//if (circinfo->top=='C') {
			angN = 360-gpsPos.course;
			angA += angN; if(angA>=360) angA-=360;
			angB += angN; if(angB>=360) angB-=360;
		}
		Serial.printf("GPS0: %c%c%c N=%d, A=%d, B=%d\n", circinfo->top, circinfo->arr, circinfo->bul, angN, angA, angB);
		// "N" in direction angN
#if 1
	// TODO
#else
		static_cast<ILI9225Display *>(rdis)->tft->drawGFXcharBM(x0 + circinfo->radius*sin(angN*PI/180)-6, y0 - circinfo->radius*cos(angN*PI/180)+7, 'N', 0xffff, bitmap, size, size);
#endif

		// small circle in direction angB
		if(validB) {
			circ(bitmap, size, x0+circinfo->radius*sin(angB*PI/180), y0-circinfo->radius*cos(angB*PI/180), circinfo->brad,
				circinfo->bcol, true, validB==1?circinfo->bcol:0);
		}
		rdis->drawBitmap(de->x, de->y, bitmap, size, size);
		// triangle in direction angA
		uint16_t xa,ya,xb,yb,xc,yc;
		float xf=sin(angA*PI/180);
		float yf=cos(angA*PI/180);
		xa = de->x + x0 + xf*circinfo->radius;
		ya = de->y + y0 - yf*circinfo->radius;
		xb = de->x + x0 + yf*circinfo->awidth;
		yb = de->y + y0 + xf*circinfo->awidth;
		xc = de->x + x0 - yf*circinfo->awidth;
		yc = de->y + y0 - xf*circinfo->awidth;
		Serial.printf("%d: %d,%d\n", alpha, xa, ya);
		if(validA==-1)
			rdis->drawTriangle(xa,ya,xb,yb,xc,yc,circinfo->acol, false);
		else if(validA==1)
			rdis->drawTriangle(xa,ya,xb,yb,xc,yc,circinfo->acol, true);
		free(bitmap);
		}
		break;
	case 'E':
		// elevation
		break;
	}
}

void Display::drawBatt(DispEntry *de) {
	float val;
	char buf[30];
	if (!axp192_found) {
		if (sonde.config.batt_adc<0) return;
		switch (de->extra[0])
		{
		case 'V':
				val = (float)(analogRead(sonde.config.batt_adc)) / 4095 * 2 * 3.3 * 1.1;
				snprintf(buf, 30, "%.2f%s", val, de->extra + 1);
			break;
		default:
			*buf = 0;
		}
		rdis->setFont(de->fmt);
		drawString(de, buf);
	} else {
	xSemaphoreTake( axpSemaphore, portMAX_DELAY );
	switch(de->extra[0]) {
	case 'S':
		if(!axp.isBatteryConnect()) { 
			if(axp.isVBUSPlug()) { strcpy(buf, "U"); }
			else { strcpy(buf, "N"); } // no battary
		}
		else if (axp.isChargeing()) { strcpy(buf, "C"); } // charging
		else { strcpy(buf, "B"); }  // battery, but not charging
		break;
	case 'V':
		val = axp.getBattVoltage();
		snprintf(buf, 30, "%.2f%s", val/1000, de->extra+1);
		break;
	case 'C':
		val = axp.getBattChargeCurrent();
		snprintf(buf, 30, "%.2f%s", val, de->extra+1);
		break;
	case 'D':
		val = axp.getBattDischargeCurrent();
		snprintf(buf, 30, "%.2f%s", val, de->extra+1);
		break;
	case 'U':
		if(sonde.config.type == TYPE_M5_CORE2) {
		  val = axp.getAcinVoltage();
		} else {
		  val = axp.getVbusVoltage();
		}
		snprintf(buf, 30, "%.2f%s", val/1000, de->extra+1);
		break;
	case 'I':
		if(sonde.config.type == TYPE_M5_CORE2) {
		  val = axp.getAcinCurrent();
		} else {
		  val = axp.getVbusCurrent();
		}
		snprintf(buf, 30, "%.2f%s", val, de->extra+1);
		break;
	case 'T':
		val = axp.getTemp();  // fixed in newer versions of libraray: -144.7 no longer needed here!
		snprintf(buf, 30, "%.2f%s", val, de->extra+1);
		break;
	default:
		*buf=0;
	}
	xSemaphoreGive( axpSemaphore );
        rdis->setFont(de->fmt);
	drawString(de, buf);
	}
}

void Display::drawText(DispEntry *de) {
        rdis->setFont(de->fmt);
	drawString(de, de->extra);
}

void Display::updateDisplayPos() {
	if( dispstate == 0 ) return; // do not display anything
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawLat && di->func != disp.drawLon) continue;
		di->func(di);
	}
}
void Display::updateDisplayPos2() {
	if( dispstate == 0 ) return; // do not display anything
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawAlt && di->func != disp.drawHS && di->func != disp.drawVS) continue;
		di->func(di);
	}
}
void Display::updateDisplayID() {
	if( dispstate == 0 ) return; // do not display anything
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawID) continue;
		di->func(di);
	}
}
void Display::updateDisplayRSSI() {
	if( dispstate == 0 ) return; // do not display anything
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawRSSI) continue;
		di->func(di);
	}
}
void Display::updateStat() {
	if( dispstate == 0 ) return; // do not display anything
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		if(di->func != disp.drawQS) continue;
		di->func(di);
	}
}

void Display::updateDisplayRXConfig() {
	if( dispstate == 0 ) return; // do not display anything
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
	if( dispstate == 0 ) return; // do not display anything
	calcGPS();
	for(DispEntry *di=layout->de; di->func != NULL; di++) {
		di->func(di);
	}
}

// Called when key is pressed or new RX starts
void Display::dispsavectlON() {
	// nothing to do to turn display on, may add power on code here later
	dispstate = 1;
}

// Should be called 1x / sec to update display
// parameter: rxactive (1=currently receiving something, 0=no rx)
void Display::dispsavectlOFF(int rxactive) {
	if( sonde.config.dispsaver == 0 ) return;  // screensaver disabled
	if( dispstate == 0 ) return; // already OFF
	if( rxactive && ((sonde.config.dispsaver%10)==2) ) return; // OFF only if no RX, but rxactive is 0
	dispstate++;
	if( dispstate > (sonde.config.dispsaver/10) ) {
		rdis->clear();
		dispstate = 0;
	}
}

void Display::setContrast() {
	if(sonde.config.dispcontrast<0) return;
	rdis->setContrast(sonde.config.dispcontrast);
}

Display disp = Display();

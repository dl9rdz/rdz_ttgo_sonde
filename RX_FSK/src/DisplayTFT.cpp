#include "../features.h"

#define TAG "Display"
#include "logger.h"

#include "DisplayTFT.h"
#include "DisplayGfxFonts.h"
#include "Sonde.h"

extern const char *version_name;
extern const char *version_id;

extern Sonde sonde;

extern SemaphoreHandle_t globalLock;
#define SPI_MUTEX_LOCK() \
	do                     \
{                      \
} while (xSemaphoreTake(globalLock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK() xSemaphoreGive(globalLock)

#define TFT_LED 0 // 0 if wired to +5V directly
#define TFT_BRIGHTNESS 100 // Initial brightness of TFT backlight (optional)

Arduino_DataBus *bus;

void TFTDisplay::begin() {
	LOG_I(TAG, "TFT display (ILI9225/ILI9341/ILI9342/ST7789) init\n");
	init_gfx_fonts();
	calc_gfx_offsets();
	// On the M5, the display and the Lora chip are on the same SPI interface (VSPI default pins),
	// we must use the same SPI bus with correct locking 
	// The bus init also must use the same miso as the sx1278 (even if we don't use miso at all for the display)
	if(sonde.config.type == TYPE_M5_CORE2 || sonde.config.type == TYPE_M5_CORE ) {
		LOG_D(TAG, "Type is M5 core/core2, using VSPI\n");
		bus = new Arduino_ESP32SPI( sonde.config.tft_rs, sonde.config.tft_cs,
				sonde.config.oled_scl, sonde.config.oled_sda, sonde.config.sx1278_miso, VSPI);
	} else if( sonde.config.oled_scl == sonde.config.sd.clk) {
		LOG_D(TAG, "Sharing with SDcard and TFT on HSPI\n");
		// if we share the bus between SDcard and TFT then we must configure MISO even if the display's MISO is not configured/used
		bus = new Arduino_ESP32SPI( sonde.config.tft_rs, sonde.config.tft_cs,
				sonde.config.oled_scl, sonde.config.oled_sda, sonde.config.sd.miso, HSPI);
        } else {
		LOG_D(TAG, "TFT on HSPI\n");
		bus = new Arduino_ESP32SPI( sonde.config.tft_rs, sonde.config.tft_cs,
				sonde.config.oled_scl, sonde.config.oled_sda, -1, HSPI);

	}
	if(_type == 3) 
		tft = new Arduino_ILI9341(bus, sonde.config.oled_rst);
	else if(_type == 4) 
		tft = new Arduino_ILI9342(bus, sonde.config.oled_rst);
	else if(_type == 5) 
		tft = new Arduino_ST7789(bus, sonde.config.oled_rst);
	else if(_type == 6)
		tft = new Arduino_ST7796(bus, sonde.config.oled_rst);
	else
		tft = new Arduino_ILI9225(bus, sonde.config.oled_rst);
	tft->begin(sonde.config.tft_spifreq);
	tft->fillScreen(BLACK);
	tft->setRotation(sonde.config.tft_orient);
	tft->setTextWrap(false);
	if(sonde.config.type == TYPE_M5_CORE2||sonde.config.type==TYPE_M5_CORE) 
		tft->invertDisplay(true);
	LOG_D(TAG, "TFT init: done\n");
}

void TFTDisplay::clear() {
	SPI_MUTEX_LOCK();
	tft->fillScreen(BLACK);
	SPI_MUTEX_UNLOCK();
}

void TFTDisplay::setContrast(uint8_t /*contrast*/) {
}

// for now, 0=small=FreeSans9pt7b, 1=large=FreeSans18pt7b
void TFTDisplay::setFont(uint8_t fontindex) {
	//if(fontindex==1 || fontindex==2) { fontindex=3; }
	findex = fontindex;
	switch(fontindex) {
		case 0: tft->setFont(); tft->setTextSize(1); break;
			//case 1: tft->setFont(NULL); tft->setTextSize(2); break;
			//case 2: tft->setFont(NULL); tft->setTextSize(2); break;
		default: tft->setFont(gfl[fontindex-1]);
	}
}

// 0=OLED/SSD1306, 1=ILI9225, 2=OLED/SH1106, 3=ILI9341, 4=ILI9342, 5=ST7789, 6=ST7796)
static uint16_t dispSize[7][2] = { {0,0}, {220,176}, {0,0}, {320,240}, {320,240}, {320,240}, {480,320} };

void TFTDisplay::getDispSize(uint8_t *height, uint8_t *width, uint8_t *lineskip, uint8_t *colskip) {
	if(height) *height = dispSize[sonde.config.disptype][1];
	if(width) *width = dispSize[sonde.config.disptype][0];
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
				if(lineskip&&colskip) { LOG_I(TAG, "skip size from bounds: %d, %d\n", *lineskip, *colskip); }
			}
	}
}



// Note: alignright means that there is a box from x to x+(-width), with text right-justified
//       x is the *left* corner! not the right...
void TFTDisplay::drawString(uint16_t x, uint16_t y, const char *s, int16_t width, uint16_t fg, uint16_t bg) {
	int16_t w,h;
	boolean alignright=false;
	char buf[50];
	utf2latin15(s, buf, 50);	

	if(width<0) {
		width = -width;
		alignright = true;
	}
	int16_t x1, y1;
	// Standard font
	if(findex==0) {
		SPI_MUTEX_LOCK();
		LOG_D(TAG, "Simple Text %s at %d,%d [%d]\n", s, x, y, width); 
		// for gpx fonts and new library, cursor is at baseline!!
		int h = 6;
		if( alignright ) {
			tft->getTextBounds(buf, x, y + gfxoffsets[findex-1].yofs, &x1, &y1, (uint16_t *)&w, (uint16_t *)&h);
			LOG_D(TAG, "Simple Text: width is %d\n", w);
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
	LOG_D(TAG, "GFX Text %s at %d,%d+%d in color %x, width=%d (w=%d)\n", s, x, y, gfxoffsets[findex-1].yofs, fg, width, w);
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
		LOG_D(TAG, "rtext fill %d %d %d %d -- %d %d\n", x, y, width-w, height, x1, y1);
	} else {
		// fill with bg from x+w to width
		if(width>w) tft->fillRect( x+w, y, width-w, height, bg);
		LOG_D(TAG, "ltext fill %d %d %d %d -- %d %d\n", x+w, y, width-w, height, x1, y1);
	}
#else
	uint16_t height = gfxoffsets[findex-1].yclear;
	uint16_t *bitmap = (uint16_t *)malloc(sizeof(uint16_t) * width * height);
	if(!bitmap) {
		LOG_E(TAG, "FATAL: OUT OF MEMORY when allocating bitmap\n");
		LOG_E(TAG, "w=%d, h=%d, s==%d\n",width, height, 2*width*height);
		heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
		return;
	}
	for(int i=0; i<width*height; i++) { bitmap[i] = bg; }   // fill with background
	int x0 = 0;
	if(alignright) { x0 = width - w; }
	int y0 = gfxoffsets[findex-1].yofs;
	LOG_D(TAG, "GFX: w=%d h=%d\n", width, height);
	for (uint8_t k = 0; k < strlen(s); k++) {	
		x0 += tft->drawGFXcharBM(x0, y0, s[k], fg, bitmap, width, height) + 1;
		LOG_D(TAG, "[%c->%d]",s[k],x0);
	}
	// TODO: if x+width exceeds display width, garbage is generated....
	drawBitmap(x, y, bitmap, width, height);
	free(bitmap);
#endif
#endif
	SPI_MUTEX_UNLOCK();
}

void TFTDisplay::drawTile(uint16_t x, uint16_t y, uint8_t cnt, uint8_t *tile_ptr) {
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

void TFTDisplay::drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color, boolean fill) {
	SPI_MUTEX_LOCK();
	if(fill)
		tft->fillTriangle(x1, y1, x2, y2, x3, y3, color);
	else
		tft->drawTriangle(x1, y1, x2, y2, x3, y3, color);
	SPI_MUTEX_UNLOCK();
}

void TFTDisplay::drawBitmap(uint16_t x1, uint16_t y1, const uint16_t* bitmap, int16_t w, int16_t h) {
	SPI_MUTEX_LOCK();
	tft->draw16bitRGBBitmap(x1, y1, bitmap, w, h);
	SPI_MUTEX_UNLOCK();
}

void TFTDisplay::drawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
	tft->drawFastVLine(x, y, h, color);
}

void TFTDisplay::welcome() {
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

void TFTDisplay::drawIP(uint16_t x, uint16_t y, int16_t width, uint16_t fg, uint16_t bg) {
	char buf[20];
	if(sonde.isAP) strcpy(buf, "A "); else *buf=0;   
	strncat(buf, sonde.ipaddr.c_str(), 16);
	drawString(x, y, buf, width, fg, bg);		
}

// size: 3=> 3x5 symbols; 4=> 4x7 symbols
void TFTDisplay::drawQS(uint16_t x, uint16_t y, uint8_t len, uint8_t size, uint8_t *stat, uint16_t fg, uint16_t bg) {
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

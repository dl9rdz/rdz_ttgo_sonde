#include <U8x8lib.h>
#include <U8g2lib.h>

#include "Display.h"
#include "Sonde.h"

extern Sonde sonde;

extern U8X8_SSD1306_128X64_NONAME_SW_I2C *u8x8;

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
   0x1F, 0x02, 0x04, 0x02, 0x1F, 0x40, 0x20, 0x10, 0x08, 0x04, 0x12, 0xA4, 0xA4, 0xA4, 0x40, 0x00
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


#define SETFONT(large) u8x8->setFont((large)?u8x8_font_7x14_1x2_r:u8x8_font_chroma48medium8_r);

/* Description of display layouts.
 * for each display, the content is described by a DispEntry structure
 * timeout values are in milliseconds, for view activ, rx signal present, no rx signal present
 * for each displey, actions (switching to different sonde or different view) can be defined
 * based on key presses or on expired timeouts
 */
DispEntry searchLayout[] = {
	{0, 0, FONT_LARGE, disp.drawText, "Scan:"},
	{0, 8, FONT_LARGE, disp.drawType, NULL},
	{3, 0, FONT_LARGE, disp.drawFreq, " MHz"},
	{5, 0, FONT_LARGE, disp.drawSite, NULL},
	{7, 5, 0, disp.drawIP, NULL},	
	{-1, -1, -1, NULL, NULL},
};
int16_t searchTimeouts[] = { -1, 0, 0 };
int8_t searchActions[] = {
	ACT_NONE,
	ACT_DISPLAY_DEFAULT, ACT_NONE, ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_NONE, ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_DISPLAY_DEFAULT, ACT_NEXTSONDE};
DispEntry legacyLayout[] = {
	{0, 5, FONT_SMALL, disp.drawFreq, " MHz"},
	{1, 8, FONT_SMALL, disp.drawAFC, NULL},
	{0, 0, FONT_SMALL, disp.drawType, NULL},
	{1, 0, FONT_SMALL, disp.drawID, NULL},
	{2, 0, FONT_LARGE, disp.drawLat, NULL},
	{4, 0, FONT_LARGE, disp.drawLon, NULL},
	{2, 10, FONT_SMALL, disp.drawAlt, NULL},
	{3, 10, FONT_SMALL, disp.drawHS, NULL},
	{4, 9, FONT_SMALL, disp.drawVS, NULL},
	{6, 0, FONT_LARGE, disp.drawRSSI, NULL},
	{6, 7, 0, disp.drawQS, NULL},
	{7, 5, 0, disp.drawIP, NULL},	
	{-1, -1, -1, NULL, NULL},
};
int16_t legacyTimeouts[] = { -1, -1, 20000 };
int8_t legacyActions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(2), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_DISPLAY(0)};
DispEntry fieldLayout[] = {
	{2, 0, FONT_LARGE, disp.drawLat, NULL},
	{4, 0, FONT_LARGE, disp.drawLon, NULL},
	{3, 10, FONT_SMALL, disp.drawHS, NULL},
	{4, 9, FONT_SMALL, disp.drawVS, NULL},
	{0, 0, FONT_LARGE, disp.drawID, NULL},
	{6, 0, FONT_LARGE, disp.drawAlt, NULL},
	{6, 7, 0, disp.drawQS, NULL},
	{-1, -1, -1, NULL, NULL},
};
int16_t fieldTimeouts[] = { -1, -1, -1 };
int8_t fieldActions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(3), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_NONE};
DispEntry field2Layout[] = {
	{2, 0, FONT_LARGE, disp.drawLat, NULL},
	{4, 0, FONT_LARGE, disp.drawLon, NULL},
	{1, 12, FONT_SMALL, disp.drawType, NULL},
	{0, 9, FONT_SMALL, disp.drawFreq, ""},
	{3, 10, FONT_SMALL, disp.drawHS, NULL},
	{4, 9, FONT_SMALL, disp.drawVS, NULL},
	{0, 0, FONT_LARGE, disp.drawID, NULL},
	{6, 0, FONT_LARGE, disp.drawAlt, NULL},
	{6, 7, 0, disp.drawQS, NULL},
	{-1, -1, -1, NULL, NULL},
};
int8_t field2Actions[] = {
	ACT_NONE,
	ACT_NEXTSONDE, ACT_DISPLAY(0), ACT_DISPLAY_SPECTRUM, ACT_DISPLAY_WIFI,
	ACT_DISPLAY(1), ACT_NONE, ACT_NONE, ACT_NONE,
	ACT_NONE, ACT_NONE, ACT_NONE};

DispInfo layouts[4] = {
  { searchLayout, searchActions, searchTimeouts },
  { legacyLayout, legacyActions, legacyTimeouts },
  { fieldLayout, fieldActions, fieldTimeouts },
  { field2Layout, field2Actions, fieldTimeouts } };

char Display::buf[17];

Display::Display() {
	setLayout(1);
}

void Display::setLayout(int layoutIdx) {
	layout = &layouts[layoutIdx];
}

void Display::drawLat(DispEntry *de) {
	SETFONT(de->fmt);
	if(!sonde.si()->validPos) {
	   u8x8->drawString(de->x,de->y,"<?""?>      ");
	   return;
	}
	snprintf(buf, 16, "%2.5f", sonde.si()->lat);
	u8x8->drawString(de->x,de->y,buf);
}
void Display::drawLon(DispEntry *de) {
	SETFONT(de->fmt);
	if(!sonde.si()->validPos) {
	   u8x8->drawString(de->x,de->y,"<?""?>      ");
	   return;
	}
	snprintf(buf, 16, "%2.5f", sonde.si()->lon);
	u8x8->drawString(de->x,de->y,buf);
}
void Display::drawAlt(DispEntry *de) {
	SETFONT(de->fmt);
	if(!sonde.si()->validPos) {
	   u8x8->drawString(de->x,de->y,"     ");
	   return;
	}
	snprintf(buf, 16, sonde.si()->alt>=1000?"   %5.0fm":"   %3.1fm", sonde.si()->alt);
	u8x8->drawString(de->x,de->y,buf+strlen(buf)-6);
}
void Display::drawHS(DispEntry *de) {
	SETFONT(de->fmt);
	if(!sonde.si()->validPos) {
	   u8x8->drawString(de->x,de->y,"     ");
	   return;
	}
	snprintf(buf, 16, sonde.si()->hs>99?" %3.0f":" %2.1f", sonde.si()->hs);
	u8x8->drawString(de->x,de->y,buf+strlen(buf)-4);
	u8x8->drawTile(de->x+4,de->y,2,kmh_tiles);
}
void Display::drawVS(DispEntry *de) {
	SETFONT(de->fmt);
	if(!sonde.si()->validPos) {
	   u8x8->drawString(de->x,de->y,"     ");
	   return;
	}
	snprintf(buf, 16, "  %+2.1f", sonde.si()->vs);
	u8x8->drawString(de->x, de->y, buf+strlen(buf)-5);
	u8x8->drawTile(de->x+5,de->y,2,ms_tiles);

}
void Display::drawID(DispEntry *de) {
	SETFONT((de->fmt&0x01));
	if(!sonde.si()->validID) {
		u8x8->drawString(de->x, de->y, "nnnnnnnn ");
		return;
	}
	u8x8->drawString(de->x, de->y, sonde.si()->id);
}
void Display::drawRSSI(DispEntry *de) {
	SETFONT(de->fmt);
	snprintf(buf, 16, "-%d   ", sonde.si()->rssi/2);
	int len=strlen(buf)-3;
	Serial.printf("drawRSSI: %d %d %d (%d)[%d]\n", de->y, de->x, sonde.si()->rssi/2, sonde.currentSonde, len);
	buf[5]=0;
	u8x8->drawString(de->x,de->y,buf);
	u8x8->drawTile(de->x+len, de->y, 1, (sonde.si()->rssi&1)?halfdb_tile1:empty_tile1);
	u8x8->drawTile(de->x+len, de->y+1, 1, (sonde.si()->rssi&1)?halfdb_tile2:empty_tile2);
}
void Display::drawQS(DispEntry *de) {
	uint8_t *stat = sonde.si()->rxStat;
	for(int i=0; i<18; i+=2) {
	        uint8_t tile[8];
	        *(uint32_t *)(&tile[0]) = *(uint32_t *)(&(stattiles[stat[i]]));
	        *(uint32_t *)(&tile[4]) = *(uint32_t *)(&(stattiles[stat[i+1]]));
	        u8x8->drawTile(de->x+i/2, de->y, 1, tile);
	}
}
void Display::drawType(DispEntry *de) {
	SETFONT(de->fmt);
        u8x8->drawString(de->x, de->y, sondeTypeStr[sonde.si()->type]);
}
void Display::drawFreq(DispEntry *de) {
	SETFONT(de->fmt);
        snprintf(buf, 16, "%3.3f%s", sonde.si()->freq, de->extra?de->extra:"");
        u8x8->drawString(de->x, de->y, buf);
}
void Display::drawAFC(DispEntry *de) {
 	if(!sonde.config.showafc) return;
	SETFONT(de->fmt);
	if(sonde.si()->afc==0) { strcpy(buf, "        "); }
	else { snprintf(buf, 15, "     %+3.2fk", sonde.si()->afc*0.001); }
        u8x8->drawString(de->x, de->y, buf+strlen(buf)-8);
}
void Display::drawIP(DispEntry *de) {
        u8x8->drawTile(de->x, de->y, 11, myIP_tiles);

}
void Display::drawSite(DispEntry *de) {
        SETFONT(de->fmt);
	u8x8->drawString(de->x, de->y, sonde.si()->launchsite);
}
void Display::drawTelemetry(DispEntry *de) {
}
void Display::drawGPSdist(DispEntry *de) {
}
void Display::drawText(DispEntry *de) {
        SETFONT(de->fmt);
	u8x8->drawString(de->x, de->y, de->extra);
}


void Display::clearIP() {
  memset(myIP_tiles, 0, 11*8);
}

void Display::setIP(const char *ip, bool AP) {
  memset(myIP_tiles, 0, 11*8);
  int len = strlen(ip);
  int pix = (len-3)*6+6;
  int tp = 80-pix+8;
  if(AP) memcpy(myIP_tiles+(tp<16?0:8), ap_tile, 8);
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

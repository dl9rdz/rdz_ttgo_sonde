#include <U8x8lib.h>
#include <U8g2lib.h>

#include "Sonde.h"

extern U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8;

SondeInfo si = { "RS41", 403.450, "P1234567", true, 48.1234, 14.9876, 543, 3.97, -0.5, true, 120 };

static unsigned char kmh_tiles[] U8X8_PROGMEM = {
   0x1F, 0x04, 0x0A, 0x11, 0x00, 0x1F, 0x02, 0x04, 0x42, 0x3F, 0x10, 0x08, 0xFC, 0x22, 0x20, 0xF8
   };
static unsigned char ms_tiles[] U8X8_PROGMEM = {
   0x1F, 0x02, 0x04, 0x02, 0x1F, 0x40, 0x20, 0x10, 0x08, 0x04, 0x12, 0xA4, 0xA4, 0xA4, 0x40, 0x00
   };

void Sonde::updateDisplayPos() {
	char buf[16];
	u8x8.setFont(u8x8_font_7x14_1x2_r);
	if(si.validPos) {
		snprintf(buf, 16, "%2.5f", si.lat);
		u8x8.drawString(0,2,buf);
		snprintf(buf, 16, "%2.5f", si.lon);
		u8x8.drawString(0,4,buf);
	} else {
		u8x8.drawString(0,2,"<??>     ");
		u8x8.drawString(0,4,"<??>     ");
	}
}

void Sonde::updateDisplayPos2() {
	char buf[16];
	u8x8.setFont(u8x8_font_chroma48medium8_r);
	if(!si.validPos) {
		u8x8.drawString(10,2,"      ");
		u8x8.drawString(10,3,"      ");
		u8x8.drawString(10,4,"      ");
		return;
	}
	snprintf(buf, 16, si.hei>999?"%5.0fm":"%3.1fm", si.hei);
	u8x8.drawString((10+6-strlen(buf)),2,buf);
	snprintf(buf, 16, si.hs>99?"%3.0f":"%2.1f", si.hs);
	u8x8.drawString((10+4-strlen(buf)),3,buf);
	snprintf(buf, 16, "%+2.1f", si.vs);
	u8x8.drawString((10+4-strlen(buf)),4,buf);
	u8x8.drawTile(14,3,2,kmh_tiles);
	u8x8.drawTile(14,4,2,ms_tiles);
}

void Sonde::updateDisplayID() {
        u8x8.setFont(u8x8_font_chroma48medium8_r);
	if(si.validID) {
        	u8x8.drawString(0,1, si.id);
	} else {
		u8x8.drawString(0,1, "nnnnnnnn        ");
	}
}
	
void Sonde::updateDisplayRSSI() {
	char buf[16];
	u8x8.setFont(u8x8_font_7x14_1x2_r);
	snprintf(buf, 16, "%ddB ", si.rssi);
	u8x8.drawString(0,6,buf);
}

void Sonde::updateDisplayRXConfig() {
	char buf[16];
	u8x8.setFont(u8x8_font_chroma48medium8_r);
	u8x8.drawString(0,0, si.type);
	snprintf(buf, 16, "%3.3f MHz", si.freq);
	u8x8.drawString(5,0, buf);

}

void Sonde::updateDisplay()
{
	char buf[16];
	updateDisplayRXConfig();
	updateDisplayID();
	updateDisplayPos();
	updateDisplayPos2();
	updateDisplayRSSI();
}

Sonde sonde = Sonde();

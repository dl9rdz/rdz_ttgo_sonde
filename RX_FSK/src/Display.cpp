#include "../features.h"

#define TAG "Display"
#include "logger.h"

#include <MicroNMEA.h>
#include <LittleFS.h>
#include "Display.h"
#include "DisplayU8x8.h"
#include "DisplayTFT.h"
#include "Sonde.h"
#include "pmu.h"


int readLine(Stream &stream, char *buffer, int maxlen);

extern const char *version_name;
extern const char *version_id;

extern Sonde sonde;

extern PMU *pmu;
extern SemaphoreHandle_t axpSemaphore;

static unsigned char kmh_tiles[] = {
	0x1F, 0x04, 0x0A, 0x11, 0x00, 0x1F, 0x02, 0x04, 0x42, 0x3F, 0x10, 0x08, 0xFC, 0x22, 0x20, 0xF8
};
static unsigned char ms_tiles[] = {
	0x1F, 0x02, 0x04, 0x02, 0x1F, 0x40, 0x20, 0x10, 0x08, 0x04, 0x12, 0xA8, 0xA8, 0xA4, 0x40, 0x00
};
unsigned char stattiles[5][4] =  {
	0x00, 0x1F, 0x00, 0x00 ,   // | == ok
	0x00, 0x10, 0x10, 0x00 ,   // . == no header found
	0x1F, 0x15, 0x15, 0x00 ,   // E == decode error
	0x00, 0x00, 0x00, 0x00 ,   // ' ' == unknown/unassigned
	0x07, 0x05, 0x07, 0x00 };  // ° = rx ok, but no valid position
unsigned char stattilesXL[5][5] =  {
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

void utf2latin15(const char *src, char *dst, int dstlen) {
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

#include <pgmspace.h>
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))


///////////////


char Display::buf[17];
char Display::lineBuf[Display::LINEBUFLEN];

RawDisplay *Display::rdis = NULL;

//TODO: maybe merge with initFromFile later?
void Display::init() {
	LOG_I(TAG, "init: disptype is %d\n",sonde.config.disptype);
	if(sonde.config.disptype==0 || sonde.config.disptype==2) {
		rdis = new U8x8Display(sonde.config.disptype);
	} else {
		rdis = new TFTDisplay(sonde.config.disptype);
	}
	LOG_D(TAG, "Display created\n");
	rdis->begin();
	delay(100);
	LOG_D(TAG, "Display initialized\n");
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
	LOG_I(TAG, "replaceLayouts: idx=%d n(new)=%d\n", layoutIdx, nLayouts);
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
	LOG_I(TAG, "%s: alloc %d bytes (%d entries) for %p (addr=%p)\n", label, totalsize, entries, d, d->de);
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
			break;
		case 'z':
			de->func = disp.drawInfo;
			de->extra = strdup(text+1);
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
			LOG_W(TAG, "parseDispElement: unknown: %c\n", type);
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
	printf("getting index: %d\n", index);

	index = getScreenIndex(index);  // auto selection for index==0
	snprintf(file, 20, "/screens%d.txt", index);
	LOG_I(TAG, "init: reading file %s\n", file);
	d = LittleFS.open(file, "r");
	if(!d || d.available()==0 ) {
		LOG_E(TAG, "init: %s not found\n", file);
		return;
	}

	DispInfo *newlayouts = (DispInfo *)malloc(MAXSCREENS * sizeof(DispInfo));
	if(!newlayouts) {
		LOG_E(TAG, "initFromFail: FAILED, not updating layouts\n");
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
	LOG_I(TAG, "Reading from screen config: available=%d\n", d.available());
	while(d.available()) {
		const char *ptr;
		readLine(d, lineBuf, LINEBUFLEN);
		const char *s = trim(lineBuf);
		//LOG_D(TAG, "Line: '%s'\n", s);
		if(*s == '#') continue;  // ignore comments
		switch(what) {
			case -1:	// wait for start of screen (@)
				{
					if(*s != '@') {
						if(*s==0 || *s==10 || *s==13) continue;
						LOG_I(TAG, "initFromFile: Illegal start of screen: %s\n", s);
						continue;
					}
					char *label = strdup(s+1);
					entrysize = countEntries(d);
					LOG_D(TAG, "Reading entry with %d elements\n", entrysize);
					idx++;
					int res = allocDispInfo(entrysize, &newlayouts[idx], label);
					if(res<0) {
						LOG_E(TAG, "Error allocating memory for disp info");
						continue;
					}
					what = 0;
				}
				break;
			default:	// parse content... (additional data or line `what`)
				if(strncmp(s,"timer=",6)==0) {  // timer values
					char t1[10],t2[10],t3[10];
					sscanf(s+6, "%5[0-9a-zA-Z-] , %5[0-9a-zA-Z-] , %5[0-9a-zA-Z-]", t1, t2, t3);
					LOG_D(TAG, "timers are %s, %s, %s\n", t1, t2, t3);
					newlayouts[idx].timeouts[0] = (*t1=='n'||*t1=='N')?sonde.config.norx_timeout:atoi(t1);
					newlayouts[idx].timeouts[1] = (*t2=='n'||*t2=='N')?sonde.config.norx_timeout:atoi(t2);
					newlayouts[idx].timeouts[2] = (*t3=='n'||*t3=='N')?sonde.config.norx_timeout:atoi(t3);
					// Code later assumes milliseconds, but config.txt and screens.txt use values in seconds
					if(newlayouts[idx].timeouts[0]>0) newlayouts[idx].timeouts[0]*=1000;
					if(newlayouts[idx].timeouts[1]>0) newlayouts[idx].timeouts[1]*=1000;
					if(newlayouts[idx].timeouts[2]>0) newlayouts[idx].timeouts[2]*=1000;
					//sscanf(s+6, "%hd,%hd,%hd", newlayouts[idx].timeouts, newlayouts[idx].timeouts+1, newlayouts[idx].timeouts+2);
					//LOG_I(TAG, "timer values: %d, %d, %d\n", newlayouts[idx].timeouts[0], newlayouts[idx].timeouts[1], newlayouts[idx].timeouts[2]);
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
					//LOG_I(TAG, "parsing key2action: %c %c %c %c\n", c1, c2, c3, c4);
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
					//LOG_D(TAG, "entry at %d,%d width=%d font %d, color=%x,%x\n", (int)x, (int)y, newlayouts[idx].de[what].width, newlayouts[idx].de[what].fmt,
					//		newlayouts[idx].de[what].fg, newlayouts[idx].de[what].bg);
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
	LOG_D(TAG, "setLayout: %d (max is %d)\n", newidx, nLayouts);
	if(newidx>=nLayouts) newidx = 0; 
	layout = &layouts[newidx];
	layoutIdx = newidx;
}

void Display::drawString(DispEntry *de, const char *str) {
	rdis->drawString(de->x, de->y, str, de->width, de->fg, de->bg);
}

void Display::drawInfo(DispEntry *de) {
	rdis->setFont(de->fmt);
        switch(de->extra[0]) {
	case 's':  // software name
		drawString(de, version_name);
		break;
	case 'v':  // software version
		drawString(de, version_id);
		break;
	}
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
	LOG_D(TAG, "drawVS: extra is %s width=%d\n", de->extra?de->extra:"<null>", de->width);
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
		LOG_D(TAG, "drawRSSI: %d %d %d (%d)[%d]\n", de->y, de->x, sonde.si()->rssi/2, sonde.currentSonde, len);
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
			snprintf(buf, 3, "%2d ", sonde.currentSonde+1);
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

	LOG_D(TAG, "GPS data: valid%d  GPS at %f,%f (alt=%d,cog=%d);  sonde at dist=%d, dir=%d rel.bear=%d\n",gpsPos.valid?1:0,
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
		case 'S':
			// GPS speed over ground
			// extra[1]: m for m/s, k for km/h
			// extra[2...]: additional text that is appended, like "m/s" or "km/h"
			//              if omitted, the m/s or km/h tile is appended on LCD displays (not TFT)
			if(!de->extra[1]) break; // missing m or k
			{
				float speed = gpsPos.speed;
				if(de->extra[1] != 'm') speed *= 3.6;
				snprintf(buf, 16, "%.2f%s", speed, de->extra+2);
				drawString(de, buf);
				if(!de->extra[2]) {
					rdis->drawTile(de->x+5, de->y, 2, de->extra[1]=='m' ? ms_tiles : kmh_tiles);
				}
			}
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
				LOG_D(TAG, "Drawing circle with size %d at %d,%d\n",size,de->x, de->y);
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
				LOG_D(TAG, "GPS0: %c%c%c N=%d, A=%d, B=%d\n", circinfo->top, circinfo->arr, circinfo->bul, angN, angA, angB);
				// "N" in direction angN
#if 1
				// TODO
#else
				static_cast<TFTDisplay *>(rdis)->tft->drawGFXcharBM(x0 + circinfo->radius*sin(angN*PI/180)-6, y0 - circinfo->radius*cos(angN*PI/180)+7, 'N', 0xffff, bitmap, size, size);
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
				LOG_D(TAG, "%d: %d,%d\n", alpha, xa, ya);
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
		if (!pmu) {
			if (sonde.config.batt_adc<0) return;
			switch (de->extra[0])
			{
				case 'V':
					val = getBattNoPMU();
					snprintf(buf, 30, "%.2f%s", val, de->extra + 1);
					break;
				default:
					*buf = 0;
			}
			rdis->setFont(de->fmt);
			drawString(de, buf);
			LOG_D(TAG, "drawBatt [%c]: %s\n", de->extra[0], buf);
			return;
		}
		*buf = 0;
		xSemaphoreTake( axpSemaphore, portMAX_DELAY );
		switch(de->extra[0]) {
			case 'S':
				if(!pmu->isBatteryConnected()) { 
					if(pmu->isVbusIn()) { strcpy(buf, "U"); }
					else { strcpy(buf, "N"); } // no battary
				}
				else if (pmu->isCharging()) { strcpy(buf, "C"); } // charging
				else { strcpy(buf, "B"); }  // battery, but not charging
				break;
			case 'V':
				val = pmu->getBattVoltage();
				snprintf(buf, 30, "%.2f%s", val/1000, de->extra+1);
				break;
			case 'U':
				if(sonde.config.type == TYPE_M5_CORE2) {
					val = pmu->getAcinVoltage();
				} else {
					val = pmu->getVbusVoltage();
				}
				if(val<0) { // make sure to clear old text...
					int len = strlen(de->extra)-1 + 4;
					strcpy(buf, "                    ");
					if(len>29) len=29;
					buf[len] = 0;
					break;
				}
				snprintf(buf, 30, "%.2f%s", val/1000, de->extra+1);
				break;
			case 'T':
				val = pmu->getTemperature();
				snprintf(buf, 30, "%.2f%s", val, de->extra+1);
				break;
		}
		if(pmu->type==TYPE_AXP192) {
			switch(de->extra[0]) {
				case 'C':
					val = pmu->getBattChargeCurrent();
					snprintf(buf, 30, "%.2f%s", val, de->extra+1);
					break;
				case 'D':
					val = pmu->getBattDischargeCurrent();
					snprintf(buf, 30, "%.2f%s", val, de->extra+1);
					break;
				case 'I':
					if(sonde.config.type == TYPE_M5_CORE2) {
						val = pmu->getAcinCurrent();
					} else {
						val = pmu->getVbusCurrent();
					}
					snprintf(buf, 30, "%.2f%s", val, de->extra+1);
					break;
			}   
		}
		else if (pmu->type == TYPE_AXP2101) {
			// AXP2101-specific items here... I guess there are none for now...
		}
		xSemaphoreGive( axpSemaphore );
		rdis->setFont(de->fmt);
		drawString(de, buf);
		LOG_D(TAG, "drawBatt [%c]: %s\n", de->extra[0], buf);
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
			LOG_D(TAG, "updateDisplayIP: %d %d\n",di->x, di->y);
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
		LOG_D(TAG, "DISP SAVE: ON");
		dispstate = 1;
	}

	// Should be called 1x / sec to update display
	// parameter: rxactive (1=currently receiving something, 0=no rx)
	void Display::dispsavectlOFF(int rxactive) {
		LOG_D(TAG, "DISP SAVE: OFF %d (state is %d)\n", rxactive, dispstate);
		if( sonde.config.dispsaver == 0 ) return;  // screensaver disabled
		if( dispstate == 0 ) return; // already OFF
		if( rxactive && ((sonde.config.dispsaver%10)==2) ) return; // OFF only if no RX, but rxactive is 0
		dispstate++;
		LOG_D(TAG, "dispstate is %d\n", dispstate);
		if( dispstate > (sonde.config.dispsaver/10) ) {
			rdis->clear();
			dispstate = 0;
			LOG_D(TAG, "Clearning display\n");
		}
	}

	void Display::setContrast() {
		if(sonde.config.dispcontrast<0) return;
		rdis->setContrast(sonde.config.dispcontrast);
	}

	Display disp = Display();

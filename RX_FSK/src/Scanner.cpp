#include "Scanner.h"

#include <U8x8lib.h>

#include "SX1278FSK.h"
#include "Sonde.h"
#include "Display.h"


double STARTF;


struct scancfg {
	int PLOT_W;		// Width of plot, in pixel
	int PLOT_H8;		// Height of plot, in 8 pixel units
	int TICK1;		// Pixel per MHz marker
	int TICK2;		// Pixel per sub-Mhz marker (250k or 200k)
	double CHANSTEP;	// Scanner frequenz steps
	int SMPL_PIX;		// Frequency steps per pixel
	int NCHAN;		// number of channels to scan, PLOT_W * SMPL_PIX
	int SMOOTH;
	int ADDWAIT;
	int VSCALE;
};

//struct scancfg scanLCD={ 121, 7,  120/6, 120/6/4, 6000.0/120.0/20.0, 20, 120*20, 1 };
struct scancfg scanLCD={ 121, 7,  120/6, 120/6/4, 6000.0/120.0/10.0, 10, 120*10, 2, 40, 1 };
struct scancfg scanTFT={ 210, 16, 210/6, 210/6/5, 6000.0/210.0/10.0, 10, 210*10, 1, 0, 1 };
struct scancfg scan934x={ 300, 22, 300/6, 300/6/5, 6000.0/300.0/7.0, 7, 300*5, 1, 10, 2 };

struct scancfg &scanconfig = scanTFT;

#define CHANBW 12500
//#define PIXSAMPL (50/CHANBW)
//#define STARTF 401000000

// max of 120*5 and 210*3
//#define MAXN 210*10
//#define MAXN 120*20
#define MAXN 300*10

// max of 120 and 210 (ceil(210/8)*8)) -- now ceil(300/8)*8
//#define MAXDISP 216
#define MAXDISP 304

int scanresult[MAXN];
int scandisp[MAXDISP];
double peakf=0;

//#define PLOT_MIN -250
#define PLOT_MIN (sonde.config.noisefloor*2)
#define PLOT_SCALE(x) (x<PLOT_MIN?0:(x-PLOT_MIN)/2)

const byte tilepatterns[9]={0,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF};
void Scanner::fillTiles(uint8_t *row, int value) {
	for(int y=0; y<scanconfig.PLOT_H8; y++) {
		int nbits = scanconfig.VSCALE*value - 8*(scanconfig.PLOT_H8-1-y);
		if(nbits<0) { row[8*y]=0; continue; }
		if(nbits>=8) { row[8*y]=255; continue; }
		row[8*y] = tilepatterns[nbits];
	}
}
/* LCD:
 * There are 16*8 columns to plot, NPLOT must be lower than that
 * currently, we use 128 * 50kHz channels
 * There are 8*8 values to plot; MIN is bottom end, 
 * TFT:
 * There are 210 columns to plot
 * Currently we use 210 * (6000/120)kHz channels, i.e. 28.5714kHz
 */
///// unused????  uint8_t tiles[16] = { 0x0f,0x0f,0x0f,0x0f,0xf0,0xf0,0xf0,0xf0, 1, 3, 7, 15, 31, 63, 127, 255};

// type 0: lcd, 1: tft(ILI9225), 2: lcd(sh1106) 3:TFT(ili9341), 4: TFT(ili9342)
#define ISTFT (sonde.config.disptype!=0 && sonde.config.disptype!=2)
void Scanner::plotResult()
{
	int yofs = 0;
	char buf[30];
	if(ISTFT) {
		yofs = 2;
  		if (sonde.config.marker != 0) {
    			itoa((sonde.config.startfreq), buf, 10);
    			disp.rdis->drawString(0, 1, buf);
    			disp.rdis->drawString(scanconfig.PLOT_W/2-10, 1, "MHz");
    			itoa((sonde.config.startfreq + 6), buf, 10);
    			disp.rdis->drawString(scanconfig.PLOT_W-15, 1, buf);
		}	
	}
	else {
  		if (sonde.config.marker != 0) {
    			itoa((sonde.config.startfreq), buf, 10);
    			disp.rdis->drawString(0, 1, buf);
    			disp.rdis->drawString(7, 1, "MHz");
    			itoa((sonde.config.startfreq + 6), buf, 10);
    			disp.rdis->drawString(13, 1, buf);
		}	
  	}
	uint8_t row[scanconfig.PLOT_H8*8];
	for(int i=0; i<scanconfig.PLOT_W; i+=8) {
		for(int j=0; j<8; j++) {
			fillTiles(row+j, PLOT_SCALE(scandisp[i+j]));
			if( (i+j)>=scanconfig.PLOT_W ) { for(int y=0; y<scanconfig.PLOT_H8; y++) row[j+8*y]=0; }
		        if( ((i+j)%scanconfig.TICK1)==0) { row[j] |= 0x07; }
		        if( ((i+j)%scanconfig.TICK2)==0) { row[j] |= 0x01; }
		}
		for(int y=0; y<scanconfig.PLOT_H8; y++) {
			if(sonde.config.marker && y==1 && !ISTFT ) {
				// don't overwrite MHz marker text
				if(i<3*8 || (i>=7*8&&i<10*8) || i>=13*8) continue;
			}
			disp.rdis->drawTile(i/8, y+yofs, 1, row+8*y);
		}
	}
	if(ISTFT) { // large TFT
		sprintf(buf, "Peak: %03.3f MHz", peakf*0.000001);	
		disp.rdis->drawString(0, (yofs+scanconfig.PLOT_H8+1)*8, buf);
	} else {
		sprintf(buf, "Peak: %03.3fMHz", peakf*0.000001);	
		disp.rdis->drawString(0, 7, buf);
	}
}

void Scanner::scan()
{
	if(!ISTFT) { // LCD small
		scanconfig = scanLCD;
	} else if (sonde.config.disptype==1) {
		scanconfig = scanTFT;
	} else {
		scanconfig = scan934x;
	}
	// Configure 
 	STARTF = (sonde.config.startfreq * 1000000);
	sx1278.writeRegister(REG_PLL_HOP, 0x80);   // FastHopOn
	sx1278.setRxBandwidth((int)(scanconfig.CHANSTEP*1000));
	double bw = sx1278.getRxBandwidth();
	Serial.print("RX Bandwith for scan: "); Serial.println(bw);
	sx1278.writeRegister(REG_RSSI_CONFIG, scanconfig.SMOOTH&0x07);
	sx1278.setFrequency(STARTF);
	Serial.print("Start freq = "); Serial.println(STARTF);
	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

	unsigned long start = millis();
	uint32_t lastfrf= STARTF * (1<<19) / SX127X_CRYSTAL_FREQ;
	float freq = STARTF;
	int wait = scanconfig.ADDWAIT + 20 + 1000*(1<<(scanconfig.SMOOTH+1))/4/(0.001*CHANBW);
	Serial.print("wait time (us) is: "); Serial.println(wait);
	for(int iter=0; iter<3; iter++) {   // three interations, to catch all RS41 transmissions
	    delayMicroseconds(20000); yield();
	    for(int i=0; i<scanconfig.PLOT_W*scanconfig.SMPL_PIX; i++) {
		freq = STARTF + 1000.0*i*scanconfig.CHANSTEP;
		//freq = 404000000 + 100*i*scanconfig.CHANSTEP;
		uint32_t frf = freq * 1.0 * (1<<19) / SX127X_CRYSTAL_FREQ;
		if( (lastfrf>>16)!=(frf>>16) ) {
        		sx1278.writeRegister(REG_FRF_MSB, (frf&0xff0000)>>16);
		}
		if( ((lastfrf&0x00ff00)>>8) != ((frf&0x00ff00)>>8) ) {
        		sx1278.writeRegister(REG_FRF_MID, (frf&0x00ff00)>>8);
		}
        	sx1278.writeRegister(REG_FRF_LSB, (frf&0x0000ff));
		lastfrf = frf;
		// Wait TS_HOP (20us) + TS_RSSI ( 2^(scacconfig.SMOOTH+1) / 4 / CHANBW us)
		delayMicroseconds(wait);
		int rssi = -(int)sx1278.readRegister(REG_RSSI_VALUE_FSK);
		if(iter==0) { scanresult[i] = rssi; } else {
			if(rssi>scanresult[i]) scanresult[i]=rssi;
		}
	    }
	}
	yield();
	unsigned long duration = millis()-start;
	Serial.print("wait: ");
	Serial.println(wait);
	Serial.print("Scan time: ");
	Serial.println(duration);
	Serial.print("Final freq: ");
	Serial.println(freq);
	int peakidx=-1;
	int peakres=-9999;
	for(int i=0; i<scanconfig.PLOT_W; i+=1) {
		int r=scanresult[i*scanconfig.SMPL_PIX];
                if(r>peakres+1) { peakres=r; peakidx=i*scanconfig.SMPL_PIX; }
		scandisp[i] = r;
		for(int j=1; j<scanconfig.SMPL_PIX; j++) { 
			r = scanresult[i*scanconfig.SMPL_PIX+j]; 
			scandisp[i]+=r;
			if(r>peakres+1) { peakres=r; peakidx=i*scanconfig.SMPL_PIX+j; }
		}
		//for(int j=1; j<PIXSAMPL; j++) { if(scanresult[i+j]>scandisp[i/PIXSAMPL]) scandisp[i/PIXSAMPL] = scanresult[i+j]; }
		Serial.print(scanresult[i]); Serial.print(", ");
	}
	peakidx--;
	double newpeakf = STARTF + scanconfig.CHANSTEP*1000.0*peakidx;
	if(newpeakf<peakf-20000 || newpeakf>peakf+20000) peakf=newpeakf; 		// different frequency
	else if (newpeakf < peakf) peakf = 0.75*newpeakf + 0.25*peakf;		// averaging on frequency, some bias towards lower...
	else peakf = 0.25*newpeakf + 0.75*peakf;
	Serial.println("\n");
	for(int i=0; i<scanconfig.PLOT_W; i++) { 
		scandisp[i]/=scanconfig.SMPL_PIX;
                Serial.print(scandisp[i]); Serial.print(", ");
	}
	Serial.println("\n");
	Serial.print("Peak: ");
	Serial.print(peakf);
}

Scanner scanner = Scanner();

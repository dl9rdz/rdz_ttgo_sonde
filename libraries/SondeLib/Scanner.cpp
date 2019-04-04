#include "Scanner.h"
#include <SX1278FSK.h>

#define CHANBW 7.5
#define SMOOTH 1
#define STARTF 400000000
#define NCHAN ((int)(6000/CHANBW))

int scanresult[NCHAN];
int scandisp[NCHAN/2];

void Scanner::scan()
{
	// Configure 
	sx1278.writeRegister(REG_PLL_HOP, 0x80);   // FastHopOn
	sx1278.setRxBandwidth(CHANBW*1000);
	sx1278.writeRegister(REG_RSSI_CONFIG, SMOOTH&0x07);
	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

	unsigned long start = millis();
	uint32_t lastfrf=-1;
	for(int i=0; i<NCHAN; i++) {
		float freq = STARTF + 1000.0*i*CHANBW;
		uint32_t frf = freq * 1.0 * (1<<19) / SX127X_CRYSTAL_FREQ;
		if( (lastfrf>>16)!=(frf>>16) ) {
        		sx1278.writeRegister(REG_FRF_MSB, (frf&0xff0000)>>16);
		}
		if( ((lastfrf&0x00ff00)>>8) != ((frf&0x00ff00)>>8) ) {
        		sx1278.writeRegister(REG_FRF_MID, (frf&0x00ff00)>>8);
		}
        	sx1278.writeRegister(REG_FRF_LSB, (frf&0x0000ff));
		lastfrf = frf;
		// Wait TS_HOP (20us) + TS_RSSI ( 2^(SMOOTH+1) / 4 / CHANBW us)
		int wait = 20 + 1000*(1<<(SMOOTH+1))/4/CHANBW;
		delayMicroseconds(wait);
		int rssi = -sx1278.readRegister(REG_RSSI_VALUE_FSK)/2;
		scanresult[i] = rssi;
	}
	unsigned long duration = millis()-start;
	Serial.print("Scan time: ");
	Serial.println(duration);
	for(int i=0; i<NCHAN; i+=2) {
		scandisp[i/2] = scanresult[i];
		for(int j=1; j<2; j++) { if(scanresult[i+j]>scandisp[i/2]) scandisp[i/2] = scanresult[i+j]; }
		Serial.print(scanresult[i]); Serial.print(", ");
		if(((i+1)%32) == 0) Serial.println();
	}
	Serial.println("\n");
}

Scanner scanner = Scanner();

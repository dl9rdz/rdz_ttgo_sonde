
/* M10 and M20 decoder functions */

#include "M10M20.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

#define M10M20_DEBUG 1

#if M10M20_DEBUG
#define M10M20_DBG(x) x
#else
#define M10M20_DBG(x)
#endif


static byte data1[512];
static byte *dataptr=data1;

static uint8_t rxbitc;
static uint16_t rxbyte;
static int rxp=0;
static int haveNewFrame = 0;
//static int lastFrame = 0;
static int headerDetected = 0;

int M10M20::setup(float frequency) 
{
	M10M20_DBG(Serial.println("Setup sx1278 for M10/M20 sonde"));;
	if(sx1278.ON()!=0) {
		M10M20_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	// setFSK: switches to FSK standby mode
	if(sx1278.setFSK()!=0) {
		M10M20_DBG(Serial.println("Setting FSK mode FAILED"));
		return 1;
	}
        Serial.print("M10/M20: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
	// Test: maybe fix issue after spectrum display?
	sx1278.writeRegister(REG_PLL_HOP, 0);

        if(sx1278.setAFCBandwidth(sonde.config.m10m20.agcbw)!=0) {
                M10M20_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.m10m20.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.m10m20.rxbw)!=0) {
                M10M20_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.m10m20.rxbw));
                return 1;
        }

/// TODO: Maybe do this conditionally? -- maybe skip if afc if agcbw set to 0 or -1? 
//// Step 1: Tentative AFC mode
        sx1278.clearIRQFlags();
        // preamble detector + AFC + AGC on
        // wait for preamble interrupt within 2sec
        sx1278.setBitrate(4800);
        // DetectorOn=1, Preamble detector size 01, preamble tol 0x0A (10)
        sx1278.setPreambleDetect(0x80 | 0x20 | 0x0A);
        // Manual start RX, Enable Auto-AFC, Auto-AGC, RX Trigger (AGC+AFC)by preamble
        sx1278.setRxConf(0x20 | 0x10 | 0x08 | 0x06);
        // Packet config 1: fixed len, no mancecer, no crc, no address filter
        // Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
        if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
                M10M20_DBG(Serial.println("Setting Packet config FAILED"));
                return 1;
        }
        // enable RX
        sx1278.setPayloadLength(0);
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
        unsigned long t0 = millis();
        M10M20_DBG(Serial.printf("M10M20::setup() AFC preamble search start at %ld\n",t0));
        while( millis() - t0 < 1000 ) {
                uint8_t value = sx1278.readRegister(REG_IRQ_FLAGS1);
               if(value & 2) {
                       int32_t afc = sx1278.getAFC();
                       int16_t rssi = sx1278.getRSSI();
                       Serial.printf("M10M20::setup: preamble: AFC is %d, RSSI is %.1f\n", afc, rssi/2.0);
                       sonde.sondeList[rxtask.currentSonde].rssi = rssi;
                       sonde.sondeList[rxtask.currentSonde].afc = afc;
                       break;
               }
               yield();
        }
        if( millis() - t0 >= 1000) {
               Serial.println("Preamble scan for AFC: TIMEOUT\n");
               return 1; // no preamble, so we may fail fast....
        }

//// Step 2: Real reception
	// FSK standby mode, seems like otherweise baudrate cannot be changed?
	sx1278.setFSK();
	if(sx1278.setBitrate(9600)!=0) {
		M10M20_DBG(Serial.println("Setting bitrate 9600bit/s FAILED"));
		return 1;
	}
	M10M20_DBG(Serial.printf("Exact bitrate is %f\n", sx1278.getBitrate()));
	// Probably not necessary, as this was set before
        if(sx1278.setAFCBandwidth(sonde.config.m10m20.agcbw)!=0) {
                M10M20_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.m10m20.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.m10m20.rxbw)!=0) {
                M10M20_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.m10m20.rxbw));
                return 1;
        }

	///// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	//if(sx1278.setRxConf(0x1E)!=0) {
	// Disable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x00)!=0) {
		M10M20_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// version 1, working with continuous RX
	const char *SYNC="\x66\x66";
	if(sx1278.setSyncConf(0x70, 1, (const uint8_t *)SYNC)!=0) {
		M10M20_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
        // Preamble detection off (+ size 1 byte, maximum tolerance; should not matter for "off"?)
        if(sx1278.setPreambleDetect(0x00 | 0x00 | 0x1F)!=0) {
		M10M20_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		M10M20_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

        // enable RX
        sx1278.setPayloadLength(0);  // infinite for now...
        sx1278.setRxConf(0x20);
	uint16_t afc = sx1278.getRawAFC();
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	delay(50);
	sx1278.setRawAFC(afc);
	delay(50);
        Serial.printf("after RX_MODE: AFC is %d\n", sx1278.getAFC());

#if M10M20_DEBUG
	M10M20_DBG(Serial.println("Setting SX1278 config for M10 finished\n"); Serial.println());
#endif
        return res;
}


M10M20::M10M20() {
}

#define M10_FRAMELEN 101
#define M10_CRCPOS 99

#define M20_FRAMELEN 88
#define M20_CRCPOSB 22

void M10M20::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X ", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

static uint16_t update_checkM10M20(int c, uint8_t b) {
	int c0, c1, t, t6, t7, s;
	c1 = c & 0xFF;
	// B
	b  = (b >> 1) | ((b & 1) << 7);
	b ^= (b >> 2) & 0xFF;
	// A1
	t6 = ( c     & 1) ^ ((c >> 2) & 1) ^ ((c >> 4) & 1);
	t7 = ((c >> 1) & 1) ^ ((c >> 3) & 1) ^ ((c >> 5) & 1);
	t = (c & 0x3F) | (t6 << 6) | (t7 << 7);
	// A2
	s  = (c >> 7) & 0xFF;
	s ^= (s >> 2) & 0xFF;
	c0 = b ^ t ^ s;
	return ((c1 << 8) | c0) & 0xFFFF;
}
static uint16_t crc_M10M20(int len, uint8_t *msg) {
	uint16_t cs = 0;
	for (int i = 0; i < len; i++) {
		cs = update_checkM10M20(cs, msg[i]);
	}
	return cs;
}
static bool checkM10M20crc(int crcpos, uint8_t *msg) {
	uint16_t cs, cs1;
	cs = crc_M10M20(crcpos, msg);
	cs1 = (msg[crcpos] << 8) | msg[crcpos+1];
	return (cs1 == cs);
}

typedef uint32_t SET256[8];
static SET256 sondeudp_VARSET = {0x03BBBBF0UL,0x80600000UL,0x06A001A0UL,
                0x0000001CUL,0x00000000UL,0x00000000UL,0x00000000UL,
                0x00000000UL};
// VARSET=SET256{4..9,11..13,15..17,19..21,23..25,53..54,63,69,71,72,85,87,89,90,98..100}; 
static SET256 sondeudp_VARSETM20 = {0xF3E27F54UL,0x0000000FUL,0x00000030UL,
                0x00000000UL, 0x00444C39UL, 0x53445A00UL, 0x00000000UL,
                0x00000000UL};
// VARSET=SET256{2,4,6,8..10,11..14,17,21..25,28..35,68,69}; (* known as variable *)

static uint8_t fixcnt[M10_FRAMELEN];
static uint8_t fixbytes[M10_FRAMELEN];

static int32_t getint32(uint8_t *data) {
	return (int32_t)( data[3]|(data[2]<<8)|(data[1]<<16)|(data[0]<<24) );
}

static int32_t getint24(uint8_t *data) {
    return (int32_t)(data[2]|(data[1]<<8)|(data[0]<<16) );
}

static int16_t getint16(uint8_t *data) {
	return (int16_t)(data[1]|((uint16_t)data[0]<<8));
}

static int16_t getint16_r(uint8_t *data) {
	return (int16_t)(((uint16_t)data[1]<<8) |data[0]);
}

static char dez(uint8_t nr) {
	nr = nr%10;
	return '0'+nr;
}
static char hex(uint8_t nr) {
	nr = nr&0x0f;
	if(nr<10) return '0'+nr;
	else return 'A'+nr-10;
}
const static float DEGMUL = 1.0/0xB60B60;

#define VMUL 0.005
#define VMUL_M20 0.01
#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
#define RAD (PI/180)


// ret: 1=frame ok; 2=frame with errors; 0=ignored frame (m10dop-alternativ)
int M10M20::decodeframeM10(uint8_t *data) {
	int repairstep = 16;
	int repl = 0;
	bool crcok;
	// error correction, inspired by oe5dxl's sondeudp
	do {
		crcok = checkM10M20crc(M10_CRCPOS, data);
		if(crcok || repairstep==0) break;
		repl = 0;
		for(int i=0; i<M10_CRCPOS; i++) {
			if( ((sondeudp_VARSET[i/32]&(1<<(i%32))) == 0)  && (fixcnt[i]>=repairstep) ) {
				repl++;	
				data[i] = fixbytes[i];
			}
		}
		repairstep >>= 1;
	} while(true);
	if(crcok) {
		for(int i=0; i<M10_CRCPOS; i++) {
			if(fixbytes[i]==data[i] &&fixcnt[i]<255) fixcnt[i]++;
			else { fixcnt[i]=0; fixbytes[i]=data[i]; }
		}
	}
	Serial.println(crcok?"CRC OK":"CRC NOT OK");
	Serial.printf(" repair: %d/%d\n", repl, repairstep);
	if(!crcok) return 2;

	if(data[1]==0x9F && data[2]==0x20) {
		Serial.println("Decoding...");
		//SondeInfo *si = sonde.si();
		SondeData *si = &(sonde.si()->d);
		
		// Its a M10
		// getid...
		char ids[12];
		ids[0] = 'M';
		ids[1] = 'E';
		ids[2] = hex(data[95]/16);
		ids[3] = hex(data[95]);
		ids[4] = hex(data[93]);
		uint32_t id = data[96] + data[97]*256;
		ids[5] = hex(id/4096);
		ids[6] = hex(id/256);
		ids[7] = hex(id/16);
		ids[8] = hex(id);
		ids[9] = 0;
		strncpy(si->id, ids, 10);
		ids[0] = hex(data[95]/16);
		ids[1] = dez((data[95]&0x0f)/10);
		ids[2] = dez((data[95]&0x0f));
		ids[3] = '-';
		ids[4] = dez(data[93]);
		ids[5] = '-';
		ids[6] = dez(id>>13);
		id &= 0x1fff;
		ids[7] = dez(id/1000); 
		ids[8] = dez((id/100)%10);
		ids[9] = dez((id/10)%10);
		ids[10] = dez(id%10);
		ids[11] = 0;
		strncpy(si->ser, ids, 12);
		si->validID = true;
		Serial.printf("ID is %s [%02x %02x %d]\n", ids, data[95], data[93], id);
		// ID printed on sonde is ...-.-abbbb, with a=id>>13, bbbb=id&0x1fff in decimal
		// position data
		si->lat = getint32(data+14) * DEGMUL;
		si->lon = getint32(data+18) * DEGMUL;
		si->alt = getint32(data+22) * 0.001;
		float ve = getint16(data+4)*VMUL;
		float vn = getint16(data+6)*VMUL;
		si->vs = getint16(data+8) * VMUL;
		si->hs = sqrt(ve*ve+vn*vn);
		si->sats = data[30];
		float dir = atan2(ve, vn)*(1.0/RAD);
		if(dir<0) dir+=360;
		si->dir = dir;
		si->validPos = 0x3f;
		// m10 temp
		const float p0 = 1.07303516e-03, p1 = 2.41296733e-04, p2 = 2.26744154e-06, p3 = 6.52855181e-08;
  		const float Rs[3] = { 12.1e3 ,  36.5e3 ,  475.0e3 }; 
  		const float Rp[3] = { 1e20   , 330.0e3 , 2000.0e3 };
		uint8_t sct = data[62];
		float rt = getint16_r(data+63) & (0xFFF);
		float T = NAN;
		if(rt!=0 && sct<3) {
			rt = (4095-rt)/rt - (Rs[sct]/Rp[sct]);
			if(rt>0) {
				rt = Rs[sct] / rt;
				if(rt>0) {
					rt = log(rt);
					rt = 1/( p0 + p1*rt + p2*rt*rt + p3*rt*rt*rt ) - 273.15;
					if(rt>-99 && rt<50) { T = rt; }
				}
			}
		}
		si->temperature = T;

 		uint32_t gpstime = getint32(data+10);
                uint16_t gpsweek = getint16(data+32);
                        // UTC is GPSTIME - 18s (24*60*60-18 = 86382)
                        // one week = 7*24*60*60 = 604800 seconds
                        // unix epoch starts jan 1st 1970 0:00
                        // gps time starts jan 6, 1980 0:00. thats 315964800 epoch seconds.
                        // subtracting 86400 yields 315878400UL
                si->time = (gpstime/1000) + 86382 + gpsweek*604800 + 315878400UL;
		// consistent with autorx, vframe is based on GPS time without the -18 seconds adjustment 
		// for the GPS time / UTC time difference (included in 86382 above)
		si->vframe = si->time - 315964800 + 18;
                si->validTime = true;
	} else {
		Serial.printf("data is %02x %02x %02x\n", data[0], data[1], data[2]);
		return 0;
	}
	return 1;
}

static uint32_t rxdata;
static bool rxsearching=true;
static bool isM20=false;

// search for
// //101001100110011010011010011001100110100110101010100110101001
// //1010011001100110100110100110 0110.0110 1001.1010 1010.1001 1010.1001 => 0x669AA9A9
void M10M20::processM10data(uint8_t dt)
{
	for(int i=0; i<8; i++) {
		uint8_t d = (dt&0x80)?1:0;
		dt <<= 1;
		rxdata = (rxdata<<1) | d;
		if( (rxbitc&1)==0 ) {
			// "bit1"
			rxbyte = (rxbyte<<1) | d;
		} else {
			// "bit2" ==> 01 or 10 => 1, otherweise => 0
			rxbyte = rxbyte ^ d;
		}
		//
		if(rxsearching) {
			if( rxdata == 0xcccca64c || rxdata == 0x333359b3 ) {
				rxsearching = false;
				rxbitc = 0;
				rxp = 0;
				isM20 = false;
				headerDetected = 1;
#if 1
                                int rssi=sx1278.getRSSI();
                                int fei=sx1278.getFEI();
                                int afc=sx1278.getAFC();
                                Serial.print("SYNC!!! Test: RSSI="); Serial.print(rssi);
                                Serial.print(" FEI="); Serial.print(fei);
                                Serial.print(" AFC="); Serial.println(afc);
                                sonde.si()->rssi = rssi;
                                sonde.si()->afc = afc;
#endif
			}
		} else {
			rxbitc = (rxbitc+1)%16; // 16;
			if(rxbitc == 0) { // got 8 data bit
				//Serial.printf("%03x ",rxbyte);
				dataptr[rxp++] = rxbyte&0xff; // (rxbyte>>1)&0xff;
				// detect type of sonde:
				// 64 9F 20 => M10
				// 64 49 0x => M10 (?) -- not used here
				// 45 20 7x => M20
				if(rxp==2 && dataptr[0]==0x45 && dataptr[1]==0x20) { isM20 = true; }
				if(isM20) {
					memcpy(sonde.si()->d.typestr, "M20 ", 5);
					sonde.si()->d.subtype = 2;
					if(rxp>=M20_FRAMELEN) {
						rxsearching = true;
						haveNewFrame = decodeframeM20(dataptr);
					}
				} else {
					memcpy(sonde.si()->d.typestr, "M10 ", 5);
					sonde.si()->d.subtype = 1;
					if(rxp>=M10_FRAMELEN) {
						rxsearching = true;
						haveNewFrame = decodeframeM10(dataptr);
					}
				}
			}
		}
	}
}

int M10M20::receive() {
	unsigned long t0 = millis();
	Serial.printf("M10M20::receive() start at %ld\n",t0);
   	while( millis() - t0 < 1100 ) {
		uint8_t value = sx1278.readRegister(REG_IRQ_FLAGS2);
		if ( bitRead(value, 7) ) {
			Serial.println("FIFO full");
      		}
      		if ( bitRead(value, 4) ) {
        		Serial.println("FIFO overflow");
      		}
      		if ( bitRead(value, 2) == 1 ) {
        		Serial.println("FIFO: ready()");
        		sx1278.clearIRQFlags();
      		}
		if(bitRead(value, 6) == 0) { // while FIFO not empty
      			byte data = sx1278.readRegister(REG_FIFO);
			//Serial.printf("%02x:",data);
      			processM10data(data);
      			value = sx1278.readRegister(REG_IRQ_FLAGS2);
    		} else {
			if(headerDetected) {
				t0 = millis(); // restart timer... don't time out if header detected...
				headerDetected = 0;
			}
    			if(haveNewFrame) {
				Serial.printf("M10M20::receive(): new frame complete after %ldms\n", millis()-t0);
				printRaw(dataptr, M10_FRAMELEN);
				int retval = haveNewFrame==1 ? RX_OK: RX_ERROR;
				haveNewFrame = 0;
				return retval;
			}
			delay(2);
    		}
    	}
                       int32_t afc = sx1278.getAFC();
                       int16_t rssi = sx1278.getRSSI();
                       Serial.printf("receive: AFC is %d, RSSI is %.1f\n", afc, rssi/2.0);
	Serial.printf("M10M20::receive() timed out\n");
    	return RX_TIMEOUT; // TODO RX_OK;
}

#define M10MAXLEN (240)

int M10M20::waitRXcomplete() {
	return 0;
}



// ret: 1=frame ok; 2=frame with errors; 0=ignored frame (m20dop-alternativ)
int M10M20::decodeframeM20(uint8_t *data) {
	int repairstep = 16;
	int frl;
	int repl = 0;
	bool crcok = false;
	bool crcbok = false;
	//SondeInfo *si = sonde.si();
	SondeData *si = &(sonde.si()->d);
	// error correction, inspired by oe5dxl's sondeudp
	// check first block
	uint8_t s[200];
	s[0] = 0x16;
	for(int i=1; i<=M20_CRCPOSB-1; i++) { s[i] = data[i+1]; }
	crcbok = (crc_M10M20(M20_CRCPOSB-1, s) == 
	         ((data[M20_CRCPOSB] << 8) | data[M20_CRCPOSB+1]));

	frl = data[0] + 1;     // frame len? (0x45+1 => 70)
	if(frl>M20_FRAMELEN) { frl = M20_FRAMELEN; }
	do {
		crcok = checkM10M20crc(frl-2, data);
		if(crcok || repairstep == 0) break;
		repl = 0;
		for(int i=crcbok?M20_CRCPOSB+2:0; i<frl-2; i++) {
			if( ((sondeudp_VARSETM20[i/32]&(1<<(i%32))) == 0)  && (fixcnt[i]>=repairstep) ) {
				repl++;	
				data[i] = fixbytes[i];
			}
		}
		repairstep >>= 1;
	} while(true);
	if(crcbok) {
		int oklen = crcok ? frl-2 : 21;
		for(int i=0; i<oklen; i++) {
			if(fixbytes[i]==data[i]) { if(fixcnt[i]<255) fixcnt[i]++; }
			else { fixcnt[i]=0; fixbytes[i]=data[i]; }
		}
	}
	Serial.println(crcok?"CRC OK":"CRC NOT OK");
        Serial.printf(" repair: %d/%d\n", repl, repairstep);

	Serial.println("Decoding...");
	// Its a M20
	// getid...
	// TODO: Adjust ID calculation and serial number reconstruction
	char ids[11]={'M','E','0','0','0','0','0','0','0','0','0'};
		
	ids[0] = 'M';
	ids[1] = 'E';
	uint32_t id = data[18];  // getint16(data+18);
	ids[2] = hex(id/16);
	ids[3] = hex(id);
	//
	id = getint16_r(data+19)/4;
	ids[4] = (char)((id/10000)%10+48);
	ids[5] = (char)((id/1000)%10+48);
	ids[6] = (char)((id/100)%10+48);
	ids[7] = (char)((id/10)%10+48);
	ids[8] = (char)(id%10+48);
	strncpy(si->id, ids, 10);
	// Serial: AAB-C-DDEEE
	char *ser = si->ser;
	uint8_t tmp = data[18] & 0x7F;
	ser[0] = (tmp/12) + '0';
	ser[1] = ((tmp%12 + 1) / 10 ) + '0';
	ser[2] = ((tmp%12 + 1) % 10 ) + '0';
	ser[3] = '-';
	ser[4] = (data[18]/128) + 1 + '0';
	ser[5] = '-';
	ser[6] = ids[4];
	ser[7] = ids[5];
	ser[8] = ids[6];
	ser[9] = ids[7];
	ser[10] = ids[8];
	ser[11] = 0;

	// TODO
	if(crcok) {
	si->validID = true;
	//Serial.printf("ID is %s [%02x %02x %d]\n", ids, data[95], data[93], id);
	// ID printed on sonde is ...-.-abbbb, with a=id>>13, bbbb=id&0x1fff in decimal
	// position data
	// 0x1C  4 byte
	si->lat = getint32(data+28) * 1e-6;
	//0x20  4 byte
	si->lon = getint32(data+32) * 1e-6;
	//0x08  3 byte
	si->alt = getint24(data+8) * VMUL_M20;
	//0x0B  2 byte
	//VMUL_M20 specific
	float ve = getint16(data+11)*VMUL_M20;
	//0x0D  2 byte
	float vn = getint16(data+13)*VMUL_M20;
	//0x18  2 byte
	si->vs = getint16(data+24) * VMUL_M20;
	si->hs = sqrt(ve*ve+vn*vn);
	float dir = atan2(ve, vn)*(1.0/RAD);
	if(dir<0) dir+=360;
	si->dir = dir;
	si->validPos = 0x3f;

        //0x0F  3 byte
 	uint32_t tow = getint24(data+15);
        uint16_t week = getint16(data+26);
        si->time = (tow+week*604800+315964800)-18;
	si->vframe =si->time - 315964800;
                
        si->validTime = true;
	}
	return crcok?1:2;
}


M10M20 m10m20 = M10M20();

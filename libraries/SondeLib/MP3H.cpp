
/* MP3H decoder functions */

#include "MP3H.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

#define MP3H_DEBUG 1

#if MP3H_DEBUG
#define MP3H_DBG(x) x
#else
#define MP3H_DBG(x)
#endif

static struct st_mp3hstate {
	uint32_t id1, id2;
	uint8_t idok;
	uint32_t gpsdate;
	bool dateok;
} mp3hstate;

static byte data1[512];
static byte *dataptr=data1;

static uint8_t rxbitc;
static uint16_t rxbyte;
static int rxp=0;
static int haveNewFrame = 0;
//static int lastFrame = 0;
static int headerDetected = 0;

int MP3H::setup(float frequency) 
{
	MP3H_DBG(Serial.println("Setup sx1278 for MP3H sonde"));;
	if(sx1278.ON()!=0) {
		MP3H_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	// setFSK: switches to FSK standby mode
	if(sx1278.setFSK()!=0) {
		MP3H_DBG(Serial.println("Setting FSK mode FAILED"));
		return 1;
	}
        Serial.print("MP3H: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
	// Test: maybe fix issue after spectrum display?
	sx1278.writeRegister(REG_PLL_HOP, 0);

        if(sx1278.setAFCBandwidth(sonde.config.mp3h.agcbw)!=0) {
                MP3H_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.mp3h.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.mp3h.rxbw)!=0) {
                MP3H_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.mp3h.rxbw));
                return 1;
        }

#if 0
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
                MP3H_DBG(Serial.println("Setting Packet config FAILED"));
                return 1;
        }
        // enable RX
        sx1278.setPayloadLength(0);
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
        unsigned long t0 = millis();
        MP3H_DBG(Serial.printf("MP3H::setup() AFC preamble search start at %ld\n",t0));
        while( millis() - t0 < 1000 ) {
                uint8_t value = sx1278.readRegister(REG_IRQ_FLAGS1);
               if(value & 2) {
                       int32_t afc = sx1278.getAFC();
                       int16_t rssi = sx1278.getRSSI();
                       Serial.printf("MP3H::setup: preamble: AFC is %d, RSSI is %.1f\n", afc, rssi/2.0);
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
#endif

//// Step 2: Real reception
	// FSK standby mode, seems like otherweise baudrate cannot be changed?
	sx1278.setFSK();
	if(sx1278.setBitrate(2400)!=0) {
		MP3H_DBG(Serial.println("Setting bitrate 2400bit/s FAILED"));
		return 1;
	}
	MP3H_DBG(Serial.printf("Exact bitrate is %f\n", sx1278.getBitrate()));
	// Probably not necessary, as this was set before
        if(sx1278.setAFCBandwidth(sonde.config.mp3h.agcbw)!=0) {
                MP3H_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.mp3h.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.mp3h.rxbw)!=0) {
                MP3H_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.mp3h.rxbw));
                return 1;
        }

	///// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	//if(sx1278.setRxConf(0x1E)!=0) {
	// Disable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x00)!=0) {
		MP3H_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// version 1, working with continuous RX
	const char *SYNC="\x66\x66";
	if(sx1278.setSyncConf(0x70, 1, (const uint8_t *)SYNC)!=0) {
		MP3H_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
        // Preamble detection off (+ size 1 byte, maximum tolerance; should not matter for "off"?)
        if(sx1278.setPreambleDetect(0x00 | 0x00 | 0x1F)!=0) {
		MP3H_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		MP3H_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

        // enable RX
        sx1278.setPayloadLength(0);  // infinite for now...
        //sx1278.setRxConf(0x20);
	uint16_t afc = sx1278.getRawAFC();
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	delay(50);
	sx1278.setRawAFC(afc);
	delay(50);
        Serial.printf("after RX_MODE: AFC is %d\n", sx1278.getAFC());

	memset((void *)&mp3hstate, 0, sizeof(mp3hstate));
#if MP3H_DEBUG
	MP3H_DBG(Serial.println("Setting SX1278 config for MP3H finished\n"); Serial.println());
#endif
        return res;
}


MP3H::MP3H() {
}

#define MP3H_FRAMELEN 49

// offsets from zilog
// https://github.com/rs1729/RS/blob/master/demod/mod/mp3h1mod.c
#define OFS -3
#define pos_CNT1        (OFS+ 3)  //   1 nibble (0x80..0x8F ?)
#define pos_TIME        (OFS+ 4)  // 3*1 byte
#define pos_GPSecefX    (OFS+ 8)  //   4 byte
#define pos_GPSecefY    (OFS+12)  //   4 byte
#define pos_GPSecefZ    (OFS+16)  //   4 byte
#define pos_GPSecefV    (OFS+20)  // 3*2 byte
#define pos_GPSnSats    (OFS+26)  //   1 byte (num Sats ?)
#define pos_PTU1        (OFS+35)  //   4 byte
#define pos_PTU2        (OFS+39)  //   4 byte
#define pos_CNT2        (OFS+43)  //   1 byte   (0x01..0x10 ?)
#define pos_CFG         (OFS+44)  // 2/4 byte
#define pos_CRC         (OFS+48)  //   2 byte


#define crc16poly 0xA001
static bool checkMP3CRC(uint8_t *data)
{
	int start = pos_CNT1;
	int len = 45;
	uint16_t rem = 0xffff;
	for(int i=0; i<len; i++) {
		rem ^= data[start+i];
		for(int j=0; j<8; j++) {
			if(rem&0x1) rem = (rem>>1) ^ crc16poly;
			else rem = rem>>1;
		}
	}
	uint16_t crcdat = data[pos_CRC] | (data[pos_CRC+1]<<8);
	return rem == crcdat ? true : false;
}

void MP3H::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X ", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
#define RAD (PI/180)
#define DEG (180/PI)

static uint32_t u4(uint8_t *d)
{
	return d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24);
}
#define i4(d) ((int32_t)u4(d))

static uint16_t u2(uint8_t *d)
{
	return d[0] | (d[1]<<8);
}
#define i2(d) ((int16_t)u2(d))


// defined in RS41.cpp
extern void wgs84r(double x, double y, double z, double * lat, double * long0, double * heig);
extern double atang2(double x, double y);


void calcgps(uint8_t *buf) {
	SondeInfo *si = sonde.si();
	double wx = i4(buf+pos_GPSecefX) * 0.01;
	double wy = i4(buf+pos_GPSecefY) * 0.01;
	double wz = i4(buf+pos_GPSecefZ) * 0.01;
	double vx = i2(buf+pos_GPSecefV) * 0.01;
	double vy = i2(buf+pos_GPSecefV+2) * 0.01;
	double vz = i2(buf+pos_GPSecefV+4) * 0.01;
	// wgs84r
	double lat, lng, alt;
	wgs84r(wx, wy, wz, &lat, &lng, &alt);
	si->lat = (float)(lat*DEG);
	si->lon = (float)(lng*DEG);
	si->alt = alt;
	// speeddir
	double sinlat = sin(lat);
	double coslat = cos(lat);
	double sinlng = sin(lng);
	double coslng = cos(lng);
	double vn = -vx*sinlat*coslng - vy*sinlat*sinlng + vz*coslat;
	double ve = -vx*sinlng + vy*coslng;
	double clb = vx*coslat*coslng + vy*coslat*sinlng + vz*sinlat;
	double dir = atang2(vn, ve)/RAD;
	if(dir<0.0) dir+=360.0;
	si->dir = dir;
	si->vs = clb;
	si->hs = sqrt(vn*vn + ve*ve);

	Serial.printf("Pos: %f %f  alt %f  dir %f vs %f hs %f\n", si->lat, si->lon, si->alt, si->dir, si->vs, si->hs);
	si->validPos = 0x3f;  // maybe do some checks first...
}

static uint8_t hex(uint32_t n) {
	n = n % 16;
	return (n<10) ? (n+'0') : (n-10+'A');
}
// ret: 1=frame ok; 2=frame with errors; 0=ignored frame (m10dop-alternativ)
int MP3H::decodeframeMP3H(uint8_t *data) {
	printRaw(data, MP3H_FRAMELEN);

	//
	if(!checkMP3CRC(data)) {
		// maybe add repairing frames later...
		return 2;
	}
	
	// data is a frame with correct CRC
	SondeInfo *si = sonde.si();
	uint8_t cnt = data[pos_CNT1] & 0x0F;
	uint32_t cfg = u4(data+pos_CFG);
	if(cnt==15) {
		// date
		mp3hstate.gpsdate = cfg;
		mp3hstate.dateok = true;
	} else if(cnt==13) {
		// id2
		mp3hstate.id2 = cfg;
		mp3hstate.idok |= 2;
	} else if(cnt==12) {
		// id1
		mp3hstate.id1 = cfg;
		mp3hstate.idok |= 1;
	}
	// get id
	if((mp3hstate.idok&3) == 3) {
		//...
		si->type = STYPE_MP3H;
		uint32_t n = mp3hstate.id1*100000 + mp3hstate.id2;
		si->id[0] = 'M';
		si->id[1] = 'R';
		si->id[2] = 'Z';
		si->id[3] = hex(n/0x100000);
		si->id[4] = hex(n/0x10000);
		si->id[5] = hex(n/0x1000);
		si->id[6] = hex(n/0x100);
		si->id[7] = hex(n/0x10);
		si->id[8] = hex(n);
		si->id[9] = 0;
		snprintf(si->ser, 12, "MRZ-%d-%d", mp3hstate.id1, mp3hstate.id2);
		si->validID = true;
	}

	// position
	calcgps(data);
	return 1;
#if 0
	int repairstep = 16;
	int repl = 0;
	bool crcok;
	// error correction, inspired by oe5dxl's sondeudp
	do {
		crcok = checkMP3Hcrc(M10_CRCPOS, data);
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

	if(data[1]==0x9F && data[2]==0x20) {
		Serial.println("Decoding...");
		// Its a M10
		// getid...
		char ids[11];
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
		strncpy(sonde.si()->id, ids, 10);
		ids[0] = hex(data[95]/16);
		ids[1] = dez((data[95]&0x0f)/10);
		ids[2] = dez((data[95]&0x0f));
		ids[3] = dez(data[93]);
		ids[4] = dez(id>>13);
		id &= 0x1fff;
		ids[5] = dez(id/1000); 
		ids[6] = dez((id/100)%10);
		ids[7] = dez((id/10)%10);
		ids[8] = dez(id%10);
		strncpy(sonde.si()->ser, ids, 10);
		sonde.si()->validID = true;
		Serial.printf("ID is %s [%02x %02x %d]\n", ids, data[95], data[93], id);
		// ID printed on sonde is ...-.-abbbb, with a=id>>13, bbbb=id&0x1fff in decimal
		// position data
		sonde.si()->lat = getint32(data+14) * DEGMUL;
		sonde.si()->lon = getint32(data+18) * DEGMUL;
		sonde.si()->alt = getint32(data+22) * 0.001;
		float ve = getint16(data+4)*VMUL;
		float vn = getint16(data+6)*VMUL;
		sonde.si()->vs = getint16(data+8) * VMUL;
		sonde.si()->hs = sqrt(ve*ve+vn*vn);
		float dir = atan2(vn, ve)*(1.0/RAD);
		if(dir<0) dir+=360;
		sonde.si()->dir = dir;
		sonde.si()->validPos = 0x3f;

 		uint32_t gpstime = getint32(data+10);
                uint16_t gpsweek = getint16(data+32);
                        // UTC is GPSTIME - 18s (24*60*60-18 = 86382)
                        // one week = 7*24*60*60 = 604800 seconds
                        // unix epoch starts jan 1st 1970 0:00
                        // gps time starts jan 6, 1980 0:00. thats 315964800 epoch seconds.
                        // subtracting 86400 yields 315878400UL
                sonde.si()->time = (gpstime/1000) + 86382 + gpsweek*604800 + 315878400UL;
                sonde.si()->validTime = true;
	} else {
		Serial.printf("data is %02x %02x %02x\n", data[0], data[1], data[2]);
		return 0;
	}
	return crcok?1:2;
#endif
	return 0;
}

static uint32_t rxdata;
static bool rxsearching=true;

// search for
// 0xBF3H (or inverse)
void MP3H::processMP3Hdata(uint8_t dt)
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
			// rxbyte = rxbyte ^ d;
		}
		// BF3H => 1011 1111 0011 0101 => 10011010 10101010 01011010 01100110 => 9AAA5A66 // 6555a599
		if(rxsearching) {
			if( rxdata == 0x9AAA5A66 || rxdata == 0x6555a599 ) {
//if( rxdata == 0x9AAA5A66 || rxdata == 0x6555a599 ) {
				rxsearching = false;
				rxbitc = 0;
				rxp = 0;
				headerDetected = 1;
				Serial.print("SYNC\n");
#if 0
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
				dataptr[rxp++] = rxbyte&0xff; // (rxbyte>>1)&0xff;
				//if(rxp==2 && dataptr[0]==0x45 && dataptr[1]==0x20) { isM20 = true; }
				if(rxp>=MP3H_FRAMELEN) {
					rxsearching = true;
					haveNewFrame = decodeframeMP3H(dataptr);
				}
			}
		}
	}
}

int MP3H::receive() {
	unsigned long t0 = millis();
	Serial.printf("MP3H::receive() start at %ld\n",t0);
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
			Serial.printf("%02x:",data);
      			processMP3Hdata(data);
      			value = sx1278.readRegister(REG_IRQ_FLAGS2);
    		} else {
			if(headerDetected) {
				t0 = millis(); // restart timer... don't time out if header detected...
				headerDetected = 0;
			}
    			if(haveNewFrame) {
				Serial.printf("MP3H::receive(): new frame complete after %ldms\n", millis()-t0);
				printRaw(dataptr, MP3H_FRAMELEN);
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
	Serial.printf("MP3H::receive() timed out\n");
    	return RX_TIMEOUT; // TODO RX_OK;
}

int MP3H::waitRXcomplete() {
	return 0;
}


MP3H mp3h = MP3H();
	

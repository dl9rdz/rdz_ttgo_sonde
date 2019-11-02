
/* M10 decoder functions */
#include "M10.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

// well...
//#include "rs92gps.h"

#define M10_DEBUG 1

#if M10_DEBUG
#define M10_DBG(x) x
#else
#define M10_DBG(x)
#endif


static byte data1[512];
static byte *dataptr=data1;

static uint8_t rxbitc;
static uint16_t rxbyte;
static int rxp=0;
static int haveNewFrame = 0;
static int lastFrame = 0;
static int headerDetected = 0;

int M10::setup(float frequency) 
{
#if M10_DEBUG
	Serial.println("Setup sx1278 for M10 sonde");
#endif
	//if(!initialized) {
		//Gencrctab();
		//initrsc();
        	// not here for now.... get_eph("/brdc.19n");
	//	initialized = true;
	//}

	if(sx1278.ON()!=0) {
		M10_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	if(sx1278.setFSK()!=0) {
		M10_DBG(Serial.println("Setting FSJ mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(9600)!=0) {
		M10_DBG(Serial.println("Setting bitrate 9600bit/s FAILED"));
		return 1;
	}
#if M10_DEBUG
	float br = sx1278.getBitrate();
	Serial.print("Exact bitrate is ");
	Serial.println(br);
#endif
        if(sx1278.setAFCBandwidth(sonde.config.rs92.rxbw)!=0) {
                M10_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.rs92.rxbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.rs92.rxbw)!=0) {
                M10_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.rs92.rxbw));
                return 1;
        }

	// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x1E)!=0) {
		M10_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// Set autostart_RX to 01, preamble 0, SYNC detect==on, syncsize=3 (==4 byte
	//char header[] = "0110.0101 0110.0110 1010.0101 1010.1010";

	//const char *SYNC="\x10\xB6\xCA\x11\x22\x96\x12\xF8";
	//const char *SYNC="\x08\x6D\x53\x88\x44\x69\x48\x1F";
	// was 0x57
	//const char *SYNC="\x99\x9A";
#if 1
	// version 1, working with continuous RX
	const char *SYNC="\x66\x65";
	if(sx1278.setSyncConf(0x70, 2, (const uint8_t *)SYNC)!=0) {
		M10_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
	//if(sx1278.setPreambleDetect(0xA8)!=0) {
	if(sx1278.setPreambleDetect(0x9F)!=0) {
		M10_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}
#endif
#if 0
	// version 2, with per-packet rx start, untested
	// header is 2a 10 65, i.e. with lsb first
	// 0 0101 0100 1            0 0000 1000 1            0 1010 0110 1
 	// 10 10011001 10011010 01  10 10101010 01101010 01  10 01100110 10010110 01
	//                             preamble 0x6A 0x66 0x6A
	// i.e. preamble detector on (0x80), preamble detector size 1 (0x00), preample chip errors??? (0x0A)
	// after 2a2a2a2a2a1065
        if(sx1278.setPreambleDetect(0xA8)!=0) {
                M10_DBG(Serial.println("Setting PreambleDetect FAILED"));
                return 1;
        }
	// sync config: ato restart (01), preamble polarity AA (0), sync on (1), resevered (0), syncsize 2+1 (010) => 0x52
	const char *SYNC="\x6A\x66\x69";
        if(sx1278.setSyncConf(0x52, 3, (const uint8_t *)SYNC)!=0) {
                M10_DBG(Serial.println("Setting SYNC Config FAILED"));
                return 1;
        }
	// payload length is ((240 - 7)*10 +6)/8 = 292
#endif

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		M10_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

	Serial.print("M10: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
        // enable RX
        sx1278.setPayloadLength(0);  // infinite for now...
	//sx1278.setPayloadLength(292);
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

#if M10_DEBUG
	M10_DBG(Serial.println("Setting SX1278 config for M10 finished\n"); Serial.println());
#endif
        return res;
}

#if 0
int M10::setFrequency(float frequency) {
	Serial.print("M10: setting RX frequency to ");
	Serial.println(frequency);
	int res = sx1278.setFrequency(frequency);
	// enable RX
        sx1278.setPayloadLength(0);  // infinite for now...

	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	return res;
}
#endif

#if 0
uint32_t M10::bits2val(const uint8_t *bits, int len) {
	uint32_t val = 0;
	for (int j = 0; j < len; j++) {
		val |= (bits[j] << (len-1-j));
	}
	return val;
}
#endif

M10::M10() {
}

#define M10_FRAMELEN 101
#define M10_CRCPOS 99

void M10::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X ", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

static int update_checkM10(int c, uint8_t b) {
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
static bool checkM10crc(uint8_t *msg) {
	int i, cs, cs1;
	cs = 0;
	for (i = 0; i < M10_CRCPOS; i++) {
		cs = update_checkM10(cs, msg[i]);
	}
	cs = cs & 0xFFFF;
	cs1 = (msg[M10_CRCPOS] << 8) | msg[M10_CRCPOS+1];
	return (cs1 == cs);
}

typedef uint32_t SET256[8];
static SET256 sondeudp_VARSET = {0x03BBBBF0UL,0x80600000UL,0x06A001A0UL,
                0x0000001CUL,0x00000000UL,0x00000000UL,0x00000000UL,
                0x00000000UL};
// VARSET=SET256{4..9,11..13,15..17,19..21,23..25,53..54,63,69,71,72,85,87,89,90,98..100}; 

static uint8_t fixcnt[M10_FRAMELEN];
static uint8_t fixbytes[M10_FRAMELEN];

static int32_t getint32(uint8_t *data) {
	return (int32_t)( data[3]|(data[2]<<8)|(data[1]<<16)|(data[0]<<24) );
}
static int16_t getint16(uint8_t *data) {
	return (int16_t)(data[1]|((uint16_t)data[0]<<8));
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
#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
#define RAD (PI/180)


bool M10::decodeframeM10(uint8_t *data) {
	int repairstep = 16;
	int repl = 0;
	bool crcok;
	// error correction, inspired by oe5dxl's sondeudp
	do {
		crcok = checkM10crc(data);
		if(crcok) break;
		repl = 0;
		for(int i=0; i<M10_CRCPOS; i++) {
			if( ((sondeudp_VARSET[i/32]&(1<<(i%32))) != 1)  && (fixcnt[i]>=repairstep) ) {
				repl++;	
				data[i] = fixbytes[i];
			}
		}
		repairstep >>= 1;
	} while(repairstep>0);
	if(crcok) {
		for(int i=0; i<M10_CRCPOS; i++) {
			if(fixbytes[i]==data[i] &&fixcnt[i]<255) fixcnt[i]++;
			else { fixcnt[i]=0; fixbytes[i]=data[i]; }
		}
	}
	Serial.println(crcok?"CRC OK":"CRC NOT OK");

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
	}
	return crcok;
}

static uint32_t rxdata;
static bool rxsearching=true;

// search for
// //101001100110011010011010011001100110100110101010100110101001
// //1010011001100110100110100110 0110.0110 1001.1010 1010.1001 1010.1001 => 0x669AA9A9
void M10::processM10data(uint8_t dt)
{
	for(int i=0; i<8; i++) {
		uint8_t d = (dt&0x80)?1:0;
		dt <<= 1;
		rxdata = (rxdata<<1) | d;
		//uint8_t value = ((rxdata>>1)^rxdata)&0x01;
		//if((rxbitc&1)==1) { rxbyte = (rxbyte>>1) + ((value)<<8); } // mancester decoded data
		//rxbyte = (rxbyte>>1) | (d<<8);
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
#if 1
                                int rssi=sx1278.getRSSI();
                                int fei=sx1278.getFEI();
                                int afc=sx1278.getAFC();
                                Serial.print("Test: RSSI="); Serial.print(rssi);
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
#if 0
				if(rxp==7 && dataptr[6] != 0x65) {
					Serial.printf("wrong start: %02x\n",dataptr[6]);
					rxsearching = true;
				}
#endif
				if(rxp>=M10_FRAMELEN) {
					rxsearching = true;
					bool ok = decodeframeM10(dataptr);
					haveNewFrame = ok ? 1 : 2;
				}
			}
		}
	}
}

int M10::receive() {
	unsigned long t0 = millis();
	Serial.printf("M10::receive() start at %ld\n",t0);
   	while( millis() - t0 < 1000 ) {
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
			//Serial.printf("%02x",data);
      			processM10data(data);
      			value = sx1278.readRegister(REG_IRQ_FLAGS2);
    		} else {
			if(headerDetected) {
				t0 = millis(); // restart timer... don't time out if header detected...
				headerDetected = 0;
			}
    			if(haveNewFrame) {
				Serial.printf("M10::receive(): new frame complete after %ldms\n", millis()-t0);
				printRaw(dataptr, M10_FRAMELEN);
				int retval = haveNewFrame==1 ? RX_OK: RX_ERROR;
				haveNewFrame = 0;
				return retval;
			}
			delay(2);
    		}
    	}
	Serial.printf("M10::receive() timed out\n");
    	return RX_TIMEOUT; // TODO RX_OK;
}

#define M10MAXLEN (240)
int M10::waitRXcomplete() {
	// called after complete...
#if 0
	Serial.printf("decoding frame %d\n", lastFrame);
	print_frame(lastFrame==1?data1:data2, 240);
	SondeInfo *si = sonde.sondeList+rxtask.receiveSonde;
	si->lat = gpx.lat;
	si->lon = gpx.lon;
	si->alt = gpx.alt;
	si->vs = gpx.vU;
	si->hs = gpx.vH;
	si->dir = gpx.vD;
	si->validPos = 0x3f;
	memcpy(si->id, gpx.id, 9);
	si->validID = true;

	int res=0;
        uint32_t t0 = millis();
        while( rxtask.receiveResult == 0xFFFF && millis()-t0 < 2000) { delay(20); }

        if( rxtask.receiveResult<0 || rxtask.receiveResult==RX_TIMEOUT) {
                res = RX_TIMEOUT;
        } else if ( rxtask.receiveResult==0) {
                res = RX_OK;
        } else {
                res = RX_ERROR;
        }
        rxtask.receiveResult = 0xFFFF;
        Serial.printf("M10::waitRXcomplete returning %d (%s)\n", res, RXstr[res]);
        return res;
#endif
	return 0;
}


#if 0
int oldwaitRXcomplete() {
	Serial.println("M10: receive frame...\n");
	sx1278receiveData = true;
	delay(6000); // done in other task....
	//sx1278receiveData = false;
#if 0
	//sx1278.setPayloadLength(518-8);    // Expect 320-8 bytes or 518-8 bytes (8 byte header)
	//sx1278.setPayloadLength(0);  // infinite for now...

////// test code for continuous reception
	//  sx1278.receive();  /// active FSK RX mode -- already done above...
        uint8_t value = sx1278.readRegister(REG_IRQ_FLAGS2);
        unsigned long previous = millis();

        byte ready=0;
	uint32_t wait = 8000;
        // while not yet done or FIFO not yet empty
	// bit 6: FIFO Empty
	// bit 2 payload ready
	int by=0;
        while( (!ready || bitRead(value,6)==0) && (millis() - previous < wait) )
        {
		if( bitRead(value, 7) ) { Serial.println("FIFO full"); }
		if( bitRead(value, 4) ) { Serial.println("FIFO overflow"); }
                if( bitRead(value,2)==1 ) ready=1;
                if( bitRead(value, 6) == 0 ) { // FIFO not empty
                        byte data = sx1278.readRegister(REG_FIFO);
			process8N1data(data);
			by++;
#if 0
                        if(di==1) {
                                int rssi=getRSSI();
                                int fei=getFEI();
                                int afc=getAFC();
                                Serial.print("Test: RSSI="); Serial.println(rssi);
                                Serial.print("Test: FEI="); Serial.println(fei);
                                Serial.print("Test: AFC="); Serial.println(afc);
                                sonde.si()->rssi = rssi;
                                sonde.si()->afc = afc;
                        }
                        if(di>520) {
                                // TODO
                                Serial.println("TOO MUCH DATA");
                                break;
                        }
                        previous = millis(); // reset timeout after receiving data
#endif
                }
                value = sx1278.readRegister(REG_IRQ_FLAGS2);
        }
	Serial.printf("processed %d bytes before end/timeout\n", by);
#endif



/////
#if 0
	int e = sx1278.receivePacketTimeout(1000, data+8);
	if(e) { Serial.println("TIMEOUT"); return RX_TIMEOUT; } //if timeout... return 1

	printRaw(data, M10MAXLEN);
	//for(int i=0; i<M10MAXLEN; i++) { data[i] = reverse(data[i]); }
	//printRaw(data, MAXLEN);
	//for(int i=0; i<M10MAXLEN; i++) { data[i] = data[i] ^ scramble[i&0x3F]; }
	//printRaw(data, MAXLEN);
	//int res = decode41(data, M10MAXLEN);
#endif
	int res=0;
	return res==0 ? RX_OK : RX_ERROR;
}
#endif

M10 m10 = M10();

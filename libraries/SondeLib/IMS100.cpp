
/* IMS100 decoder functions */
#include "IMS100.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

// well...
//#include "rs92gps.h"

#define IMS100_DEBUG 1

#if IMS100_DEBUG
#define IMS100_DBG(x) x
#else
#define IMS100_DBG(x)
#endif

static uint16_t MON[]={0,0,31,59,90,120,151,181,212,243,273,304,334};

static byte data2[80];
static byte data1[80];
static byte *dataptr=data1;

static uint8_t rxbitc;
static uint16_t rxbyte;
static int rxp=0;
static int haveNewFrame = 0;
static int lastFrame = 0;
static int headerDetected = 0;

#include "bch_ecc.c"

static boolean initialized;
int IMS100::setup(float frequency) 
{
#if IMS100_DEBUG
	Serial.println("Setup sx1278 for IMS100 sonde");
#endif
	if(!initialized) {
		rs_init_BCH64();
		initialized = true;
	}	

	if(sx1278.ON()!=0) {
		IMS100_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	if(sx1278.setFSK()!=0) {
		IMS100_DBG(Serial.println("Setting FSJ mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(2400)!=0) {
		IMS100_DBG(Serial.println("Setting bitrate 9600bit/s FAILED"));
		return 1;
	}
#if IMS100_DEBUG
	float br = sx1278.getBitrate();
	Serial.print("Exact bitrate is ");
	Serial.println(br);
#endif
        if(sx1278.setAFCBandwidth(sonde.config.rs92.rxbw)!=0) {
                IMS100_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.rs92.rxbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.rs92.rxbw)!=0) {
                IMS100_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.rs92.rxbw));
                return 1;
        }

	// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x1E)!=0) {
		IMS100_DBG(Serial.println("Setting RX Config FAILED"));
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
	const char *SYNC="\x55\x55";
	if(sx1278.setSyncConf(0x70, 2, (const uint8_t *)SYNC)!=0) {
		IMS100_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
	//if(sx1278.setPreambleDetect(0xA8)!=0) {
	if(sx1278.setPreambleDetect(0x9F)!=0) {
		IMS100_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}
#endif

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		IMS100_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

	Serial.print("IMS100: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
        // enable RX
        sx1278.setPayloadLength(0);  // infinite for now...
	//sx1278.setPayloadLength(292);
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

#if IMS100_DEBUG
	IMS100_DBG(Serial.println("Setting SX1278 config for IMS100 finished\n"); Serial.println());
#endif
        return res;
}

#if 0
int IMS100::setFrequency(float frequency) {
	Serial.print("IMS100: setting RX frequency to ");
	Serial.println(frequency);
	int res = sx1278.setFrequency(frequency);
	// enable RX
        sx1278.setPayloadLength(0);  // infinite for now...

	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	return res;
}
#endif

#if 0
uint32_t IMS100::bits2val(const uint8_t *bits, int len) {
	uint32_t val = 0;
	for (int j = 0; j < len; j++) {
		val |= (bits[j] << (len-1-j));
	}
	return val;
}
#endif

IMS100::IMS100() {
}

#define IMS100_FRAMELEN (75)

#define IMS100_CRCPOS 99

void IMS100::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X ", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

static int update_checkIMS100(int c, uint8_t b) {
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
static bool checkIMS100crc(uint8_t *msg) {
	int i, cs, cs1;
	cs = 0;
	for (i = 0; i < IMS100_CRCPOS; i++) {
		cs = update_checkIMS100(cs, msg[i]);
	}
	cs = cs & 0xFFFF;
	cs1 = (msg[IMS100_CRCPOS] << 8) | msg[IMS100_CRCPOS+1];
	return (cs1 == cs);
}

typedef uint32_t SET256[8];
static SET256 sondeudp_VARSET = {0x03BBBBF0UL,0x80600000UL,0x06A001A0UL,
                0x0000001CUL,0x00000000UL,0x00000000UL,0x00000000UL,
                0x00000000UL};
// VARSET=SET256{4..9,11..13,15..17,19..21,23..25,53..54,63,69,71,72,85,87,89,90,98..100}; 

static uint8_t fixcnt[IMS100_FRAMELEN];
static uint8_t fixbytes[IMS100_FRAMELEN];

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

// 6*2 block first subframe, 6*2 blocks second subframe
uint16_t getw16(uint8_t *data, int index) {
	int bytepos;
	if(index<12) bytepos = 3 + 2*index;
	else bytepos = 27 + 3 + 2*(index-12);
	return ( data[bytepos]<<8 ) | data[bytepos+1];
}

// ret: 1=frame ok; 2=frame with errors; 0=ignored frame (m10dop-alternativ)
int IMS100::decodeframeIMS100(uint8_t *data) {
	int repairstep = 16;
	int repl = 0;
	bool crcok;
	printRaw(data, 75);
	uint8_t outdata[80];
	int outpos=0;

	// do error correction, based on RS's bch decoder
	uint8_t cw[64], err_pos[4], err_val[4];
	int8_t errcnt[12];
	int bitpos = 0; // 24 bit header
	for(int subframe = 0; subframe<2; subframe++) {
		outdata[outpos++] = data[(bitpos/8)];
		outdata[outpos++] = data[(bitpos/8)+1];
		outdata[outpos++] = data[(bitpos/8)+2];
		bitpos += 24;
		for(int block=0; block<6; block++) {
			// create codeword for decoder
                        for (int j = 46; j < 63; j++) cw[j] = 0;
			for (int j = 0; j<46; j++) {
				cw[45-j] = ( data[ (bitpos+j)>>3 ] >> (7 - ((bitpos+j)&7)) ) & 1;
			}
			errcnt[block+6*subframe] = rs_decode_bch_gf2t2(cw, err_pos, err_val);
			// TODO: Check for uncorrectable errors

			outdata[outpos] = outdata[outpos+1] = 0;
			for(int j=0; j<16; j++) { outdata[outpos + (j>>3)] |= (cw[45-j]<<(7 - (j&7))); }
			outpos += 2;
			outdata[outpos] = outdata[outpos+1] = 0;
			for(int j=0; j<16; j++) { outdata[outpos + (j>>3)] |= (cw[45-17-j]<<(7 - (j&7))); }
			outpos += 2;
			bitpos += 46;
		}
	}
	Serial.print("errcount: ");
	for(int i=0; i<12; i++) { Serial.print(errcnt[i]); Serial.print(" "); } Serial.println(".");
	printRaw(outdata, 54);

	// decoding...
	uint16_t gps_chk_sum = 0;
	for(int j= 10; j<12; j++) gps_chk_sum += getw16(outdata, j);
	if(outdata[0x12]!=0xC1) { Serial.println("Warning: Typs is not ims100 (0xC1)"); }
	uint16_t counter  = (outdata[3]<<8) | outdata[4];
	Serial.printf(" [%d] ", counter);
	uint32_t val = (outdata[7]<<8) | (outdata[8]) | (outdata[9]<<24) | (outdata[10]<<16);
	if( (counter&15)==0 && errcnt[1]>=0 ) { 
		uint32_t sn = *(float *)(&val);
		Serial.printf(" sn:%x ", sn);
		snprintf(sonde.si()->ser, 12, "%d", sn);
		snprintf(sonde.si()->id, 10, "IMS%06x", sn); 
		sonde.si()->validID = true;
	}
	if( (counter&63)==15 ) Serial.printf(" freq:%x / %f ",val,400e3+(*(float *)&val)*100);
	uint16_t ms;
	uint8_t hr, mi;
	if( (counter&1)==0 ) {
		ms = (outdata[0x17]<<8) | outdata[0x18];
		hr = outdata[0x19];
		mi = outdata[0x1A];
		Serial.printf(" %02d:%02d:%06.3f ", hr, mi, 0.001 * ms);
	}
	for(int j=0; j<11; j++) gps_chk_sum += getw16(outdata, j+12);
	boolean gps_err = gps_chk_sum != getw16(outdata, 11+12);
	Serial.printf("GPS checksum ok: %s  %d vs %d\n",gps_err?"NO":"yes",gps_chk_sum, getw16(outdata, 11+12));
	if((counter&1)==0) {
		uint16_t dat2 = (outdata[27+3]<<8) | (outdata[27+4]);
		int y = 2000+(dat2%10)+10;
		int m = (dat2/10)%100;
		int d = dat2/1000;
		Serial.printf(" (%04d-%02d-%02d) ", y, m, d);
		// time
                int tt = (y-1970)*365 + (y-1969)/4; // days since 1970
                if(m<=12) { tt += MON[m]; if((y%4)==0 && m>2) tt++; }
                tt = (tt+d-1)*(60*60*24) + hr*3600 + mi*60 + ms/1000;
                sonde.si()->time = tt;
		sonde.si()->frame = counter;
		sonde.si()->validTime = 1;	
		if(!gps_err) {
		int32_t lat = (outdata[30+2]<<24) | (outdata[30+3]<<16) | (outdata[30+4]<<8) | outdata[30+5];
		int32_t lon = (outdata[30+6]<<24) | (outdata[30+7]<<16) | (outdata[30+8]<<8) | outdata[30+9];
		int32_t alt = (outdata[30+10]<<16) | (outdata[30+11]<<8) | (outdata[30+12]);
		int32_t latdeg = lat / 1e6;
		int32_t londeg = lon / 1e6;
		float la = latdeg + ((double)lat/1.0e6-latdeg)*100/60.0;
		float lo = londeg + ((double)lon/1.0e6-londeg)*100/60.0;
		float al = alt/1.0e2;
		Serial.printf("  lat: %.5f(%d)  lon: %.5f(%d) alt: %.2f  ", la, lat, lo, lon, al);
		sonde.si()->lat = la;
		sonde.si()->lon = lo;
		sonde.si()->alt = al;
		sonde.si()->vs = 0;  // not in data?
		uint16_t vd = (outdata[30+18]<<8) | (outdata[30+19]);
		uint16_t vh = (outdata[30+20]<<8) | (outdata[30+21]);
		sonde.si()->hs = vh / 1.94384e2;
		sonde.si()->dir = vd / 1.0e2;
		sonde.si()->validPos = 0x37;
		}
	}
	return 1;
}

static uint32_t rxdata;
static bool rxsearching=true;

// search for
// //101001100110011010011010011001100110100110101010100110101001
// //1010011001100110100110100110 0110.0110 1001.1010 1010.1001 1010.1001 => 0x669AA9A9

// for now, search just for 0x04  =>   1010101010110101
//uint32_t testhead = 0b10101010101101010010101100110100;
uint32_t head[]={0b10101010101101010010101100110100, ~0b10101010101101010010101100110100, 0x396c6420, 0x207a6472};
// 600 bit => 75 byte, for now twice as this is before diff. decoding
void IMS100::processIMS100data(uint8_t dt)
{
	for(int i=0; i<8; i++) {
		uint8_t d = (dt&0x80)?1:0;
		dt <<= 1;
		rxdata = (rxdata<<1) | d;
		//
		if(rxsearching) {
			if( rxdata == head[0] || rxdata == head[1] ) {
				rxsearching = false;
				rxbitc = 0;
				rxp = 0;
				dataptr[0] = 0x04;
				dataptr[1] = 0x9D;
				rxp = 2;
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
			rxbitc = (rxbitc+1)%16;
			if(rxbitc&1) continue;
			rxbyte = (rxbyte<<1) | ( (rxdata&1)^((rxdata>>1)&1)^1 );
			if(rxbitc == 0) { // got 8 data bit
				//Serial.printf("%03x ",rxbyte);
				dataptr[rxp++] = rxbyte&0xff; // (rxbyte>>1)&0xff;
#if 0
				if(rxp==7 && dataptr[6] != 0x65) {
					Serial.printf("wrong start: %02x\n",dataptr[6]);
					rxsearching = true;
				}
#endif
				if(rxp>=IMS100_FRAMELEN) {
					rxsearching = true;
					haveNewFrame = decodeframeIMS100(dataptr);
				}
			}
		}
	}
}

int IMS100::receive() {
    for(int i=0; i<2; i++) {  // two 500ms frames
	unsigned long t0 = millis();
	Serial.printf("IMS100::receive() start at %ld\n",t0);
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
      			processIMS100data(data);
      			value = sx1278.readRegister(REG_IRQ_FLAGS2);
    		} else {
			if(headerDetected) {
				t0 = millis(); // restart timer... don't time out if header detected...
				headerDetected = 0;
			}
    			if(haveNewFrame) {
				Serial.printf("IMS100::receive(): new frame complete after %ldms\n", millis()-t0);
				printRaw(dataptr, IMS100_FRAMELEN);
				int retval = haveNewFrame==1 ? RX_OK: RX_ERROR;
				haveNewFrame = 0;
				if(i==0) continue;
				// TODO: consider return value of both frames??
				return retval;
			}
			delay(2);
    		}
    	}
    }
    Serial.printf("IMS100::receive() timed out\n");
    return RX_TIMEOUT; // TODO RX_OK;
}

#define IMS100MAXLEN (240)
int IMS100::waitRXcomplete() {
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
        Serial.printf("IMS100::waitRXcomplete returning %d (%s)\n", res, RXstr[res]);
        return res;
#endif
	return 0;
}


#if 0
int oldwaitRXcomplete() {
	Serial.println("IMS100: receive frame...\n");
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

	printRaw(data, IMS100MAXLEN);
	//for(int i=0; i<IMS100MAXLEN; i++) { data[i] = reverse(data[i]); }
	//printRaw(data, MAXLEN);
	//for(int i=0; i<IMS100MAXLEN; i++) { data[i] = data[i] ^ scramble[i&0x3F]; }
	//printRaw(data, MAXLEN);
	//int res = decode41(data, IMS100MAXLEN);
#endif
	int res=0;
	return res==0 ? RX_OK : RX_ERROR;
}
#endif

IMS100 ims100 = IMS100();

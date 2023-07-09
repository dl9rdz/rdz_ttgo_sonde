#include "../features.h"
#if FEATURE_RS92

/* RS92 decoder functions */
#include "RS92.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

// well...
#include "rs92gps.h"

#define RS92_DEBUG 1

#if RS92_DEBUG
#define RS92_DBG(x) x
#else
#define RS92_DBG(x)
#endif

uint16_t *CRCTAB = NULL;

#define X2C_DIVR(a, b) ((b) != 0.0f ? (a)/(b) : (a))
#define X2C_DIVL(a, b) ((a)/(b))
static uint32_t X2C_LSH(uint32_t a, int32_t length, int32_t n)
{
	uint32_t m;

	m = 0;
	m = (length == 32) ? 0xFFFFFFFFl : (1 << length) - 1;
	if (n > 0) {
		if (n >= (int32_t)length)
			return 0;
		return (a << n) & m;
	}

	if (n <= (int32_t)-length)
		return 0;
	return (a >> -n) & m;
}

static void Gencrctab(void)
{
   uint16_t j;
   uint16_t i;
   uint16_t crc;
   if(!CRCTAB) { CRCTAB=(uint16_t *)malloc(256*sizeof(uint16_t)); }
   for (i = 0U; i<=255U; i++) {
      crc = (uint16_t)(i*256U);
      for (j = 0U; j<=7U; j++) {
         if ((0x8000U & crc)) crc = X2C_LSH(crc,16,1)^0x1021U;
         else crc = X2C_LSH(crc,16,1);
      } /* end for */
      CRCTAB[i] = X2C_LSH(crc,16,-8)|X2C_LSH(crc,16,8);
   } /* end for */
} /* end Gencrctab() */


static byte data1[512]={0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x10};
static byte data2[512]={0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x10};
static byte *dataptr=data1;

static uint8_t rxbitc;
static int32_t asynst[10]={0};
static uint16_t rxbyte;
int rxp=0;

static int haveNewFrame = 0;
static int lastFrame = 0;
static int headerDetected = 0;

decoderSetupCfg rs92SetupCfg = {
	.bitrate = 4800,
	.rx_cfg = 0x1E,
	.sync_cfg = 0x70,
	.sync_len = 2,
	.sync_data = (const uint8_t *)"\x66\x65",
	.preamble_cfg = 0xA8,
};

int RS92::setup(float frequency, int /*type*/) 
{
#if RS92_DEBUG
	Serial.println("Setup sx1278 for RS92 sonde");
#endif
	if(!initialized) {
		Gencrctab();
		initrsc();
        	// not here for now.... get_eph("/brdc.19n");
		initialized = true;
	}

	if(sx1278.ON()!=0) {
		RS92_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	if(DecoderBase::setup(rs92SetupCfg, sonde.config.rs92.rxbw, sonde.config.rs92.rxbw)!=0) {
		return 1;
	}
#if 0
	if(sx1278.setFSK()!=0) {
		RS92_DBG(Serial.println("Setting FSJ mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(4800)!=0) {
		RS92_DBG(Serial.println("Setting bitrate 4800bit/s FAILED"));
		return 1;
	}
#if RS92_DEBUG
	float br = sx1278.getBitrate();
	Serial.print("Exact bitrate is ");
	Serial.println(br);
#endif
        if(sx1278.setAFCBandwidth(sonde.config.rs92.rxbw)!=0) {
                RS92_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.rs92.rxbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.rs92.rxbw)!=0) {
                RS92_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.rs92.rxbw));
                return 1;
        }

	// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x1E)!=0) {
		RS92_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// Set autostart_RX to 01, preamble 0, SYNC detect==on, syncsize=3 (==4 byte
	//char header[] = "0110.0101 0110.0110 1010.0101 1010.1010";

	//const char *SYNC="\x10\xB6\xCA\x11\x22\x96\x12\xF8";
	//const char *SYNC="\x08\x6D\x53\x88\x44\x69\x48\x1F";
	// was 0x57
	//const char *SYNC="\x99\x9A";
	// version 1, working with continuous RX
	const char *SYNC="\x66\x65";
	if(sx1278.setSyncConf(0x70, 2, (const uint8_t *)SYNC)!=0) {
		RS92_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
	if(sx1278.setPreambleDetect(0xA8)!=0) {
		RS92_DBG(Serial.println("Setting PreambleDetect FAILED"));
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
                RS92_DBG(Serial.println("Setting PreambleDetect FAILED"));
                return 1;
        }
	// sync config: ato restart (01), preamble polarity AA (0), sync on (1), resevered (0), syncsize 2+1 (010) => 0x52
	const char *SYNC="\x6A\x66\x69";
        if(sx1278.setSyncConf(0x52, 3, (const uint8_t *)SYNC)!=0) {
                RS92_DBG(Serial.println("Setting SYNC Config FAILED"));
                return 1;
        }
	// payload length is ((240 - 7)*10 +6)/8 = 292
#endif

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		RS92_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

	Serial.print("RS92: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
        sx1278.clearIRQFlags();

        // enable RX
        sx1278.setPayloadLength(0);  // infinite for now...
	//sx1278.setPayloadLength(292);
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

#if RS92_DEBUG
	RS92_DBG(Serial.println("Setting SX1278 config for RS92 finished\n"); Serial.println());
#endif
        return res;
}

uint32_t RS92::bits2val(const uint8_t *bits, int len) {
	uint32_t val = 0;
	for (int j = 0; j < len; j++) {
		val |= (bits[j] << (len-1-j));
	}
	return val;
}

RS92::RS92() {
}

/* RS92 reed solomon decoder, from dxlAPRS
 */


#if 0
static char crcrs(const byte frame[], uint32_t frame_len,
                int32_t from, int32_t to)
{
   uint16_t crc;
   int32_t i;
   int32_t tmp;
   crc = 0xFFFFU;
   tmp = to-3L;
   i = from;
   if (i<=tmp) for (;; i++) {
      crc = X2C_LSH(crc,16,-8)^CRCTAB[(uint32_t)((crc^(uint16_t)(uint8_t)frame[i])&0xFFU)];
      if (i==tmp) break;
   } /* end for */
   return frame[to-1L]==(char)crc && frame[to-2L]==(char)X2C_LSH(crc,
                16,-8);
} /* end crcrs() */

static int32_t getint32(const byte frame[], uint32_t frame_len,
                uint32_t p)
{
   uint32_t n;
   uint32_t i;
   n = 0UL;
   for (i = 3UL;; i--) {
      n = n*256UL+(uint32_t)(uint8_t)frame[p+i];
      if (i==0UL) break;
   } /* end for */
   return (int32_t)n;
} /* end getint32() */


static uint32_t getcard16(const byte frame[], uint32_t frame_len,
                uint32_t p)
{
   return (uint32_t)(uint8_t)frame[p]+256UL*(uint32_t)(uint8_t)
                frame[p+1UL];
} /* end getcard16() */


static int32_t getint16(const byte frame[], uint32_t frame_len,
                uint32_t p)
{
   uint32_t n;
   n = (uint32_t)(uint8_t)frame[p]+256UL*(uint32_t)(uint8_t)
                frame[p+1UL];
   if (n>=32768UL) return (int32_t)(n-65536UL);
   return (int32_t)n;
} /* end getint16() */

static void wgs84r(double x, double y, double z,
                double * lat, double * long0,
                double * heig)
{
   double sl;
   double ct;
   double st;
   double t;
   double rh;
   double xh;
   double h;
   h = x*x+y*y;
   if (h>0.0) {
      rh = (double)sqrt((float)h);
      xh = x+rh;
      *long0 = atang2(xh, y)*2.0;
      if (*long0>3.1415926535898) *long0 = *long0-6.2831853071796;
      t = (double)atan((float)(X2C_DIVL(z*1.003364089821,
                rh)));
      st = (double)sin((float)t);
      ct = (double)cos((float)t);
      *lat = (double)atan((float)
                (X2C_DIVL(z+4.2841311513312E+4*st*st*st,
                rh-4.269767270718E+4*ct*ct*ct)));
      sl = (double)sin((float)*lat);
      *heig = X2C_DIVL(rh,(double)cos((float)*lat))-(double)(X2C_DIVR(6.378137E+6f,
                sqrt((float)(1.0-6.6943799901413E-3*sl*sl))));
   }
   else {
      *lat = 0.0;
      *long0 = 0.0;
      *heig = 0.0;
   }
/*  lat:=atan(z/(rh*(1.0 - E2))); */
/*  heig:=sqrt(h + z*z) - EARTHA; */
} /* end wgs84r() */
#endif


static int32_t reedsolomon92(uint8_t *buf, uint32_t buf_len)
{  
   uint32_t i;
   int32_t res;
   uint8_t b[256];
   uint32_t eraspos[24];
   for (i = 0UL; i<=255UL; i++) {
      b[i] = 0;
   } /* end for */
   for (i = 0UL; i<=209UL; i++) {
      b[230UL-i] = buf[i+6UL];
   } /* end for */
   for (i = 0UL; i<=23UL; i++) {
      b[254UL-i] = buf[i+216UL];
   } /* end for */
   res = decodersc((char *)b, eraspos, 0L);
   if (res>0L && res<=12L) {
      for (i = 0UL; i<=209UL; i++) {
         buf[i+6UL] = b[230UL-i];
      } /* end for */
      for (i = 0UL; i<=23UL; i++) {
         buf[i+216UL] = b[254UL-i];
      } /* end for */
   }
   return res;
} /* end reedsolomon92() */

void printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

void RS92::decodeframe92(uint8_t *data)
{
	//uint32_t gpstime;
	//uint32_t flen;
	//uint32_t j;
	int32_t corr;
	corr = reedsolomon92(data, 301ul);
	//int calok;
	//int mesok;
	//uint32_t calibok;
	lastFrame = (dataptr==data1)?1:2;
	Serial.printf("rs corr is %d --- data:%p data1:%p data2:%p lastframe=%d\n", corr, data, data1, data2, lastFrame);
	dataptr = (dataptr==data1)?data2:data1;
	//print_frame(data, 240);
#if 0
	/* from sondemod*/
	int p=6;
	while(1) {
		uint8_t typ = data[p];
		if(typ==0xff) break;
		++p;
		int len = ((uint32_t)data[p])*2 + 2;
		Serial.printf("type %c: len=%d\n", typ, len);
		//printRaw(data+p, len+2);
		if(len>240) {
			Serial.print("RS92 frame too long: ");
			Serial.println(len);
			break;
		}
		++p;
		j=0;
		uint16_t crc = 0xFFFF;
		while(j<len) {
			if(j < len-2) {
				for(int ic = 0; ic<=7; ic++) {
					if (((0x8000&crc)!=0) != ( ((1<<(7-ic))&data[p])!=0 )) {
						crc <<= 1;
						crc ^= 0x1021;
					} else {
						crc <<= 1;
					}
				}
			}
			++p;
			++j;
			if(p>240) {
				Serial.println("eof");
				return;
			}
		}
		if ( (((uint8_t)(crc&0xff)) != data[p-2]) || (((uint8_t)(crc>>8)) != data[p-1])) {
			Serial.printf("************ crc error: expected %04x\n",crc);
			continue;
		}
		switch(typ) {
		case 'e':
			Serial.println("cal  "); 
			//docalib(sf, 256, objname, 9, &contextr9, &mhz, &frameno);
			// ...
			break;
		case 'i':
			if(calok && calibok==0xffffffff) {
				//domes(sf, 256, &hp, &hyg, &temp)
				mesok = 1;
			}
			break;
		case 'g':
			Serial.println("gps  "); 
			if(1||calok) {
				//dogps(data+p-len, 256, &contextr9,  &contextr9.timems, &gpstime);
			}
			break;
		case 'h':
			Serial.println("data "); break;
			if(data[p+2]!=3) Serial.println("aux ");
			// ..
			break;
		}
	}
#endif
} /* end decodeframe92() */





void RS92::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}


void RS92::stobyte92(uint8_t b)
{
	dataptr[rxp] = b;
	if(rxp>=5 || b=='*') rxp++; else rxp=0;
	if(rxp==6) { // header detected
		headerDetected = 1;	
	}
	if(rxp>=240) { // frame complete... (240 byte)
		rxp=0;
		//printRaw(data, 240);
		decodeframe92(dataptr);
		haveNewFrame = 1;
	}
} /* end stobyte92() */


uint32_t rxdata;
bool rxsearching=true;

// search for
// 101001100110011010011010011001100110100110101010100110101001
// 1010011001100110100110100110 0110.0110 1001.1010 1010.1001 1010.1001 => 0x669AA9A9
void RS92::process8N1data(uint8_t dt)
{
	for(int i=0; i<8; i++) {
		uint8_t d = (dt&0x80)?1:0;
		rxdata = (rxdata<<1) | d;
		if((rxbitc&1)==1) { rxbyte = (rxbyte>>1) + (d<<9); } // mancester decoded data
		dt <<= 1;
		//
		if(rxsearching) {
			if(rxdata == 0x669AA9A9) {
				rxsearching = false;
				rxbitc = 0;
				rxp = 6;
                                int rssi=sx1278.getRSSI();
                                int fei=sx1278.getFEI();
                                int afc=sx1278.getAFC();
                                Serial.print("Test: RSSI="); Serial.print(rssi);
                                Serial.print(" FEI="); Serial.print(fei);
                                Serial.print(" AFC="); Serial.println(afc);
                                sonde.si()->rssi = rssi;
                                sonde.si()->afc = afc;
			}
		} else {
			rxbitc = (rxbitc+1)%20;
			if(rxbitc == 0) { // got startbit, 8 data bit, stop bit
				//Serial.printf("%03x ",rxbyte);
				dataptr[rxp++] = (rxbyte>>1)&0xff;
				if(rxp==7 && dataptr[6] != 0x65) {
					Serial.printf("wrong start: %02x\n",dataptr[6]);
					rxsearching = true;
				}
				if(rxp>=240) {
					rxsearching = true;
					decodeframe92(dataptr);
					haveNewFrame = 1;
				}
			}
		}
	}
}

void process8N1dataOrig(uint8_t data)
{
	// data contains 8 bits (after mancester encoding; 4 real bit), big endian
	for(int i=0; i<4; i++) {
		uint8_t d = (data&0x80)?1:0;
		data = data << 2;
		rxbyte = (rxbyte>>1) + (d<<8);
		int maxk = 0;
		int max0 = 0;
		for(int k = 0; k< 10; k++) {
			int n = asynst[k] - asynst[(k+1)%10];
			if(abs(n)>abs(max0)) {
				max0 = n;
				maxk = k;
			}	
		}
		//Serial.printf("<%d,%d,%d>",max0,maxk,rxbitc);
		if(rxbitc == maxk) {
			if(max0<0) { rxbyte = rxbyte ^ 0xFF; }
		/////TODO	stobyte92( rxbyte&0xff );
		}
		//Serial.printf("%d:",asynst[rxbitc]);
		if(d) {
			asynst[rxbitc] += (32767-asynst[rxbitc])/16;
		} else {
			asynst[rxbitc] -= (32767+asynst[rxbitc])/16;
		}
		//Serial.printf("%d ",asynst[rxbitc]);
		rxbitc = (rxbitc+1) % 10;
	}
}

int RS92::receive() {
	unsigned long t0 = millis();
	Serial.printf("RS92::receive() start at %ld\n",t0);
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
      			process8N1data(data);
      			value = sx1278.readRegister(REG_IRQ_FLAGS2);
    		} else {
			if(headerDetected) {
				t0 = millis(); // restart timer... don't time out if header detected...
				headerDetected = 0;
			}
    			if(haveNewFrame) {
				Serial.printf("RS92::receive(): new frame complete after %ldms\n", millis()-t0);
				haveNewFrame = 0;
				return RX_OK;
			}
			delay(2);
    		}
    	}
	Serial.printf("RS92::receive() timed out\n");
    	return RX_TIMEOUT; // TODO RX_OK;
}

#define RS92MAXLEN (240)
int RS92::waitRXcomplete() {
	// called after complete...
	Serial.printf("decoding frame %d\n", lastFrame);
	print_frame(lastFrame==1?data1:data2, 240);

	SondeData *si = &( (sonde.sondeList+rxtask.receiveSonde)->d );
	si->lat = gpx.lat;
	si->lon = gpx.lon;
	si->alt = gpx.alt;
	si->vs = gpx.vU;
	si->hs = gpx.vH;
	si->dir = gpx.vD;
	si->validPos = 0x3f;
	memcpy(si->id, gpx.id, 9);
	memcpy(si->ser, gpx.id, 9);
	si->validID = true;
	si->vframe = si->frame = gpx.frnr;
	si->sats = gpx.k;
        si->time = (gpx.gpssec/1000) + 86382 + gpx.week*604800 + 315878400UL;
        si->validTime = true;	

	return 0;
}



RS92 rs92 = RS92();
#endif

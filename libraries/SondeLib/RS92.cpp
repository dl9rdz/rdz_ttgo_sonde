
/* RS92 decoder functions */
#include "RS92.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

// well...
#include "rs92gps.inc"

#define RS92_DEBUG 1

#if RS92_DEBUG
#define RS92_DBG(x) x
#else
#define RS92_DBG(x)
#endif

//static uint16_t CRCTAB[256];
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

static double atang2(double x, double y)
{
   double w;
   if (fabs(x)>fabs(y)) {
      w = (double)atan((float)(X2C_DIVL(y,x)));
      if (x<0.0) {
         if (y>0.0) w = 3.1415926535898+w;
         else w = w-3.1415926535898;
      }
   }
   else if (y!=0.0) {
      w = (double)(1.5707963267949f-atan((float)(X2C_DIVL(x,
                y))));
      if (y<0.0) w = w-3.1415926535898;
   }
   else w = 0.0;
   return w;
} /* end atang2() */


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

int RS92::setup(float frequency) 
{
#if RS92_DEBUG
	Serial.println("Setup sx1278 for RS92 sonde");
#endif
	if(!initialized) {
		Gencrctab();
		initrsc();
        	get_eph("/brdc.19n");
		initialized = true;
	}

	if(sx1278.ON()!=0) {
		RS92_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
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

	if(sx1278.setAFCBandwidth(sonde.config.rs41.agcbw)!=0) {
		RS92_DBG(Serial.println("Setting AFC bandwidth 25 kHz FAILED"));
		return 1;
	}
	if(sx1278.setRxBandwidth(sonde.config.rs41.rxbw)!=0) {
		RS92_DBG(Serial.println("Setting RX bandwidth 12kHz FAILED"));
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
	const char *SYNC="\x66\x65";
	if(sx1278.setSyncConf(0x71, 2, (const uint8_t *)SYNC)!=0) {
		RS92_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
	if(sx1278.setPreambleDetect(0xA8)!=0) {
		RS92_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		RS92_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

	Serial.print("RS92: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
        // enable RX
        sx1278.setPayloadLength(0);  // infinite for now...
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

#if RS92_DEBUG
	RS92_DBG(Serial.println("Setting SX1278 config for RS92 finished\n"); Serial.println());
#endif
        return res;
}

#if 0
int RS92::setFrequency(float frequency) {
	Serial.print("RS92: setting RX frequency to ");
	Serial.println(frequency);
	int res = sx1278.setFrequency(frequency);
	// enable RX
        sx1278.setPayloadLength(0);  // infinite for now...

	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	return res;
}
#endif

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
static void posrs41(const byte b[], uint32_t b_len, uint32_t p)
{
   double dir;
   double vu;
   double ve;
   double vn;
   double vz;
   double vy;
   double vx;
   double heig;
   double long0;
   double lat;
   double z;
   double y;
   double x;
   x = (double)getint32(b, b_len, p)*0.01;
   y = (double)getint32(b, b_len, p+4UL)*0.01;
   z = (double)getint32(b, b_len, p+8UL)*0.01;
   wgs84r(x, y, z, &lat, &long0, &heig);
   Serial.print(" ");
   sonde.si()->lat = (float)(X2C_DIVL(lat,1.7453292519943E-2));
   Serial.print(sonde.si()->lat);
   Serial.print(" ");
   sonde.si()->lon = (float)(X2C_DIVL(long0,1.7453292519943E-2));
   Serial.print(sonde.si()->lon);
   if (heig<1.E+5 && heig>(-1.E+5)) {
      Serial.print(" ");
      Serial.print((uint32_t)heig);
      Serial.print("m");
   }
   /*speed */
   vx = (double)getint16(b, b_len, p+12UL)*0.01;
   vy = (double)getint16(b, b_len, p+14UL)*0.01;
   vz = (double)getint16(b, b_len, p+16UL)*0.01;
   vn = (-(vx*(double)sin((float)lat)*(double)
                cos((float)long0))-vy*(double)
                sin((float)lat)*(double)sin((float)
                long0))+vz*(double)cos((float)lat);
   ve = -(vx*(double)sin((float)long0))+vy*(double)
                cos((float)long0);
   vu = vx*(double)cos((float)lat)*(double)
                cos((float)long0)+vy*(double)
                cos((float)lat)*(double)sin((float)
                long0)+vz*(double)sin((float)lat);
   dir = X2C_DIVL(atang2(vn, ve),1.7453292519943E-2);
   if (dir<0.0) dir = 360.0+dir;
   sonde.si()->dir = dir;
   Serial.print(" ");
   sonde.si()->hs = sqrt((float)(vn*vn+ve*ve))*3.6f;
   Serial.print(sonde.si()->hs);
   Serial.print("km/h ");
   Serial.print(dir);
   Serial.print("deg ");
   Serial.print((float)vu);
   sonde.si()->vs = vu;
   Serial.print("m/s ");
   Serial.print(getcard16(b, b_len, p+18UL)&255UL);
   Serial.print("Sats");
   sonde.si()->alt = heig;
   sonde.si()->validPos = true;
} /* end posrs41() */



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

static struct CONTEXTR9 contextr9;

void RS92::decodeframe92(uint8_t *data)
{
	uint32_t gpstime;
	uint32_t flen;
	uint32_t j;
	int32_t corr;
	corr = reedsolomon92(data, 301ul);
	int calok;
	int mesok;
	uint32_t calibok;
	Serial.printf("rs corr is %d\n", corr);
	print_frame(data, 240);
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

int RS92::bitsToBytes(uint8_t *bits, uint8_t *bytes, int len)
{
	int i;
	for(i=0; i<len*4; i++) {
	       	bytes[i/8] = (bytes[i/8]<<1) | (bits[i]?1:0);
	}
	bytes[(i-1)/8] &= 0x0F;
}

static unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

static uint8_t reverse(uint8_t n) {
   return (lookup[n&0x0f] << 4) | lookup[n>>4];
}


static uint8_t scramble[64] = {150U,131U,62U,81U,177U,73U,8U,152U,50U,5U,89U,
                14U,249U,68U,198U,38U,33U,96U,194U,234U,121U,93U,109U,161U,
                84U,105U,71U,12U,220U,232U,92U,241U,247U,118U,130U,127U,7U,
                153U,162U,44U,147U,124U,48U,99U,245U,16U,46U,97U,208U,188U,
                180U,182U,6U,170U,244U,35U,120U,110U,59U,174U,191U,123U,76U,
		193U};

static byte data[5000];

static uint8_t rxbitc;
static int32_t asynst[10]={0};
static uint16_t rxbyte;
int rxp=0;

static int haveNewFrame = 0;

void RS92::stobyte92(uint8_t b)
{
	data[rxp] = b;
	if(rxp>=5 || b=='*') rxp++; else rxp=0;
	if(rxp>=240) { // frame complete... (240 byte)
		rxp=0;
		//printRaw(data, 240);
		decodeframe92(data);
		haveNewFrame = 1;
	}
} /* end stobyte92() */



void RS92::process8N1data(uint8_t data)
{
	// data contains 8 bits, big endian
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
			stobyte92( rxbyte&0xff );
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
   	while( millis() - t0 < 2000 ) {
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
      			process8N1data(data);
      			value = sx1278.readRegister(REG_IRQ_FLAGS2);
    		} else {
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
        Serial.printf("RS92::waitRXcomplete returning %d (%s)\n", res, RXstr[res]);
        return res;
}


#if 0
int oldwaitRXcomplete() {
	Serial.println("RS92: receive frame...\n");
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

	printRaw(data, RS92MAXLEN);
	//for(int i=0; i<RS92MAXLEN; i++) { data[i] = reverse(data[i]); }
	//printRaw(data, MAXLEN);
	//for(int i=0; i<RS92MAXLEN; i++) { data[i] = data[i] ^ scramble[i&0x3F]; }
	//printRaw(data, MAXLEN);
	//int res = decode41(data, RS92MAXLEN);
#endif
	int res=0;
	return res==0 ? RX_OK : RX_ERROR;
}
#endif

RS92 rs92 = RS92();

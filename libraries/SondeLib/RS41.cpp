
/* RS41 decoder functions */
#include "RS41.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"

#define RS41_DEBUG 0

#if RS41_DEBUG
#define RS41_DBG(x) x
#else
#define RS41_DBG(x)
#endif

#define RS41MAXLEN (320)
static byte data[800];
static int dpos = 0;

static uint16_t CRCTAB[256];

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
   for (i = 0U; i<=255U; i++) {
      crc = (uint16_t)(i*256U);
      for (j = 0U; j<=7U; j++) {
         if ((0x8000U & crc)) crc = X2C_LSH(crc,16,1)^0x1021U;
         else crc = X2C_LSH(crc,16,1);
      } /* end for */
      CRCTAB[i] = X2C_LSH(crc,16,-8)|X2C_LSH(crc,16,8);
   } /* end for */
} /* end Gencrctab() */

int RS41::setup(float frequency) 
{
#if RS41_DEBUG
	Serial.println("Setup sx1278 for RS41 sonde");
#endif
	if(!initialized) {
		Gencrctab();
		initrsc();
		initialized = true;
	}

	if(sx1278.ON()!=0) {
		RS41_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	if(sx1278.setFSK()!=0) {
		RS41_DBG(Serial.println("Setting FSM mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(4800)!=0) {
		RS41_DBG(Serial.println("Setting bitrate 4800bit/s FAILED"));
		return 1;
	}
#if RS41_DEBUG
	float br = sx1278.getBitrate();
	Serial.print("Exact bitrate is ");
	Serial.println(br);
#endif

	if(sx1278.setAFCBandwidth(sonde.config.rs41.agcbw)!=0) {
		RS41_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.rs41.agcbw));
		return 1;
	}
	if(sx1278.setRxBandwidth(sonde.config.rs41.rxbw)!=0) {
		RS41_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.rs41.rxbw));
		return 1;
	}
	// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x1E)!=0) {
		RS41_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// Set autostart_RX to 01, preamble 0, SYNC detect==on, syncsize=3 (==4 byte
	//char header[] = "0110.0101 0110.0110 1010.0101 1010.1010";

	//const char *SYNC="\x10\xB6\xCA\x11\x22\x96\x12\xF8";
	const char *SYNC="\x08\x6D\x53\x88\x44\x69\x48\x1F";
	if(sx1278.setSyncConf(0x57, 8, (const uint8_t *)SYNC)!=0) {
		RS41_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
	if(sx1278.setPreambleDetect(0xA8)!=0) {
		RS41_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		RS41_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}
	Serial.print("RS41: setting RX frequency to ");
	Serial.println(frequency);
	int retval = sx1278.setFrequency(frequency);
	dpos = 0;

#if RS41_DEBUG
	RS41_DBG(Serial.println("Setting SX1278 config for RS41 finished\n"); Serial.println());
#endif
	// go go go
        sx1278.setPayloadLength(RS41MAXLEN-8);    // Expect 320-8 bytes or 518-8 bytes (8 byte header)
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	return retval;
}

uint32_t RS41::bits2val(const uint8_t *bits, int len) {
	uint32_t val = 0;
	for (int j = 0; j < len; j++) {
		val |= (bits[j] << (len-1-j));
	}
	return val;
}

RS41::RS41() {
}

/* RS41 reed solomon decoder, from dxlAPRS
 */
static int32_t reedsolomon41(byte buf[], uint32_t buf_len, uint32_t len2)
{
   uint32_t i;
   int32_t res1;
   /*tb1, */
   int32_t res;
   char b1[256];
   char b[256];
   uint32_t eraspos[24];
   uint32_t tmp;
   for (i = 0UL; i<=255UL; i++) {
      b[i] = 0;
      b1[i] = 0;
   } /* end for */
   tmp = len2;
   i = 0UL;
   if (i<=tmp) for (;; i++) {
      b[230UL-i] = buf[i*2UL+56UL];
      b1[230UL-i] = buf[i*2UL+57UL];
      if (i==tmp) break;
   } /* end for */
   for (i = 0UL; i<=23UL; i++) {
      b[254UL-i] = buf[i+8UL];
      b1[254UL-i] = buf[i+32UL];
   } /* end for */
   res = decodersc(b, eraspos, 0);
   res1 = decodersc(b1, eraspos, 0);
   if (res>0L && res<=12L) {
      tmp = len2;
      i = 0UL;
      if (i<=tmp) for (;; i++) {
         buf[i*2UL+56UL] = b[230UL-i];
         if (i==tmp) break;
      } /* end for */
      for (i = 0UL; i<=23UL; i++) {
         buf[i+8UL] = b[254UL-i];
      } /* end for */
   }
   if (res1>0L && res1<=12L) {
      tmp = len2;
      i = 0UL;
      if (i<=tmp) for (;; i++) {
         buf[i*2UL+57UL] = b1[230UL-i];
         if (i==tmp) break;
      } /* end for */
      for (i = 0UL; i<=23UL; i++) {
         buf[i+32UL] = b1[254UL-i];
      } /* end for */
   }
   if (res<0L || res1<0L) return -1L;
   else return res+res1;
   return 0;
} /* end reedsolomon41() */




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



// returns: 0: ok, -1: rs or crc error
int RS41::decode41(byte *data, int maxlen)
{
	char buf[128];	
	int crcok = 0;

	int32_t corr = reedsolomon41(data, 560, 131);  // try short frame first
	if(corr<0) {
		corr = reedsolomon41(data, 560, 230);  // try long frame
	}
	Serial.print("RS result:");
	Serial.print(corr);
	Serial.println();
	int p = 57; // 8 byte header, 48 byte RS 
	while(p<maxlen) {  /* why 555? */
		uint8_t typ = data[p++];
		uint32_t len = data[p++]+2UL;
		if(p+len>maxlen) break;

#if 1
		// DEBUG OUTPUT
		Serial.print("@");
		Serial.print(p-2);
		Serial.print(": ID:");
		Serial.print(typ, HEX);
		Serial.print(", len=");
		Serial.print(len);
		Serial.print(": ");
		for(int i=0; i<len-1; i++) {
			char buf[3];
			snprintf(buf, 4, "%02X|", data[p+i]);
			Serial.print(buf);
		}
#endif
		// check CRC
		if(!crcrs(data, 560, p, p+len)) {
			Serial.println("###CRC ERROR###");
		} else {	
		crcok = 1;
		switch(typ) {
		case 'y': // name
			{
			Serial.print("#");
			uint16_t fnr = data[p]+(data[p+1]<<8);
			Serial.print(fnr);
			Serial.print("; RS41 ID ");
			snprintf(buf, 10, "%.8s ", data+p+2);
			Serial.print(buf);
			sonde.si()->type=STYPE_RS41;
			strncpy(sonde.si()->id, (const char *)(data+p+2), 8);
			sonde.si()->id[8]=0;
			sonde.si()->validID=true;
			}
			// TODO: some more data
			break;
		case '|': // date
			break;
		case '{': // pos
			posrs41(data+p, len, 0);
			break;
		default:
			break;
		}}
		p += len;
		Serial.println();
	}
	return crcok ? 0 : -1;
}
void RS41::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

void RS41::bitsToBytes(uint8_t *bits, uint8_t *bytes, int len)
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


int RS41::receive() {
	sx1278.setPayloadLength(RS41MAXLEN-8); 
	int e = sx1278.receivePacketTimeout(1000, data+8);
	rxtask.lastSonde = rxtask.currentSonde;
	if(e) { Serial.println("TIMEOUT"); return RX_TIMEOUT; } 

        for(int i=0; i<RS41MAXLEN; i++) { data[i] = reverse(data[i]); }
        for(int i=0; i<RS41MAXLEN; i++) { data[i] = data[i] ^ scramble[i&0x3F]; }
        return decode41(data, RS41MAXLEN);
}

int RS41::waitRXcomplete() {
	// Currently not used. can be used for additinoal post-processing
	// (required for RS92 to avoid FIFO overrun in rx task)
#if 0
	int res;
	uint32_t t0 = millis();
	while(rxtask.receiveResult<0 && millis()-t0 < 3000) { delay(50); }

	if(rxtask.receiveResult<0 || rxtask.receiveResult==RX_TIMEOUT) { 
		res = RX_TIMEOUT;
	} else if (rxtask.receiveResult==0) {
		res = RX_OK;
	} else {
		res = RX_ERROR;
	}
	rxtask.receiveResult = -1;
	Serial.printf("waitRXcomplete returning %d\n", res);
	return res;
#endif
	return 0;
}

RS41 rs41 = RS41();

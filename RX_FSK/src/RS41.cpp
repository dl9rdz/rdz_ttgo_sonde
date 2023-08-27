
/* RS41 decoder functions */
#include "RS41.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"

#define RS41_DEBUG 1

#if RS41_DEBUG
#define RS41_DBG(x) x
#else
#define RS41_DBG(x)
#endif

#define RS41MAXLEN (320)
static byte data[800];
static int dpos = 0;


// whole 51 row frame as C structure
// taken from https://github.com/einergehtnochrein/ra-firmware
struct subframeBuffer {
    uint64_t valid;   // bitmask for subframe valid; lsb=frame 0, etc.
    union {
        byte rawData[51*16];
        struct __attribute__((__packed__)) {
            uint16_t crc16;                     /* CRC16 CCITT Checksum over range 0x002...0x31F */
            uint16_t frequency;                 /* 0x002: TX is on 400 MHz + (frequency / 64) * 10 kHz */
            uint8_t startupTxPower;             /* 0x004: TX power level at startup (1...7) */
            uint8_t reserved005;
            uint8_t reserved006;
            uint16_t reserved007;               /* 0x007:  ?? (some bitfield) [0],[1],[2],[3]. Init value = 0xE */
            uint16_t reserved009;               /* 0x009: ? */
            uint8_t reserved00B;
            uint8_t reserved00C;
            uint8_t serial[8];                  /* 0x00D: Sonde ID, 8 char, not terminated */
            uint16_t firmwareVersion;           /* 0x015: 10000*major + 100*minor + patch*/
            uint16_t reserved017;
            uint16_t minHeight4Flight;          /* 0x019: Height (meter above ground) where flight mode begins */
            uint8_t lowBatteryThreshold100mV;   /* 0x01B: (Default=18) Shutdown if battery voltage below this
                                                          threshold for some time (10s ?)
                                                */
            uint8_t nfcDetectorThreshold;       /* 0x01C: NFC detector threshold [25mV] (Default: 0x05 = 125mV) */
            uint8_t reserved01D;                /* 0x01D: ?? (Init value = 0xB4) */
            uint8_t reserved01E;                /* 0x01E: ?? (Init value = 0x3C) */
            uint16_t reserved01F;
            int8_t refTemperatureThreshold;     /* 0x021: Reference temperature threshold [°C] */
            uint8_t reserved022;
            uint16_t reserved023;
            uint16_t reserved025;
            int16_t flightKillFrames;           /* 0x027: Number of frames in flight until kill (-1 = disabled) */
            uint16_t reserved029;               /* 0x029: ? (Init value = 0) */
            uint8_t burstKill;                  /* 0x02B: Burst kill (0=disabled, 1=enabled) */
            uint8_t reserved02C;
            uint8_t reserved02D;
            uint16_t reserved02E;
            uint16_t reserved030;
            uint8_t reserved032;
            uint16_t reserved033;
            uint16_t reserved035;
            uint16_t reserved037;
            uint16_t reserved039;               /* 0x039: */
            uint8_t reserved03B;                /* 0x03B: */
            uint8_t reserved03C;                /* 0x03C: */
            float refResistorLow;               /* 0x03D: Reference resistor low (750 Ohms) */
            float refResistorHigh;              /* 0x041: Reference resistor high (1100 Ohms) */
            float refCapLow;                    /* 0x045: Reference capacitance low (0) */
            float refCapHigh;                   /* 0x049: Reference capacitance high (47 pF) */
            float taylorT[3];                   /* 0x04D: Tayor coefficients for main temperature calculation */
            float calT;                         /* 0x059: Calibration factor for main sensor */
            float polyT[6];                     /* 0x05D: */
            float calibU[2];                    /* 0x075: Calibration coefficients for humidity sensor */

            float matrixU[7][6];                /* 0x07D: Matrix for humidity sensor RH calculation */
            float taylorTU[3];                  /* 0x125: Coefficients for U sensor temperature calculation */
            float calTU;                        /* 0x131: Calibration factor for U temperature sensor */
            float polyTrh[6];                   /* 0x135:  */

            uint8_t reserved14D;                /* 0x14D: */
            uint32_t reserved14E;               /* 0x14E: */

            float f152;
            uint8_t u156;
            float f157;                         /* 0x157: ?? (Initialized by same value as calibU[0]) */
            uint8_t reserved15B;                /* 0x15B: */
            uint32_t reserved15C;               /* 0x15C: */
            float f160[35];
            uint8_t startIWDG;                  /* 0x1EC: If ==1 or ==2: Watchdog IWDG will not be started */
            uint8_t parameterSetupDone;         /* 0x1ED: Set (!=0) if parameter setup was done */
            uint8_t enableTestMode;             /* 0x1EE: Test mode (service menu) (0=disabled, 1=enabled) */
            uint8_t enableTX;                   /* 0x1EF: 0=TX disabled, 1=TX enabled (maybe this is autostart?) */
            float f1F0[8];
            float pressureLaunchSite[2];        /* 0x210: Pressure [hPa] at launch site */
            struct __attribute__((__packed__)){
                char variant[10];               /* 0x218: Sonde variant (e.g. "RS41-SG") */
                uint8_t mainboard[10];          /* 0x222: Name of mainboard (e.g. "RSM412") */
            } names;
            struct __attribute__((__packed__)){
                uint8_t mainboard[9];           /* 0x22C: Serial number of mainboard (e.g. "L1123553") */
                uint8_t text235[12];            /* 0x235: "0000000000" */
                uint16_t reserved241;           /* 0x241: */
                uint8_t pressureSensor[8];      /* 0x243: Serial number of pressure sensor (e.g. "N1310487") */
                uint16_t reserved24B;           /* 0x24B: */
            } serials;
            uint16_t reserved24D;               /* 0x24D: */
            uint16_t reserved24F;               /* 0x24F: */
            uint16_t reserved251;               /* 0x251: (Init value = 0x21A = 538) */
            uint8_t xdataUartBaud;              /* 0x253: 1=9k6, 2=19k2, 3=38k4, 4=57k6, 5=115k2 */
            uint8_t reserved254;
            float cpuTempSensorVoltageAt25deg;  /* 0x255: CPU temperature sensor voltage at 25°C */
            uint8_t reserved259;
            uint8_t reserved25A[0x25E -0x25A];
            float matrixP[18];                  /* 0x25E: Coefficients for pressure sensor polynomial */
            float vectorBp[3];                  /* 0x2A6: */
            uint8_t reserved2B2[8];             /* 0x2B2: */
            float matrixBt[12];                 /* 0x2BA: */
            uint8_t reserved2EA[0x2FA-0x2EA];
            uint16_t halfword2FA[9];
            float reserved30C;
            float reserved310;                  /* 0x310: */
            uint8_t reserved314;                /* 0x314: */
            uint8_t reserved315;                /* 0x315: */
            int16_t burstKillFrames;            /* 0x316: Number of active frames after burst kill */
            uint8_t reserved318[0x320-0x318];

            /* This is fragment 50. It only uses 14 valid bytes! */
            int16_t killCountdown;              /* 0x320: Counts frames remaining until kill (-1 = inactive) */
            uint8_t reserved322[6];
            int8_t intTemperatureCpu;           /* 0x328: Temperature [°C] of CPU */
            int8_t intTemperatureRadio;         /* 0x329: Temperature [°C] of radio chip */
            int8_t reserved32A;                 /* 0x32A: */
            uint8_t reserved32B;                /* 0x32B: */
            uint8_t reserved32C;                /* 0x32C: ? (the sum of two slow 8-bit counters) */
            uint8_t reserved32D;                /* 0x32D: ? (the sum of two slow 8-bit counters) */
        } value;
    };
};
// moved global variable "calibration" to sondeInfo->extra

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

double atang2(double x, double y)
{
   double w;
   if (fabs(x)>fabs(y)) {
      w = (double)atan(X2C_DIVL(y,x));
      if (x<0.0) {
         if (y>0.0) w = 3.1415926535898+w;
         else w = w-3.1415926535898;
      }
   }
   else if (y!=0.0) {
      w = (double)(1.5707963267949f-atan(X2C_DIVL(x, y)));
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

decoderSetupCfg rs41SetupCfg = {
	.bitrate = 4800,
	.rx_cfg = 0x1E, // Enable auto-AFC, auto-AGC, RX Trigger by preamble
	.sync_cfg = 0x57, // Set autostart_RX to 01, preamble 0, SYNC detect==on, syncsize=3 (==4 byte
	.sync_len = 8,
	.sync_data = (const uint8_t *)"\x08\x6D\x53\x88\x44\x69\x48\x1F",
	.preamble_cfg = 0xA8,
};


int RS41::setup(float frequency, int /*type*/) 
{
	if(!initialized) {
		Gencrctab();
		initrsc();
		initialized = true;
	}

	if(sx1278.ON()!=0) {
		RS41_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	if(DecoderBase::setup(rs41SetupCfg, sonde.config.rs41.agcbw, sonde.config.rs41.rxbw)!=0 ) {
		return 1;
	}
#if 0
	// all moved to DecoderBase now
	if(sx1278.setFSK()!=0) {
		RS41_DBG(Serial.println("Setting FSK mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(4800)!=0) {
		RS41_DBG(Serial.println("Setting bitrate 4800bit/s FAILED"));
		return 1;
	}

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
#endif

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		RS41_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}
	int retval = sx1278.setFrequency(frequency);
	dpos = 0;

        sx1278.clearIRQFlags();

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

static uint32_t getint24(const byte frame[], uint32_t frame_len, uint32_t p) {  // 24bit unsigned int
    uint32_t val24 = 0;
    val24 = frame[p] | (frame[p+1]<<8) | (frame[p+2]<<16);
    return val24;
}

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

// also used by MP3H.cpp
void wgs84r(double x, double y, double z,
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
      rh = sqrt(h);
      xh = x+rh;
      *long0 = atang2(xh, y)*2.0;
      if (*long0>3.1415926535898) *long0 = *long0-6.2831853071796;
      t = atan(X2C_DIVL(z*1.003364089821, rh));
      st = sin(t);
      ct = cos(t);
      *lat = atan((X2C_DIVL(z+4.2841311513312E+4*st*st*st,
                rh-4.269767270718E+4*ct*ct*ct)));
      sl = sin(*lat);
      *heig = X2C_DIVL(rh,cos(*lat))-(X2C_DIVR(6.378137E+6f,
                sqrt((1.0-6.6943799901413E-3*sl*sl))));
   }
   else {
      *lat = 0.0;
      *long0 = 0.0;
      *heig = 0.0;
   }
/*  lat:=atan(z/(rh*(1.0 - E2))); */
/*  heig:=sqrt(h + z*z) - EARTHA; */
} /* end wgs84r() */

// returns: 0=ok, -1=error
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
   SondeData *si = &(sonde.si()->d);
   x = (double)getint32(b, b_len, p)*0.01;
   y = (double)getint32(b, b_len, p+4UL)*0.01;
   z = (double)getint32(b, b_len, p+8UL)*0.01;
   uint8_t sats = getcard16(b, b_len, p+18UL)&255UL;
   Serial.printf("x:%g, y:%g, z:%g  sats:%d\n", x, y, z, sats);
   if( sats<4 || (x==0 && y==0 && z==0) ) {
      // RS41 sometimes sends frame with all 0
      // or, if sats<4, data is simply garbage. do not use.
      if(si->validPos) si->validPos |= 0x80; // flag as old
      return;
   }
   si->sats = sats;
   wgs84r(x, y, z, &lat, &long0, &heig);
   Serial.print(" ");
   si->lat = (float)(X2C_DIVL(lat,1.7453292519943E-2));
   Serial.print(si->lat);
   Serial.print(" ");
   si->lon = (float)(X2C_DIVL(long0,1.7453292519943E-2));
   Serial.print(si->lon);
   if (heig<1.E+5 && heig>(-1.E+5)) {
      Serial.print(" ");
      Serial.print((uint32_t)heig);
      Serial.print("m");
   }
   /*speed */
   vx = (double)getint16(b, b_len, p+12UL)*0.01;
   vy = (double)getint16(b, b_len, p+14UL)*0.01;
   vz = (double)getint16(b, b_len, p+16UL)*0.01;
   vn = (-(vx*sin(lat)*cos(long0))-vy*sin(lat)*sin(long0))+vz*cos(lat);
   ve = -(vx*sin(long0))+vy*cos(long0);
   vu = vx*cos(lat)*cos(long0)+vy*cos(lat)*sin(long0)+vz*sin(lat);
   dir = X2C_DIVL(atang2(vn, ve),1.7453292519943E-2);
   if (dir<0.0) dir = 360.0+dir;
   si->dir = dir;
   Serial.print(" ");
   si->hs = sqrt(vn*vn+ve*ve);
   Serial.print(si->hs*3.6);
   Serial.print("km/h ");
   Serial.print(dir);
   Serial.print("deg ");
   Serial.print((float)vu);
   si->vs = vu;
   Serial.print("m/s ");
   si->alt = heig;
   if( 0==(int)(lat*10000) && 0==(int)(long0*10000) ) {
      if(si->validPos) {
	// we have an old position, so keep previous position and mark it as old
	si->validPos |= 0x80;
      }
   }
   else
      si->validPos = 0x7f;
} /* end posrs41() */

void ProcessSubframe( byte *subframeBytes, int subframeNumber ) {
   // the total subframe consists of 51 rows, each row 16 bytes
   // based on https://github.com/bazjo/RS41_Decoding/tree/master/RS41-SGP#Subframe 
   struct subframeBuffer *s = (struct subframeBuffer *)sonde.si()->extra;
   // Allocate on demand
   if(!s) {
      s = (struct subframeBuffer *)malloc( sizeof(struct subframeBuffer) );
      if(!s) { Serial.println("ProcessSubframe: out of memory"); return; }
      sonde.si()->extra = s;
      s->valid = 0;
   }
   memcpy( s->rawData+16*subframeNumber, subframeBytes, 16);
   s->valid |= (1ULL << subframeNumber);
   Serial.printf("subframe %d; valid: %x%08x\n", subframeNumber, (uint32_t)(s->valid>>32), (uint32_t)s->valid);
   for(int i=0; i<16; i++) { Serial.printf("%02x[%c]", subframeBytes[i],( subframeBytes[i]>20 && subframeBytes[i]<127)? subframeBytes[i] : '.'); }
   Serial.println("");
   // subframeReceived[subframeNumber] = true; // mark this row of the total subframe as complete

   #if 0
      Serial.printf("subframe number: 0x%02X\n", subframeNumber );
      Serial.print("subframe values: ");
      for ( int i = 0; i < 16; i++ ) {
         Serial.printf( "%02X ", subframeBytes[i] ); 
      }
      Serial.println();

      Serial.println("Full subframe");
      for ( int j = 0; j<51; j++ ) {
         Serial.printf("%03X ", j*16);
         for ( int i = 0; i < 16; i++ ) {
            Serial.printf( "%02X ", s.rawData[j*16+i] ); 
         }
         Serial.println();
      }
      Serial.println();
   #endif   
}

/* Find the water vapor saturation pressure for a given temperature.
 * Uses the Hyland and Wexler equation with coefficients for T < 0°C.
 */
// taken from https://github.com/einergehtnochrein/ra-firmware
static float _RS41_waterVaporSaturationPressure (float Tcelsius)
{
    /* Convert to Kelvin */
    float T = Tcelsius + 273.15f;

    /* Apply some correction magic */
    T = 0
        - 0.4931358f
        + (1.0f + 4.61e-3f) * T
        - 1.3746454e-5f * T * T
        + 1.2743214e-8f * T * T * T
        ;

    /* Plug into H+W equation */
    float p = expf(-5800.2206f / T
                  + 1.3914993f
                  + 6.5459673f * logf(T)
                  - 4.8640239e-2f * T
                  + 4.1764768e-5f * T * T
                  - 1.4452093e-8f * T * T * T
                  );

    /* Scale result to hPa */
    return p / 100.0f;
}

#define PM(x) calibration->value.matrixP[x]
// CALIB_P: matrixP (frames 0x25..0x2A) and type (frame 0x21)
#define CALIB_P ((0x3Fll<<0x25)|(1ll<<0x21))
float GetRAP( uint32_t m, uint32_t m1, uint32_t m2, int16_t ptraw) {
   struct subframeBuffer *calibration = (struct subframeBuffer *)sonde.si()->extra;
   float pt = (float)ptraw*0.01;
   float pw[6];
   pw[0] = PM(0) + pt*PM(7) + pt*pt*PM(11) + pt*pt*pt*PM(15);
   pw[1] = PM(1) + pt*PM(8) + pt*pt*PM(12) + pt*pt*pt*PM(16);
   pw[2] = PM(2) + pt*PM(9) + pt*pt*PM(13) + pt*pt*pt*PM(17);
   pw[3] = PM(3) + pt*PM(10)+ pt*pt*PM(14);
   pw[4] = PM(4);
   pw[5] = PM(5);
   float f = (float)m; //meas[9];
   float f1 = (float)m1; //meas[10];
   float f2 = (float)m2;  //meas[11];
   float r = f-f1;
   if(r!=0.0) {
      r = (f2-f1) * PM(6) / r;
      float xx = 1.0;
      float p = 0.0;
      for(int i=0; i<=5; i++) {
         p += pw[i] * xx;
         xx = xx * r;
      }
      return p;
   }
   return NAN;
}

// taken from https://github.com/einergehtnochrein/ra-firmware
float GetRATemp( uint32_t measuredCurrent, uint32_t refMin, uint32_t refMax, float calT, float taylorT[3], float polyT[6] ) {
   struct subframeBuffer *calibration = (struct subframeBuffer *)sonde.si()->extra;
   /* Reference values for temperature are two known resistors.
    * From that we can derive the resistance of the sensor.
    */
   float current = ( float(measuredCurrent) - float(refMin) ) / float(refMax - refMin);
   float res = calibration->value.refResistorLow
            + (calibration->value.refResistorHigh - calibration->value.refResistorLow) * current;
   float x = res * calT;

   float Tuncal = 0
            + taylorT[0]
            + taylorT[1] * x
            + taylorT[2] * x * x;

   /* Apply calibration polynomial */
   float temperature =
         Tuncal + polyT[0]
         + polyT[1] * Tuncal
         + polyT[2] * Tuncal * Tuncal
         + polyT[3] * Tuncal * Tuncal * Tuncal
         + polyT[4] * Tuncal * Tuncal * Tuncal * Tuncal
         + polyT[5] * Tuncal * Tuncal * Tuncal * Tuncal * Tuncal;

   return temperature;
}

// taken from https://github.com/einergehtnochrein/ra-firmware
float GetRAHumidity( uint32_t humCurrent, uint32_t humMin, uint32_t humMax, float sensorTemp, float externalTemp, float pressure ) {
   struct subframeBuffer *calibration = (struct subframeBuffer *)sonde.si()->extra;
   float current = float( humCurrent - humMin) / float( humMax - humMin );
   /* Compute absolute capacitance from the known references */
   float C = calibration->value.refCapLow
            + (calibration->value.refCapHigh - calibration->value.refCapLow) * current;

   /* Apply calibration */
   float Cp = ( C / calibration->value.calibU[0] - 1.0f) * calibration->value.calibU[1];

   /* Compensation for low temperature and pressure at altitude */
   if(isnan(pressure)) {
      // if no pressure is available (non-SGP), estimate based on altitude
      pressure = 1013.25f * expf(-1.18575919e-4f * sonde.si()->d.alt );
   }

   float Tp = (sensorTemp - 20.0f) / 180.0f;
   float sum = 0;
   float powc = 1.0f;
   float p = pressure / 1000.0f;
   for ( int i = 0; i < 3; i++) {
      float l = 0;
      float powt = 1.0f;
      for ( int j = 0; j < 4; j++) {
         l += calibration->value.matrixBt[4*i+j] * powt;
         powt *= Tp;
      }
      float x = calibration->value.vectorBp[i];
      sum += l * (x * p / (1.0f + x * p) - x * powc / (1.0f + x));
      powc *= Cp;
   }
   Cp -= sum;

   float xj = 1.0f;
   for ( int j = 0; j < 7; j++) {
      float yk = 1.0f;
      for ( int k = 0; k < 6; k++) {
         sum += xj * yk * calibration->value.matrixU[j][k];
         yk *= Tp;
      }
      xj *= Cp;
   }

   /* Since there is always a small difference between the temperature readings for
    * the atmospheric (main) tempoerature sensor and the temperature sensor inside
    * the humidity sensor device, transform the humidity value to the atmospheric conditions
    * with its different water vapor saturation pressure.
   */
   float RH = sum
            * _RS41_waterVaporSaturationPressure(sensorTemp)
            / _RS41_waterVaporSaturationPressure(externalTemp);

   return RH;
}

// returns: 0: ok, -1: rs or crc error
int RS41::decode41(byte *data, int maxlen)
{
	char buf[128];	
	int crcok = 1;
	SondeData *si = &(sonde.si()->d);

	int32_t corr = reedsolomon41(data, 560, 131);  // try short frame first
	if(corr<0) {
		corr = reedsolomon41(data, 560, 230);  // try long frame
	}
#if 0
	Serial.print("RS result:");
	Serial.print(corr);
	Serial.println();
#endif
	int p = 57; // 8 byte header, 48 byte RS 
	while(p<maxlen) {  /* why 555? */
		uint8_t typ = data[p++];
		uint32_t len = data[p++]+2UL;
		if(p+len>maxlen) break;

#if 0
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
			crcok = 0;
		} else {	
		switch(typ) {
		case 'y': // name
			{
			if(strncmp(si->id, (const char *)(data+p+2), 8)) {
				// ID changed, i.e. new sonde on same frequency. clear calibration and all other data
				sonde.clearAllData(sonde.si());
				struct subframeBuffer *sub = (struct subframeBuffer *)sonde.si()->extra;
				if(sub) { sub->valid = 0; }
			}
			Serial.print("#");
			uint16_t fnr = data[p]+(data[p+1]<<8);
			Serial.print(fnr);
			si->vframe = si->frame = fnr;
			Serial.print("; RS41 ID ");
			snprintf(buf, 10, "%.8s ", data+p+2);
			Serial.print(buf);
			si->batteryVoltage = data[p+10] / 10.0f;
			// not needed, if we end up here, the type has to be RS41.... si->type=STYPE_RS41;
			strncpy(si->id, (const char *)(data+p+2), 8);
			si->id[8]=0;
			strncpy(si->ser, (const char *)(data+p+2), 8);
			si->ser[8]=0;
			si->validID=true;
			int calnr = data[p+23];
			// not sure about this
			if(calnr==0x31) {
				uint16_t bt = data[p+30] + 256*data[p+31];
				si->burstKT = bt;
			}
			// this should be right...
			if(calnr==0x02) {
				uint16_t kt = data[p+31] + 256*data[p+32];
				si->launchKT = kt;
			}
			// and this seems fine as well...
			if(calnr==0x32) {
				uint16_t cntdown = data[p+24] + (data[p+25]<<8);
				uint16_t min = cntdown - (cntdown/3600)*3600;
				Serial.printf("Countdown value: %d\n [%2d:%02d:%02d]", cntdown, cntdown/3600, min/60, min-(min/60)*60);
				si->countKT = cntdown;
				si->crefKT = fnr;
			}
			ProcessSubframe( data+p+24, calnr );

			}
			// TODO: some more data
			break;
		case '|': // date
			{
			uint32_t gpstime = getint32(data, 560, p+2);
			uint16_t gpsweek = getint16(data, 560, p);
			// UTC is GPSTIME - 18s (24*60*60-18 = 86382)
			// one week = 7*24*60*60 = 604800 seconds
			// unix epoch starts jan 1st 1970 0:00
			// gps time starts jan 6, 1980 0:00. thats 315964800 epoch seconds.
			// subtracting 86400 yields 315878400UL
			si->time = (gpstime/1000) + 86382 + gpsweek*604800 + 315878400UL;
			si->validTime = true;
			}
			break;
		case '{': // pos
			posrs41(data+p, len, 0);
			break;
		case 'z': // 0x7a is character z - 7A-MEAS temperature and humidity frame
		case '\x7f': //0x7f - short MEAS, no pressure
         {
      		uint32_t tempMeasMain = getint24(data, 560, p+0);
		      uint32_t tempMeasRef1 = getint24(data, 560, p+3);
			   uint32_t tempMeasRef2 = getint24(data, 560, p+6);
  			   uint32_t humidityMain = getint24(data, 560, p+9);
			   uint32_t humidityRef1 = getint24(data, 560, p+12);
			   uint32_t humidityRef2 = getint24(data, 560, p+15);
			   uint32_t tempHumiMain = getint24(data, 560, p+18);
			   uint32_t tempHumiRef1 = getint24(data, 560, p+21);
			   uint32_t tempHumiRef2 = getint24(data, 560, p+24);
			   uint32_t pressureMain;
			   uint32_t pressureRef1;
			   uint32_t pressureRef2;
			   int16_t  ptraw;
		if (typ == 'z') {
			     pressureMain = getint24(data, 560, p+27);
			     pressureRef1 = getint24(data, 560, p+30);
			     pressureRef2 = getint24(data, 560, p+33);
			     ptraw = getint16(data, 560, p+38);
		}
            #if 0
               Serial.printf( "External temp: tempMeasMain = %ld, tempMeasRef1 = %ld, tempMeasRef2 = %ld\n", tempMeasMain, tempMeasRef1, tempMeasRef2 );
               Serial.printf( "Rel  Humidity: humidityMain = %ld, humidityRef1 = %ld, humidityRef2 = %ld\n", humidityMain, humidityRef1, humidityRef2 );
               Serial.printf( "Humid  sensor: tempHumiMain = %ld, tempHumiRef1 = %ld, tempHumiRef2 = %ld\n", tempHumiMain, tempHumiRef1, tempHumiRef2 );
               if (typ == 'z') {
                  Serial.printf( "Pressure sens: pressureMain = %ld, pressureRef1 = %ld, pressureRef2 = %ld\n", pressureMain, pressureRef1, pressureRef2 );
               }
            #endif
   	    struct subframeBuffer *calibration = (struct subframeBuffer *)(sonde.si()->extra);
		 // temp: 0xF8==bits 3..7 : we need refResistorlow/high, taylorT, polyT
	         bool validExternalTemperature = calibration!=NULL && (calibration->valid & 0xF8) == 0xF8;

		 // humidity:  bits 3..20 and 37..46.  and bit 33 (variant)
	         bool validHumidity = calibration!=NULL && (calibration->valid & 0x7FE2001FFFF8) == 0x7FE2001FFFF8;

		 // pressure:  bits 33 and 37..42 (variant; x25..x2a: matrixP)    /// CALIB_P is    0x7E200000000)
		 bool validPressure = calibration!=NULL && (calibration->valid & CALIB_P)==CALIB_P && calibration->value.names.variant[7]=='P' && (typ == 'z');

	    if ( validPressure ) {
	       si->pressure = GetRAP( pressureMain, pressureRef1, pressureRef2, ptraw );
	       Serial.printf("Pressure sensor = %f\n", si->pressure);
	    }

            if ( validExternalTemperature ) {
               si->temperature = GetRATemp( tempMeasMain, tempMeasRef1, tempMeasRef2,
                                 calibration->value.calT, calibration->value.taylorT, calibration->value.polyT );
               Serial.printf("External temperature = %f\n", si->temperature );
            }

            if ( validHumidity && validExternalTemperature ) {
               si->tempRHSensor = GetRATemp( tempHumiMain, tempHumiRef1, tempHumiRef2, 
                                                    calibration->value.calTU, calibration->value.taylorTU, calibration->value.polyTrh );
               Serial.printf("Humidity Sensor temperature = %f\n", si->tempRHSensor );
               si->relativeHumidity = GetRAHumidity( humidityMain, humidityRef1, humidityRef2, si->tempRHSensor, si->temperature, si->pressure );
               Serial.printf("Relative humidity = %f\n", si->relativeHumidity );
            }
         }
         break;



		default:
			break;
		}}
		p += len;
		Serial.println();
	}
	return crcok ? 0 : RX_ERROR;
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
#if 1
	if(e) { /*Serial.println("TIMEOUT");*/ return RX_TIMEOUT; } 

        for(int i=0; i<RS41MAXLEN; i++) { data[i] = reverse(data[i]); }
        for(int i=0; i<RS41MAXLEN; i++) { data[i] = data[i] ^ scramble[i&0x3F]; }
        return decode41(data, RS41MAXLEN);
#else
	// FAKE testing data
	SondeInfo *si = sonde.si();
	si->lat = 48;
	si->lon = -100;
	si->alt = 30000;
	si->vs = 3.4;
	si->validPos = 0x7f;
	si->validID = 1;
	strcpy(si->id, "A1234");
	return 0;
#endif
}

int RS41::waitRXcomplete() {
	// Currently not used. can be used for additinoal post-processing
	// (required for RS92 to avoid FIFO overrun in rx task)
	return 0;
}

// copy variant string to buf (max buflen chars; buflen should be 11
// return 0 if subtype is available, -1 if not
int RS41::getSubtype(char *buf, int buflen, SondeInfo *si) {
	struct subframeBuffer *sf = (struct subframeBuffer *)si->extra;
	if(!sf) return -1;
	if( ( (sf->valid>>0x21) &3) != 3 ) return -1;   // or 1 instead of 3 for the first 8 chars only, as in autorx?
	if(buflen>11) buflen=11;			    // then buflen should be capped at 9 (8+trailing \0)
	strncpy(buf, sf->value.names.variant, buflen);
	buf[buflen-1]=0;
	if(*buf==0) return -1;
	Serial.printf("subframe valid: %x%08x; subtype=%s\n", (uint32_t)(sf->valid>>32), (uint32_t)sf->valid, buf);
	return 0;
}

RS41 rs41 = RS41();

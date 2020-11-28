
/* DFM decoder functions */
#include "DFM.h"
#include "SX1278FSK.h"
#include "Sonde.h"

#define DFM_DEBUG 1

#if DFM_DEBUG
#define DFM_DBG(x) x
#else
#define DFM_DBG(x)
#endif

int DFM::setup(float frequency, int inv) 
{
	inverse = inv;
#if DFM_DEBUG
	Serial.printf("Setup sx1278 for DFM sonde (inv=%d)\n",inv);
#endif
	if(sx1278.ON()!=0) {
		DFM_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	if(sx1278.setFSK()!=0) {
		DFM_DBG(Serial.println("Setting FSM mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(2500)!=0) {
		DFM_DBG(Serial.println("Setting bitrate 2500bit/s FAILED"));
		return 1;
	}
#if DFM_DEBUG
	float br = sx1278.getBitrate();
	Serial.print("Exact bitrate is ");
	Serial.println(br);
#endif

        if(sx1278.setAFCBandwidth(sonde.config.dfm.agcbw)!=0) {
                DFM_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.dfm.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.dfm.rxbw)!=0) {
                DFM_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.dfm.rxbw));
                return 1;
        }
	// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x1E)!=0) {
		DFM_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// Set autostart_RX to 01, preamble 0, SYNC detect==on, syncsize=3 (==4 byte
	//char header[] = "0110.0101 0110.0110 1010.0101 1010.1010";

	const char *SYNC=inverse?"\x9A\x99\x5A\x55":"\x65\x66\xA5\xAA";
	if(sx1278.setSyncConf(0x53, 4, (const uint8_t *)SYNC)!=0) {
		DFM_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
	if(sx1278.setPreambleDetect(0xA8)!=0) {
		DFM_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}

	// Packet config 1: fixed len, mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x28, 0x40)!=0) {
		DFM_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}
        Serial.print("DFM: setting RX frequency to ");
        Serial.println(frequency);

	int retval = sx1278.setFrequency(frequency);
        sx1278.clearIRQFlags();
	DFM_DBG(Serial.println("Setting SX1278 config for DFM finished\n"); Serial.println());
	return retval;
}


#define bitpick(value,bitpos) (((value)>>(7-(bitpos)))&0x01)
// Input: str: packed data, MSB first
void DFM::deinterleave(uint8_t *str, int L, uint8_t *block) {
	int i, j;
	for (j = 0; j < B; j++) {  // L = 7 (CFG), 13 (DAT1, DAT2)
		for (i = 0; i < L; i++) {
			block[B*i+j] = bitpick( str[(L*j+i)/8], (L*j+i)&7 )?0:1;
		}
	}
}
        
uint32_t DFM::bits2val(const uint8_t *bits, int len) {
	uint32_t val = 0;
	for (int j = 0; j < len; j++) {
		val |= (bits[j] << (len-1-j));
	}
	return val;
}

// Error correction for hamming code
// returns 0: ok   >0: 1 error was corrected -1: uncorrectable error
int DFM::check(uint8_t code[8]) {
	int i, j;            
	uint32_t synval = 0;  
	uint8_t syndrom[4];   
	int ret=0;

	for (i = 0; i < 4; i++) { 
		syndrom[i] = 0;
		for (j = 0; j < 8; j++) { 
			syndrom[i] ^= H[i][j] & code[j];
		}
	}
	synval = bits2val(syndrom, 4);
	if (synval) {
		ret = -1;
		for (j = 0; j < 8; j++) {   // 1-bit-error
			if (synval == He[j]) {  // reicht auf databits zu pruefen, d.h.
				ret = j+1;          // (systematischer Code) He[0..3]
				break;
			}
		}
	}
	else ret = 0;
	if (ret > 0) code[ret-1] ^= 0x1;

	return ret;
}

// Extended (8,4) Hamming code
// Return number of corrected bits, -1 if uncorrectable error
int DFM::hamming(uint8_t *ham, int L, uint8_t *sym) {
	int i, j;
	int ret = 0;               // DFM: length L = 7 or 13
	for (i = 0; i < L; i++) {  // L bytes (4bit data, 4bit parity)
		if (use_ecc) {
			int res = check(ham+8*i);
			if(ret>=0 && res>=0) ret += res; else ret=-1;
		}
		// systematic Hamming code: copy bits 0..3
		for (j = 0; j < 4; j++) {
			sym[4*i+j] = ham[8*i+j];
		}
	}
	return ret;
}

DFM::DFM() {
}

void DFM::printRaw(const char *label, int len, int ret, const uint8_t *data)
{
	Serial.print(label); Serial.print("(");
	Serial.print(ret);
	Serial.print("):");
	int i;
	for(i=0; i<len/2; i++) {
		char str[10];
		snprintf(str, 10, "%02X", data[i]);
		Serial.print(str);
	}
	Serial.print(data[i]&0x0F, HEX);
	Serial.print(" ");
}

// single data structure, search restarts after decoder change
static struct st_dfmstat {
	int idcnt0;
	int idcnt1;
	int lastfrid;
	int lastfrcnt;
	uint8_t start[50];
	uint16_t dat[50*2];
	uint8_t cnt[50*2];
	uint8_t nameregok;
	uint8_t nameregtop;
} dfmstate;


#define DFMIDTHRESHOLD 2
/* inspired by oe5dxl's finddnmae in sondeudp.c of dxlaprs */
void DFM::finddfname(uint8_t *b)
{
	uint8_t st;
	uint32_t thres;
	uint32_t i;
	uint8_t ix;
	uint16_t d;

	st = b[0];    /* frame start byte */
	ix = b[3];    /* hi/lo part of ser;  (LSB due to our bitsToBytes...) */
	d = (b[1]<<8) + b[2];  /* data byte */
	i = 0;
        /* find highest channel number single frame serial,
           (2 frame serial will make a single serial too) */
	if(dfmstate.idcnt0 < DFMIDTHRESHOLD && dfmstate.idcnt1 < DFMIDTHRESHOLD) {
		uint32_t v = (st<<20) | (d<<4) | ix;
		if ( st > (dfmstate.lastfrid>>20) ) {
			dfmstate.lastfrid = v;
			Serial.print("MAXCH: "); Serial.println(st>>20);
			dfmstate.lastfrcnt = 0;
		} else if ( st == (dfmstate.lastfrid>>20) ) {
			/* same id found */
            		if (v == dfmstate.lastfrid) {
				++dfmstate.lastfrcnt;
               			thres = DFMIDTHRESHOLD * 2;
 				/* may be a 2 frame serial so increase safety level */
               			if (ix <= 1) thres *= 2; 
                		/* may be not a dfm6 so increase safety level */
               			if ( (st>>4) != 6) thres *= 2;
               			if (dfmstate.lastfrcnt >= thres) {
                  			/* id found */
                  			if (dfmstate.lastfrcnt == thres) {
						uint32_t id = ((st&0x0F)<<20) | (d<<4) | ix;
						uint32_t chkid = id;
						int i;
						/* check validity */
						for(i=0; i<6; i++) {
							if((chkid&0x0f)>9)  { break; /* not ok */ }
							chkid >>= 4;
						}
						if(i==6) {
							snprintf(sonde.si()->id, 10, "D%x ", id);
							sonde.si()->validID = true;
							return;
						}
						dfmstate.lastfrcnt = 0;
						Serial.println(" NOT NUMERIC SERIAL");
                     			}
                  			//anonym->idtime = osic_time();
       			         } else {
					Serial.print(" MAXCHCNT/SECURITYLEVEL:");
					Serial.print(dfmstate.lastfrcnt);
					Serial.print("/");
					Serial.println(thres);
				}
			} else {
               			dfmstate.lastfrid = v; /* not stable ser */
               			dfmstate.lastfrcnt = 0UL;
            		}
		}
      } /*find highest channel number single frame serial */

	while (i<dfmstate.nameregtop && dfmstate.start[i]!=st) i++;
	if (i<dfmstate.nameregtop) {
        	if (ix<=1UL && (dfmstate.cnt[2*i+ix]==0 || dfmstate.dat[2*i+ix]==d)) {
        		dfmstate.dat[2*i+ix] = d;
			if(dfmstate.cnt[2*i+ix] < 255) dfmstate.cnt[2*i+ix]++;
			Serial.print(" ID:");
			Serial.print(st, HEX);
			Serial.print("[");
			Serial.print(ix);
			Serial.print("] CNT:");
			Serial.print(dfmstate.cnt[2*i]);
			Serial.print(",");
			Serial.println(dfmstate.cnt[2*i+1]);
			if(dfmstate.cnt[2*i]>DFMIDTHRESHOLD && dfmstate.cnt[2*i+1]>DFMIDTHRESHOLD) {
				if(dfmstate.idcnt0 == 0) {
					dfmstate.idcnt0 = dfmstate.cnt[2*i];
					dfmstate.idcnt1 = dfmstate.cnt[2*i+1];
					dfmstate.nameregok = i;
					// generate id.....
					snprintf(sonde.si()->id, 10, "D%d", ((dfmstate.dat[2*i]<<16)|dfmstate.dat[2*i+1])%100000000);
					Serial.print(" NEW AUTOID:");
					Serial.println(sonde.si()->id);
					sonde.si()->validID = true;
				}
				if(dfmstate.nameregok==i) {
					Serial.println("ID OK");
					// idtime = .... /* TODO */
				}
			}
		} else {
               		/* data changed so not ser */
			dfmstate.cnt[2*i] = 0;
			dfmstate.cnt[2*i+1] = 0;
			if(dfmstate.nameregok == i) { /* found id wrong */
				dfmstate.idcnt0 = 0;
				dfmstate.idcnt1 = 0;
			}
		}
      	} else if (ix<=1) {  /* add new entry for possible ID */
	    dfmstate.start[dfmstate.nameregtop] = st;
	    dfmstate.cnt[2*dfmstate.nameregtop] = 0;
	    dfmstate.cnt[2*dfmstate.nameregtop+1] = 0;
	    dfmstate.cnt[2*dfmstate.nameregtop+ix] = 1;
	    dfmstate.dat[2*dfmstate.nameregtop+ix] = d;
	    if(dfmstate.nameregtop<49) dfmstate.nameregtop++;
      	}
}

void DFM::decodeCFG(uint8_t *cfg)
{
#if 1
	// new ID
	finddfname(cfg);
#else
	// old ID
	static int lowid, highid, idgood=0, type=0;
	if((cfg[0]>>4)==0x06 && type==0) {   // DFM-6 ID
		lowid = ((cfg[0]&0x0F)<<20) | (cfg[1]<<12) | (cfg[2]<<4) | (cfg[3]&0x0f);
		Serial.print("DFM-06 ID: "); Serial.print(lowid, HEX);
		snprintf(sonde.si()->id, 10, "%x", lowid);
		sonde.si()->validID = true;
	}
	if((cfg[0]>>4)==0x0A) {  // DMF-9 ID
		type=9;
		if(cfg[3]==1) {
			lowid = (cfg[1]<<8) | cfg[2];
			idgood |= 1;
		} else {
			highid = (cfg[1]<<8) | cfg[2];
			idgood |= 2;
		}
		if(idgood==3) {
			uint32_t dfmid = (highid<<16) | lowid;
			Serial.print("DFM-09 ID: "); Serial.print(dfmid); 
			snprintf(sonde.si()->ser, 10, "%d", dfmid);
	                // dxlAPRS sonde number (DF6 (why??) and 5 last digits of serial number as hex number
			snprintf(sonde.si()->id, 9, "DF6%05X", dfmid&0xfffff);
			sonde.si()->validID = true;
		}
	}
#endif
}

static int bitCount(int x) {
    int m4 = 0x1 | (0x1<<8) | (0x1<<16) | (0x1<<24);
    int m1 = 0xFF; 
    int s4 = (x&m4) + ((x>>1)&m4) + ((x>>2)&m4) + ((x>>3)&m4) + ((x>>4)&m4) + ((x>>5)&m4) + ((x>>6)&m4) + ((x>>7)&m4);
    int s1 = (s4&m1) + ((s4>>8)&m1) + ((s4>>16)&m1) + ((s4>>24)&m1);
    return s1;
}

static uint16_t MON[]={0,0,31,59,90,120,151,181,212,243,273,304,334};

void DFM::decodeDAT(uint8_t *dat)
{
	Serial.print(" DAT["); Serial.print(dat[6]); Serial.print("]: ");
	switch(dat[6]) {
	case 0:
		Serial.print("Packet counter: "); Serial.print(dat[3]);	
		sonde.si()->frame = dat[3];
		break;
	case 1:
		{
		int val = (((uint16_t)dat[4])<<8) + (uint16_t)dat[5];
		Serial.print("UTC-msec: "); Serial.print(val);
		sonde.si()->sec = val/1000;
		uint32_t tmp = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + ((uint32_t)dat[3]);
		sonde.si()->sats = bitCount(tmp); // maybe!?!?!?
		}
		break;
	case 2:
		{
		float lat, vh;
		lat = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + ((uint32_t)dat[3]);
		vh = ((uint16_t)dat[4]<<8) + dat[5];
		Serial.print("GPS-lat: "); Serial.print(lat*0.0000001);
		Serial.print(", hor-V: "); Serial.print(vh*0.01);
		sonde.si()->lat = lat*0.0000001;
		sonde.si()->hs = vh*0.01;
		sonde.si()->validPos |= 0x11; 
		}
		break;
	case 3:
		{
		float lon, dir;
		lon = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + (uint32_t)dat[3];
		dir = ((uint16_t)dat[4]<<8) + dat[5];
		Serial.print("GPS-lon: "); Serial.print(lon*0.0000001);
		Serial.print(", dir: "); Serial.print(dir*0.01);
		sonde.si()->lon = lon*0.0000001;
		sonde.si()->dir = dir*0.01;
		sonde.si()->validPos |= 0x42;
		}
		break;
	case 4:
		{
		float alt, vv;
		alt = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + dat[3];
		vv = (int16_t)( ((int16_t)dat[4]<<8) | dat[5] );
		Serial.print("GPS-height: "); Serial.print(alt*0.01);
		Serial.print(", vv: "); Serial.print(vv*0.01);
		sonde.si()->alt = alt*0.01;
		sonde.si()->vs = vv*0.01;
		sonde.si()->validPos |= 0x0C;
		}
		break;
	case 8:
		{
		int y = (dat[0]<<4) + (dat[1]>>4);
		int m = dat[1]&0x0F;
		int d = dat[2]>>3;
		int h = ((dat[2]&0x07)<<2) + (dat[3]>>6);
		int mi = (dat[3]&0x3F);
		char buf[100];
		snprintf(buf, 100, "%04d-%02d-%02d %02d:%02dz", y, m, d, h, mi);
		Serial.print("Date: "); Serial.print(buf);
		// convert to unix time
		int tt = (y-1970)*365 + (y-1969)/4; // days since 1970
		if(m<=12) { tt += MON[m]; if((y%4)==0 && m>2) tt++; }
		tt = (tt+d-1)*(60*60*24) + h*3600 + mi*60;
		sonde.si()->time = tt;
		}
		break;
	default:
		Serial.print("(?)");
		break;
	}
}

void DFM::bitsToBytes(uint8_t *bits, uint8_t *bytes, int len)
{
	int i;
	for(i=0; i<len*4; i++) {
		//Serial.print(bits[i]?"1":"0");
	       	bytes[i/8] = (bytes[i/8]<<1) | (bits[i]?1:0);
	}
	bytes[(i-1)/8] &= 0x0F;
}

int DFM::receive() {
	byte data[1000];  // pending data from previous mode may write more than 33 bytes. TODO. 
	for(int i=0; i<2; i++) {
	sx1278.setPayloadLength(33);    // Expect 33 bytes (7+13+13 bytes)

	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	int e = sx1278.receivePacketTimeout(1000, data);
	if(e) { return RX_TIMEOUT; } //if timeout... return 1

	Serial.printf("inverse is %d\b", inverse);
	if(!inverse) { for(int i=0; i<33; i++) { data[i]^=0xFF; } }
	deinterleave(data, 7, hamming_conf);
	deinterleave(data+7, 13, hamming_dat1);
	deinterleave(data+20, 13, hamming_dat2);
  
	int ret0 = hamming(hamming_conf,  7, block_conf);
	int ret1 = hamming(hamming_dat1, 13, block_dat1);
	int ret2 = hamming(hamming_dat2, 13, block_dat2);

	byte byte_conf[4], byte_dat1[7], byte_dat2[7];
	bitsToBytes(block_conf, byte_conf, 7);
	bitsToBytes(block_dat1, byte_dat1, 13);
	bitsToBytes(block_dat2, byte_dat2, 13);

	printRaw("CFG", 7, ret0, byte_conf);
	printRaw("DAT", 13, ret1, byte_dat1);
	printRaw("DAT", 13, ret2, byte_dat2);
	decodeCFG(byte_conf);
	decodeDAT(byte_dat1);
	decodeDAT(byte_dat2);
	}
	return RX_OK;
}

// moved to a single function in Sonde(). This function can be used for additional
// processing here, that takes too long for doing in the RX task loop
int DFM::waitRXcomplete() {
#if 0
	int res=0;
	uint32_t t0 = millis();
	while( rxtask.receiveResult < 0 && millis()-t0 < 2000) { delay(50); }

	if( rxtask.receiveResult<0 || rxtask.receiveResult==RX_TIMEOUT) { 
                res = RX_TIMEOUT;
        } else if ( rxtask.receiveResult ==0) {
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

DFM dfm = DFM();


/* DFM decoder functions */
#include "DFM.h"
#include "SX1278FSK.h"
#include "Sonde.h"

#define DFM_DEBUG 0

#if DFM_DEBUG
#define DFM_DBG(x) x
#else
#define DFM_DBG(x)
#endif

#define DFM_FRAMELEN 33

#define MAXIDAGE 1800


const char *dfmSubtypeLong[] = { "", "DFMxx", "DFM06", "DFM06P", "PS15",
        "DFM09", "DFM09P", "DFM17", "DFM17P"};
const char *dfmSubtypeShort[] = { "", "DFMx", "DFM6", "DF6P", "PS15",
        "DFM9", "DF9P", "DF17", "D17P"};


/*
 * observed DAT patterns for DFM-9:
 * A: 0+1; 2+3; 4+5; 6+7; 8+0	=> keep frame in shadowFrame
 * B: 1+2; 3+4; 5+6; 7+8	=> all good => keep date in shadowDate
 * C: 0+1; 2+3; 4+5; 6+7; 8+15  => all good => keep date in shadowDate
 * D: 0+1; 2+3; 4+5; 6+7; 0+1	=> use shadowDate
 * not seen:5+6; 7+1
 * values:
 * 0:packet counter; 1:utc-msec; 2:lat,vh; 3:lon,dir, 4:alt,vv, 8=utc-date(day-hh-mm)
 */

// single data structure, search restarts after decoder change
static struct st_dfmstat {
	int idcnt0;
	int idcnt1;
	int lastfrid;
	int lastfrcnt;
	uint8_t start[50];
	uint16_t dat[50*2];
	uint8_t cnt[50*2];
	uint16_t good;
	uint32_t datesec;
	uint8_t frame;
	uint16_t msec;
	uint8_t nameregok;
	uint8_t nameregtop;
	uint8_t lastdat;
	uint8_t cycledone; // 0=no; 1=OK, 2=partially/with errors
	float meas[9];
	uint16_t measok;   // Bit-mask showing which meas entries have been received
	uint8_t ptu_chan;  // always the max channel. used as subtype before, but 0xC can be DFM09 (with P) or DFM17 (w/o P)
	uint8_t sensP;     // P channel in data (0xC DMF09 (but not 0xC DFM17), 0xD DFM17P, 0x8 DFM6P (but not PS15) (0 or 1)
} dfmstate;

decoderSetupCfg DFMSetupCfg {
	.bitrate = 2500,
	// continuous mode
	// Enable auto-AFC, auto-AGC, RX Trigger by preamble  ????
	.rx_cfg = 0x1E,
	.sync_cfg = 0x70,
	.sync_len = 2,
	.sync_data = (const uint8_t *)"\xAA\xAA",
	.preamble_cfg = 0xA8,
};

int DFM::setup(float frequency, int type) 
{
	stype = type;
#if DFM_DEBUG
	Serial.printf("Setup sx1278 for DFM sonde (type=%d)\n", stype);
#endif
	if(sx1278.ON()!=0) {
		DFM_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}

	if(DecoderBase::setup(DFMSetupCfg, sonde.config.dfm.agcbw, sonde.config.dfm.rxbw) != 0) {
		return 1;
	}
#if 0
	// This is now all done by the generic setup method in base class
	if(sx1278.setFSK()!=0) {
		DFM_DBG(Serial.println("Setting FSM mode FAILED"));
		return 1;
	}
	if(sx1278.setBitrate(2500)!=0) {
		DFM_DBG(Serial.println("Setting bitrate 2500bit/s FAILED"));
		return 1;
	}
        if(sx1278.setAFCBandwidth(sonde.config.dfm.agcbw)!=0) {
                DFM_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.dfm.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.dfm.rxbw)!=0) {
                DFM_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.dfm.rxbw));
                return 1;
        }
	{
                if(sx1278.setRxConf(0x1E)!=0) {
                        DFM_DBG(Serial.println("Setting RX Config FAILED"));
                        return 1;
                }
                // working with continuous RX
                const char *SYNC="\xAA\xAA";
                if(sx1278.setSyncConf(0x70, 2, (const uint8_t *)SYNC)!=0) {
                        DFM_DBG(Serial.println("Setting SYNC Config FAILED"));
                        return 1;
                }
                if(sx1278.setPreambleDetect(0xA8)!=0) {
                //if(sx1278.setPreambleDetect(0x9F)!=0) {
                        DFM_DBG(Serial.println("Setting PreambleDetect FAILED"));
                        return 1;
                }
	}
#endif
        if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
                DFM_DBG(Serial.println("Setting Packet config FAILED"));
                return 1;
        }
        sx1278.setPayloadLength(0);  // infinite for now...
        Serial.print("DFM: setting RX frequency to ");
        Serial.println(frequency);

	int retval = sx1278.setFrequency(frequency);
        sx1278.clearIRQFlags();

	// Do this only once in setup in continous mode
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);

	memset((void *)&dfmstate, 0, sizeof(dfmstate));
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
			if( res<0 ) ret = -1;
			else if ( ret >= 0 && res > 0 ) ret++;
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

const char* typestr[16]={
  "", "", "", "", "", "",   // 00..05
  "DFM6",                   // 06 => DFM6
  "PS15",		    // 07 => PS15 (untested)
  "", "",
  "DFM9",		    // 0A => DFM9
  "DF17",		    // 0B => DFM17?
  "DF9P",		    // 0C => DFM9P or DFM17 test
  "DF17",		    // 0D => DFM17
  "", ""
};

void DFM::killid() {
	SondeData *sd = &(sonde.si()->d);
	sd->validID = false;
	*(sd->id) = 0;
	*(sd->ser) = 0;
	memset((void *)&dfmstate, 0, sizeof(dfmstate));
}

#define DFMIDTHRESHOLD 2
/* inspired by oe5dxl's finddnmae in sondeudp.c of dxlaprs */
void DFM::finddfname(uint8_t *b)
{
	uint8_t st;
	uint32_t thres;
	uint32_t i;
	uint8_t ix;
	uint16_t d;
	SondeData *sd = &(sonde.si()->d);

	st = b[0];    /* frame start byte */
	ix = b[3];    /* hi/lo part of ser;  (LSB due to our bitsToBytes...) */
	d = (b[1]<<8) + b[2];  /* data byte */
        /* find highest channel number single frame serial,
           (2 frame serial will make a single serial too) */
	if(dfmstate.idcnt0 < DFMIDTHRESHOLD && dfmstate.idcnt1 < DFMIDTHRESHOLD) {
		uint32_t v = (st<<20) | (d<<4) | ix;
		if ( st > (dfmstate.lastfrid>>20) ) {
			dfmstate.lastfrid = v;
			Serial.print(" MAXCH:"); Serial.print(st);
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
							snprintf(sd->id, 10, "D%x ", id);
							memcpy(sd->ser, sd->id+1, 9);
							sd->validID = true;
							//sd->subtype = (st>>4)&0x0F;
							//strncpy(sd->typestr, typestr[ (st>>4)&0x0F ], 5);
							// Subtype is set later, as we need more data to distingish 0xC DFM09P and DFM17
							sd->subtype = 0;
							dfmstate.ptu_chan = (st>>4)&0x0F;
							strcpy(sd->typestr, "DFM");
							return;
						}
						dfmstate.lastfrcnt = 0;
						Serial.print(" NOT NUMERIC SERIAL");
                     			}
                  			//anonym->idtime = osic_time();
       			         } else {
					Serial.print(" MAXCHCNT/SECLVL:");
					Serial.print(dfmstate.lastfrcnt);
					Serial.print("/");
					Serial.print(thres);
				}
			} else {
               			dfmstate.lastfrid = v; /* not stable ser */
               			dfmstate.lastfrcnt = 0UL;
            		}
		}
      } /*find highest channel number single frame serial */

	i = 0;
	while (i<dfmstate.nameregtop && dfmstate.start[i]!=st) i++;
	Serial.printf(" %02x:i=%d,top=%d", st, i, dfmstate.nameregtop);
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
			Serial.print(dfmstate.cnt[2*i+1]);
			Serial.print(",st=");
			Serial.print(st);
			Serial.print(",lastfrid=");
			Serial.print(dfmstate.lastfrid>>20);
			if( (dfmstate.cnt[2*i]>DFMIDTHRESHOLD && dfmstate.cnt[2*i+1]>DFMIDTHRESHOLD) ||
			    (dfmstate.cnt[2*i]>0 && dfmstate.cnt[2*i+1]>0 &&  st == (dfmstate.lastfrid>>20) && (st>>4)>6) ) {
				if(dfmstate.idcnt0 <= 1) {
					dfmstate.idcnt0 = dfmstate.cnt[2*i];
					dfmstate.idcnt1 = dfmstate.cnt[2*i+1];
					dfmstate.nameregok = i;
					// generate id.....
					snprintf(sd->id, 10, "D%d", ((dfmstate.dat[2*i]<<16)|dfmstate.dat[2*i+1])%100000000);
					Serial.print("\nNEW AUTOID:");
					Serial.println(sd->id);
					memcpy(sd->ser, sd->id+1, 9);
					sd->validID = true;
					//sd->subtype = (st>>4)&0x0F;
					//strncpy(sd->typestr, typestr[ (st>>4)&0x0F ], 5);
					// Subtype is set later, as we need more data to distingish 0xC DFM09P and DFM17
					sd->subtype = 0;
					dfmstate.ptu_chan = (st>>4)&0x0F;
					strcpy(sd->typestr, "DFM");
				}
				if(dfmstate.nameregok==i) {
					Serial.print(" ID OK");
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

static float get_Temp() {
	SondeData *si = &(sonde.si()->d);
	if(!si->subtype) { // type not yet known, so don't try to decode
		return NAN;
	}
	float f = dfmstate.meas[0],
		f1 = dfmstate.meas[3],
		f2 = dfmstate.meas[4];
	if(dfmstate.sensP) {
		f = dfmstate.meas[1];
		f1 = dfmstate.meas[5];
		f2 = dfmstate.meas[6];
	}
	Serial.printf("Meas: %f %f %f\n", f, f1, f2);
    // as in autorx / dfm
    float BB0 = 3260.0;       // B/Kelvin, fit -55C..+40C
    float T0 = 25 + 273.15;  // t0=25C
    float R0 = 5.0e3;        // R0=R25=5k
    float Rf = 220e3;        // Rf = 220k, for DFM17: 332k
    if( si->subtype==DFM_17 || si->subtype==DFM_17P ) Rf = 332e3;
    float g = f2/Rf;
    float R = (f-f1) / g; // meas[0,3,4] > 0 ?
    float T = 0;                     // T/Kelvin
    if (f*f1*f2 == 0) R = 0;
    if (R > 0)  T = 1/(1/T0 + 1/BB0 * log(R/R0));
    T =  T - 273.15; // Celsius
    if(T<-100 || T>50) {
	Serial.printf("Temperature invalid: %f\n", T);
	return NAN;
    }
    return T;
}

void DFM::decodeCFG(uint8_t *cfg)
{
	SondeData *si = &(sonde.si()->d);
	// new ID
	finddfname(cfg);
	// get meas
	uint8_t conf_id = (*cfg)>>4;
	if(conf_id<=8) {
		uint32_t val = (cfg[1]<<12) | (cfg[2]<<4) | cfg[3];
		uint8_t exp = cfg[0] & 0xF;
		dfmstate.meas[conf_id] = val / (float)(1<<exp);
		dfmstate.measok |= (1 << conf_id);
		Serial.printf("meas %d is %f (%d,%d)\n", conf_id, dfmstate.meas[conf_id], val, exp);
	}

	// get type (if we have an ID, but no type yet, and (needed only for 0xC), meas[6])
	if( si->validID && si->subtype==0 && (dfmstate.measok & (1<<6)) ) {
		switch(dfmstate.ptu_chan) {
		case 0x6: si->subtype = DFM_06; break;
		case 0x7: case 0x8:
			si->subtype = DFM_06P; dfmstate.sensP = 1; break;  // (TODO: OR PS15)
		case 0xA: si->subtype = DFM_09; break;
		case 0xB: si->subtype = DFM_17; break;
		case 0xC: // DFM-09P or DFM-17
			if(dfmstate.meas[6]<220e3) { si->subtype = DFM_09P; dfmstate.sensP = 1; }
			else si->subtype = DFM_17;
			break;
		case 0xD: si->subtype = DFM_17P; dfmstate.sensP = 1; break;
		default: si->subtype = DFM_UNK;
		}
		if( si->subtype == DFM_UNK ) {
			snprintf(si->typestr, 5, "DFx%x", dfmstate.ptu_chan);
			si->subtype |= (dfmstate.ptu_chan<<4);
		} else {
			strcpy(si->typestr, dfmSubtypeShort[si->subtype]);
		}
	}

	// get batt
	if(si->validID && dfmstate.ptu_chan>=0x0A && si->subtype>0 ) {
		// otherwise don't try, as we might not have the right type yet...
		int cid = (dfmstate.sensP) ? 0x7 : 0x5;
		if(conf_id == cid) {
			uint16_t val = cfg[1]<<8 | cfg[2];
			si->batteryVoltage = val / 1000.0;
			Serial.printf("battery: %f\n", si->batteryVoltage);
		}
	}
}

#if 0
// not used any more
static int bitCount(int x) {
    int m4 = 0x1 | (0x1<<8) | (0x1<<16) | (0x1<<24);
    int m1 = 0xFF; 
    int s4 = (x&m4) + ((x>>1)&m4) + ((x>>2)&m4) + ((x>>3)&m4) + ((x>>4)&m4) + ((x>>5)&m4) + ((x>>6)&m4) + ((x>>7)&m4);
    int s1 = (s4&m1) + ((s4>>8)&m1) + ((s4>>16)&m1) + ((s4>>24)&m1);
    return s1;
}
#endif

uint16_t MON[]={0,0,31,59,90,120,151,181,212,243,273,304,334};

void DFM::decodeDAT(uint8_t *dat)
{
	// TODO: Here we need to work on a shadow copy of SondeData in order to prevent concurrent changes while using data in main loop
	SondeData *si = &(sonde.si()->d);
	Serial.print(" DAT["); Serial.print(dat[6]); Serial.print("]: ");

	// We handle this case already here, because we need to update dfmstate.datesec before the cycle complete handling 
	if( dat[6]==8 ) {
		int y = (dat[0]<<4) + (dat[1]>>4);
		int m = dat[1]&0x0F;
		int d = dat[2]>>3;
		int h = ((dat[2]&0x07)<<2) + (dat[3]>>6);
		int mi = (dat[3]&0x3F);
		Serial.printf("Date: %04d-%02d-%02d %02d:%02dz ", y, m, d, h, mi);
		si->sats = dat[4];
		si->validPos |= 0x40;
		// convert to unix time
		int tt = (y-1970)*365 + (y-1969)/4; // days since 1970
		if(m<=12) { tt += MON[m]; if((y%4)==0 && m>2) tt++; }
		tt = (tt+d-1)*(60*60*24) + h*3600 + mi*60;
		// If we get a time stamp much different to the previously received one, kill the ID.
		// most likely, we have a new sonde now, so wait for the new ID.
		if(tt-dfmstate.datesec > MAXIDAGE) killid();
		dfmstate.datesec = tt;
		dfmstate.good |= 0x100;
	}
	else if( dat[6]>8 ) return; // we ignore those...

	/* Here we update data that should be updated only after receiving a "complete" frame (mainly for consistent SondeHub exports)
	 * We do this (a) when there is a DAT8 block and (b) when there is wrap from DAT7 to DAT0, for these fields:
	 * => frame
	 * => vframe (only used for SondeHub as virtual frame number)
	 * => time (calculated with using date and msec)
	 * [assuming that if there is no DAT8, then the date value did not change.]
	 */

	if( dat[6]==8 || dat[6] < dfmstate.lastdat) { // After DAT8, or after a "warp around"
		if( dfmstate.good&1 ) si->frame = dfmstate.frame;
		if( (dfmstate.good&0x102)==0x102 ) {
			si->time = dfmstate.datesec + dfmstate.msec/1000;
	 		// Lets be consistent with autorx: the timestamp uses the msec value truncated to seconds,
    			// whereas the virtual frame number for DFM uses the msec value rounded to full seconds.
			// Actually, tt is real UTC, and the transformation to GPS seconds lacks adjusting for leap seconds
			si->vframe = dfmstate.datesec + (dfmstate.msec+500)/1000 - 315964800;
		}
		// All fields updated? 1=OK, 2=with errors
		Serial.printf("Cycle done: good is %x\n", dfmstate.good);
		si->temperature = get_Temp();
		Serial.printf("Temp: %f\n", si->temperature);
		dfmstate.cycledone = ((dfmstate.good&0x11F)==0x11F) ? 1 : 2; 
		dfmstate.good = 0;
		dfmstate.lastdat = 0;
	} else {
		dfmstate.lastdat = dat[6];
	}
	dfmstate.good |= (1<<dat[6]);
	switch(dat[6]) {
	case 0:
		Serial.print("Packet counter: "); Serial.print(dat[3]);	
		dfmstate.frame = dat[3];
		break;
	case 1:
		{
		int val = (((uint16_t)dat[4])<<8) + (uint16_t)dat[5];
		Serial.print("UTC-msec: "); Serial.print(val);
		dfmstate.msec = val; 
		//uint32_t tmp = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + ((uint32_t)dat[3]);
		//si->sats = bitCount(tmp); 
		}
		break;
	case 2:
		{
		float lat, vh;
		lat = (int32_t)(((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + ((uint32_t)dat[3]));
		vh = ((uint16_t)dat[4]<<8) + dat[5];
		Serial.print("GPS-lat: "); Serial.print(lat*0.0000001);
		Serial.print(", hor-V: "); Serial.print(vh*0.01);
		lat = lat*0.0000001;
		if( lat!=0 && si->lat!=0 && abs(lat-si->lat)>.25 ) killid();
		si->lat = lat;
		si->hs = vh*0.01;
		if(lat!=0 || vh!=0) si->validPos |= 0x11;  else si->validPos &= ~0x11;
		}
		break;
	case 3:
		{
		float lon, dir;
		lon = (int32_t)(((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + (uint32_t)dat[3]);
		dir = ((uint16_t)dat[4]<<8) + dat[5];
		lon = lon*0.0000001;
		if( lon!=0 && si->lon!=0 && abs(lon-si->lon)>.25 ) killid();
		si->lon = lon;
		si->dir = dir*0.01;
		Serial.print("GPS-lon: "); Serial.print(si->lon);
		Serial.print(", dir: "); Serial.print(si->dir);
		if(lon != 0 || dir != 0) si->validPos |= 0x22; else si->validPos &= ~0x22;
		}
		break;
	case 4:
		{
		float alt, vv;
		alt = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + dat[3];
		vv = (int16_t)( ((int16_t)dat[4]<<8) | dat[5] );
		Serial.print("GPS-height: "); Serial.print(alt*0.01);
		Serial.print(", vv: "); Serial.print(vv*0.01);
		si->alt = alt*0.01;
		si->vs = vv*0.01;
		if(alt!=0 || vv != 0) si->validPos |= 0x0C; else si->validPos &= ~0x0C;
		}
		break;
	case 8:
		// handled above
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

static int haveNewFrame = 0;

int DFM::processDFMdata(uint8_t dt) {
	static uint8_t data[1024];
	static uint32_t rxdata = 0;
	static uint8_t rxbitc = 0;
	static uint8_t rxbyte = 0;
	static uint8_t rxsearching = 1;
	static uint8_t rxp;
	static int rssi=0, fei=0, afc=0;
	static uint8_t invers = 0;
	
       	for(int i=0; i<8; i++) {
                uint8_t d = (dt&0x80)?1:0;
                dt <<= 1;
                rxdata = (rxdata<<1) | d;
                if( (rxbitc&1)==0 ) {
                        // "bit1"
                        rxbyte = (rxbyte<<1) | d;
                } else {
                        // "bit2" ==> 01 or 10 => 1, otherweise => 0
                        // not here: (10=>1, 01=>0)!!! rxbyte = rxbyte ^ d;
                }
                //
                if(rxsearching) {
                        if( rxdata == 0x6566A5AA || rxdata == 0x9A995A55 ) {
                                rxsearching = false;
                                rxbitc = 0;
                                rxp = 0;
				rxbyte = 0;
                                rssi=sx1278.getRSSI();
                                fei=sx1278.getFEI();
                                afc=sx1278.getAFC();
                                sonde.si()->rssi = rssi;
                                sonde.si()->afc = afc;
				invers = (rxdata == 0x6566A5AA)?1:0;
                        }
                } else {
                        rxbitc = (rxbitc+1)%16; // 16;
                        if(rxbitc == 0) { // got 8 data bit
				if(invers) rxbyte=~rxbyte;
                                data[rxp++] = rxbyte&0xff; // (rxbyte>>1)&0xff;
                                if(rxp>=DFM_FRAMELEN) {
                                        rxsearching = true;
					//Serial.println("Got a DFM frame!");
                                	Serial.print("[RSSI="); Serial.print(rssi);
                                	Serial.print(" FEI="); Serial.print(fei);
                                	Serial.print(" AFC="); Serial.print(afc); Serial.print("] ");
                                        decodeFrameDFM(data);
					haveNewFrame = 1;
                                }
                        }
                }
        }
	return 0;
}

int DFM::receive() {
	int rxframes = 5;  // UP TO 5 frames, stop at type 8 frame

	// tentative continuous RX version...
	unsigned long t0 = millis();
	dfmstate.cycledone = 0;
	while( ( millis() - t0 ) < 1300 ) {
		uint8_t value = sx1278.readRegister(REG_IRQ_FLAGS2);	
                if ( bitRead(value, 7) ) {
                        Serial.println("FIFO full");
                }
                if ( bitRead(value, 4) ) {
                        Serial.println("FIFO overflow");
			// new: (maybe clear only overflow??) TODO
                        sx1278.clearIRQFlags();
                }
                if ( bitRead(value, 2) == 1 ) {
                        Serial.println("FIFO: payload ready()");
			// does not make much sence? (from m10): TODO
                        // ???????  sx1278.clearIRQFlags();
                }
		if(bitRead(value, 6) == 0) { // while FIFO not empty
                        byte data = sx1278.readRegister(REG_FIFO);
                        processDFMdata(data);
                        value = sx1278.readRegister(REG_IRQ_FLAGS2);
                } else {
                        if(haveNewFrame) {
                                //Serial.printf("DFM::receive(): new frame complete after %ldms\n", millis()-t0);
                                haveNewFrame = 0;
				rxframes--;
				// OK: All DAT frames (0/1/2/3/4/8) have been received in the last cycle
				if(dfmstate.cycledone==1) return RX_OK;
				if(dfmstate.cycledone>1 || rxframes==0) return RX_ERROR;
                        }
                        delay(2);
                }
	}
	return rxframes == 5 ? RX_TIMEOUT : RX_ERROR;
}

int DFM::decodeFrameDFM(uint8_t *data) {
	deinterleave(data, 7, hamming_conf);
	deinterleave(data+7, 13, hamming_dat1);
	deinterleave(data+20, 13, hamming_dat2);
  
	int ret0 = hamming(hamming_conf,  7, block_conf);
	int ret1 = hamming(hamming_dat1, 13, block_dat1);
	int ret2 = hamming(hamming_dat2, 13, block_dat2);
	//Serial.printf("Hamming returns %d %d %d -- %d\n", ret0, ret1, ret2, ret0|ret1|ret2);

	byte byte_conf[4], byte_dat1[7], byte_dat2[7];
	bitsToBytes(block_conf, byte_conf, 7);
	bitsToBytes(block_dat1, byte_dat1, 13);
	bitsToBytes(block_dat2, byte_dat2, 13);

	printRaw("CFG", 7, ret0, byte_conf);
	printRaw("DAT", 13, ret1, byte_dat1);
	printRaw("DAT", 13, ret2, byte_dat2);
	if (ret0>=0) decodeCFG(byte_conf);
	if (ret1>=0 && ret1<=4) decodeDAT(byte_dat1);
	if (ret2>=0 && ret2<=4) decodeDAT(byte_dat2);
	Serial.println("");
	// Consistent with autorx: If more than 4 corrected bit errors in DAT block, assume it is possibly corrupt and
	// don't treat it as a correct frame (ttgo display shows data anyway, but it is not sent to external sites)
	if(ret1>4 || ret2>4) return RX_ERROR;
	return (ret0|ret1|ret2)>=0 ? RX_OK : RX_ERROR;
}

// moved to a single function in Sonde(). This function can be used for additional
// processing here, that takes too long for doing in the RX task loop
int DFM::waitRXcomplete() {
	return 0;
}

DFM dfm = DFM();

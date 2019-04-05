
/* DFM decoder functions */
#include "DFM.h"
#include "SX1278FSK.h"

#define DFM_DEBUG 1

#if DFM_DEBUG
#define DFM_DBG(x) x
#else
#define DFM_DBG(x)
#endif

int DFM::setup(int inverse) 
{
#if DFM_DEBUG
	Serial.println("Setup sx1278 for DFM sonde");
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

	if(sx1278.setAFCBandwidth(25000)!=0) {
		DFM_DBG(Serial.println("Setting AFC bandwidth 25 kHz FAILED"));
		return 1;
	}
	if(sx1278.setRxBandwidth(12000)!=0) {
		DFM_DBG(Serial.println("Setting RX bandwidth 12kHz FAILED"));
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
	DFM_DBG(Serial.println("Setting SX1278 config for DFM finished\n"); Serial.println());
	return 0;
}

int DFM::setFrequency(float frequency) {
        Serial.print("DFM: setting RX frequency to ");
        Serial.println(frequency);
	return sx1278.setFrequency(frequency);
}

#define bit(value,bitpos) ((value>>(7-bitpos))&0x01)
// Input: str: packed data, MSB first
void DFM::deinterleave(uint8_t *str, int L, uint8_t *block) {
	int i, j;
	for (j = 0; j < B; j++) {  // L = 7 (CFG), 13 (DAT1, DAT2)
		for (i = 0; i < L; i++) {
			block[B*i+j] = bit( str[(L*j+i)/8], (L*j+i)&7 )?0:1;
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

void DFM::printRaw(char *label, int len, int ret, uint8_t *data)
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

int DFM::decodeCFG(uint8_t *cfg)
{
	static int lowid, highid, idgood=0, type=0;
	if((cfg[0]>>4)==0x06 && type==0) {   // DFM-6 ID
		lowid = ((cfg[0]&0x0F)<<20) | (cfg[1]<<12) | (cfg[2]<<4) | (cfg[3]&0x0f);
		Serial.print("DFM-06 ID: "); Serial.print(lowid, HEX);
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
		}
	}
}

int DFM::decodeDAT(uint8_t *dat)
{
	Serial.print(" DAT["); Serial.print(dat[6]); Serial.print("]: ");
	switch(dat[6]) {
	case 0:
		Serial.print("Packet counter: "); Serial.print(dat[3]);
		break;
	case 1:
		{
		int val = (((uint16_t)dat[4])<<8) + (uint16_t)dat[5];
		Serial.print("UTC-msec: "); Serial.print(val);
		}
		break;
	case 2:
		{
		float lat, vh;
		lat = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + ((uint32_t)dat[3]);
		vh = (dat[4]<<8) + dat[5];
		Serial.print("GPS-lat: "); Serial.print(lat*0.0000001);
		Serial.print(", hor-V: "); Serial.print(vh*0.01);
		}
		break;
	case 3:
		{
		float lon, dir;
		lon = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + (uint32_t)dat[3];
		dir = ((uint16_t)dat[4]<<8) + dat[5];
		Serial.print("GPS-lon: "); Serial.print(lon*0.0000001);
		Serial.print(", dir: "); Serial.print(dir*0.01);
		}
		break;
	case 4:
		{
		float hei, vv;
		hei = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + dat[3];
		vv = (int16_t)( (dat[4]<<8) | dat[5] );
		Serial.print("GPS-height: "); Serial.print(hei*0.01);
		Serial.print(", vv: "); Serial.print(vv*0.01);
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
		}
		break;
	default:
		Serial.print("(?)");
		break;
	}
}

int DFM::bitsToBytes(uint8_t *bits, uint8_t *bytes, int len)
{
	int i;
	for(i=0; i<len*4; i++) {
		//Serial.print(bits[i]?"1":"0");
	       	bytes[i/8] = (bytes[i/8]<<1) | (bits[i]?1:0);
	}
	bytes[(i-1)/8] &= 0x0F;
}

int DFM::receiveFrame() {
	byte data[33];
	sx1278.setPayloadLength(33);    // Expect 33 bytes (7+13+13 bytes

	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	int e = sx1278.receivePacketTimeout(1000, data);
	if(e) { return 1; } //if timeout... return 1

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

DFM dfm = DFM();

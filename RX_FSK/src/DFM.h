/*
 * DFM.h
 * Functions for decoding DFM sondes with SX127x chips
 * Copyright (C) 2019 Hansi Reiser, dl9rdz
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef DFM_h
#define DFM_h

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif
#include "DecoderBase.h"

#define DFM_NORMAL 0
#define DFM_INVERSE 1

enum DFMSubtype { DFM_UNDEF, DFM_UNK, DFM_06, DFM_06P, DFM_PS15, DFM_09, DFM_09P, DFM_17, DFM_17P };

extern const char *dfmSubtypeLong[];
extern const char *dfmSubtypeShort[];

/* Main class */
class DFM : public DecoderBase
{
private:
	int stype;
	char *stypename=NULL;

	void deinterleave(uint8_t *str, int L, uint8_t *block);
	uint32_t bits2val(const uint8_t *bits, int len);
	int check(uint8_t code[8]);
	int hamming(uint8_t *ham, int L, uint8_t *sym);
	void printRaw(const char *prefix, int len, int ret, const uint8_t* data);
	void finddfname(uint8_t *cfg);
	void decodeCFG(uint8_t *cfg);
	void decodeDAT(uint8_t *dat);
	void bitsToBytes(uint8_t *bits, uint8_t *bytes, int len);
	int processDFMdata(uint8_t dt);
        int decodeFrameDFM(uint8_t *data);
	void killid();


#define B 8
#define S 4
	uint8_t hamming_conf[ 7*B];  //  7*8=56
	uint8_t hamming_dat1[13*B];  // 13*8=104
	uint8_t hamming_dat2[13*B];

	uint8_t block_conf[ 7*S];  //  7*4=28
	uint8_t block_dat1[13*S];  // 13*4=52
	uint8_t block_dat2[13*S];

	uint8_t H[4][8] =  // extended Hamming(8,4) particy check matrix
             {{ 0, 1, 1, 1, 1, 0, 0, 0},
              { 1, 0, 1, 1, 0, 1, 0, 0},
              { 1, 1, 0, 1, 0, 0, 1, 0},
              { 1, 1, 1, 0, 0, 0, 0, 1}};
	uint8_t He[8] = { 0x7, 0xB, 0xD, 0xE, 0x8, 0x4, 0x2, 0x1};  // Spalten von H:
                                                            // 1-bit-error-Syndrome

public:
	DFM();
	// main decoder API
	int setup(float frequency, int type);
	int receive();
	int waitRXcomplete();

	int use_ecc = 1;
};

// DFM state structure for collision detection
struct st_dfmstat {
	int idcnt0;
	int idcnt1;
	int lastfrid;
	int lastfrcnt;
	uint8_t invers;
	uint8_t start[50];
	uint16_t dat[50*2];
	uint8_t cnt[50*2];
	uint16_t good;
	uint32_t datesec;
	uint8_t frame;
	uint8_t posmode;
	uint16_t msec;
	uint8_t nameregok;
	uint8_t nameregtop;
	uint8_t lastdat;
	uint8_t cycledone; // 0=no; 1=OK, 2=partially/with errors
	float meas[9];
	uint16_t measok;   // Bit-mask showing which meas entries have been received
	uint8_t ptu_chan;  // always the max channel. used as subtype before, but 0xC can be DFM09 (with P) or DFM17 (w/o P)
	uint8_t sensP;     // P channel in data (0xC DMF09 (but not 0xC DFM17), 0xD DFM17P, 0x8 DFM6P (but not PS15) (0 or 1)
};

extern DFM dfm;

// Expose DFM state for collision detection
extern struct st_dfmstat dfmstate;

#endif

/*
 * M10M20.h
 * Functions for decoding Meteomodem M10M20 sondes with SX127x chips
 * Copyright (C) 2019 Hansi Reiser, dl9rdz
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef M10M20_h
#define M10M20_h

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif
#include "DecoderBase.h"

/* Main class */
class M10M20 : public DecoderBase
{
private:
	void printRaw(uint8_t *data, int len);
	void processM10data(uint8_t data);
        int decodeframeM10(uint8_t *data);
        int decodeframeM20(uint8_t *data);
#if 0
	void stobyte92(uint8_t byte);
	void dogps(const uint8_t *sf, int sf_len,
                struct CONTEXTR9 * cont, uint32_t * timems,
                uint32_t * gpstime);
	uint32_t bits2val(const uint8_t *bits, int len);
	int bitsToBytes(uint8_t *bits, uint8_t *bytes, int len);
	int decode92(byte *data, int maxlen);

	uint8_t hamming_conf[ 7*8];  //  7*8=56
	uint8_t hamming_dat1[13*8];  // 13*8=104
	uint8_t hamming_dat2[13*8];

	uint8_t block_conf[ 7*4];  //  7*4=28
	uint8_t block_dat1[13*4];  // 13*4=52
	uint8_t block_dat2[13*4];

	uint8_t H[4][8] =  // extended Hamming(8,4) particy check matrix
             {{ 0, 1, 1, 1, 1, 0, 0, 0},
              { 1, 0, 1, 1, 0, 1, 0, 0},
              { 1, 1, 0, 1, 0, 0, 1, 0},
              { 1, 1, 1, 0, 0, 0, 0, 1}};
	uint8_t He[8] = { 0x7, 0xB, 0xD, 0xE, 0x8, 0x4, 0x2, 0x1};  // Spalten von H:
                                                            // 1-bit-error-Syndrome
	boolean initialized = false;
#endif

public:
	M10M20();
	int setup(float frequency, int type = 0);
	int receive();
	int waitRXcomplete();

	//int use_ecc = 1;
};

extern M10M20 m10m20;

#endif

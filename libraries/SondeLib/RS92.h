/*
 * RS92.h
 * Functions for decoding RS92 sondes with SX127x chips
 * Copyright (C) 2019 Hansi Reiser, dl9rdz
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef RS92_h
#define RS92_h

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif


struct CONTEXTR9 {
   char calibdata[512];
   uint32_t calibok;
   char mesok;
   char posok;
   char framesent;
   double lat;
   double lon;
   double heig;
   double speed;
   double dir;
   double climb;
   double lastlat;
   double laslong;
   double lastalt;
   double lastspeed;
   double lastdir;
   double lastclb;
   float hrmsc;
   float vrmsc;
   double hp;
   double hyg;
   double temp;
   double ozontemp;
   double ozon;
   uint32_t goodsats;
   uint32_t timems;
   uint32_t framenum;
};


/* Main class */
class RS92
{
private:
	void process8N1data(uint8_t data);
	void stobyte92(uint8_t byte);
        void decodeframe92(uint8_t *data);
#if 0
	void dogps(const uint8_t *sf, int sf_len,
                struct CONTEXTR9 * cont, uint32_t * timems,
                uint32_t * gpstime);
#endif
	uint32_t bits2val(const uint8_t *bits, int len);
	void printRaw(uint8_t *data, int len);
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

public:
	RS92();
	int setup(float frequency);
	int receive();
	int waitRXcomplete();

	int use_ecc = 1;
};

extern RS92 rs92;

#endif

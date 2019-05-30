/*
 * RS41.h
 * Functions for decoding RS41 sondes with SX127x chips
 * Copyright (C) 2019 Hansi Reiser, dl9rdz
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef RS41_h
#define RS41_h

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif

/* Main class */
class RS41
{
private:
	uint32_t bits2val(const uint8_t *bits, int len);
	void printRaw(uint8_t *data, int len);
	void bitsToBytes(uint8_t *bits, uint8_t *bytes, int len);
	int decode41(byte *data, int maxlen);

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
	boolean initialized = false;

public:
	RS41();
	// New interface:
	// setup() is called when channel is activated (sets mode and frequency and activates receiver)
	int setup(float frequency);
	// processRXbyte is called by background task for each received byte
	// should be fast enough to not cause sx127x fifo buffer overflow
    //    void processRXbyte(uint8_t data);
	// is called approx. 1x per second, may do some post-processing of received data
	// and update information in sonde data structure
	// returns infomration about sucess/error (for timers and for quality bar in display)
	int receive();
	int waitRXcomplete();
	//int receiveFrame();

	int use_ecc = 1;
};

extern RS41 rs41;

#endif

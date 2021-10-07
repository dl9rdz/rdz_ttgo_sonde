/*
 * MP3H.h
 * Functions for decoding MP3H radiosonde
 * Copyright (C) 2021 Hansi Reiser, dl9rdz
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef MP3H_h
#define MP3H_h

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif

/* Main class */
class MP3H
{
private:
	void printRaw(uint8_t *data, int len);
	void processMP3Hdata(uint8_t data);
        int decodeframeMP3H(uint8_t *data);
public:
	MP3H();
	int setup(float frequency);
	int receive();
	int waitRXcomplete();
};

extern MP3H mp3h;

#endif

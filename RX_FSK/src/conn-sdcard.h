/*
 * conn-sdcard.h
 * Data exporter to SD card
 * Copyright (c) 2023 Hansi Reiser, dl9rdz
 * 
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef conn_sdcard_h
#define conn_sdcard_h

#include "conn.h"

//#include "FS.h"
#include "SD.h"

class ConnSDCard : public Conn
{
public:
        /* Called once on startup */
        void init();

        /* Called whenever the network becomes available */
        void netsetup();


        /* Called approx 1x / second (maybe only if good data is available) */
        virtual void updateSonde( SondeInfo *si );

        /* Called approx 1x / second* */
        virtual void updateStation( PosInfo *pi );


private:
	File file;
	uint8_t initok = 0;
	uint16_t wcount = 0;

};


#endif

#include "../features.h"

#if FEATURE_SDCARD

#include "conn-sdcard.h"

// TODO: Move into config
#define CS 13
#define SYNC_INTERVAL 10

void ConnSDCard::init() {
	/* Initialize SD card */
	initok = SD.begin(CS);
	Serial.printf("SD card init: %s\n", initok?"OK":"Failed");
}

void ConnSDCard::netsetup() {
	/* empty function, we don't use any network here */
}

void ConnSDCard::updateSonde( SondeInfo *si ) {
	if (!initok) return;
	if (!file) {
		file = SD.open("/data.csv", FILE_APPEND);
	}
	if (!file) {
		Serial.println("Error opening file");
		return;
	}
	SondeData *sd = &si->d;
	file.printf("%d,%s,%s,%d,"
		"%f,%f,%f,%f,%f,%f,%d,%d,"
		"%d,%d,%d,%d\n",
		sd->validID, sd->ser, sd->typestr, sd->subtype,
		sd->lat, sd->lon, sd->alt, sd->vs, sd->hs, sd->dir, sd->sats, sd->validPos,
		sd->time, sd->frame, sd->vframe, sd->validTime);
	wcount++;
	if(wcount >= SYNC_INTERVAL) {
		file.flush();
		wcount = 0;
	}
}


void ConnSDCard::updateStation( PosInfo *pi ) {
}


ConnSDCard connSDCard;

#endif

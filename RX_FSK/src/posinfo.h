
#ifndef _posinfo_h
#define _posinfo_h

#include <inttypes.h>
#include "Sonde.h"
#include <SPIFFS.h>

enum { SH_LOC_OFF, SH_LOC_FIXED, SH_LOC_CHASE, SH_LOC_AUTO };


// Handling of station position (GPS, fixed location)

struct StationPos {
        double lat;
        double lon;
        int alt;
        float speed;
        int16_t course;
        int16_t accuracy;
        int16_t hdop;
        int8_t sat;
        int8_t valid;
	int8_t chase;
};

extern struct StationPos gpsPos, posInfo;


// Initialize GPS chip
void initGPS();

// Update position from app (if not local GPS chip)
void parseGpsJson(char *data, int len); 

// Update position from static config
void fixedToPosInfo();

#endif


#ifndef _posinfo_h
#define _posinfo_h

#include <inttypes.h>
#include "Sonde.h"

#include "conn.h"

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
        char time[20];
};

extern struct StationPos gpsPos, posInfo;

// Initialize GPS chip
void initGPS();

// Update position from app (if not local GPS chip)
void parseGpsJson(char *data, int len); 

// Update position from static config
void fixedToPosInfo();


// We implement the connector interface for the GPS position just to provide the status in a uniform way....
class ConnGPS : public Conn
{
public:
        /* Called once on startup */
        void init();

        /* Called whenever the network becomes available */
        void netsetup();

        /* Close connections */
        void netshutdown();

        /* Called approx 1x / second (maybe only if good data is available) */
        void updateSonde( SondeInfo *si );

        /* Called approx 1x / second* */
        void updateStation( PosInfo *pi );

        String getStatus();

        String getName();
};

extern ConnGPS connGPS;


#endif

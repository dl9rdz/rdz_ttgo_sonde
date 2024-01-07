#include "../features.h"
#if FEATURE_APRS

#ifndef conn_aprs_h
#define conn_aprs_h

#include "conn.h"
#include "aprs.h"

// Times in ms, i.e. station: 10 minutes, mobile: 20 seconds
#define APRS_STATION_UPDATE_TIME (10*60*1000)
#define APRS_MOBILE_STATION_UPDATE_TIME (20*1000)

static unsigned long time_last_aprs_update = -APRS_STATION_UPDATE_TIME;


class ConnAPRS : public Conn
{
public:
        /* Called once on startup */
        void init();

        /* Called whenever the network becomes available */
        void netsetup();

        /* Called approx 1x / second (maybe only if good data is available) */
        void updateSonde( SondeInfo *si );

        /* Called approx 1x / second* */
        void updateStation( PosInfo *pi );

private:
	void aprs_station_update();
};

extern ConnAPRS connAPRS;
#endif

#endif

/* 
 * conn.h
 * Interface for external data exporters
 * Copyright (c) 2023 Hansi Reiser, dl9rdz
 */

#ifndef conn_h
#define conn_h

#include "Sonde.h"


// to be moved elsewhere
struct PosInfo {
public:
	float lat;
	float lon;
};


/* Interface for all data exporters */
class Conn
{
public: 
	/* Called once on startup */
	virtual void init();

	/* Called whenever the network becomes available */
	virtual void netsetup();

	/* Called approx 1x / second (maybe only if good data is available) */
	virtual void updateSonde( SondeInfo *si );

	/* Called approx 1x / second* */
	virtual void updateStation( PosInfo *pi );

};
#endif

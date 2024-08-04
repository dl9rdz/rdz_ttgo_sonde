#include "../features.h"
#if FEATURE_SONDEHUB

#ifndef conn_sondehub_h
#define conn_sondehub_h

#include "conn.h"


class ConnSondehub : public Conn
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

        String getStatus();

private:
	void sondehub_station_update();
	void sondehub_reply_handler();
	void sondehub_send_fimport();
	void sondehub_send_data(SondeInfo * s);
	void sondehub_finish_data();
	void sondehub_send_header(SondeInfo * s, struct tm * now);
	void sondehub_send_next(SondeInfo * s, char *chunk, int chunklen, int first);
	void sondehub_send_last();
};


extern ConnSondehub connSondehub;
#endif

#endif

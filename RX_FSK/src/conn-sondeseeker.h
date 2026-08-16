#ifndef _SONDESEEKER_H
#define _SONDESEEKER_H
 
#include "Sonde.h"
#include "conn.h"
 
class ConnSondeseeker : public Conn {
public:
    void init();
    void netsetup();
    void netshutdown();
    void updateSonde( SondeInfo *si );
    void updateStation( PosInfo *pi );
    void updateQRG( int nextIndex );
    String getStatus();
    String getName();
};
 
extern ConnSondeseeker connSondeseeker;

#endif

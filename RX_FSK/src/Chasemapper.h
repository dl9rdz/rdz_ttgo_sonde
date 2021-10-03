#ifndef _CHASEMAPPER_H
#define _CHASEMAPPER_H

#include "Sonde.h"
//#include <WiFi.h>	
#include <WiFiUdp.h>
#include <time.h>
class Chasemapper {
public:
	 static int send(WiFiUDP &udb, SondeInfo *si);
};

#endif

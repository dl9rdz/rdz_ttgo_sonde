#ifndef _JSON_H
#define _JSON_H

#include "Sonde.h"

const char *getType(SondeInfo *si);
int sonde2json(char *buf, int maxlen, SondeInfo *si, bool rssi_as_dbm=false);

#endif

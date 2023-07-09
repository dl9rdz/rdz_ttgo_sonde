
#ifndef _aprs_h
#define _aprs_h

#include "Sonde.h"


#define APRS_MAXLEN 201
void aprs_gencrctab(void);
int aprsstr_mon2raw(const char *mon, char raw[], int raw_len);
int aprsstr_mon2kiss(const char *mon, char raw[], int raw_len);
char *aprs_send_beacon(const char *call, float lat, float lon, const char *sym, const char *comment);
char *aprs_senddata(SondeInfo *s, const char *usercall, const char *objcall, const char *sym);


#endif

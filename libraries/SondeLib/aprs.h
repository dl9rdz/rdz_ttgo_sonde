
#ifndef _aprs_h
#define _aprs_h

#define MAXLEN 201
void aprs_gencrctab(void);
int aprsstr_mon2raw(const char *mon, char raw[], int raw_len);
char * aprs_senddata(float lat, float lon, float hei, float speed, float dir, float climb, const char *type, const char *objname, const char *usercall, const char *sym);


#endif


#ifndef _aprs_h
#define _aprs_h

enum IDTYPE { ID_DFMDXL, ID_DFMREAL, ID_DFMAUTO };

typedef struct st_feedinfo {
	bool active;
	int type;	// 0:UDP(axudp), 1:TCP(aprs.fi)
	char *host;
	int port;
	int rate;
	int idformat;	// 0: dxl  1: real  2: auto
}
#define MAXLEN 201
void aprs_gencrctab(void);
int aprsstr_mon2raw(const char *mon, char raw[], int raw_len);
char * aprs_senddata(float lat, float lon, float hei, float speed, float dir, float climb, const char *type, const char *objname, const char *usercall, const char *sym);


#endif

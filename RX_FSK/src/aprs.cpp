/* Copyright (C) Hansi Reiser, dl9rdz
 *
 * partially based on dxlAPRS toolchain
 *
 * Copyright (C) Christian Rabler <oe5dxl@oevsv.at>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <stdio.h>
#include <WString.h>
#include <stdlib.h>
//#include <arpa/inet.h>
//#include <sys/socket.h>
#include <math.h>
#include <unistd.h>
#include <inttypes.h>
#include "aprs.h"
#include "RS41.h"

extern const char *version_name;
extern const char *version_id;
#if 0
int openudp(const char *ip, int port, struct sockaddr_in *si) {
	int fd;
	if((fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) return -1;
	memset((char *)&si, 0, sizeof(si));
	si->sin_family = AF_INET;
	si->sin_port = htons(port);
	if(inet_aton(ip, &(si->sin_addr))==0) {
		return -1;
	}
	return fd;
}

int sendudp(int fd, struct sockaddr_in *si, char *frame, int framelen)
{
	if(sendto(fd, frame, framelen, 0, (struct sockaddr *)si, sizeof(struct sockaddr_in))==-1)  {
		return -1;
	}	
	return 0;
}
#endif



void aprsstr_append(char *b, const char *data)
{
	int blen=strlen(b);
	int len=strlen(data);
	if(blen+len>APRS_MAXLEN) len=APRS_MAXLEN-blen;
	strncat(b, data, len);
}

uint32_t realcard(float x) {
	if(x<0) return 0;
	else return (uint32_t)x;
}


/* CRC for AXUDP frames */

#define APRSCRC_POLY 0x8408 
static uint8_t CRCL[256];
static uint8_t CRCH[256];

void aprs_gencrctab(void)
{
   uint32_t c;
   uint32_t crc;
   uint32_t i;
   for (c = 0UL; c<=255UL; c++) {
      crc = 255UL-c;
      for (i = 0UL; i<=7UL; i++) {
         if ((crc&1)) crc = (uint32_t)((uint32_t)(crc>>1)^APRSCRC_POLY);
         else crc = crc>>1;
      } /* end for */
      CRCL[c] = (uint8_t)crc;
      CRCH[c] = (uint8_t)(255UL-(crc>>8));
   } /* end for */
} /* end Gencrctab() */

static void aprsstr_appcrc(char frame[], uint32_t frame_len, int32_t size)
{
   uint8_t h;
   uint8_t l;
   uint8_t b;
   int32_t i;
   int32_t tmp;
   l = 0U;
   h = 0U;
   tmp = size-1L;
   i = 0L;
   if (i<=tmp) for (;; i++) {
      b = (uint8_t)((uint8_t)(uint8_t)frame[i]^l);
      l = CRCL[b]^h;
      h = CRCH[b];
      if (i==tmp) break;
   } /* end for */
   frame[size] = (char)l;
   frame[size+1L] = (char)h;
} /* end aprsstr_appcrc() */


static int mkaprscall(int32_t * p, char raw[],
                uint32_t * i, const char mon[],
                char sep1, char sep2, char sep3,
                uint32_t sbase)
{
   uint32_t s;
   uint32_t l;
   l = 0UL;
   while ((((mon[*i] && mon[*i]!=sep1) && mon[*i]!=sep2) && mon[*i]!=sep3)
                && mon[*i]!='-') {
      s = (uint32_t)(uint8_t)mon[*i]*2UL&255UL;
      if (s<=64UL) return 0;
      raw[*p] = (char)s;
      ++*p;
      ++*i;
      ++l;
      if (l>=7UL) return 0;
   }
   while (l<6UL) {
      raw[*p] = '@';
      ++*p;
      ++l;
   }
   s = 0UL;
   if (mon[*i]=='-') {
      ++*i;
      while ((uint8_t)mon[*i]>='0' && (uint8_t)mon[*i]<='9') {
         s = (s*10UL+(uint32_t)(uint8_t)mon[*i])-48UL;
         ++*i;
      }
      if (s>15UL) return 0;
   }
   raw[*p] = (char)((s+sbase)*2UL);
   ++*p;
   return 1;
} /* end call() */



// returns raw len, 0 in case of error
extern int aprsstr_mon2raw(const char *mon, char raw[], int raw_len)
{
   uint32_t r;
   uint32_t n;
   uint32_t i;
   uint32_t tmp;
   int p = 7L;
   i = 0UL;
   fprintf(stderr,"mon2raw for %s\n", mon);
   if (!mkaprscall(&p, raw, &i, mon, '>', 0, 0, 48UL)) {
      return 0;
   }
   p = 0L;
   if (mon[i]!='>') return 0;
   /* ">" */
   ++i;
   if (!mkaprscall(&p, raw, &i, mon, ':', ',', 0, 112UL)) {
      return 0;
   }
   p = 14L;
   n = 0UL;
   while (mon[i]==',') {
      ++i;
      if (!mkaprscall(&p, raw, &i, mon, ':', ',', '*', 48UL)) {
         return 0;
      }
      ++n;
      if (n>8UL) {
         return 0;
      }
      if (mon[i]=='*') {
         /* "*" has repeatet sign */
         ++i;
         r = (uint32_t)p;
         if (r>=21UL) for (tmp = (uint32_t)(r-21UL)/7UL;;) {
            raw[r-1UL] = (char)((uint32_t)(uint8_t)raw[r-1UL]+128UL);
                 /* set "has repeated" flags */
            if (!tmp) break;
            --tmp;
            r -= 7UL;
         } /* end for */
      }
   }
 if (p==0L || mon[i]!=':') {
	return 0;
   }
   raw[p-1L] = (char)((uint32_t)(uint8_t)raw[p-1L]+1UL);
                /* end address field mark */
   raw[p] = '\003';
   ++p;
   raw[p] = '\360';
   ++p;
   ++i;
   n = 256UL;
   while (mon[i]) {
      /* copy info part */
      if (p>=(int32_t)(raw_len-1)-2L || n==0UL) {
         return 0;
      }
      raw[p] = mon[i];
      ++p;
      ++i;
      --n;
   }
   aprsstr_appcrc(raw, raw_len, p);
   //fprintf(stderr,"results in %s\n",raw);
   return p+2;
} /* end mon2raw() */

extern int aprsstr_mon2kiss(const char *mon, char raw[], int raw_len)
{
	char tmp[201];
	int len = aprsstr_mon2raw(mon, tmp, 201);
	if(len==0) return 0;
	int idx=0;
	raw[idx++] = '\xC0';
	raw[idx++] = 0; // channel 0
	for(int i=0; i<len-2; i++) { // -2: discard CRC, not used in KISS
		if(tmp[i]=='\xC0') {	
			raw[idx++] = '\xDB';
			raw[idx++] = '\xDC';
		} else if (tmp[i]=='\xDB') {
			raw[idx++] = '\xDB';
			raw[idx++] = '\xDD';
		} else {
			raw[idx++] = tmp[i];
		}
		if(idx>=raw_len)
			return 0;
	}
	return idx;
}

#define FEET (1.0/0.3048)
#define KNOTS (1.851984)

static uint32_t truncc(double r)
{
   if (r<=0.0) return 0UL;
   else if (r>=2.E+9) return 2000000000UL;
   else return (uint32_t)r;
   return 0;
} /* end truncc() */



static uint32_t dao91(double x)
/* radix91(xx/1.1) of dddmm.mmxx */
{
   double a;
   a = fabs(x);
   return ((truncc((a-(double)(float)truncc(a))*6.E+5)%100UL)
                *20UL+11UL)/22UL;
} /* end dao91() */


char b[251];
//char raw[201];
const char *destcall="APRRDZ";

char *aprs_send_beacon(const char *usercall, float lat, float lon, const char *sym, const char *comment) {
	*b = 0;
	aprsstr_append(b, usercall);
	aprsstr_append(b, ">");
	aprsstr_append(b, destcall);
#if 0
	aprsstr_append(b, ":/");   //  / is report with timestamp
	int i = strlen(b);
	int sec = 0; // TODO: NOW!!!
	snprintf(b+i, APRS_MAXLEN, "%02d%02d%02dh", sec/(60*60), (sec%(60*60))/60, sec%60);
#else
	// report without timestamp
	aprsstr_append(b, ":!");  //  ! is report w/p timestamp
#endif
	// lat
	int i = strlen(b);
	int lati = abs((int)lat);
	int latm = (fabs(lat)-lati)*6000;
	snprintf(b+i, APRS_MAXLEN-i, "%02d%02d.%02d%c%c", lati, latm/100, latm%100, lat<0?'S':'N', sym[0]);
	// lon
	i = strlen(b);
	int loni = abs((int)lon);
	int lonm = (fabs(lon)-loni)*6000;
	snprintf(b+i, APRS_MAXLEN-i, "%03d%02d.%02d%c%c", loni, lonm/100, lonm%100, lon<0?'W':'E', sym[1]);
	// maybe add alt
	// maybe add DAO?
	i = strlen(b);
	snprintf(b+i, APRS_MAXLEN-i, "%s", comment);

	i = strlen(b);
	snprintf(b+i, APRS_MAXLEN-i, " %s-%s", version_name, version_id);
	//sprintf(b + strlen(b), "%s", version_name);
	return b;
}

char *aprs_senddata(SondeInfo *si, const char *usercall, const char *objcall, const char *sym) {
	SondeData *s = &(si->d);
	*b=0;
	aprsstr_append(b, *objcall ? objcall : usercall);
	aprsstr_append(b, ">");
//	const char *destcall="APRARX,SONDEGATE,TCPIP,qAR,oh3bsg";
	aprsstr_append(b, destcall);
//	if(*objcall) { aprsstr_append(b, ","); aprsstr_append(b, usercall); }
	// uncompressed
	aprsstr_append(b, ":;");
	char tmp[10];
	snprintf(tmp,10,"%s         ",s->id);
	aprsstr_append(b, tmp);
	aprsstr_append(b, "*");
	// time
	int i = strlen(b);
	int sec = s->time % 86400;
	snprintf(b+i, APRS_MAXLEN-1, "%02d%02d%02dh", sec/(60*60), (sec%(60*60))/60, sec%60);
	i = strlen(b);
	//aprsstr_append_data(time, ds);
	int lati = abs((int)s->lat);
	int latm = (fabs(s->lat)-lati)*6000;
	snprintf(b+i, APRS_MAXLEN-i, "%02d%02d.%02d%c%c", lati, latm/100, latm%100, s->lat<0?'S':'N', sym[0]);
	i = strlen(b);
	int loni = abs((int)s->lon);
	int lonm = (fabs(s->lon)-loni)*6000;
	snprintf(b+i, APRS_MAXLEN-i, "%03d%02d.%02d%c%c", loni, lonm/100, lonm%100, s->lon<0?'W':'E', sym[1]);
	if(s->hs>0.5) {
		i=strlen(b);
		snprintf(b+i, APRS_MAXLEN-i, "%03d/%03d", realcard(s->dir+1.5), realcard(s->hs*3.6/KNOTS+0.5));
	}
	if(s->alt>0.5) {
		i=strlen(b);
		snprintf(b+i, APRS_MAXLEN-i, "/A=%06d", realcard(s->alt*FEET+0.5));
	}
	// always use DAO
	i=strlen(b);
	snprintf(b+i, APRS_MAXLEN-i, "!w%c%c!", 33+dao91(s->lat), 33+dao91(s->lon));

	// ??? strcat(b, "&");
	i=strlen(b);
        i += snprintf(b+i, APRS_MAXLEN-i, "Clb=%.1fm/s ", s->vs );
	if( !isnan(s->pressure) ) {
		sprintf(b+strlen(b), "p=%.1fhPa ", s->pressure);
	}
	if( !isnan(s->temperature) ) {
		sprintf(b+strlen(b), "t=%.1fC ", s->temperature);
	}
	if( !isnan(s->relativeHumidity) ) {
		sprintf(b+strlen(b), "h=%.1f%% ", s->relativeHumidity);
	}
	char type[12];
        if ( si->type == STYPE_RS41 && RS41::getSubtype(type, 11, si) == 0 ) {
	    // type was copied to type
        } else {
	    strncpy(type, sondeTypeStr[sonde.realType(si)], 11);  type[11]=0; 
        }
	
	sprintf(b+strlen(b), "%.3fMHz Type=%s ", si->freq, type /* sondeTypeStr[sonde.realType(si)] */ );
	if( s->countKT != 0xffff && s->vframe - s->crefKT < 51 ) {
		sprintf(b+strlen(b), "TxOff=%dh%dm ", s->countKT/3600, (s->countKT-s->countKT/3600*3600)/60);
	}
	if( TYPE_IS_DFM(si->type) || TYPE_IS_METEO(si->type) ) {
		sprintf(b + strlen(b), "ser=%s ", s->ser);
	}
	sprintf(b + strlen(b), "%s", version_name);
	return b;
}


#if 0
int main(int argc, char *argv[])
{
	Gencrctab();

	struct sockaddr_in si;
	int fd = openudp("127.0.0.1",9002,&si);
	if(fd<0) { fprintf(stderr,"open failed\n"); return 1; }

	float lat=48, lon=10;
	while(1) {
	const char *str = aprs_senddata(lat, lon, 543, 5, 180, 1.5, "RS41", "TE0ST", "TE1ST", "EO");
	int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
	sendudp(fd, raw, rawlen);

	str = "OE3XKC>APMI06,qAR,OE3XLR:;ER-341109*111111z4803.61NE01532.39E0145.650MHz R15k OE3XPA";
	rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
	sendudp(fd, &si, raw, rawlen);
	lat += 0.002; lon += 0.01;
	sleep(5);
	}
}
#endif

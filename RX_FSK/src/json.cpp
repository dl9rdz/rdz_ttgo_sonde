#include "json.h"
#include "RS41.h"

extern const char *sondeTypeStrSH[];
extern const char *dfmSubtypeStrSH[];

static char typestr[11];

const char *getType(SondeInfo *si) {
    if( si->type == STYPE_RS41 ) {
        if ( RS41::getSubtype(typestr, 11, si) == 0 ) return typestr;
    } else if ( TYPE_IS_DFM(si->type) && si->d.subtype > 0 && si->d.subtype < 16 ) {
        const char *t = dfmSubtypeStrSH[si->d.subtype];
        if(t) return t;
        sprintf(typestr, "DFMx%X", si->d.subtype);
        return typestr;
    }
    return sondeTypeStrSH[sonde.realType(si)];
}

// To be used by
// - MQTT
// - rdzJSON (for Android app)
// - Web map
int sonde2json(char *buf, int maxlen, SondeInfo *si)
{
    SondeData *s = &(si->d);
    int n = snprintf(buf, maxlen, 
        "\"type\":\"%s\","
        "\"id\": \"%s\","       // TODO: maybe remove in the future, ser is enough, client can calculate APRS id if needed
        "\"ser\": \"%s\","
        "\"frame\": %u,"	// raw frame, from sonde, can be 0. (TODO: add virtual frame # as in sondehub?)
        "\"vframe\": %d,"	
        "\"time\": %u,"
        "\"lat\": %.5f,"
        "\"lon\": %.5f,"
        "\"alt\": %.1f,"
        "\"vs\": %.1f,"
        "\"hs\": %.1f," 
        "\"climb\": %.1f,"	// used by HTML map, to be removed (-> vs)
        "\"speed\": %.1f,"	// used by HTML map, to be removed (-> hs)
        "\"dir\": %.1f,"
        "\"sats\": %d,"
        "\"freq\": %.2f,"
        "\"rssi\": %d,"
        "\"afc\": %d,"
        "\"launchKT\": %d,"
        "\"burstKT\": %d,"
        "\"countKT\": %d,"
        "\"crefKT\": %d,"
	"\"launchsite\": \"%d\","
	"\"res\": %d",
        getType(si),
        s->id,
        s->ser,
        s->frame,
        s->vframe,
        s->time,
        s->lat,
        s->lon,
        s->alt,
        s->vs,
        s->hs,
        s->vs,
        s->hs,
        s->dir,
        s->sats,
        si->freq,
        si->rssi,
        si->afc,
        s->launchKT,
        s->burstKT,
        s->countKT,
        s->crefKT,
	si->launchsite,
	(int)si->rxStat[0]
    );
    if(n>=maxlen) return -1;
    buf += n; maxlen -= n;

    // add only if available
    if(s->batteryVoltage > 0) {
	n = snprintf(buf, maxlen, ",\"bat\": %.1f", s->batteryVoltage);
	if(n>=maxlen) return -1;
	buf += n; maxlen -= n;
    }
    if ( !isnan( s->temperature ) ) {
        n = snprintf(buf, maxlen, ",\"temp\": %.1f", s->temperature );
	if(n>=maxlen) return -1;
	buf += n; maxlen -=n;
    }
    if ( !isnan( s->relativeHumidity) ) {
        n = snprintf(buf, maxlen, ",\"humidity\": %.1f", s->relativeHumidity);
	if(n>=maxlen) return -1;
	buf += n; maxlen -=n;
    }
    if ( !isnan( s->pressure) ) {
        n = snprintf(buf, maxlen, ",\"pressure\": %.1f", s->pressure );
	if(n>=maxlen) return -1;
	buf += n; maxlen -=n;
    }
    return 0;
}



#include "json.h"
#include <string.h>
#include <ctype.h>
#include "RS41.h"
#include "DFM.h"

extern const char *sondeTypeStrSH[];
//extern const char *dfmSubtypeStrSH[];

static char typestr[11];

const char *getType(SondeInfo *si) {
    if( si->type == STYPE_RS41 ) {
        if ( RS41::getSubtype(typestr, 11, si) == 0 ) return typestr;
    } else if ( TYPE_IS_DFM(si->type) && si->d.subtype > 0 ) {
        const char *t = dfmSubtypeLong[si->d.subtype & 0xf];
	if( (si->d.subtype & 0xf) == DFM_UNK) {
           sprintf(typestr, "DFMx%X", si->d.subtype>>4);
           return typestr;
	}
        return t;
    }
    return sondeTypeStrSH[sonde.realType(si)];
}

static int float2json(char **buf, int *maxlen, const char *fmt, float value) {
   if(isnan(value)) return 0;
   int n = snprintf(*buf, *maxlen, fmt, value);
   if(n<0 || n>=*maxlen) return -1;
   *buf += n; *maxlen -= n;
   return n;
}

/* Escapes as much of src as fits; *rest points to the first unconsumed char (NUL if all done). */
static size_t json_escape_impl(char *dst, size_t cap, const char *src, const char **rest) {
    size_t o = 0;
    if (!src) src = "";
    for (; *src; src++) {
        unsigned char c = (unsigned char)*src;
        if (c == '"' || c == '\\') {
            if (o + 2 >= cap) break;
            dst[o++] = '\\';
            dst[o++] = (char)c;
        } else if (c < 0x20) {
            if (o + 6 >= cap) break;
            snprintf(dst + o, cap - o, "\\u%04x", c);
            o += 6;
        } else {
            if (o + 1 >= cap) break;
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
    *rest = src;
    return o;
}

size_t json_escape(char *dst, size_t cap, const char *src) {
    if (cap == 0) return 0;
    const char *rest;
    return json_escape_impl(dst, cap, src, &rest);
}

bool json_put_str(char **p, char *end, const char *src) {
    char *w = *p;
    if (end - w < 3) return false;          /* opening + closing quote + NUL */
    const char *rest;
    /* body may use everything except the opening quote, closing quote and NUL */
    size_t n = json_escape_impl(w + 1, (size_t)(end - w) - 2, src, &rest);
    if (*rest) { *w = '\0'; return false; }
    w[0] = '"';
    w[1 + n] = '"';
    w[2 + n] = '\0';
    *p = w + 2 + n;
    return true;
}

static void json_put_utf8(char *out, size_t *o, size_t out_cap, unsigned cp) {
    char tmp[3];
    size_t n;
    if (cp < 0x80) { tmp[0] = (char)cp; n = 1; }
    else if (cp < 0x800) { tmp[0] = (char)(0xC0 | (cp >> 6)); tmp[1] = (char)(0x80 | (cp & 0x3F)); n = 2; }
    else { tmp[0] = (char)(0xE0 | (cp >> 12)); tmp[1] = (char)(0x80 | ((cp >> 6) & 0x3F)); tmp[2] = (char)(0x80 | (cp & 0x3F)); n = 3; }
    if (*o + n < out_cap) { memcpy(out + *o, tmp, n); *o += n; }
}

bool json_get_str(const char **pp, const char *name, char *out, size_t out_cap) {
    if (!pp || !*pp || !name || !out || out_cap == 0) return false;
    char keypat[48];
    snprintf(keypat, sizeof(keypat), "\"%s\"", name);
    const char *p = strstr(*pp, keypat);
    if (!p) return false;
    p += strlen(keypat);
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    if (*p != ':') return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    if (*p != '"') return false;
    p++;
    size_t o = 0;
    while (*p && *p != '"') {
        unsigned char c = (unsigned char)*p++;
        if (c == '\\' && *p) {
            char e = *p++;
            switch (e) {
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'n': c = '\n'; break;
            case 'r': c = '\r'; break;
            case 't': c = '\t'; break;
            case 'u': {
                unsigned cp = 0;
                int k;
                for (k = 0; k < 4 && isxdigit((unsigned char)p[k]); k++) {
                    char h = p[k];
                    cp = cp * 16 + (h <= '9' ? h - '0' : (h | 0x20) - 'a' + 10);
                }
                if (k == 4) {
                    p += 4;
                    json_put_utf8(out, &o, out_cap, cp);
                    continue;
                }
                c = 'u';
                break;
            }
            default: c = (unsigned char)e; break;  /* \" \\ \/ */
            }
        }
        if (o + 1 < out_cap) out[o++] = (char)c;
    }
    out[o] = '\0';
    if (*p == '"') p++;
    *pp = p;
    return true;
}

// To be used by
// - MQTT
// - rdzJSON (for Android app)
// - Web map
int sonde2json(char *buf, int maxlen, SondeInfo *si, bool rssi_as_dbm)
{
    SondeData *s = &(si->d);
    int n;
    float rssi_conversion = 1.0;

    // this allows callers to get the raw reading and convert it themselves (mobile app)
    // or to request the conversion to be done internally for the benefit of downstream
    // consumers (mqtt subsystem, and anything else that calls this function)
    if (rssi_as_dbm)
      rssi_conversion = -0.5;

    n = float2json(&buf, &maxlen, "\"lat\": %.5f,", s->lat);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"lon\": %.5f,", s->lon);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"alt\": %.1f,", s->alt);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"vs\": %.1f,", s->vs);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"hs\": %.1f,", s->hs);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"climb\": %.1f,", s->vs); // used by HTML map, to be removed (-> vs)
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"speed\": %.1f,", s->hs); // used by HTML map, to be removed (-> hs)
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"dir\": %.1f,", s->dir);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"temp\": %.1f,", s->temperature );
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"humidity\": %.1f,", s->relativeHumidity);
    if(n<0) return -1;
    n = float2json(&buf, &maxlen, "\"pressure\": %.1f,", s->pressure);
    if(n<0) return -1;
    n = snprintf(buf, maxlen, 
        "\"type\":\"%s\","
        "\"id\": \"%s\","       // TODO: maybe remove in the future, ser is enough, client can calculate APRS id if needed
        "\"ser\": \"%s\","
        "\"frame\": %u,"	// raw frame, from sonde, can be 0. (TODO: add virtual frame # as in sondehub?)
        "\"vframe\": %d,"	
        "\"time\": %u,"
        "\"sats\": %d,"
        "\"freq\": %.2f,"
        "\"rssi\": %.1f,"
        "\"afc\": %d,"
        "\"launchKT\": %d,"
        "\"burstKT\": %d,"
        "\"countKT\": %d,"
        "\"crefKT\": %d,"
	"\"launchsite\": \"%s\","
	"\"res\": %d",
        getType(si),
        s->id,
        s->ser,
        s->frame,
        s->vframe,
        s->time,
        s->sats,
        si->freq,
        si->rssi * rssi_conversion,
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
	n = snprintf(buf, maxlen, ",\"batt\": %.1f", s->batteryVoltage);
	if(n>=maxlen) return -1;
	buf += n; maxlen -= n;
    }
    return 0;
}

const char *json_object_end(const char *p) {
    if (!p || *p != '{') return nullptr;
    int depth = 0;
    bool in_str = false;
    for (; *p; p++) {
        if (in_str) {
            if (*p == '\\' && p[1]) p++;
            else if (*p == '"') in_str = false;
        } else if (*p == '"') {
            in_str = true;
        } else if (*p == '{') {
            depth++;
        } else if (*p == '}') {
            if (--depth == 0) return p;
        }
    }
    return nullptr;
}

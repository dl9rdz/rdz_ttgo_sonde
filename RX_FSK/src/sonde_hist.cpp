#include "sonde_hist.h"

#include <LittleFS.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define TAG "sonde_hist"
#include "logger.h"

#define LAST_RX_PATH "/last_rx.csv"

struct HistPt {
	float lat, lon, alt;
	uint32_t t;
};

struct HistEntry {
	char ser[12];
	char type[8];
	double lat, lon;
	float alt;
	float freq;
	float vs, hs, dir;
	uint32_t time;
	HistPt buf[SONDE_HIST_BUF];
	uint8_t nBuf;
	uint16_t nPoints;
	uint32_t lastPtT;
	uint32_t lastHeard; /* unix seconds of last updateSonde for this ser */
	bool dirty;
};

static HistEntry store[SONDE_HIST_MAX];
static int nStore = 0;
static bool indexDirty = false;

static int findSer(const char *ser)
{
	for (int i = 0; i < nStore; i++) {
		if (strcmp(store[i].ser, ser) == 0)
			return i;
	}
	return -1;
}

static bool freqMatch(float a, float b)
{
	return fabsf(a - b) < 0.005f; /* ~5 kHz */
}

static uint16_t sampleInterval(float alt)
{
	return (alt > 2000.0f) ? 60 : 10;
}

static void ltPath(const char *ser, char *buf, int buflen)
{
	int w = 0;
	buf[w++] = '/';
	buf[w++] = 'l';
	buf[w++] = 't';
	buf[w++] = '_';
	for (const char *p = ser; *p && w < buflen - 5; p++) {
		char c = *p;
		if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
		    (c >= 'a' && c <= 'z') || c == '-' || c == '_')
			buf[w++] = c;
	}
	if (w <= 4) {
		buf[0] = 0;
		return;
	}
	strcpy(buf + w, ".csv");
}

static void ltRemove(const char *ser)
{
	char path[40];
	ltPath(ser, path, sizeof(path));
	if (path[0] && LittleFS.exists(path))
		LittleFS.remove(path);
}

bool sondeHistIsManagedFile(const char *basename)
{
	if (!basename || !*basename)
		return false;
	if (strcasecmp(basename, "last_rx.csv") == 0)
		return true;
	if (strncmp(basename, "lt_", 3) != 0)
		return false;
	size_t n = strlen(basename);
	return n > 7 && strcasecmp(basename + n - 4, ".csv") == 0;
}

static void writeLastRx()
{
	File f = LittleFS.open(LAST_RX_PATH, "w");
	if (!f) {
		LOG_E(TAG, "cannot open %s\n", LAST_RX_PATH);
		return;
	}
	f.println("ser,type,lat,lon,alt,freq,time,vs,hs,dir");
	for (int i = 0; i < nStore; i++) {
		f.printf("%s,%s,%.6f,%.6f,%.0f,%.3f,%lu,%.1f,%.1f,%.0f\n",
			 store[i].ser, store[i].type, store[i].lat, store[i].lon,
			 store[i].alt, store[i].freq, (unsigned long)store[i].time,
			 store[i].vs, store[i].hs, store[i].dir);
	}
	f.close();
	indexDirty = false;
	for (int i = 0; i < nStore; i++)
		store[i].dirty = false;
	LOG_I(TAG, "wrote %s (%d entries)\n", LAST_RX_PATH, nStore);
}

static void flushEntry(HistEntry &e, bool ensureLast)
{
	int mode = sonde.config.hist.mode;
	if (mode < 1)
		return;

	if (ensureLast && e.time) {
		bool need = true;
		if (e.nBuf > 0) {
			HistPt &last = e.buf[e.nBuf - 1];
			if (last.t == e.time &&
			    fabsf(last.lat - (float)e.lat) < 1e-5f &&
			    fabsf(last.lon - (float)e.lon) < 1e-5f)
				need = false;
		}
		if (need) {
			if (e.nBuf >= SONDE_HIST_BUF) {
				/* Write current buffer, then keep the latest point for this flush. */
				if (mode >= 2) {
					char path[40];
					ltPath(e.ser, path, sizeof(path));
					if (path[0] && e.nPoints < SONDE_HIST_TRACK_MAX) {
						File f = LittleFS.open(path, e.nPoints ? "a" : "w");
						if (f) {
							if (e.nPoints == 0)
								f.println("lat,lon,alt,t");
							for (int i = 0; i < e.nBuf && e.nPoints < SONDE_HIST_TRACK_MAX; i++) {
								f.printf("%.5f,%.5f,%.0f,%lu\n", e.buf[i].lat,
									 e.buf[i].lon, e.buf[i].alt,
									 (unsigned long)e.buf[i].t);
								e.nPoints++;
							}
							f.close();
						}
					}
				}
				e.nBuf = 0;
			}
			e.buf[e.nBuf].lat = (float)e.lat;
			e.buf[e.nBuf].lon = (float)e.lon;
			e.buf[e.nBuf].alt = e.alt;
			e.buf[e.nBuf].t = e.time;
			e.nBuf++;
			e.lastPtT = e.time;
		}
	}

	if (mode >= 2 && e.nBuf > 0) {
		char path[40];
		ltPath(e.ser, path, sizeof(path));
		if (path[0] && e.nPoints < SONDE_HIST_TRACK_MAX) {
			File f = LittleFS.open(path, e.nPoints ? "a" : "w");
			if (f) {
				if (e.nPoints == 0)
					f.println("lat,lon,alt,t");
				for (int i = 0; i < e.nBuf && e.nPoints < SONDE_HIST_TRACK_MAX; i++) {
					f.printf("%.5f,%.5f,%.0f,%lu\n", e.buf[i].lat, e.buf[i].lon,
						 e.buf[i].alt, (unsigned long)e.buf[i].t);
					e.nPoints++;
				}
				f.close();
			} else {
				LOG_E(TAG, "cannot open %s\n", path);
			}
		}
	}
	e.nBuf = 0;
	indexDirty = true;
	writeLastRx();
}

static void addPoint(HistEntry &e, double lat, double lon, float alt, uint32_t t)
{
	if (!t)
		return;
	if (lat == 0.0 && lon == 0.0)
		return;
	if (e.lastPtT && (t - e.lastPtT) < sampleInterval(alt))
		return;
	if (e.nBuf >= SONDE_HIST_BUF)
		flushEntry(e, false);
	if (e.nBuf >= SONDE_HIST_BUF)
		return;
	e.buf[e.nBuf].lat = (float)lat;
	e.buf[e.nBuf].lon = (float)lon;
	e.buf[e.nBuf].alt = alt;
	e.buf[e.nBuf].t = t;
	e.nBuf++;
	e.lastPtT = t;
	if (e.nBuf >= SONDE_HIST_BUF)
		flushEntry(e, false);
}

static int allocSlot(const char *ser)
{
	int i = findSer(ser);
	if (i >= 0)
		return i;
	if (nStore < SONDE_HIST_MAX) {
		i = nStore++;
		memset(&store[i], 0, sizeof(store[i]));
		strncpy(store[i].ser, ser, sizeof(store[i].ser) - 1);
		ltRemove(ser);
		return i;
	}
	/* Evict oldest by time */
	i = 0;
	for (int k = 1; k < nStore; k++) {
		if (store[k].time < store[i].time)
			i = k;
	}
	flushEntry(store[i], true);
	ltRemove(store[i].ser);
	memset(&store[i], 0, sizeof(store[i]));
	strncpy(store[i].ser, ser, sizeof(store[i].ser) - 1);
	ltRemove(ser);
	return i;
}

void sondeHistUpdate(SondeInfo *si)
{
	if (!si || sonde.config.hist.mode < 1)
		return;
	SondeData *d = &si->d;
	if (!d->validID || !VALIDPOS(d->validPos))
		return;
	if (!d->ser[0])
		return;

	const char *type = d->typestr[0] ? d->typestr : sondeTypeStr[sonde.realType(si)];
	int i = allocSlot(d->ser);
	HistEntry &e = store[i];
	strncpy(e.type, type, sizeof(e.type) - 1);
	e.type[sizeof(e.type) - 1] = 0;
	e.lat = d->lat;
	e.lon = d->lon;
	e.alt = VALIDALT(d->validPos) ? d->alt : 0;
	e.freq = si->freq;
	e.vs = VALIDVS(d->validPos) ? d->vs : 0;
	e.hs = VALIDHS(d->validPos) ? d->hs : 0;
	e.dir = VALIDDIR(d->validPos) ? d->dir : 0;
	if (d->time)
		e.time = d->time;
	e.lastHeard = e.time ? e.time : (uint32_t)time(NULL);
	e.dirty = true;
	indexDirty = true;

	if (sonde.config.hist.mode >= 2) {
		addPoint(e, e.lat, e.lon, e.alt, e.time);
	} else {
		/* mode 1: keep one RAM point; rewrite last_rx when sample interval elapses */
		e.buf[0].lat = (float)e.lat;
		e.buf[0].lon = (float)e.lon;
		e.buf[0].alt = e.alt;
		e.buf[0].t = e.time;
		e.nBuf = 1;
		bool due = !e.lastPtT || (e.time && (e.time - e.lastPtT) >= sampleInterval(e.alt));
		if (due) {
			e.lastPtT = e.time;
			writeLastRx();
		}
	}
}

void sondeHistTick()
{
	if (sonde.config.hist.mode < 1)
		return;
	static unsigned long lastCheck = 0;
	if (millis() - lastCheck < 10000UL)
		return;
	lastCheck = millis();

	time_t now;
	time(&now);
	if ((uint32_t)now < 1600000000UL)
		return;

	for (int i = 0; i < nStore; i++) {
		if (!store[i].dirty && store[i].nBuf == 0)
			continue;
		uint32_t heard = store[i].lastHeard ? store[i].lastHeard : store[i].time;
		if (!heard)
			continue;
		if ((uint32_t)now - heard < SONDE_HIST_STALE_SEC)
			continue;
		LOG_I(TAG, "%s stale, flushing\n", store[i].ser);
		flushEntry(store[i], true);
		return; /* one per tick */
	}
}

/* decoder.json only lists slots with lastRx!=0; map age from wall clock when possible. */
static void setLastRxFromHist(SondeInfo *si, uint32_t histTime)
{
	uint32_t nowMs = millis();
	time_t nowu = time(nullptr);
	if (histTime && (uint32_t)nowu > 1600000000UL && (uint32_t)nowu >= histTime) {
		uint32_t ageSec = (uint32_t)nowu - histTime;
		if (ageSec <= nowMs / 1000UL)
			si->lastRx = nowMs - ageSec * 1000UL;
		else
			si->lastRx = 1; /* older than uptime; age will look large */
	} else {
		/* No usable wall clock: still non-zero so the Decoder panel includes the slot. */
		si->lastRx = nowMs ? nowMs : 1;
	}
}

static void seedDecoder()
{
	for (int s = 0; s < sonde.nSonde; s++) {
		SondeInfo *si = &sonde.sondeList[s];
		int best = -1;
		for (int i = 0; i < nStore; i++) {
			if (!freqMatch(si->freq, store[i].freq))
				continue;
			if (best < 0 || store[i].time > store[best].time)
				best = i;
		}
		if (best < 0)
			continue;
		HistEntry &e = store[best];
		SondeData *d = &si->d;
		memset(d->ser, 0, sizeof(d->ser));
		memset(d->id, 0, sizeof(d->id));
		strncpy(d->ser, e.ser, sizeof(d->ser) - 1);
		strncpy(d->id, e.ser, sizeof(d->id) - 1);
		d->validID = true;
		memset(d->typestr, 0, sizeof(d->typestr));
		strncpy(d->typestr, e.type, sizeof(d->typestr) - 1);
		d->lat = (float)e.lat;
		d->lon = (float)e.lon;
		d->alt = e.alt;
		d->vs = e.vs;
		d->hs = e.hs;
		d->dir = e.dir;
		d->time = e.time;
		d->validTime = e.time != 0;
		d->validPos = 0x03 | 0x04 | 0x08 | 0x10 | 0x20 | 0x80; /* pos+alt+vs+hs+dir+old */
		setLastRxFromHist(si, e.time);
		LOG_I(TAG, "seeded slot %d freq=%.3f with %s lastRx=%u\n", s, si->freq, e.ser,
		      (unsigned)si->lastRx);
	}
}

void sondeHistLoad()
{
	nStore = 0;
	indexDirty = false;
	if (sonde.config.hist.mode < 1)
		return;

	File f = LittleFS.open(LAST_RX_PATH, "r");
	if (!f) {
		LOG_I(TAG, "no %s\n", LAST_RX_PATH);
		return;
	}
	char line[200];
	while (f.available() && nStore < SONDE_HIST_MAX) {
		int n = f.readBytesUntil('\n', line, sizeof(line) - 1);
		line[n] = 0;
		if (n < 5 || line[0] == '#' || strncmp(line, "ser,", 4) == 0)
			continue;
		char ser[12] = "", type[8] = "";
		double lat = 0, lon = 0;
		float alt = 0, freq = 0, vs = 0, hs = 0, dir = 0;
		unsigned long t = 0;
		if (sscanf(line, "%11[^,],%7[^,],%lf,%lf,%f,%f,%lu,%f,%f,%f",
			   ser, type, &lat, &lon, &alt, &freq, &t, &vs, &hs, &dir) < 7)
			continue;
		HistEntry &e = store[nStore++];
		memset(&e, 0, sizeof(e));
		strncpy(e.ser, ser, sizeof(e.ser) - 1);
		strncpy(e.type, type, sizeof(e.type) - 1);
		e.lat = lat;
		e.lon = lon;
		e.alt = alt;
		e.freq = freq;
		e.vs = vs;
		e.hs = hs;
		e.dir = dir;
		e.time = (uint32_t)t;
		e.lastHeard = e.time;
		char path[40];
		ltPath(e.ser, path, sizeof(path));
		if (path[0] && LittleFS.exists(path)) {
			/* approximate point count from file size is unnecessary; leave 0 and append */
			File tf = LittleFS.open(path, "r");
			if (tf) {
				int lines = 0;
				while (tf.available()) {
					char discard[64];
					int m = tf.readBytesUntil('\n', discard, sizeof(discard) - 1);
					if (m > 0)
						lines++;
				}
				tf.close();
				if (lines > 1)
					e.nPoints = (uint16_t)(lines - 1);
			}
		}
	}
	f.close();
	LOG_I(TAG, "loaded %d entries from %s\n", nStore, LAST_RX_PATH);
	seedDecoder();
}

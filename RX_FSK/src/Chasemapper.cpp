#include "Chasemapper.h"

extern const char *sondeTypeStrSH[];

int Chasemapper::send(WiFiUDP &udp, SondeInfo *si) {
	char buf[1024];
	struct tm tim;
	time_t t = si->d.time;
	gmtime_r(&t, &tim);
	uint8_t realtype = si->type;
	if (TYPE_IS_METEO(realtype)) {
		realtype = si->d.subtype == 1 ? STYPE_M10 : STYPE_M20;
	}
	sprintf(buf, "{ \"type\": \"PAYLOAD_SUMMARY\","
		"\"callsign\": \"%s\","
		"\"latitude\": %g,"
		"\"longitude\": %g,"
		"\"altitude\": %d,"
		"\"speed\": %d,"
		"\"heading\": %d,"
		"\"time\": \"%02d:%02d:%02d\","
		"\"model\": \"%s\","
		"\"freq\": \"%.3f MHz\","
		"\"temp\": %g }",
		si->d.ser,
		si->d.lat,
		si->d.lon,
		(int)si->d.alt,
		(int)(si->d.hs * 1.9438445),  // m/s into knots
		(int)si->d.dir,
		tim.tm_hour, tim.tm_min, tim.tm_sec,
		sondeTypeStrSH[realtype],
		si->freq,
		si->d.temperature);
	Serial.printf("Sending chasemapper json: %s\n", buf);
	udp.beginPacket(sonde.config.cm.host, sonde.config.cm.port);
	udp.write((const uint8_t *)buf, strlen(buf));
	udp.endPacket();	
	return 0;
}


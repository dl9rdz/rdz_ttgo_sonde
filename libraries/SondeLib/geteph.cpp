#include "time.h"
#include "geteph.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <rom/miniz.h>
#include <inttypes.h>
#include <WiFi.h>
#include "Display.h"
#include "Sonde.h"

extern WiFiClient client;

//static const char *ftpserver = "www.ngs.noaa.gov";
char outbuf[128];

uint8_t getreply() {
	String s = client.readStringUntil('\n');
	Serial.println(s);
	const char *str = s.c_str();
	if(strlen(str)<4) return 255; // something unusual...
	if(str[3]=='-') { // multi-line resonse...
		String s2;
		const char *str2;
		do {
			s2 = client.readStringUntil('\n');
			Serial.println(s2);
			str2 = s2.c_str();
			if(strlen(str2)<4) return 255; // something is wrong
		} while( str[0]!=str2[0] || str[1]!=str2[1] || str[2]!=str2[2] || str2[3]!=' ' );
	}
	return str[0];
}

void writeFully(File &file, uint8_t *buf, size_t len)
{
	size_t olen;

	while(len) {
		olen = file.write(buf, len);
		Serial.printf("written: %d of %d\n", olen, len);
		len -= olen;
		buf += olen;
	}
}

void geteph() {
	// Set current time via network...	
	struct tm tinfo;
	configTime(0, 0, "pool.ntp.org");
	bool ok = getLocalTime(&tinfo, 2000);  // wait max 2 seconds to get current time via ntp
	if(!ok) {
		Serial.println("Failed to get current date/time");
		return;
	}

	// Check time of last update
	int year = tinfo.tm_year + 1900;
	int day = tinfo.tm_yday + 1;
	Serial.printf("year %d, day %d\n", year, day);
	char nowstr[20];
	snprintf(nowstr, 20, "%04d%03d%02d", year, day, tinfo.tm_hour);
	File status = SPIFFS.open("/brdc.time", "r");
	if(status) {
		String ts = status.readStringUntil('\n');
		const char *tsstr = ts.c_str();
		if(tsstr && strlen(tsstr)>=9) {
			if(strcmp(nowstr, ts.c_str())<=0) {
				Serial.println("local brdc is up to date\n");
				return;
			}
		}
		Serial.printf("now: %s, existing: %s => updating\n", nowstr, tsstr);
	}
	status.close();
	File fh = SPIFFS.open("/brdc.gz","w");
	if(!fh) {	
		Serial.println("cannot open file\n");
		return;
	}
	char host[252];
	strcpy(host, sonde.config.ephftp);
	char *buf  = strchr(host, '/');
	if(!buf) { Serial.println("Invalid FTP host config"); return; }
        *buf = 0;
	buf++;	
	uint8_t dispw, disph, dispxs, dispys;
  	disp.rdis->getDispSize(&disph, &dispw, &dispxs, &dispys);
	disp.rdis->clear();
	disp.rdis->setFont(FONT_SMALL);
	disp.rdis->drawString(0, 0, host);
	// fetch rinex from server
	char *ptr = buf + strlen(buf);
	snprintf(ptr, 128, "%04d/%03d/brdc%03d0.%02dn.gz", year, day, day, year-2000);
	Serial.println("running geteph\n");
	disp.rdis->drawString(0, 1*dispys, ptr+9);
	
	if(!client.connect(host, 21)) {
		Serial.printf("FTP connection to %s failed\n", host);
		return;
	}
	if(getreply()>='4') { Serial.println("connected failed"); return; }
	client.print("USER anonymous\r\n");
	if(getreply()>='4') { Serial.println("USER failed"); return; }
	client.print("PASS anonymous\r\n");
	if(getreply()>='4') { Serial.println("PASS failed"); return; }
	client.print("TYPE I\r\n");
	if(getreply()>='4') { Serial.println("TYPE I failed"); return; }
	client.print("PASV\r\n");
	String s = client.readStringUntil('\n');
	Serial.println(s);
	if(s.c_str()[0]>='4') { Serial.println("PASV failed"); return; }
	int array_pasv[6];
	char *tStr = strtok((char *)s.c_str(), "(,");
	for(int i=0; i<6; i++) {
		tStr = strtok(NULL, "(,");
		if(tStr==NULL) {
			Serial.println("strange response to PASV");
			return;
		}
		array_pasv[i] = atoi(tStr);  
		Serial.println(array_pasv[i]);
	}
	uint16_t port = (array_pasv[4]<<8) | (array_pasv[5]&0xff);
	WiFiClient dclient;
	Serial.printf("connecting to %s:%d\n", host, port);
	dclient.connect(host, port);
	if(!dclient) {
		Serial.println("data connection failed");
		return;
	}
	client.print("RETR ");
	Serial.printf("fetching %s with FTP...\n", buf);
	client.println(buf);
	s = client.readStringUntil('\n');
	Serial.println(s);
	if(s.c_str()[0]>='4') { Serial.println("RETR failed"); return; }
	int len=0;
	while(dclient.connected()) {
		while(dclient.available()) {
			int c = dclient.read();
			if(c==-1) {
				Serial.println("dclient.read() returned -1 inspite of available() being true?!");
			} else {
				fh.write(c);
				len++;
			}
		}
	}
	Serial.printf("fetched %d bytes\n", len);
	fh.close();
	snprintf(buf, 16, "Fetched %d B    ",len);
	buf[16]=0;
	disp.rdis->drawString(0,2*dispys,buf);

	disp.rdis->drawString(0,4*dispys,"Decompressing...");
	// decompression
	tinfl_decompressor *decomp = (tinfl_decompressor *)malloc(sizeof(tinfl_decompressor));
	tinfl_init(decomp);
	File file = SPIFFS.open("/brdc.gz","r");
	if(!file) {	
		Serial.println("cannot open file\n");
		return;
	}
	File ofile = SPIFFS.open("/brdc", "w");
	if(!ofile) {
		Serial.println("cannot open file /brdc for writing");
		return;
	}
	file.readBytes(buf, 10);  // skip gzip header
	char flags = buf[3];
	if(flags&0x07) {
		Serial.println("Unsupported flags in gzip header, may or may not cause a problem");
	}
	if(flags&0x08) { // skip file name extra header
		do {
			int res=file.readBytes(buf, 1);
			if(res!=1) return;
		} while(*buf);
	}
	if(flags&0x10) { // skip file name extra header
		do {
			int res=file.readBytes(buf, 1);
			if(res!=1) return;
		} while(*buf);
	}
	int opos = 0;
	int total = 0;
	Serial.println("Decompressing ephemeris data...\n");
	char *obuf =(char *)malloc(32768);
	char *ibuf =(char *)malloc(8192);
	while(file.available()) {
		size_t len = file.readBytes(ibuf, 8192);
		size_t inofs = 0;
		size_t inlen = len;
		while(inofs<len) {
			size_t outlen=32768-opos;
			int res = tinfl_decompress(decomp, (const mz_uint8 *)ibuf+inofs, &inlen, (uint8_t *)obuf, (mz_uint8 *)obuf+opos, &outlen, TINFL_FLAG_HAS_MORE_INPUT);
			if(res<0) break;
			if(outlen==0) break;
			Serial.printf("... (res=%d) decompressed %d into %d bytes\n", res, inlen, outlen);
			inofs += inlen;
			inlen = len - inofs;
			//size_t retv = ofile.write((uint8_t *)(obuf+opos), outlen);
			//Serial.printf("write %d bytes\n", retv);
			writeFully(ofile, (uint8_t *)(obuf+opos), outlen);
			//Serial.write((uint8_t *)(obuf+opos), outlen);
			total += outlen;
			opos += outlen;
			if(res==0) break; // done indication
			if(opos>=32768) {
				Serial.printf("... decompressed %d bytes\n", total);
				opos=0;
			}
		}
	}
	// maybe todo: check crc?!?
	Serial.printf("done extracing content (total length: %d)\n", total);
	status = SPIFFS.open("/brdc.time","w");
	status.println(nowstr);
	status.close();
        snprintf(buf, 16, "Done: %d B    ",total);
        buf[16]=0;
        disp.rdis->drawString(0,5*dispys,buf);
	delay(1000);

	free(obuf);
	free(ibuf);
	free(decomp);
	file.close();
	ofile.close();
}

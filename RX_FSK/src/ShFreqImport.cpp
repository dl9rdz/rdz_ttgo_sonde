#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <Arduino.h>

#include "ShFreqImport.h"
#include "Sonde.h"

static int ppos;
static int quotes;
static char id[20];
static int idpos;
static float lat, lon, freq;
static char type[20];

static uint8_t inuse[1+99/8]; // MAXSONDE is 99

static char keyword[40];
static int keywordpos;
static char value[40];
static int valuepos;

static int importState;
static float homelat, homelon;


// Map SondeHub type string to Stype. -1 for not supported types.
int ShFreqImport::stringToStype(const char *type) {
    if(type[2]=='4') return STYPE_RS41;
    if(type[2]=='9') return STYPE_RS92;
    if(type[1]=='1') return STYPE_M10;
    if(type[1]=='2') return STYPE_M20;
    if(type[0]=='D') return STYPE_DFM;
    if(type[2]=='3') return STYPE_MP3H;  // TODO: check if '3' is correct
    return -1;  // iMet is not supported
}

// in Display.cpp
extern float calcLatLonDist(float lat1, float lon1, float lat2, float lon2);

void ShFreqImport::setLabel(int idx, char *id, float lat, float lon) {
	snprintf(sonde.sondeList[idx].launchsite, 18, "@%s/%d", id, (int)(calcLatLonDist(homelat, homelon, lat, lon)/1000));
	sonde.sondeList[idx].launchsite[17] = 0;
}

void ShFreqImport::usekeyvalue() {
        if(strcmp(keyword,"lat")==0) lat = atof(value);
        if(strcmp(keyword,"lon")==0) lon = atof(value);
        if(strcmp(keyword,"frequency")==0) freq = atof(value);
        if(strcmp(keyword,"type")==0) strcpy(type, value);
}

/* populate qrg.txt with frequency of near sonde */
void ShFreqImport::populate(char *id, float lat, float lon, float freq, const char *type) 
{
    //printf(" ID %s:  %.5f, %.5f  f=%.3f, type=%s \n", id, lat, lon, freq, type);
    // Skip if freq already exists
    int stype = stringToStype(type);
    if(stype<0) return;  // unsupported type

    // check if frequency exists already
    // don't do anything if its a static entry
    // update label if its a dynamic SH entry
    int i;
    for(i=0; i<sonde.config.maxsonde; i++) {
	if( abs(sonde.sondeList[i].freq-freq)<0.0015 ) { // exists already, max error 1500 Hz
	    Serial.printf("id %s close to %d\n", id, i);
	    if( sonde.sondeList[i].type == stype) {
		char *l = sonde.sondeList[i].launchsite;
		if( *l=='@' || *l==' ' || *l==0 ) {
		    setLabel(i, id, lat, lon);
		    inuse[i/8] |= (1<<(i&7));
		}
		sonde.sondeList[i].active = 1;
		return;
	    }
	}
    }

    // find slot
    // slots with empty launchsite are considered available for automated entries
    while(ppos < sonde.config.maxsonde) {
	if( *sonde.sondeList[ppos].launchsite==' ' || *sonde.sondeList[ppos].launchsite== 0 ) break;
	ppos++;
    }
    if(ppos >= sonde.config.maxsonde) { 
	Serial.println("populate: out of free slots");
	return;
    } // no more free slots

    sonde.sondeList[ppos].active = 1;
    sonde.sondeList[ppos].freq = freq;
    sonde.sondeList[ppos].type = (SondeType)stype;
    setLabel(ppos, id, lat, lon);
    inuse[ppos/8] |= (1<<(ppos&7));
    ppos++;
}

// clears all remaining automatically filled slots (no longer in SH data)
void ShFreqImport::cleanup() {
    //Serial.println("Cleanup called ********");
    for(int i=0; i<sonde.config.maxsonde; i++) {
	if( (((inuse[i/8]>>(i&7))&1) == 0) && *sonde.sondeList[i].launchsite=='@' ) {
	    Serial.printf("removing #%d\n", i);
	    sonde.sondeList[i].launchsite[0] = 0;
	    sonde.sondeList[i].active = 0;
	    sonde.sondeList[i].freq = 400;
        }
    }
}

#define BUFLEN 128
#define VALLEN 20
int ShFreqImport::handleChar(char c) {
        switch(importState) {
        case START:
                // wait for initial '{'
                if(c=='{') {
                        lat = NAN; lon = NAN; freq = NAN; *type = 0;
                        importState++; 
                }       
                break;
        case BEFOREID:
                // what for first '"' in { "A1234567" : { ... } }; or detect end
                if(c=='"') { idpos = 0; importState++; }
                if(c=='}') { importState = ENDREACHED; }
                break;
        case COPYID:
                // copy ID "A1234567" until second '"' is earched
                if(c=='"') { id[idpos] = 0; importState++; }
                else id[idpos++] = c;
                break;
        case AFTERID:
                // wait for '{' in '"A1234567": { ...'
                if(c=='{') importState++;
                break;
        case BEFOREKEY:
                if(c=='"') { keywordpos = 0; importState++; }
                break;
        case COPYKEY:
                if(c=='"') { importState++; keyword[keywordpos] = 0; /* printf("Key: >%s<\n", keyword);*/ }
                else keyword[keywordpos++] = c;
                break;
        case AFTERKEY:
                if(c==':') {
                        valuepos = 0;
                        quotes = 0;
                        if(strcmp(keyword,"lat")==0 || strcmp(keyword, "lon")==0 || strcmp(keyword, "frequency")==0 )
                                importState = BEFORENUMVAL;
                        else {  
                                if (strcmp(keyword, "type")==0)
                                        importState = BEFORESTRINGVAL;
                                else    
                                        importState = SKIPVAL;
                        }               
                }       
                break;
        case BEFORENUMVAL:
                if( (c>='0'&&c<='9') || c=='-') { value[0] = c; valuepos=1; importState++; }
                break;
        case COPYNUMVAL:
                if( !(c>='0'&&c<='9') && c!='-' && c!='.' ) {
                        value[valuepos]=0; importState=SKIPVAL; usekeyvalue();
                        if(c!=',' && c!='}') break;
                }       
                else { value[valuepos++] = c; break; }
                // intenionall fall-through
        case SKIPVAL:
                // This is rather fragile, we *should* handle more escaping and so on but do not do so so far, only simple quotes
                if(c=='"') quotes = !quotes;
                if(quotes) break; 
                if(c==',') importState = BEFOREKEY;
                if(c=='}') {
                        // we have an ID and all key/value pairs, check if its good....
                        if( !isnan(lat) && !isnan(lon) && !isnan(freq) && type[0] ) {
                                printf("SondeHub import: populate %s %f %f %f %s\n", id, lat, lon, freq, type);
                                populate(id, lat, lon, freq, type);
                        } else {
                                printf("Skipping incomplete %s\n", id);
                        }       
                        importState = ENDORNEXT;
                }       
                break;
        case BEFORESTRINGVAL:
                if(c=='"') importState++;
                break;
        case COPYSTRINGVAL:
                if(c=='"') { importState=SKIPVAL; value[valuepos]=0; usekeyvalue(); }
                else value[valuepos++] = c;
                break;
        case ENDORNEXT:
                // next we have to see either a final "}', or a comma before the next id
                if(c==',') importState = BEFOREID;
                else if (c=='}') { importState = ENDREACHED; cleanup(); return 1; }
                break;
        case ENDREACHED:
                return 1;
        }       
        return 0;
}

// lat lon in deg, dist in km, time in minutes
int ShFreqImport::shImportSendRequest(WiFiClient *client, float lat, float lon, int dist, int time) {
	if(!client->connected()) {
		if(!client->connect(sonde.config.sondehub.host, 80)) {
			Serial.println("Connection FAILED");
			return 1;
		}
	}
	Serial.println("Sending SondeHub import request");
	char req[300];
	snprintf(req, 200, "GET /sondes?lat=%f&lon=%f&distance=%d&last=%d HTTP/1.1\r\n"
		"Host: %s\r\n"
		"Accept: application/json\r\n"
		"Cache-Control: no-cache\r\n\r\n",
		lat, lon, dist*1000, time*60, sonde.config.sondehub.host);
	client->print(req);
	Serial.print(req);
	importState = START;
	homelat = lat;
	homelon = lon;
	memset(inuse, 0, sizeof(inuse));
	ppos = 0;
	return 0;
}

// return 0 if more data should be read (later), 1 if finished (close connection...)
int ShFreqImport::shImportHandleReply(WiFiClient *client) {
	while(client->available()) {
		int res = handleChar(client->read());
		if(res) return res;
	}
	return 0;
}

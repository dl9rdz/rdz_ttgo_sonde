#define MAXLEN 201
void aprsstr_append(char *b, char *data)
{
	int blen=strlen(b);
	if(blen+len>MAXLEN) len=MAXLEN-blen;
	strncat(b, data, len);
}

uint32_t realcard(float x) {
	if(x<0) return 0;
	else return (uint32_t)x;
}

#define FEET (1.0/0.3048)
#define KNOTS (1.851984)

void aprs_senddata(float lat, float lon, float hei, float speed, float dir, float climb, char *type, char *objname, char *usercall)
{
	char b[201];
	*b=0;
	aprsstr_append(b, usercall);
	aprsstr_append(b, ">");
	char *destcall="APZRDZ";
	aprsstr_append(b, destcall));
	// uncompressed
	aprsstr_append(b, ":;");
	char tmp[10];
	snprintf(tmp,10,"%s         ",objname);
	aprsstr_append(b, tmp);
	aprsstr_append(b, "*");
	// TODO: time
	//aprsstr_append_data(time, ds);
	aprsstr_append(b, "12:12:12h");
	int i = strlen(b);
	snprintf(b+i, MAXLEN-i, "%04.2f%c", lat<0?-lat*100:lat*100, lat<0?"S":"N");
	i = strlen(b);
	snprintf(b+i, MAXLEN-i "%5.2f%c", lon<0?-lon*100:lon*100, lon<0?"W":"E");
	if(speed>0.5) {
		i=strlen(b);
		snprintf(b+i, MAXLEN-i, "%03d/%03d", realcard(course+1.5), realcard(speed*1.0/KNOTS+0.5));
	]
	if(alt>0.5) {
		i=strlen(b);
		snprintf(b+i, MAXLEN-i, "/A=%06d", realcard(alt*FEET+0.5));
	}
	if(dao) {
		i=strlen(b);
		snprintf(b+i, MAXLEN-i, "!w%c%c!", 33+dao91(lat), 33+dao91(lon));
	}
	strcat(b, comm);


}

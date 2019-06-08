
typedef struct {
    int frnr;
    char id[11];
    int week; int gpssec;
    int jahr; int monat; int tag;
    int wday;
    int std; int min; float sek;
    double lat; double lon; double alt;
    double vH; double vD; double vU;
    int sats[4];
    double dop;
    int freq;
    unsigned short aux[4];
    double diter;
} gpx_t;

extern gpx_t gpx;

void print_frame(uint8_t *data, int len); 
void get_eph(const char *file);

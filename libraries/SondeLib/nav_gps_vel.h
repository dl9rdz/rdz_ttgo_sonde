

typedef struct {
    uint16_t prn;
    uint16_t week;
    uint32_t toa;
    char   epoch[20];
    double toe;
    double toc;
    double e;
    double delta_n;
    double delta_i;
    double i0;
    double OmegaDot;
    double sqrta;
    double Omega0;
    double w;
    double M0;
    double tgd;
    double idot;
    double cuc;
    double cus;
    double crc;
    double crs;
    double cic;
    double cis;
    double af0;
    double af1;
    double af2;
    int gpsweek;
    uint16_t svn;
    uint8_t  ura;
    uint8_t  health;
    uint8_t  conf;
} EPHEM_t;


typedef struct {
    uint32_t t;
    double pseudorange;
    double pseudorate;
    double clock_corr;
    double clock_drift;
    double X;
    double Y;
    double Z;
    double vX;
    double vY;
    double vZ;
    int ephhr;
    double PR;
    double ephtime;
    int prn;
} SAT_t;


typedef struct {double X; double Y; double Z;} LOC_t;

typedef struct {double  X;  double Y;  double Z;
                double vX; double vY; double vZ;} VEL_t;


double dist(double X1, double Y1, double Z1, double X2, double Y2, double Z2);

void GPS_SatelliteClockCorrection(
  const unsigned short transmission_gpsweek,   // GPS week when signal was transmit (0-1024+)            [weeks]
  const double         transmission_gpstow,    // GPS time of week when signal was transmit              [s]
  const unsigned short ephem_week,             // ephemeris: GPS week (0-1024+)                          [weeks]
  const double         toe,                    // ephemeris: time of week                                [s]
  const double         toc,                    // ephemeris: clock reference time of week                [s]
  const double         af0,                    // ephemeris: polynomial clock correction coefficient     [s],
  const double         af1,                    // ephemeris: polynomial clock correction coefficient     [s/s],
  const double         af2,                    // ephemeris: polynomial clock correction coefficient     [s/s^2]
  const double         ecc,                    // ephemeris: eccentricity of satellite orbit             []
  const double         sqrta,                  // ephemeris: square root of the semi-major axis of orbit [m^(1/2)]
  const double         delta_n,                // ephemeris: mean motion difference from computed value  [rad]
  const double         m0,                     // ephemeris: mean anomaly at reference time              [rad]
  const double         tgd,                    // ephemeris: group delay differential between L1 and L2  [s]
  double*  clock_correction );

void GPS_SatellitePosition_Ephem(
  const unsigned short gpsweek,      // gps week of signal transmission (0-1024+)            [week]
  const double         gpstow,       // time of week of signal transmission  (gpstow-psr/c)  [s]
  EPHEM_t ephem,
  double* clock_correction,  // clock correction for this satellite for this epoch           [m]
  double* satX,              // satellite X position WGS84 ECEF                              [m]
  double* satY,              // satellite Y position WGS84 ECEF                              [m]
  double* satZ               // satellite Z position WGS84 ECEF                              [m]
  );

void GPS_SatelliteClockDriftCorrection(
  const unsigned short transmission_gpsweek,   // GPS week when signal was transmit (0-1024+)            [weeks]
  const double         transmission_gpstow,    // GPS time of week when signal was transmit              [s]
  const unsigned short ephem_week,             // ephemeris: GPS week (0-1024+)                          [weeks]
  const double         toe,                    // ephemeris: time of week                                [s]
  const double         toc,                    // ephemeris: clock reference time of week                [s]
  const double         af0,                    // ephemeris: polynomial clock correction coefficient     [s],
  const double         af1,                    // ephemeris: polynomial clock correction coefficient     [s/s],
  const double         af2,                    // ephemeris: polynomial clock correction coefficient     [s/s^2]
  const double         ecc,                    // ephemeris: eccentricity of satellite orbit             []
  const double         sqrta,                  // ephemeris: square root of the semi-major axis of orbit [m^(1/2)]
  const double         delta_n,                // ephemeris: mean motion difference from computed value  [rad]
  const double         m0,                     // ephemeris: mean anomaly at reference time              [rad]
  const double         tgd,                    // ephemeris: group delay differential between L1 and L2  [s]
  double*  clock_correction,                   // ephemeris: satellite clock correction                  [m]
  double*  clock_drift )  ;

void GPS_SatellitePositionVelocity_Ephem(
  const unsigned short gpsweek,      // gps week of signal transmission (0-1024+)            [week]
  const double         gpstow,       // time of week of signal transmission  (gpstow-psr/c)  [s]
  EPHEM_t ephem,
  double* clock_correction,   // clock correction for this satellite for this epoch           [m]
  double* clock_drift,        // clock correction for this satellite for this epoch           [m]
  double* satX,               // satellite X position WGS84 ECEF                              [m]
  double* satY,               // satellite Y position WGS84 ECEF                              [m]
  double* satZ,               // satellite Z position WGS84 ECEF                              [m]
  double* satvX,              // satellite X velocity WGS84 ECEF                              [m]
  double* satvY,              // satellite Y velocity WGS84 ECEF                              [m]
  double* satvZ               // satellite Z velocity WGS84 ECEF                              [m]
  );


  int NAV_ClosedFormSolution_FromPseudorange(
  SAT_t sats[4],          // input:  satellite position and pseudorange
  double* latitude,       // output: ellipsoid latitude  [rad]
  double* longitude,      //         ellipsoid longitude [rad]
  double* height,         //         ellipsoid height    [m]
  double* rx_clock_bias,  //         receiver clock bias [m]
  double pos_ecef[3] );  

int calc_DOPn(int n, SAT_t satss[], double pos_ecef[3], double DOP[4]);

int NAV_LinP(int N, SAT_t satv[], double pos_ecef[3], double dt,
                    double dpos_ecef[3], double *cc); 
void ecef2elli(double X, double Y, double Z, double *lat, double *lon, double *alt);

int NAV_LinP(int N, SAT_t satv[], double pos_ecef[3], double dt,
                    double dpos_ecef[3], double *cc); 

int NAV_LinV(int N, SAT_t satv[], double pos_ecef[3],
                    double vel_ecef[3], double dt,
                    double dvel_ecef[3], double *cc);

int NAV_bancroft1(int N, SAT_t sats[], double pos_ecef[3], double *cc);

EPHEM_t *read_RNXpephs(const char *file);



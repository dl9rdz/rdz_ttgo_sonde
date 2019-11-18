/*  SPDX-License-Identifier:        GPL-3.0
 *
 *  BCH / Reed-Solomon
 *    encoder()
 *    decoder()  (Euklid. Alg.)
 *
 *
 * based on https://github.com/rs1729/RS/blob/master/meisei/meisei_ims.c
 * author: zilog80
 *

   Meisei:
       bin.BCH(63, 51), t=2
       g(X) = (X^6+X+1)(X^6+X^4+X^2+X+1)
       g(a) = 0 fuer a = alpha^1,...,alpha^4
   Es koennen 2 Fehler korrigiert werden; diese koennen auch direkt mit
       L(x) = 1 + L1 x + L2 x^2, L1=L1(S1), L2=L2(S1,S3)
   gefunden werden. Problem: 3 Fehler und mehr erkennen.
   Auch bei 3 Fehlern ist deg(Lambda)=2 und Lambda hat auch 2 Loesungen.
   Meisei-Bloecke sind auf 46 bit gekuerzt und enthalten 2 parity bits.
   -> Wenn decodierte Bloecke bits in Position 46-63 schalten oder
   einer der parity-checks fehlschlaegt, dann Block nicht korrigierbar.
   Es werden
   - 54% der 3-Fehler-Bloecke erkannt
   - 39% der 3-Fehler-Bloecke werden durch Position/Parity erkannt
   -  7% der 3-Fehler-Bloecke werden falsch korrigiert

 *
 */


/*
//
// --- bch_ecc.h ---
//

typedef unsigned char  ui8_t;
typedef unsigned int  ui32_t;

*/
#include <inttypes.h>

typedef uint8_t ui8_t;
typedef uint32_t ui32_t;


int rs_init_BCH64(void);
int rs_decode_bch_gf2t2(ui8_t cw[], ui8_t *err_pos, ui8_t *err_val);

// ---


#define MAX_DEG 63  // max N-1


typedef struct {
    ui32_t f;
    ui32_t ord;
    ui8_t alpha;
} GF_t;

static GF_t GF/*64BCH*/ = { 0x43,   // BCH-GF(2^6): X^6 + X + 1 : 0x43
                        64,     // 2^6
                        0x02 }; // generator: alpha = X

typedef struct {
    ui8_t N;
    ui8_t t;
    ui8_t R;  // RS: R=2t, BCH: R<=mt
    ui8_t K;  // K=N-R
    ui8_t b;
    ui8_t p; ui8_t ip; // p*ip = 1 mod N
    ui8_t g[MAX_DEG+1];  // ohne g[] eventuell als init_return
} RS_t;


static RS_t RS/*BCH64*/ = {  63,  2, 12,  51, 1, 1, 1, {114, 101, 97, 108, 108, 121, 32, 40, 99, 41, 32, 100, 108, 57, 114, 100, 122, 32, 71, 80, 76, 32, 108, 105, 99, 101, 110, 115, 101, 0}};

static ui8_t exp_a[256],
             log_a[256];


/* --------------------------------------------------------------------------------------------- */

static
int GF_deg(ui32_t p) {
    ui32_t d = 31;
    if (p == 0) return -1;  /* deg(0) = -infty */
    else {
        while (d && !(p & (1<<d)))  d--;  /* d<32, 1L = 1 */
    }
    return d;
}

static
ui8_t GF2m_mul(ui8_t a, ui8_t b) {
    ui32_t aa = a;
    ui8_t ab = 0;
    int i, m = GF_deg(b);

    if (b & 1) ab = a;

    for (i = 0; i < m; i++) {
        aa = (aa << 1);  // a = a * X
        if (GF_deg(aa) == GF_deg(GF.f))
             aa ^= GF.f; // a = a - GF.f
        b >>= 1;
        if (b & 1) ab ^= (ui8_t)aa;        /* b_{i+1} > 0 ? */
    }
    return ab;
}

static
int GF_genTab(GF_t gf, ui8_t expa[], ui8_t loga[]) {
    int i, j;
    ui8_t b;

//    GF.f = f;
//    GF.ord = 1 << GF_deg(GF.f);

    b = 0x01;
    for (i = 0; i < gf.ord; i++) {
        expa[i] = b;         // b_i = a^i
        b = GF2m_mul(gf.alpha, b);
    }

    loga[0] = -00;  // log(0) = -inf
    for (i = 1; i < gf.ord; i++) {
        b = 0x01; j = 0;
        while (b != i) {
            b = GF2m_mul(gf.alpha, b);
            j++;
            if (j > gf.ord-1) {
                return -1;  // a not primitive
            }
        }   // j = log_a(i)
        loga[i] = j;
    }

    return 0;
}

/* --------------------------------------------------------------------------------------------- */

static ui8_t GF_mul(ui8_t p, ui8_t q) {
  ui32_t x;
  if ((p == 0) || (q == 0)) return 0;
  x = (ui32_t)log_a[p] + log_a[q];
  return exp_a[x % (GF.ord-1)];     // a^(ord-1) = 1
}

static ui8_t GF_inv(ui8_t p) {
  if (p == 0) return 0;             // DIV_BY_ZERO
  return exp_a[GF.ord-1-log_a[p]];  // a^(ord-1) = 1
}

/* --------------------------------------------------------------------------------------------- */

/*
 *  p(x) = p[0] + p[1]x + ... + p[N-1]x^(N-1)
 */

static
ui8_t poly_eval(ui8_t poly[], ui8_t x) {
    int n;
    ui8_t xn, y;

    y = poly[0];
    if (x != 0) {
        for (n = 1; n < GF.ord-1; n++) {
            xn = exp_a[(n*log_a[x]) % (GF.ord-1)];
            y ^= GF_mul(poly[n], xn);
        }
    }
    return y;
}

static
ui8_t poly_evalH(ui8_t poly[], ui8_t x) {
    int n;
    ui8_t y;
    y = poly[GF.ord-1];
    for (n = GF.ord-2; n >= 0; n--) {
        y = GF_mul(y, x) ^ poly[n];
    }
    return y;
}


static
int poly_deg(ui8_t p[]) {
    int n = MAX_DEG;
    while (p[n] == 0 && n > 0) n--;
    if (p[n] == 0) n--;  // deg(0) = -inf
    return n;
}

static
int poly_divmod(ui8_t p[], ui8_t q[], ui8_t *d, ui8_t *r) {
  int deg_p, deg_q;            // p(x) = q(x)d(x) + r(x)
  int i;                       //        deg(r) < deg(q)
  ui8_t c;

  deg_p = poly_deg(p);
  deg_q = poly_deg(q);

  if (deg_q < 0) return -1;  // q=0: DIV_BY_ZERO

  for (i = 0; i <= MAX_DEG; i++) d[i] = 0;
  for (i = 0; i <= MAX_DEG; i++) r[i] = 0;


  c = GF_mul( p[deg_p], GF_inv(q[deg_q]));

  if (deg_q == 0) {
      for (i = 0; i <= deg_p;   i++) d[i] = GF_mul(p[i], c);
      for (i = 0; i <= MAX_DEG; i++) r[i] = 0;
  }
  else if (deg_p < 0) {  // p=0
      for (i = 0; i <= MAX_DEG; i++) {
          d[i] = 0;
          r[i] = 0;
      }
  }
  else if (deg_p < deg_q) {
      for (i = 0; i <= MAX_DEG; i++) d[i] = 0;
      for (i = 0; i <= deg_p; i++) r[i] = p[i]; // r(x)=p(x), deg(r)<deg(q)
      for (i = deg_p+1; i <= MAX_DEG; i++) r[i] = 0;
  }
  else {
      for (i = 0; i <= deg_p; i++) r[i] = p[i];
      while (deg_p >= deg_q) {
          d[deg_p-deg_q] = c;
          for (i = 0; i <= deg_q; i++) {
              r[deg_p-i] ^= GF_mul( q[deg_q-i], c);
          }
          while (r[deg_p] == 0 && deg_p > 0) deg_p--;
          if (r[deg_p] == 0) deg_p--;
          if (deg_p >= 0) c = GF_mul( r[deg_p], GF_inv(q[deg_q]));
      }
  }
  return 0;
}

static
int poly_add(ui8_t a[], ui8_t b[], ui8_t *sum) {
    int i;
    ui8_t c[MAX_DEG+1];

    for (i = 0; i <= MAX_DEG; i++) {
            c[i] = a[i] ^ b[i];
    }

    for (i = 0; i <= MAX_DEG; i++) { sum[i] = c[i]; }

    return 0;
}

static
int poly_mul(ui8_t a[], ui8_t b[], ui8_t *ab) {
    int i, j;
    ui8_t c[MAX_DEG+1];

    if (poly_deg(a)+poly_deg(b) > MAX_DEG) {
       return -1;
    }

    for (i = 0; i <= MAX_DEG; i++) { c[i] = 0; }

    for (i = 0; i <= poly_deg(a); i++) {
        for (j = 0; j <= poly_deg(b); j++) {
            c[i+j] ^= GF_mul(a[i], b[j]);
        }
    }

    for (i = 0; i <= MAX_DEG; i++) { ab[i] = c[i]; }

    return 0;
}


static
int polyGF_eggT(int deg, ui8_t a[], ui8_t b[],  // in
                ui8_t *u, ui8_t *v, ui8_t *ggt  // out
               ) {
// deg = 0:
// a(x)u(x)+b(x)v(x) = ggt(x)
// RS:
// deg=t, a(x)=S(x), b(x)=x^2t

    int i;
    ui8_t r0[MAX_DEG+1], r1[MAX_DEG+1], r2[MAX_DEG+1],
          s0[MAX_DEG+1], s1[MAX_DEG+1], s2[MAX_DEG+1],
          t0[MAX_DEG+1], t1[MAX_DEG+1], t2[MAX_DEG+1],
          quo[MAX_DEG+1];

    for (i = 0; i <= MAX_DEG; i++) { u[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { v[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { ggt[i] = 0; }

    for (i = 0; i <= MAX_DEG; i++) { r0[i] = a[i]; }
    for (i = 0; i <= MAX_DEG; i++) { r1[i] = b[i]; }
    s0[0] = 1; for (i = 1; i <= MAX_DEG; i++) { s0[i] = 0; } // s0 = 1
    s1[0] = 0; for (i = 1; i <= MAX_DEG; i++) { s1[i] = 0; } // s1 = 0
    t0[0] = 0; for (i = 1; i <= MAX_DEG; i++) { t0[i] = 0; } // t0 = 0
    t1[0] = 1; for (i = 1; i <= MAX_DEG; i++) { t1[i] = 0; } // t1 = 1
    for (i = 0; i <= MAX_DEG; i++) { r2[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { s2[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { t2[i] = 0; }

    while ( poly_deg(r1) >= deg ) {
        poly_divmod(r0, r1, quo, r2);
        for (i = 0; i <= MAX_DEG; i++) { r0[i] = r1[i]; }
        for (i = 0; i <= MAX_DEG; i++) { r1[i] = r2[i]; }
        poly_mul(quo, s1, s2);
        poly_add(s0, s2, s2);
        for (i = 0; i <= MAX_DEG; i++) { s0[i] = s1[i]; }
        for (i = 0; i <= MAX_DEG; i++) { s1[i] = s2[i]; }
        poly_mul(quo, t1, t2);
        poly_add(t0, t2, t2);
        for (i = 0; i <= MAX_DEG; i++) { t0[i] = t1[i]; }
        for (i = 0; i <= MAX_DEG; i++) { t1[i] = t2[i]; }
    }

    if (deg > 0) {
        for (i = 0; i <= MAX_DEG; i++) { ggt[i] = r1[i]; } // deg=0: r0
        for (i = 0; i <= MAX_DEG; i++) { u[i] = s1[i]; }   // deg=0: s0
        for (i = 0; i <= MAX_DEG; i++) { v[i] = t1[i]; }
    }
    else {
        for (i = 0; i <= MAX_DEG; i++) { ggt[i] = r0[i]; }
        for (i = 0; i <= MAX_DEG; i++) { u[i] = s0[i]; }
        for (i = 0; i <= MAX_DEG; i++) { v[i] = t0[i]; }
    }

    return 0;
}

static
int polyGF_lfsr(int deg, int x2t, ui8_t S[],
                ui8_t *Lambda, ui8_t *Omega ) {
// BCH/RS/LFSR: deg=t+e/2, e=#erasures
// S(x)Lambda(x) = Omega(x) mod x^2t
    int i;
    ui8_t r0[MAX_DEG+1], r1[MAX_DEG+1], r2[MAX_DEG+1],
          s0[MAX_DEG+1], s1[MAX_DEG+1], s2[MAX_DEG+1],
          quo[MAX_DEG+1];

    for (i = 0; i <= MAX_DEG; i++) { Lambda[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { Omega[i] = 0; }

    for (i = 0; i <= MAX_DEG; i++) { r0[i] = S[i]; }
    for (i = 0; i <= MAX_DEG; i++) { r1[i] = 0; } r1[x2t] = 1; //x^2t
    s0[0] = 1; for (i = 1; i <= MAX_DEG; i++) { s0[i] = 0; }
    s1[0] = 0; for (i = 1; i <= MAX_DEG; i++) { s1[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { r2[i] = 0; }
    for (i = 0; i <= MAX_DEG; i++) { s2[i] = 0; }

    while ( poly_deg(r1) >= deg ) { // deg=t+e/2
        poly_divmod(r0, r1, quo, r2);
        for (i = 0; i <= MAX_DEG; i++) { r0[i] = r1[i]; }
        for (i = 0; i <= MAX_DEG; i++) { r1[i] = r2[i]; }

        poly_mul(quo, s1, s2);
        poly_add(s0, s2, s2);
        for (i = 0; i <= MAX_DEG; i++) { s0[i] = s1[i]; }
        for (i = 0; i <= MAX_DEG; i++) { s1[i] = s2[i]; }
    }

// deg > 0:
    for (i = 0; i <= MAX_DEG; i++) { Omega[i]  = r1[i]; }
    for (i = 0; i <= MAX_DEG; i++) { Lambda[i] = s1[i]; }

    return 0;
}

static
int poly_D(ui8_t a[], ui8_t *Da) {
    int i;

    for (i = 0; i <= MAX_DEG; i++) { Da[i] = 0; } // unten werden nicht immer
                                                  // alle Koeffizienten gesetzt
    for (i = 1; i <= poly_deg(a); i++) {
        if (i % 2) Da[i-1] = a[i];   // GF(2^n): b+b=0
    }

    return 0;
}

static
ui8_t forney(ui8_t x, ui8_t Omega[], ui8_t Lambda[]) {
    ui8_t DLam[MAX_DEG+1];
    ui8_t w, z, Y;         //  x=X^(-1), Y = x^(b-1) * Omega(x)/Lambda'(x)
                           //            Y = X^(1-b) * Omega(X^(-1))/Lambda'(X^(-1))
    poly_D(Lambda, DLam);
    w = poly_eval(Omega, x);
    z = poly_eval(DLam, x); if (z == 0) { return -00; }
    Y = GF_mul(w, GF_inv(z));
    if (RS.b == 0) Y = GF_mul(GF_inv(x), Y);
    else if (RS.b > 1) {
        ui8_t xb1 = exp_a[((RS.b-1)*log_a[x]) % (GF.ord-1)];
        Y = GF_mul(xb1, Y);
    }

    return Y;
}

static
int era_sigma(int n, ui8_t era_pos[], ui8_t *sigma) {
    int i;
    ui8_t Xa[MAX_DEG+1], sig[MAX_DEG+1];
    ui8_t a_i;

    for (i = 0; i <= MAX_DEG; i++) sig[i] = 0;
    for (i = 0; i <= MAX_DEG; i++) Xa[i] = 0;

    // sigma(X)=(1 - alpha^j1 X)...(1 - alpha^jn X)
    // j_{i+1} = era_pos[i]
    sig[0] = 1;
    Xa[0] = 1;
    for (i = 0; i < n; i++) { // n <= 2*RS.t
        a_i = exp_a[(RS.p*era_pos[i]) % (GF.ord-1)];
        Xa[1] = a_i;  // Xalp[0..1]: (1-alpha^(j_)X)
        poly_mul(sig, Xa, sig);
    }

    for (i = 0; i <= MAX_DEG; i++) sigma[i] = sig[i];

    return 0;
}

static
int syndromes(ui8_t cw[], ui8_t *S) {
    int i, errors = 0;
    ui8_t a_i;

    // syndromes: e_j=S((alpha^p)^(b+i))  (wie in g(X))
    for (i = 0; i < 2*RS.t; i++) {
        a_i = exp_a[(RS.p*(RS.b+i)) % (GF.ord-1)];  // (alpha^p)^(b+i)
        S[i] = poly_eval(cw, a_i);
        if (S[i]) errors = 1;
    }
    return errors;
}

#if 0
static
int prn_GFpoly(ui32_t p) {
  int i, s;
  s = 0;
  if (p == 0) Serial.println("0");
  else {
    if (p != (p & 0x1FF)) Serial.println(" (& 0x1FF) ");
    for (i=8; i>1; i--) {
      if ( (p>>i) & 1 ) {
        if (s) Serial.println(" + ");
        Serial.printf("X^%d", i);
        s = 1;
      }
    }
    if ( (p>>1) & 1 ) {
      if (s) Serial.printf(" + ");
      Serial.printf("X");
      s = 1;
    }
    if ( p & 1 ) {
      if (s) Serial.printf(" + ");
      Serial.printf("1");
    }
  }
  return 0;
}
#endif

#if 0
static
void prn_table(void) {
    int i;

    Serial.printf("F2[X] mod ");
    prn_GFpoly(GF.f);
    Serial.printf(" : 0x%X\n", GF.f);
    Serial.printf("\n");

    Serial.printf("a^n[%d] = {\n", GF.ord);
    Serial.printf(" ");
    for (i=0; i<GF.ord; i++) {
        Serial.printf(" 0x");
        Serial.printf("%02X", exp_a[i]);
        if (i<GF.ord-1) Serial.printf(","); else printf("}");
        if (i%16 == 15) Serial.printf("\n ");
    }
    Serial.printf("\n");

    Serial.printf("log_a(n)[%d] = {\n", GF.ord);
    Serial.printf(" ");
    Serial.printf(" -oo ,"); // log(0)
    for (i=1; i<GF.ord; i++) {
        Serial.printf(" 0x");
        Serial.printf("%02X", log_a[i]);
        if (i<GF.ord-1) Serial.printf(","); else printf("}");
        if (i%16 == 15) Serial.printf("\n ");
    }
    Serial.printf("\n");
}
#endif

#if 0
int rs_init_RS255() {
    int i, check_gen;
    ui8_t Xalp[MAX_DEG+1];

    GF = GF256RS;
    check_gen = GF_genTab( GF, exp_a, log_a);

    RS = RS256; // N=255, t=12, b=0, p=1
    for (i = 0; i <= MAX_DEG; i++) RS.g[i] = 0;
    for (i = 0; i <= MAX_DEG; i++) Xalp[i] = 0;

    // g(X)=(X-alpha^b)...(X-alpha^(b+2t-1)), b=0
    RS.g[0] = 0x01;
    Xalp[1] = 0x01; // X
    for (i = 0; i < 2*RS.t; i++) {
        Xalp[0] = exp_a[(RS.b+i) % (GF.ord-1)];  // Xalp[0..1]: X - alpha^(b+i)
        poly_mul(RS.g, Xalp, RS.g);
    }

    return check_gen;
}

int rs_init_RS255ccsds() {
    int i, check_gen;
    ui8_t Xalp[MAX_DEG+1];

    GF = GF256RSccsds;
    check_gen = GF_genTab( GF, exp_a, log_a);

    RS = RS256ccsds; // N=255, t=16, b=112, p=11
    for (i = 0; i <= MAX_DEG; i++) RS.g[i] = 0;
    for (i = 0; i <= MAX_DEG; i++) Xalp[i] = 0;

    // beta=alpha^p primitive root of g(X)
    // beta^ip=alpha  // N=255, p=11 -> ip=116
    for (i = 1; i < GF.ord-1; i++) {
        if ( (RS.p * i) % (GF.ord-1) == 1 ) {
            RS.ip = i;
            break;
        }
    }

    // g(X)=(X-(alpha^p)^b)...(X-(alpha^p)^(b+2t-1)), b=112
    RS.g[0] = 0x01;
    Xalp[1] = 0x01; // X
    for (i = 0; i < 2*RS.t; i++) {
        Xalp[0] = exp_a[(RS.p*(RS.b+i)) % (GF.ord-1)];  // Xalp[0..1]: X - (alpha^p)^(b+i)
        poly_mul(RS.g, Xalp, RS.g);
    }
/*
    RS.g[ 0] = RS.g[32] = exp_a[0];
    RS.g[ 1] = RS.g[31] = exp_a[249];
    RS.g[ 2] = RS.g[30] = exp_a[59];
    RS.g[ 3] = RS.g[29] = exp_a[66];
    RS.g[ 4] = RS.g[28] = exp_a[4];
    RS.g[ 5] = RS.g[27] = exp_a[43];
    RS.g[ 6] = RS.g[26] = exp_a[126];
    RS.g[ 7] = RS.g[25] = exp_a[251];
    RS.g[ 8] = RS.g[24] = exp_a[97];
    RS.g[ 9] = RS.g[23] = exp_a[30];
    RS.g[10] = RS.g[22] = exp_a[3];
    RS.g[11] = RS.g[21] = exp_a[213];
    RS.g[12] = RS.g[20] = exp_a[50];
    RS.g[13] = RS.g[19] = exp_a[66];
    RS.g[14] = RS.g[18] = exp_a[170];
    RS.g[15] = RS.g[17] = exp_a[5];
    RS.g[16] = exp_a[24];
*/
    return check_gen;
}
#endif

int rs_init_BCH64() {
    int i, check_gen;

    //GF = GF64BCH;
    check_gen = GF_genTab( GF, exp_a, log_a);

    //RS = BCH64; // N=63, t=2, b=1
    for (i = 0; i <= MAX_DEG; i++) RS.g[i] = 0;

    // g(X)=X^12+X^10+X^8+X^5+X^4+X^3+1
    //     =(X^6+X+1)(X^6+X^4+X^2+X+1)
    RS.g[0] = RS.g[3] = RS.g[4] = RS.g[5] = RS.g[8] = RS.g[10] = RS.g[12] = 1;

    return check_gen;
}

#if 0
int rs_init_RS15ccsds() {
    int i, check_gen;
    ui8_t Xalp[MAX_DEG+1];

    GF = GF16RS;
    check_gen = GF_genTab( GF, exp_a, log_a);

    //RS = RS16_0; // N=15, t=3, b=0, p=1
    RS = RS16ccsds; // N=15, t=2, b=6, p=1
    for (i = 0; i <= MAX_DEG; i++) RS.g[i] = 0;
    for (i = 0; i <= MAX_DEG; i++) Xalp[i] = 0;

    // g(X)=(X-alpha^b)...(X-alpha^(b+2t-1))
    RS.g[0] = 0x01;
    Xalp[1] = 0x01; // X
    for (i = 0; i < 2*RS.t; i++) {
        Xalp[0] = exp_a[(RS.b+i) % (GF.ord-1)];  // Xalp[0..1]: X - alpha^(b+i)
        poly_mul(RS.g, Xalp, RS.g);
    }

    return check_gen;
}
#endif

#if 0
int rs_encode(ui8_t cw[]) {
    int j;
    ui8_t __cw[MAX_DEG+1],
          parity[MAX_DEG+1],
          d[MAX_DEG+1];
    for (j = 0; j <= MAX_DEG; j++) parity[j] = 0;
    for (j = 0; j <= MAX_DEG; j++) __cw[j] = 0;
    for (j = RS.R; j < RS.N; j++) __cw[j] = cw[j];
    poly_divmod(__cw, RS.g, d, parity);
    //if (poly_deg(parity) >= RS.R) return -1;
    for (j = 0; j < RS.R; j++) cw[j] = parity[j];
    return 0;
}
#endif

#if 0
// 2*Errors + Erasure <= 2*t
int rs_decode_ErrEra(ui8_t cw[], int nera, ui8_t era_pos[],
                     ui8_t *err_pos, ui8_t *err_val) {
    ui8_t x, gamma;
    ui8_t S[MAX_DEG+1],
          Lambda[MAX_DEG+1],
          Omega[MAX_DEG+1],
          sigma[MAX_DEG+1],
          sigLam[MAX_DEG+1];
    int deg_sigLam, deg_Lambda, deg_Omega;
    int i, nerr, errera = 0;

    if (nera > 2*RS.t) { return -4; }

    for (i = 0; i < 2*RS.t; i++) { err_pos[i] = 0; }
    for (i = 0; i < 2*RS.t; i++) { err_val[i] = 0; }

    // IF: erasures set 0
    //    for (i = 0; i < nera; i++) cw[era_pos[i]] = 0x00; // erasures
    // THEN: restore cw[era_pos[i]], if errera < 0

    for (i = 0; i <= MAX_DEG; i++) { S[i] = 0; }
    errera = syndromes(cw, S);
    // wenn  S(x)=0 ,  dann poly_divmod(cw, RS.g, d, rem): rem=0

    for (i = 0; i <= MAX_DEG; i++) { sigma[i] = 0; }
    sigma[0] = 1;


    if (nera > 0) {
        era_sigma(nera, era_pos, sigma);
        poly_mul(sigma, S, S);
        for (i = 2*RS.t; i <= MAX_DEG; i++) S[i] = 0; // S = sig*S mod x^2t
    }

    if (errera)
    {
        polyGF_lfsr(RS.t+nera/2, 2*RS.t, S, Lambda, Omega);

        deg_Lambda = poly_deg(Lambda);
        deg_Omega  = poly_deg(Omega);
        if (deg_Omega >= deg_Lambda + nera) {
            errera = -3;
            return errera;
        }
        gamma = Lambda[0];
        if (gamma) {
            for (i = deg_Lambda; i >= 0; i--) Lambda[i] = GF_mul(Lambda[i], GF_inv(gamma));
            for (i = deg_Omega ; i >= 0; i--)  Omega[i] = GF_mul( Omega[i], GF_inv(gamma));
            poly_mul(sigma, Lambda, sigLam);
            deg_sigLam = poly_deg(sigLam);
        }
        else {
            errera = -2;
            return errera;
        }

        nerr = 0; // Errors + Erasures (erasure-pos bereits bekannt)
        for (i = 1; i < GF.ord ; i++) { // Lambda(0)=1
            x = (ui8_t)i;    // roll-over
            if (poly_eval(sigLam, x) == 0) { // Lambda(x)=0 fuer x in erasures[] moeglich
                // error location index
                ui8_t x1 = GF_inv(x);
                err_pos[nerr] = (log_a[x1]*RS.ip) % (GF.ord-1);
                // error value;   bin-BCH: err_val=1
                err_val[nerr] = forney(x, Omega, sigLam);
                //err_val[nerr] == 0, wenn era_val[pos]=0, d.h. cw[pos] schon korrekt
                nerr++;
            }
            if (nerr >= deg_sigLam) break;
        }

        // 2*Errors + Erasure <= 2*t
        if (nerr < deg_sigLam) errera = -1; // uncorrectable errors
        else {
            errera = nerr;
            for (i = 0; i < errera; i++) cw[err_pos[i]] ^= err_val[i];
        }
    }

    return errera;
}

// Errors <= t
int rs_decode(ui8_t cw[], ui8_t *err_pos, ui8_t *err_val) {
    ui8_t tmp[1] = {0};
    return rs_decode_ErrEra(cw, 0, tmp, err_pos, err_val);
}
#endif

int rs_decode_bch_gf2t2(ui8_t cw[], ui8_t *err_pos, ui8_t *err_val) {
// binary 2-error correcting BCH

    ui8_t x, gamma,
          S[MAX_DEG+1],
          L[MAX_DEG+1], L2,
          Lambda[MAX_DEG+1],
          Omega[MAX_DEG+1];
    int i, n, errors = 0;


    for (i = 0; i < RS.t; i++) { err_pos[i] = 0; }
    for (i = 0; i < RS.t; i++) { err_val[i] = 0; }

    for (i = 0; i <= MAX_DEG; i++) { S[i] = 0; }
    errors = syndromes(cw, S);
    // wenn  S(x)=0 ,  dann poly_divmod(cw, RS.g, d, rem): rem=0

    if (errors) {
        polyGF_lfsr(RS.t, 2*RS.t, S, Lambda, Omega);
        gamma = Lambda[0];
        if (gamma) {
            for (i = poly_deg(Lambda); i >= 0; i--) Lambda[i] = GF_mul(Lambda[i], GF_inv(gamma));
            for (i = poly_deg(Omega) ; i >= 0; i--)  Omega[i] = GF_mul( Omega[i], GF_inv(gamma));
        }
        else {
            errors = -2;
            return errors;
        }

        // GF(2)-BCH, t=2:
        // S1 = S[0]
        //   S1^2 = S2 , S2^2 = S4
        // L(x) = 1 + L1 x + L2 x^2  (1-2 errors)
        //   L1 = S1 , L2 = (S3 + S1^3)/S1
        if ( RS.t == 2 ) {

            for (i = 0; i <= MAX_DEG; i++) { L[i] = 0; }
            L[0] = 1;
            L[1] = S[0];
            L2 = GF_mul(S[0], S[0]); L2 = GF_mul(L2, S[0]); L2 ^= S[2];
            L2 = GF_mul(L2, GF_inv(S[0]));
            L[2] = L2;

            if (S[1] != GF_mul(S[0],S[0]) || S[3] != GF_mul(S[1],S[1])) {
                errors = -2;
                return errors;
            }
            if (L[1] != Lambda[1] || L[2] != Lambda[2] ) {
                errors = -2;
                return errors;
            }
        }

        n = 0;
        for (i = 1; i < GF.ord ; i++) { // Lambda(0)=1
            x = (ui8_t)i;    // roll-over
            if (poly_eval(Lambda, x) == 0) {
                // error location index
                err_pos[n] = log_a[GF_inv(x)];
                // error value;   bin-BCH: err_val=1
                err_val[n] = 1; // = forney(x, Omega, Lambda);
                n++;
            }
            if (n >= poly_deg(Lambda)) break;
        }

        if (n < poly_deg(Lambda)) errors = -1; // uncorrectable errors
        else {
            errors = n;
            for (i = 0; i < errors; i++) cw[err_pos[i]] ^= err_val[i];
        }
    }

    return errors;
}


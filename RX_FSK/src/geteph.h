#include <inttypes.h>

#ifndef GETEPH_H
#define GETEPH_H
void geteph();

enum EPHSTATE { EPH_NOTUSED, EPH_PENDING, EPH_TIMEERR, EPH_ERROR, EPH_EPHERROR, EPH_GOOD };

extern uint8_t ephstate;
extern const char *ephtxt[];
#endif

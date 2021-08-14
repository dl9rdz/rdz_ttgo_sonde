#ifndef MAVLINK_H
#define MAVLINK_H

#include <inttypes.h>

#include "Sonde.h"

uint8_t *mavlink_send_hearhbeat(SondeInfo *s);
uint8_t *mavlink_send_vfr_hud(SondeInfo *s);
uint8_t *mavlink_send_gps_set_raw(SondeInfo *s);
uint8_t *mavlink_send_attitude(SondeInfo *s);

#endif
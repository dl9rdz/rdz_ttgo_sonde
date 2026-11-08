/*
 * sonde_hist.h
 * Optional LittleFS last-RX + light-track persistence across reboot.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef sonde_hist_h
#define sonde_hist_h

#include "Sonde.h"

#define SONDE_HIST_MAX 8
#define SONDE_HIST_BUF 10
#define SONDE_HIST_TRACK_MAX 500
#define SONDE_HIST_STALE_SEC 120

/* Call after LittleFS + channel list are ready. Loads last_rx.csv and seeds QRG slots. */
void sondeHistLoad();

/* ~1 Hz on a good decode with ID+position (wifi not required). */
void sondeHistUpdate(SondeInfo *si);

/* Call from main loop; flushes stale entries. */
void sondeHistTick();

/* True if basename is last_rx.csv or lt_*.csv (for allowlisted LittleFS delete). */
bool sondeHistIsManagedFile(const char *basename);

#endif

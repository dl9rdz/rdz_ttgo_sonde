

#ifndef _SCANNER_H
#define _SCANNER_H

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif

/* Last completed scan, for decoder.json (display bins, not every hop sample). */
#define SPECTRUM_MAX_BINS 488
extern int scandisp[SPECTRUM_MAX_BINS];
extern int spectrumLen;
extern float spectrumStartMHz;
extern float spectrumStepMHz;
extern float spectrumPeakMHz;
extern int spectrumPeakRssi;
extern int spectrumPlotMax;	/* full-scale PLOT_SCALE units (= display height / VSCALE) */

class Scanner
{
private:
	void fillTiles(uint8_t *row, int value);

public:	
	void plotResult();
	void scan(void);
};

extern Scanner scanner;
#endif

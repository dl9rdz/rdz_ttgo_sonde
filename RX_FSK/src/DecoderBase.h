
#ifndef DECODER_BASE_H
#define DECODER_BASE_H

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif
#include "Sonde.h"

typedef struct _decoderSetupCfg {
	uint16_t bitrate;
	//uint16_t agcbw;
	//uint16_t rxbw;
	uint8_t rx_cfg;
	uint8_t sync_cfg;
	uint8_t sync_len;
	const uint8_t *sync_data;
	uint8_t preamble_cfg;
} decoderSetupCfg;

/* Generic base class for all sonde decoders */
class DecoderBase
{
protected:

public:
	int setup(decoderSetupCfg &setupcfg, uint16_t agcbw, uint16_t rxbw);
	virtual int setup(float frequency, int type=0) = 0;

	virtual int receive() = 0;
	virtual int waitRXcomplete() = 0;
};


#endif

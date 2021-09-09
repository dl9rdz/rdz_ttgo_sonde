

#ifndef _SCANNER_H
#define _SCANNER_H

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif
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

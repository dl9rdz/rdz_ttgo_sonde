#include <inttypes.h>
#include "gfxfont.h"

#define PROGMEM

#include "FreeMono12pt8b.h"
#include "FreeMono9pt8b.h"
#include "FreeSans12pt8b.h"
#include "FreeSans18pt8b.h"
#include "FreeSans9pt8b.h"
#include "Picopixel.h"
#include "Terminal11x16.h"

__attribute__((used)) const GFXfont * const allfonts[]={
	(const GFXfont *)0x544E4F46,
	&Terminal11x16Font,
	&Terminal11x16Font,
	&FreeSans9pt8b,
	&FreeSans12pt8b,
	&Picopixel,
	&FreeSans18pt8b,
	&FreeMono9pt8b,
	&FreeMono12pt8b,
	0,
};

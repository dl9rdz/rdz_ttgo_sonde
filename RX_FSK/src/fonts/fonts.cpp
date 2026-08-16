#include <inttypes.h>
#include "gfxfont.h"

#define PROGMEM

extern const GFXfont FreeMono12pt7b;
extern const GFXfont FreeMono12pt8b;
extern const GFXfont FreeMono9pt7b;
extern const GFXfont FreeMono9pt8b;
extern const GFXfont FreeSans12pt7b;
extern const GFXfont FreeSans12pt8b;
extern const GFXfont FreeSans18pt7b;
extern const GFXfont FreeSans18pt8b;
extern const GFXfont FreeSans9pt7b;
extern const GFXfont FreeSans9pt8b;
extern const GFXfont JetBrainsMonoNL_Regular10pt8b;
extern const GFXfont JetBrainsMonoNL_Regular12pt8b;
extern const GFXfont JetBrainsMonoNL_Regular9pt8b;
extern const GFXfont Picopixel;
extern const GFXfont Terminal11x16Font;
extern const GFXfont Logo;

// This needs to be first in the object file / .rodata section
__attribute__((used)) const GFXfont * const allfonts[]={
	(const GFXfont *)0x544E4F46,
	&Terminal11x16Font,
	&Terminal11x16Font,
        &Terminal11x16Font,
        &Terminal11x16Font,
	&FreeSans9pt7b,         // 5
        &FreeSans12pt7b,        // 6
        &Picopixel,             // 7
        &FreeSans18pt7b,
	&FreeMono9pt8b,
	&FreeMono12pt8b,
        &Logo,
	0,
};

#include "FreeMono12pt8b.h"
#include "FreeMono9pt8b.h"
#include "FreeSans12pt7b.h"
#include "FreeSans18pt7b.h"
#include "FreeSans9pt7b.h"
#include "Picopixel.h"
#include "Terminal11x16.h"
#include "ttgologo.h"


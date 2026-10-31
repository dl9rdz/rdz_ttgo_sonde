#ifndef DisplayGfxFonts_h
#define DisplayGfxFonts_h

#include <Arduino_GFX_Library.h>

#define MAXFONT 20

struct gfxoffset_t {
	uint8_t yofs, yclear;
};

extern const GFXfont **gfl;
extern int ngfx;
extern struct gfxoffset_t gfxoffsets[MAXFONT];

void init_gfx_fonts();
void calc_gfx_offsets();

#endif

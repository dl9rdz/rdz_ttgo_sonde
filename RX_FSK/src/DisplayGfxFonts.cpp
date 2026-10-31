#include "../features.h"

#define TAG "Display"
#include "logger.h"

#include "DisplayGfxFonts.h"

#include "fonts/FreeMono9pt7b.h"
#include "fonts/FreeMono12pt7b.h"
#include "fonts/FreeSans9pt7b.h"
#include "fonts/FreeSans12pt7b.h"
#include "fonts/FreeSans18pt7b.h"
#include "fonts/Picopixel.h"
#include "fonts/Terminal11x16.h"
#include "fonts/ttgologo.h"

#if LEGACY_FONTS_IN_CODEBIN
const GFXfont *legacygfl[] = {
	&Terminal11x16Font,		// 1 (replacement for 1 with old library)
	&Terminal11x16Font,		// 2 (replacement for 2 with old library)
	&Terminal11x16Font,		// 3 (replacement for 1 or 2 with old library)
	&Terminal11x16Font,		// 4 (replacement for 1 or 2 with old library)
	&FreeSans9pt7b,		// 5
	&FreeSans12pt7b,	// 6
	&Picopixel,             // 7
	&FreeSans18pt7b,        // 8
        &Logo,
};

const GFXfont **gfl = legacygfl;
int ngfx = sizeof(legacygfl)/sizeof(GFXfont *);
#else
// will crash if no font partition exists........
const GFXfont **gfl = NULL;
int ngfx = 0;
#endif


void init_gfx_fonts() {
	// if we have a fonts partition, relocate and use that...
	const esp_partition_t *part;
	esp_partition_mmap_handle_t handle;
	void *data;
	esp_err_t err;

	part = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "fonts");
	if(part) {
		LOG_I(TAG, "FONT partition found, size is %d!", part->size);
		err = esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, (const void **)&data, &handle);
		if( err != ESP_OK ) {
			LOG_E(TAG, "mmap of font partition failed\n");
			return;
		}
		LOG_D(TAG, "font partition successfully mmaped\n");
		// do relocation stuff....
		const GFXfont **fptr = (const GFXfont **)data;
		if((uint32_t)*fptr != 0x544E4F46) { // FONT
			LOG_E(TAG, "No font data in font partition\n");
			return; 
		}
		fptr++;
		int n=0; 
		for(const GFXfont **g=fptr; *g!=NULL; g++) { LOG_I(TAG, "fptr=%p *=%p\n", g, *g); n++; }
		LOG_I(TAG, "There a %d fonts in the font partition\n", n);
		if(n>MAXFONT) n=MAXFONT;
		GFXfont **newgfl = (GFXfont **)malloc( n*sizeof(GFXfont) + MAXFONT*sizeof(GFXfont *)  );
		if(!newgfl) {
			LOG_E(TAG, "no memory for gfx fonts");
			return;
		}
		GFXfont *fonts = (GFXfont *)(((char *)newgfl) + MAXFONT*sizeof(GFXfont *));
		// create GFXfont list
		for(int i=0; i<n; i++) {
			newgfl[i] = fonts+i;
			GFXfont *orig = (GFXfont *)(((char *)data) + (uint32_t)fptr[i]);
			memcpy(newgfl[i], orig, sizeof(GFXfont));
			newgfl[i]->bitmap = newgfl[i]->bitmap + (uint32_t)data;  // relocate bitmap pointer to mmap partition
			newgfl[i]->glyph = (GFXglyph *)( ((char *)newgfl[i]->glyph) + (uint32_t)data );
			LOG_D(TAG, "font %d: gfl[i] is %p,  gfl[i]->bitmap is %p, gfl[i]->glyph is %p\n", i, newgfl[i], newgfl[i]->bitmap, newgfl[i]->glyph);
			LOG_D(TAG, "  font data: first=%d, last=%d, yAdv=%d\n", newgfl[i]->first, newgfl[i]->last, newgfl[i]->yAdvance);
		}
		gfl = (const GFXfont **)newgfl;
		ngfx = n;
	} else {
		LOG_E(TAG, "FATAL ERROR: NO FONT PARTITION FOUND\n");
		while(1);
	}
}

#if 0
///// **** this is now calculated automatically
//
// obtained as max offset from font (last column) and maximum height (3rd column) in glyphs
// first value: offset: max offset from font glyphs (last column * (-1))   (check /, \, `, $)`
// yclear:max height: max of (height in 3rd column) + (yofs + 6th column)  (check j)
const struct gfxoffset_t gfxoffsets[]={
	{ 16, 18},
	{ 16, 18},
	{ 13, 18 },  // 17+13-12 "j" 
	{ 17, 23 }, // 23+17-17
	{  4, 6},       // 6+4-4
	{ 25, 34 },   // 34 25 -25
};
#endif

struct gfxoffset_t gfxoffsets[MAXFONT];

void calc_gfx_offsets() {
	for(int i=0; i<ngfx; i++) {

		// find offset from top to baseline
		int bofs = 0;
		GFXglyph *g = gfl[i]->glyph;
		for(int j=0; j<=gfl[i]->last-gfl[i]->first; j++) {
			if(g[j].yOffset < bofs) bofs = g[j].yOffset;  // yOffset are negative values
		}
		gfxoffsets[i].yofs = -bofs;
		// find max. height (yofs + yOffset + height) -- note that yOffset is negative
		int hgt = 0;
		for(int j=0; j<=gfl[i]->last-gfl[i]->first; j++) {
			int h = gfxoffsets[i].yofs + g[j].yOffset + g[j].height;
			if(h>hgt) hgt=h;
		}
		gfxoffsets[i].yclear = hgt;
///// 	The GFX library does things differently, and if we end up with a smaller baseline then positions might be out of bounds
/////   causing a lack of background on ILI9342.  Make sure that we use a large enough baseline!
	        gfxoffsets[i].yofs = gfl[i]->yAdvance * 2 / 3;
		printf("Font %d: yofs=%d, yclear=%d\n", i, gfxoffsets[i].yofs, gfxoffsets[i].yclear);
	}
}

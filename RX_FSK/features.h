
// Configuration flags for including/excluding fuctionality from the compiled binary
// set flag to 0 for exclude/1 for include

// Selection of data output connectors to be included in firmware
// APRS includes AXUDP (e.g. for aprsmap) and APRS-IS (TCP) (e.g. for wettersonde.net, radiosondy.info)
#define FEATURE_SONDEHUB 1

#define FEATURE_CHASEMAPPER 1
#define FEATURE_MQTT 1
#define FEATURE_SDCARD 0
#define FEATURE_APRS 1


// Additional optional components
#define FEATURE_RS92 1

/* Most recent version support fonts in a dedicated flash parition "fonts".
 * This is incomabtible (in terms of code and flash layout) to previous versions.
 * If LEGACY_FONTS_IN_CODEBIN is sets, fonts are also included in the bin image.
 * This maintains compatibility for OTA with previous versions (in which case the
 * bin image fonts will be used as before).
 * The code automatically uses fonts in flash partition if that exists, otherwise
 * fonts in code.
 * The flash partition fonts support latin15 codeset (instead of 7bit ascii).
 * Also, it is easier to use different fonts :) just flash the font partition w/ something else...
 * This option will likely be removed post-master1.0
 */
#define LEGACY_FONTS_IN_CODEBIN 1


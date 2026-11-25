#ifndef _JSON_H
#define _JSON_H

#include <stddef.h>
#include "Sonde.h"

const char *getType(SondeInfo *si);
int sonde2json(char *buf, int maxlen, SondeInfo *si, bool rssi_as_dbm=false);

/* Escape src as a JSON string body (no surrounding quotes) into dst, always NUL-terminated.
 * '"' and '\\' are backslash-escaped, control characters become \u00XX. An escape sequence is
 * never split: output is truncated before an escape that does not fit. src may be NULL.
 * Returns the number of characters written (excluding NUL). */
size_t json_escape(char *dst, size_t cap, const char *src);

/* Append src as a quoted JSON string at *p (buffer ends at end), advance *p, keep NUL-terminated.
 * Returns false (and leaves *p unchanged) if it does not fit. */
bool json_put_str(char **p, char *end, const char *src);

/* Find string field "name" in a JSON text, starting at *pp, and unescape its value into out
 * (truncated to out_cap-1). Handles \" \\ \/ \b \f \n \r \t and \uXXXX (BMP, encoded as UTF-8).
 * On success advances *pp past the value. Flat lookup: does not track object nesting. */
bool json_get_str(const char **pp, const char *name, char *out, size_t out_cap);

/* Given p pointing at '{', return pointer to the matching '}' (string-aware), or NULL. */
const char *json_object_end(const char *p);

#endif

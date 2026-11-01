/* httpd.h -- ESP-IDF esp_http_server (replaced ESP32AsyncWebServer from earlier version) */
#ifndef RDZ_HTTPD_H
#define RDZ_HTTPD_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Optional %VAR% resolver for LittleFS templates (index/livemap/upd/login).
 * Write replacement into out (NUL-terminated if out_cap>0); return strlen.
 * Unknown vars -> empty string (return 0). If unset, all vars resolve empty.
 */
typedef size_t (*httpd_template_cb)(const char *var, char *out, size_t out_cap);

/** Return non-zero if basename may be written (legacy checkAllowed). */
typedef int (*httpd_allowed_cb)(const char *basename);

/** After a successful LittleFS write via /file (e.g. reload screens*). */
typedef void (*httpd_after_write_cb)(const char *basename);

void httpd_set_template_cb(httpd_template_cb cb);
void httpd_set_allowed_cb(httpd_allowed_cb cb);
void httpd_set_after_write_cb(httpd_after_write_cb cb);

/** Start esp_http_server on :80. Stops any existing instance first. */
void httpd_start_server(void);

/** Stop httpd if running. */
void httpd_stop_server(void);

/**
 * Fields from the last application/x-www-form-urlencoded body parsed by httpd.
 * Pointers refer into that body and stay valid until the next parse, or until
 * the body buffer is overwritten (create*Form reuses message[] after apply).
 */
const char *formGet(const char *name);
int formCount(void);
const char *formNameAt(int i);
const char *formValueAt(int i);

#ifdef __cplusplus
}
#endif

#endif /* RDZ_HTTPD_H */

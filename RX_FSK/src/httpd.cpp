/* httpd.cpp -- ESP-IDF esp_http_server (:80)
 * Replaced ESP32AsyncWebServer in earlier version.
 */
#include "httpd.h"

#include "../features.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <LittleFS.h>
#include <dirent.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "user.h"
#include "posinfo.h"
#include "Startup.h"
#include "../version.h"
#if FEATURE_RS92
#include "geteph.h"
#endif

static const char *TAG = "httpd";
static httpd_handle_t s_server = nullptr;
static httpd_template_cb s_template_cb = nullptr;
static httpd_allowed_cb s_allowed_cb = nullptr;
static httpd_after_write_cb s_after_write_cb = nullptr;

/* Sketch-owned (RX_FSK.ino). */
extern uint8_t defaultUserLevel;
extern char *localUpdates;

#if FEATURE_SDCARD
extern SemaphoreHandle_t globalLock;

#define SPI_MUTEX_LOCK()                                                       \
	do {                                                                   \
	} while (xSemaphoreTake(globalLock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK() xSemaphoreGive(globalLock)
#endif

void httpd_set_template_cb(httpd_template_cb cb)
{
	s_template_cb = cb;
}

void httpd_set_allowed_cb(httpd_allowed_cb cb)
{
	s_allowed_cb = cb;
}

void httpd_set_after_write_cb(httpd_after_write_cb cb)
{
	s_after_write_cb = cb;
}

/* %VAR% substitutions for LittleFS templates (index/livemap/upd/login). */
static size_t httpd_template_resolve(const char *var, char *out, size_t out_cap)
{
	if (out == nullptr || out_cap == 0 || var == nullptr) {
		return 0;
	}
	out[0] = '\0';
	ESP_LOGD(TAG, "%s", var);
	if (strcmp(var, "MAPCENTER") == 0) {
		if (posInfo.valid) {
			snprintf(out, out_cap, "%.8g,%.8g", posInfo.lat, posInfo.lon);
		} else {
			snprintf(out, out_cap, "48,13");
		}
	} else if (strcmp(var, "VERSION_NAME") == 0) {
		snprintf(out, out_cap, "%s", version_name);
	} else if (strcmp(var, "VERSION_ID") == 0) {
		snprintf(out, out_cap, "%s", version_id);
	} else if (strcmp(var, "FULLNAMEID") == 0) {
		snprintf(out, out_cap, "%s-%c%d", version_id, FS_MAJOR + 'A' - 1, FS_MINOR);
	} else if (strcmp(var, "AUTODETECT_INFO") == 0) {
		const char *fpstr;
		int i = 0;
		while (fingerprintValue[i] != sonde.fingerprint && fingerprintValue[i] != -1) {
			i++;
		}
		if (fingerprintValue[i] == -1) {
			fpstr = "Unknown board";
		} else {
			fpstr = fingerprintText[i];
		}
		snprintf(out, out_cap, "Fingerprint %d (%s)", sonde.fingerprint, fpstr);
	} else if (strcmp(var, "EPHSTATE") == 0) {
#if FEATURE_RS92
		snprintf(out, out_cap, "%s", ephtxt[ephstate]);
#else
		snprintf(out, out_cap, "Not supported");
#endif
	} else if (strcmp(var, "LOCAL_UPDATES") == 0) {
		if (localUpdates) {
			snprintf(out, out_cap, "%s", localUpdates);
		}
	} else if (strcmp(var, "PREAUTH") == 0) {
		char preauth[COOKIE_SIZE];
		generateRandomCookie("preauth", preauth);
		storeCookie(preauth, -1); /* preauth value */
		snprintf(out, out_cap, "%s", preauth);
	}
	return strlen(out);
}

static size_t resolve_var(const char *var, char *out, size_t out_cap)
{
	if (out_cap == 0) {
		return 0;
	}
	out[0] = '\0';
	if (var == nullptr) {
		return 0;
	}
	if (s_template_cb != nullptr) {
		return s_template_cb(var, out, out_cap);
	}
	return httpd_template_resolve(var, out, out_cap);
}

static bool write_allowed(const char *basename)
{
	if (s_allowed_cb == nullptr) {
		return true;
	}
	return s_allowed_cb(basename) != 0;
}

static void notify_after_write(const char *basename)
{
	if (s_after_write_cb != nullptr && basename != nullptr) {
		s_after_write_cb(basename);
	}
}

/* Forward declaration, definition is further below: check if current user has required authorization */
static bool require_auth(httpd_req_t *req, int level);

static const char *content_type_for_path(const char *path)
{
	size_t n = strlen(path);
	if (n >= 3 && strcasecmp(path + n - 3, ".js") == 0) {
		return "text/javascript";
	}
	if (n >= 4 && strcasecmp(path + n - 4, ".css") == 0) {
		return "text/css";
	}
	if (n >= 5 && strcasecmp(path + n - 5, ".html") == 0) {
		return "text/html";
	}
	if (n >= 5 && strcasecmp(path + n - 5, ".json") == 0) {
		return "application/json";
	}
	if (n >= 4 && strcasecmp(path + n - 4, ".png") == 0) {
		return "image/png";
	}
	if (n >= 4 && strcasecmp(path + n - 4, ".ico") == 0) {
		return "image/x-icon";
	}
	return "text/html";
}

static const char *content_type_for_basename(const char *name)
{
	size_t n = strlen(name);
	if (n >= 5 && strcasecmp(name + n - 5, ".html") == 0) {
		return "text/html; charset=utf-8";
	}
	if (n >= 4 && strcasecmp(name + n - 4, ".css") == 0) {
		return "text/css; charset=utf-8";
	}
	if (n >= 3 && strcasecmp(name + n - 3, ".js") == 0) {
		return "application/javascript";
	}
	return "text/plain; charset=utf-8";
}

/* Copy req URI path (no query) into out; ensure leading '/'. Reject ".." . */
static bool uri_to_lfs_path(httpd_req_t *req, char *out, size_t out_cap)
{
	const char *uri = req->uri;
	if (uri == nullptr || out_cap < 2) {
		return false;
	}
	if (strstr(uri, "..") != nullptr) {
		return false;
	}
	size_t i = 0;
	if (uri[0] != '/') {
		out[i++] = '/';
	}
	while (uri[0] != '\0' && uri[0] != '?' && i + 1 < out_cap) {
		out[i++] = *uri++;
	}
	out[i] = '\0';
	if (i == 0 || (i == 1 && out[0] == '/')) {
		return false;
	}
	return true;
}

static int hex_nibble(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	return -1;
}

/* Percent-decode src into dst (NUL-terminated). Returns false on overflow. */
static bool url_decode(const char *src, size_t src_n, char *dst, size_t dst_cap)
{
	if (dst_cap == 0) {
		return false;
	}
	size_t o = 0;
	for (size_t i = 0; i < src_n; i++) {
		char c = src[i];
		if (c == '%' && i + 2 < src_n) {
			int hi = hex_nibble(src[i + 1]);
			int lo = hex_nibble(src[i + 2]);
			if (hi >= 0 && lo >= 0) {
				c = (char)((hi << 4) | lo);
				i += 2;
			}
		} else if (c == '+') {
			c = ' ';
		}
		if (o + 1 >= dst_cap) {
			return false;
		}
		dst[o++] = c;
	}
	dst[o] = '\0';
	return true;
}

/* In-place: decode only shrinks, so src and dst may be the same buffer. */
static void url_decode_inplace(char *s)
{
	size_t n = strlen(s);
	(void)url_decode(s, n, s, n + 1);
}

#define FORM_KV_MAX 96
struct FormKV {
	const char *name;
	const char *value;
};
static FormKV form_kv[FORM_KV_MAX];
static int form_n = 0;

static void formClear(void)
{
	form_n = 0;
}

const char *formGet(const char *name)
{
	if (name == nullptr) {
		return nullptr;
	}
	for (int i = 0; i < form_n; i++) {
		if (strcmp(form_kv[i].name, name) == 0) {
			return form_kv[i].value;
		}
	}
	return nullptr;
}

int formCount(void)
{
	return form_n;
}

const char *formNameAt(int i)
{
	return (i >= 0 && i < form_n) ? form_kv[i].name : nullptr;
}

const char *formValueAt(int i)
{
	return (i >= 0 && i < form_n) ? form_kv[i].value : nullptr;
}

/** Parse a mutable NUL-terminated application/x-www-form-urlencoded body. */
static int formParseUrlencodedMutable(char *body)
{
	formClear();
	if (body == nullptr || body[0] == '\0') {
		return 0;
	}
	char *p = body;
	while (*p && form_n < FORM_KV_MAX) {
		char *name = p;
		char *eq = strchr(p, '=');
		char *amp = strchr(p, '&');
		if (eq == nullptr || (amp != nullptr && amp < eq)) {
			if (amp == nullptr) {
				break;
			}
			p = amp + 1;
			continue;
		}
		*eq = '\0';
		char *val = eq + 1;
		if (amp != nullptr) {
			*amp = '\0';
			p = amp + 1;
		} else {
			p += strlen(p);
		}
		url_decode_inplace(name);
		url_decode_inplace(val);
		form_kv[form_n].name = name;
		form_kv[form_n].value = val;
		form_n++;
	}
	return form_n;
}

static bool basename_ok(const char *name)
{
	if (name == nullptr || name[0] == '\0') {
		return false;
	}
	size_t n = strlen(name);
	if (n > 64) {
		return false;
	}
	for (size_t i = 0; i < n; i++) {
		unsigned char c = (unsigned char)name[i];
		if (!(isalnum(c) || c == '.' || c == '_' || c == '-')) {
			return false;
		}
	}
	return true;
}

/* Map /file/<name> or /file/<name>?... -> basename in out. */
static bool basename_from_file_uri(httpd_req_t *req, char *out, size_t out_cap)
{
	const char *uri = req->uri;
	if (uri == nullptr) {
		return false;
	}
	if (strncmp(uri, "/file/", 6) != 0) {
		return false;
	}
	const char *rest = uri + 6;
	size_t n = 0;
	while (rest[n] != '\0' && rest[n] != '?') {
		n++;
	}
	while (n > 0 && rest[0] == '/') {
		rest++;
		n--;
	}
	if (n == 0) {
		return false;
	}
	char decoded[80];
	if (!url_decode(rest, n, decoded, sizeof(decoded))) {
		return false;
	}
	if (!basename_ok(decoded)) {
		return false;
	}
	if (strlen(decoded) + 1 > out_cap) {
		return false;
	}
	strcpy(out, decoded);
	return true;
}

static void lfs_path_for(const char *basename, char *path, size_t path_cap)
{
	snprintf(path, path_cap, "/%s", basename);
}

static esp_err_t send_chunks_raw(httpd_req_t *req, File &file)
{
	char buf[512];
	while (file.available()) {
		int n = file.read((uint8_t *)buf, sizeof(buf));
		if (n < 0) {
			return ESP_FAIL;
		}
		if (n == 0) {
			break;
		}
		esp_err_t err = httpd_resp_send_chunk(req, buf, (size_t)n);
		if (err != ESP_OK) {
			return err;
		}
	}
	return httpd_resp_send_chunk(req, nullptr, 0);
}

/* Stream file with Async-style %VAR% substitution (spans read chunks). */
static esp_err_t send_chunks_template(httpd_req_t *req, File &file)
{
	char inbuf[512];
	char varbuf[48];
	size_t var_len = 0;
	bool in_var = false;
	char repl[160];
	char lit[512];
	size_t lit_len = 0;

	auto flush_lit = [&]() -> esp_err_t {
		if (lit_len == 0) {
			return ESP_OK;
		}
		esp_err_t err = httpd_resp_send_chunk(req, lit, lit_len);
		lit_len = 0;
		return err;
	};

	auto emit_byte = [&](char c) -> esp_err_t {
		if (lit_len >= sizeof(lit)) {
			esp_err_t err = flush_lit();
			if (err != ESP_OK) {
				return err;
			}
		}
		lit[lit_len++] = c;
		return ESP_OK;
	};

	while (file.available()) {
		int n = file.read((uint8_t *)inbuf, sizeof(inbuf));
		if (n < 0) {
			return ESP_FAIL;
		}
		for (int i = 0; i < n; i++) {
			char c = inbuf[i];
			if (!in_var) {
				if (c == '%') {
					esp_err_t err = flush_lit();
					if (err != ESP_OK) {
						return err;
					}
					in_var = true;
					var_len = 0;
				} else {
					esp_err_t err = emit_byte(c);
					if (err != ESP_OK) {
						return err;
					}
				}
			} else if (c == '%') {
				varbuf[var_len] = '\0';
				size_t rlen = resolve_var(varbuf, repl, sizeof(repl));
				/* buf_len==0 ends the chunked response -- skip empty
				 * substitutions (e.g. %LOCAL_UPDATES% when disabled). */
				esp_err_t err = ESP_OK;
				if (rlen > 0) {
					err = httpd_resp_send_chunk(req, repl, (ssize_t)rlen);
				}
				if (err != ESP_OK) {
					return err;
				}
				in_var = false;
				var_len = 0;
			} else if (var_len + 1 < sizeof(varbuf) &&
				   ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) {
				varbuf[var_len++] = c;
			} else {
				esp_err_t err = emit_byte('%');
				if (err != ESP_OK) {
					return err;
				}
				for (size_t j = 0; j < var_len; j++) {
					err = emit_byte(varbuf[j]);
					if (err != ESP_OK) {
						return err;
					}
				}
				err = emit_byte(c);
				if (err != ESP_OK) {
					return err;
				}
				in_var = false;
				var_len = 0;
			}
		}
	}

	esp_err_t err = flush_lit();
	if (err != ESP_OK) {
		return err;
	}
	if (in_var) {
		err = emit_byte('%');
		if (err != ESP_OK) {
			return err;
		}
		for (size_t j = 0; j < var_len; j++) {
			err = emit_byte(varbuf[j]);
			if (err != ESP_OK) {
				return err;
			}
		}
		err = flush_lit();
		if (err != ESP_OK) {
			return err;
		}
	}
	return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t send_lfs(httpd_req_t *req, const char *path, const char *ctype,
			  const char *cache_control, bool with_template)
{
	if (!LittleFS.exists(path)) {
		return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not Found");
	}
	File file = LittleFS.open(path, "r");
	if (!file) {
		return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not Found");
	}

	httpd_resp_set_type(req, ctype);
	if (cache_control != nullptr) {
		httpd_resp_set_hdr(req, "Cache-Control", cache_control);
	}

	esp_err_t err;
	if (with_template) {
		err = send_chunks_template(req, file);
	} else {
		err = send_chunks_raw(req, file);
	}
	file.close();
	return err;
}

static esp_err_t send_no_content(httpd_req_t *req)
{
	httpd_resp_set_status(req, "204 No Content");
	return httpd_resp_send(req, nullptr, 0);
}

static const uint8_t *find_bytes(const uint8_t *hay, size_t hay_n, const uint8_t *needle,
				 size_t needle_n)
{
	if (needle_n == 0 || hay_n < needle_n) {
		return nullptr;
	}
	for (size_t i = 0; i + needle_n <= hay_n; i++) {
		if (memcmp(hay + i, needle, needle_n) == 0) {
			return hay + i;
		}
	}
	return nullptr;
}

/* Extract boundary= from Content-Type (into out, stripped of quotes). */
static bool multipart_boundary(httpd_req_t *req, char *out, size_t out_cap)
{
	char ctype[160];
	if (httpd_req_get_hdr_value_str(req, "Content-Type", ctype, sizeof(ctype)) != ESP_OK) {
		return false;
	}
	const char *p = strstr(ctype, "boundary=");
	if (p == nullptr) {
		return false;
	}
	p += 9;
	if (*p == '"') {
		p++;
	}
	size_t i = 0;
	while (*p && *p != ';' && *p != '"' && *p != ' ' && i + 1 < out_cap) {
		out[i++] = *p++;
	}
	out[i] = '\0';
	return i > 0;
}

/* From multipart header block, copy filename="..." into out (basename only). */
static bool filename_from_part_headers(const uint8_t *hdrs, size_t hdr_n, char *out,
				       size_t out_cap)
{
	static const char key[] = "filename=\"";
	const uint8_t *p = find_bytes(hdrs, hdr_n, (const uint8_t *)key, sizeof(key) - 1);
	if (p == nullptr || out_cap == 0) {
		return false;
	}
	p += sizeof(key) - 1;
	size_t remain = (size_t)(hdrs + hdr_n - p);
	const uint8_t *start = p;
	for (size_t k = 0; k < remain && p[k] != '"'; k++) {
		if (p[k] == '/' || p[k] == '\\') {
			start = p + k + 1;
		}
	}
	size_t len = 0;
	while ((size_t)(start - hdrs) + len < hdr_n && start[len] != '"' && len + 1 < out_cap) {
		out[len] = (char)start[len];
		len++;
	}
	out[len] = '\0';
	return basename_ok(out);
}

static esp_err_t write_raw_body_to_lfs(httpd_req_t *req, const char *basename)
{
	char path[80];
	lfs_path_for(basename, path, sizeof(path));
	File file = LittleFS.open(path, "w");
	if (!file) {
		ESP_LOGE(TAG, "open %s for write failed", path);
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");
	}

	char buf[512];
	int remaining = req->content_len;
	while (remaining > 0) {
		int to_read = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
		int n = httpd_req_recv(req, buf, (size_t)to_read);
		if (n <= 0) {
			if (n == HTTPD_SOCK_ERR_TIMEOUT) {
				continue;
			}
			file.close();
			LittleFS.remove(path);
			return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
		}
		if ((int)file.write((const uint8_t *)buf, (size_t)n) != n) {
			file.close();
			LittleFS.remove(path);
			return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
		}
		remaining -= n;
	}
	file.close();
	ESP_LOGI(TAG, "wrote %s (%d bytes, raw)", path, req->content_len);
	notify_after_write(basename);
	return send_no_content(req);
}

/*
 * Stream multipart body part to LittleFS. Fixed hold buffer only.
 * If basename_inout is empty, fill from Content-Disposition filename=.
 */
static esp_err_t write_multipart_to_lfs(httpd_req_t *req, char *basename_inout, size_t basename_cap)
{
	char boundary[72];
	if (!multipart_boundary(req, boundary, sizeof(boundary))) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no boundary");
	}

	char open_delim[80];
	char close_delim[88];
	int open_n = snprintf(open_delim, sizeof(open_delim), "--%s", boundary);
	int close_n = snprintf(close_delim, sizeof(close_delim), "\r\n--%s", boundary);
	if (open_n <= 0 || close_n <= 0 || (size_t)open_n >= sizeof(open_delim) ||
	    (size_t)close_n >= sizeof(close_delim)) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "boundary too long");
	}

	uint8_t hold[1536];
	size_t hold_n = 0;
	bool headers_done = false;
	File file;
	bool file_open = false;
	char name_local[72];
	name_local[0] = '\0';
	if (basename_inout[0] != '\0') {
		strncpy(name_local, basename_inout, sizeof(name_local) - 1);
		name_local[sizeof(name_local) - 1] = '\0';
	}

	int remaining = req->content_len;
	const size_t overlap = (size_t)close_n > 0 ? (size_t)close_n - 1 : 1;

	auto ensure_file = [&]() -> bool {
		if (file_open) {
			return true;
		}
		if (!basename_ok(name_local)) {
			return false;
		}
		if (!write_allowed(name_local)) {
			ESP_LOGE(TAG, "write %s prohibited", name_local);
			return false;
		}
		char path[80];
		lfs_path_for(name_local, path, sizeof(path));
		file = LittleFS.open(path, "w");
		if (!file) {
			return false;
		}
		file_open = true;
		if (basename_inout[0] == '\0' && basename_cap > 0) {
			strncpy(basename_inout, name_local, basename_cap - 1);
			basename_inout[basename_cap - 1] = '\0';
		}
		return true;
	};

	while (remaining > 0 || hold_n > 0) {
		if (!headers_done) {
			if (hold_n < sizeof(hold) && remaining > 0) {
				size_t space = sizeof(hold) - hold_n;
				int to_read = remaining > (int)space ? (int)space : remaining;
				int n = httpd_req_recv(req, (char *)hold + hold_n, (size_t)to_read);
				if (n <= 0) {
					if (n == HTTPD_SOCK_ERR_TIMEOUT) {
						continue;
					}
					if (file_open) {
						file.close();
					}
					return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
								   "recv failed");
				}
				hold_n += (size_t)n;
				remaining -= n;
			}
			const uint8_t *sep =
				find_bytes(hold, hold_n, (const uint8_t *)"\r\n\r\n", 4);
			if (sep == nullptr) {
				if (hold_n >= sizeof(hold) || remaining <= 0) {
					if (file_open) {
						file.close();
					}
					return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
								   "multipart headers");
				}
				continue;
			}
			size_t hdr_end = (size_t)(sep - hold) + 4;
			if (find_bytes(hold, hdr_end, (const uint8_t *)open_delim, (size_t)open_n) ==
			    nullptr) {
				if (file_open) {
					file.close();
				}
				return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no part");
			}
			if (name_local[0] == '\0') {
				if (!filename_from_part_headers(hold, hdr_end - 4, name_local,
								sizeof(name_local))) {
					return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
								   "no filename");
				}
			}
			if (!ensure_file()) {
				return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "forbidden");
			}
			memmove(hold, hold + hdr_end, hold_n - hdr_end);
			hold_n -= hdr_end;
			headers_done = true;
			continue;
		}

		/* Body: look for close delimiter; keep overlap. */
		if (hold_n < sizeof(hold) && remaining > 0 && hold_n <= overlap + 64) {
			size_t space = sizeof(hold) - hold_n;
			int to_read = remaining > (int)space ? (int)space : remaining;
			int n = httpd_req_recv(req, (char *)hold + hold_n, (size_t)to_read);
			if (n <= 0) {
				if (n == HTTPD_SOCK_ERR_TIMEOUT) {
					continue;
				}
				file.close();
				return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
							   "recv failed");
			}
			hold_n += (size_t)n;
			remaining -= n;
		}

		const uint8_t *found =
			find_bytes(hold, hold_n, (const uint8_t *)close_delim, (size_t)close_n);
		if (found != nullptr) {
			size_t body_n = (size_t)(found - hold);
			if (body_n > 0) {
				if ((size_t)file.write(hold, body_n) != body_n) {
					file.close();
					return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
								   "write failed");
				}
			}
			file.close();
			file_open = false;
			/* Drain remaining request body. */
			while (remaining > 0) {
				char sink[256];
				int to_read = remaining > (int)sizeof(sink) ? (int)sizeof(sink)
									   : remaining;
				int n = httpd_req_recv(req, sink, (size_t)to_read);
				if (n <= 0) {
					break;
				}
				remaining -= n;
			}
			ESP_LOGI(TAG, "wrote /%s (multipart)", name_local);
			notify_after_write(name_local);
			return send_no_content(req);
		}

		if (hold_n > overlap) {
			size_t emit = hold_n - overlap;
			if ((size_t)file.write(hold, emit) != emit) {
				file.close();
				return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
							   "write failed");
			}
			memmove(hold, hold + emit, overlap);
			hold_n = overlap;
		} else if (remaining <= 0) {
			/* EOF without delimiter -- write remainder (lenient). */
			if (hold_n > 0) {
				file.write(hold, hold_n);
			}
			file.close();
			ESP_LOGW(TAG, "multipart ended without boundary; wrote /%s", name_local);
			notify_after_write(name_local);
			return send_no_content(req);
		}
	}

	if (file_open) {
		file.close();
	}
	return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty multipart");
}

static esp_err_t file_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	char name[72];
	if (!basename_from_file_uri(req, name, sizeof(name))) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid path");
	}
	char path[80];
	lfs_path_for(name, path, sizeof(path));
	return send_lfs(req, path, content_type_for_basename(name), nullptr, false);
}

static esp_err_t file_put(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	char name[72];
	if (!basename_from_file_uri(req, name, sizeof(name))) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid path");
	}
	if (!write_allowed(name)) {
		return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "forbidden");
	}
	return write_raw_body_to_lfs(req, name);
}

static esp_err_t file_post(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	char name[72];
	name[0] = '\0';
	bool have_name = basename_from_file_uri(req, name, sizeof(name));

	char boundary[72];
	bool is_mp = multipart_boundary(req, boundary, sizeof(boundary));

	if (is_mp) {
		if (!have_name) {
			/* Legacy POST /file -- name from Content-Disposition. */
			name[0] = '\0';
		}
		return write_multipart_to_lfs(req, name, sizeof(name));
	}

	if (!have_name) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
	}
	if (!write_allowed(name)) {
		return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "forbidden");
	}
	return write_raw_body_to_lfs(req, name);
}

/* Legacy POST /file (no path) -- multipart upload with filename= in part headers. */
static esp_err_t file_post_root(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	char name[72];
	name[0] = '\0';
	return write_multipart_to_lfs(req, name, sizeof(name));
}

static esp_err_t edit_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	/* IDF: static shell; JS fetches/saves via GET/PUT /file/* (no POST /edit.html). */
	return send_lfs(req, "/edit.html", "text/html; charset=utf-8", nullptr, false);
}

/* ---- query / path helpers  ---- */

static bool query_get(httpd_req_t *req, const char *key, char *out, size_t out_cap)
{
	size_t qlen = httpd_req_get_url_query_len(req);
	if (qlen == 0 || out_cap == 0) {
		return false;
	}
	char qbuf[192];
	if (qlen + 1 > sizeof(qbuf)) {
		return false;
	}
	if (httpd_req_get_url_query_str(req, qbuf, qlen + 1) != ESP_OK) {
		return false;
	}
	return httpd_query_key_value(qbuf, key, out, out_cap) == ESP_OK;
}

static bool rel_path_ok(const char *rel)
{
	if (rel == nullptr) {
		return false;
	}
	size_t n = strlen(rel);
	if (n > 80) {
		return false;
	}
	if (strstr(rel, "..") != nullptr) {
		return false;
	}
	return true;
}

/* ---- cookie auth handling (SESSION=...) ---- */

static bool session_from_cookie_hdr(httpd_req_t *req, char *out, size_t out_cap)
{
	char cookie[256];
	if (out_cap == 0) {
		return false;
	}
	out[0] = '\0';
	if (httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof(cookie)) != ESP_OK) {
		return false;
	}
	const char *p = strstr(cookie, "SESSION=");
	if (p == nullptr) {
		return false;
	}
	p += 8;
	size_t i = 0;
	while (*p == ' ') {
		p++;
	}
	while (p[i] != '\0' && p[i] != ';' && i + 1 < out_cap) {
		out[i] = p[i];
		i++;
	}
	out[i] = '\0';
	while (i > 0 && (out[i - 1] == ' ' || out[i - 1] == '\r' || out[i - 1] == '\n')) {
		out[--i] = '\0';
	}
	return i > 0;
}

/** Legacy isAuthenticated: defaultUserLevel or SESSION cookie >= level. */
static bool require_auth(httpd_req_t *req, int level)
{
	if ((int)defaultUserLevel >= level) {
		return true;
	}
	char session[COOKIE_SIZE];
	if (session_from_cookie_hdr(req, session, sizeof(session))) {
		int ulvl = getCookieAuthLevel(session);
		if (ulvl >= level) {
			return true;
		}
	}
	httpd_resp_set_status(req, "401 Unauthorized");
	httpd_resp_set_type(req, "text/plain");
	httpd_resp_send(req, "Permission denied", HTTPD_RESP_USE_STRLEN);
	return false;
}

static const char CLEAR_SESSION_COOKIE[] = "SESSION=; Path=/; SameSite=Strict; Max-Age=0";

/** If SESSION is present but not a live login (reboot / expiry), Set-Cookie to drop it. */
static void maybe_clear_stale_session(httpd_req_t *req)
{
	char session[COOKIE_SIZE];
	if (!session_from_cookie_hdr(req, session, sizeof(session))) {
		return;
	}
	if (getCookieAuthLevel(session) < (int)PERM_RO) {
		httpd_resp_set_hdr(req, "Set-Cookie", CLEAR_SESSION_COOKIE);
	}
}

/* Extract key=value from application/x-www-form-urlencoded body. */
static bool form_get(const char *body, const char *key, char *out, size_t out_cap)
{
	if (body == nullptr || key == nullptr || out_cap == 0) {
		return false;
	}
	size_t key_len = strlen(key);
	const char *p = body;
	while (*p) {
		if ((p == body || p[-1] == '&') && strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
			p += key_len + 1;
			size_t n = 0;
			while (p[n] != '\0' && p[n] != '&') {
				n++;
			}
			return url_decode(p, n, out, out_cap);
		}
		p++;
	}
	return false;
}

static esp_err_t login_html_get(httpd_req_t *req)
{
	return send_lfs(req, "/login.html", "text/html; charset=utf-8", nullptr, true);
}

static esp_err_t login_html_post(httpd_req_t *req)
{
	char body[384];
	int remaining = req->content_len;
	if (remaining <= 0 || remaining >= (int)sizeof(body)) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Request");
	}
	int got = 0;
	while (got < remaining) {
		int n = httpd_req_recv(req, body + got, (size_t)(remaining - got));
		if (n <= 0) {
			if (n == HTTPD_SOCK_ERR_TIMEOUT) {
				continue;
			}
			return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Request");
		}
		got += n;
	}
	body[got] = '\0';

	char username[32];
	char preauth[COOKIE_SIZE];
	char auth[72];
	if (!form_get(body, "user", username, sizeof(username)) ||
	    !form_get(body, "preauth", preauth, sizeof(preauth)) ||
	    !form_get(body, "auth", auth, sizeof(auth))) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Request");
	}

	int ulvl = getUserPermissions(username, preauth, auth);
	if (ulvl > 0) {
		char cookie[COOKIE_SIZE];
		generateRandomCookie(username, cookie);
		if (upgradeCookie(preauth, cookie, (char)ulvl) == 0) {
			char set_cookie[80];
			snprintf(set_cookie, sizeof(set_cookie),
				 "SESSION=%s; Path=/; SameSite=Strict", cookie);
			httpd_resp_set_status(req, "302 Found");
			httpd_resp_set_hdr(req, "Location", "/index.html");
			httpd_resp_set_hdr(req, "Set-Cookie", set_cookie);
			return httpd_resp_send(req, nullptr, 0);
		}
	}
	httpd_resp_set_status(req, "401 Unauthorized");
	httpd_resp_set_type(req, "text/plain");
	return httpd_resp_send(req, "Invalid credentials or session expired",
			       HTTPD_RESP_USE_STRLEN);
}

#if FEATURE_SDCARD

#define FILES_JSON_MAX_SIZE 4096
#define FILES_JSON_MAX_ENTRY 128

static const char *sd_content_type(const char *path)
{
	size_t n = strlen(path);
	if (n >= 4 && strcasecmp(path + n - 4, ".csv") == 0) {
		return "text/plain; charset=utf-8";
	}
	if (n >= 4 && strcasecmp(path + n - 4, ".txt") == 0) {
		return "text/plain; charset=utf-8";
	}
	if (n >= 5 && strcasecmp(path + n - 5, ".json") == 0) {
		return "application/json";
	}
	if (n >= 4 && strcasecmp(path + n - 4, ".log") == 0) {
		return "text/plain; charset=utf-8";
	}
	if (n >= 5 && strcasecmp(path + n - 5, ".html") == 0) {
		return "text/html; charset=utf-8";
	}
	return "application/octet-stream";
}

/* Map /sd/<rel> -> VFS path /sd/<rel> in out. */
static bool vfs_from_sd_uri(httpd_req_t *req, char *out, size_t out_cap)
{
	const char *uri = req->uri;
	if (uri == nullptr || strncmp(uri, "/sd/", 4) != 0) {
		return false;
	}
	const char *rest = uri + 4;
	size_t n = 0;
	while (rest[n] != '\0' && rest[n] != '?') {
		n++;
	}
	char decoded[96];
	if (!url_decode(rest, n, decoded, sizeof(decoded))) {
		return false;
	}
	if (decoded[0] == '\0' || decoded[strlen(decoded) - 1] == '/') {
		return false;
	}
	if (!rel_path_ok(decoded)) {
		return false;
	}
	int w = snprintf(out, out_cap, "/sd/%s", decoded);
	return w > 0 && (size_t)w < out_cap;
}

static bool delete_sd_dir_recursive(const char *vfs_path)
{
	DIR *d;
	SPI_MUTEX_LOCK();
	d = opendir(vfs_path);
	SPI_MUTEX_UNLOCK();
	if (d == nullptr) {
		return false;
	}

	struct dirent *e;
	for (;;) {
		char child[160];
		struct stat st;
		int is_dir = 0;
		int got = 0;

		SPI_MUTEX_LOCK();
		e = readdir(d);
		if (e == nullptr) {
			closedir(d);
			SPI_MUTEX_UNLOCK();
			break;
		}
		if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) {
			SPI_MUTEX_UNLOCK();
			continue;
		}
		snprintf(child, sizeof(child), "%s/%s", vfs_path, e->d_name);
		if (stat(child, &st) == 0) {
			is_dir = S_ISDIR(st.st_mode) ? 1 : 0;
			got = 1;
		}
		SPI_MUTEX_UNLOCK();

		if (!got) {
			continue;
		}
		if (is_dir) {
			if (!delete_sd_dir_recursive(child)) {
				SPI_MUTEX_LOCK();
				closedir(d);
				SPI_MUTEX_UNLOCK();
				return false;
			}
		} else {
			SPI_MUTEX_LOCK();
			int rc = unlink(child);
			SPI_MUTEX_UNLOCK();
			if (rc != 0) {
				SPI_MUTEX_LOCK();
				closedir(d);
				SPI_MUTEX_UNLOCK();
				return false;
			}
		}
	}

	SPI_MUTEX_LOCK();
	int rc = rmdir(vfs_path);
	SPI_MUTEX_UNLOCK();
	return rc == 0;
}

static esp_err_t files_json_flush(httpd_req_t *req, char *chunk, size_t *chunk_len)
{
	if (*chunk_len == 0) {
		return ESP_OK;
	}
	esp_err_t err = httpd_resp_send_chunk(req, chunk, *chunk_len);
	*chunk_len = 0;
	return err;
}

static esp_err_t files_json_get(httpd_req_t *req)
{
	char dir_param[64];
	char subdir[96];
	/* Stack fill buffer -- same size/pattern as sd_file_get (httpd stack is 8 KiB). */
	char chunk[FILES_JSON_MAX_SIZE];
	size_t chunk_len = 0;
	bool first = true;

	dir_param[0] = '\0';
	if (query_get(req, "dir", dir_param, sizeof(dir_param))) {
		char decoded[64];
		if (!url_decode(dir_param, strlen(dir_param), decoded, sizeof(decoded))) {
			httpd_resp_set_type(req, "application/json");
			return httpd_resp_send(req, "[]", 2);
		}
		if (!rel_path_ok(decoded) || strlen(decoded) > 32) {
			httpd_resp_set_status(req, "400 Bad Request");
			httpd_resp_set_type(req, "application/json");
			return httpd_resp_send(req, "[]", 2);
		}
		if (strcmp(decoded, ".int") == 0) {
			strncpy(subdir, "/littlefs/", sizeof(subdir) - 1);
			subdir[sizeof(subdir) - 1] = '\0';
		} else {
			snprintf(subdir, sizeof(subdir), "/sd/%s/", decoded);
		}
	} else {
		strncpy(subdir, "/sd/", sizeof(subdir) - 1);
		subdir[sizeof(subdir) - 1] = '\0';
	}

	/* ttgoconfig pages with start=; we stream the full dir on start=0.
	 * start>0 -> empty page so the client loop terminates. */
	char start_buf[12];
	if (query_get(req, "start", start_buf, sizeof(start_buf))) {
		int start = atoi(start_buf);
		if (start > 0) {
			httpd_resp_set_type(req, "application/json");
			return httpd_resp_send(req, "[]", 2);
		}
	}

	DIR *dir = nullptr;
	SPI_MUTEX_LOCK();
	dir = opendir(subdir);
	SPI_MUTEX_UNLOCK();
	if (dir == nullptr) {
		httpd_resp_set_status(req, "500 Internal Server Error");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_send(req, "[]", 2);
	}

	httpd_resp_set_type(req, "application/json");
	chunk[chunk_len++] = '[';

	for (;;) {
		char d_name_copy[64];
		char entry[FILES_JSON_MAX_ENTRY];
		int is_dir = 0;
		time_t mtime = 0;
		long fsize = 0;
		int got = 0;

		SPI_MUTEX_LOCK();
		struct dirent *dent = readdir(dir);
		if (dent == nullptr) {
			closedir(dir);
			dir = nullptr;
			SPI_MUTEX_UNLOCK();
			break;
		}
		if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
			SPI_MUTEX_UNLOCK();
			continue;
		}
		strncpy(d_name_copy, dent->d_name, sizeof(d_name_copy) - 1);
		d_name_copy[sizeof(d_name_copy) - 1] = '\0';
		char fname[128];
		struct stat attr;
		snprintf(fname, sizeof(fname), "%s%s", subdir, dent->d_name);
		if (stat(fname, &attr) == 0) {
			is_dir = S_ISDIR(attr.st_mode) ? 1 : 0;
			mtime = attr.st_mtime;
			fsize = (long)attr.st_size;
			got = 1;
		}
		SPI_MUTEX_UNLOCK();

		if (!got) {
			continue;
		}

		char ftim[50];
		strftime(ftim, sizeof(ftim), "%Y-%m-%dT%H:%M:%SZ", gmtime(&mtime));
		int n;
		if (is_dir) {
			n = snprintf(entry, sizeof(entry),
				     "%s{\"name\":\"%s\",\"dir\":1,\"ts\":\"%s\"}",
				     first ? "" : ",", d_name_copy, ftim);
		} else {
			n = snprintf(entry, sizeof(entry),
				     "%s{\"name\":\"%s\",\"size\":%ld,\"ts\":\"%s\"}",
				     first ? "" : ",", d_name_copy, fsize, ftim);
		}
		if (n < 0 || (size_t)n >= sizeof(entry)) {
			continue;
		}

		if (chunk_len + (size_t)n > sizeof(chunk)) {
			esp_err_t err = files_json_flush(req, chunk, &chunk_len);
			if (err != ESP_OK) {
				SPI_MUTEX_LOCK();
				closedir(dir);
				SPI_MUTEX_UNLOCK();
				return err;
			}
		}
		memcpy(chunk + chunk_len, entry, (size_t)n);
		chunk_len += (size_t)n;
		first = false;
	}

	if (chunk_len + 1 > sizeof(chunk)) {
		esp_err_t err = files_json_flush(req, chunk, &chunk_len);
		if (err != ESP_OK) {
			return err;
		}
	}
	chunk[chunk_len++] = ']';
	esp_err_t err = files_json_flush(req, chunk, &chunk_len);
	if (err != ESP_OK) {
		return err;
	}
	return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t sd_file_get(httpd_req_t *req)
{
	char vfs[96];
	if (!vfs_from_sd_uri(req, vfs, sizeof(vfs))) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid path");
	}

	SPI_MUTEX_LOCK();
	FILE *f = fopen(vfs, "rb");
	SPI_MUTEX_UNLOCK();
	if (f == nullptr) {
		return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not Found");
	}

	httpd_resp_set_type(req, sd_content_type(vfs));
	/* 4 KiB matches gsnext-rdz SD_STREAM_CHUNK -- fewer SPI/TCP round-trips than 512. */
	char buf[4096];
	for (;;) {
		SPI_MUTEX_LOCK();
		size_t n = fread(buf, 1, sizeof(buf), f);
		SPI_MUTEX_UNLOCK();
		if (n == 0) {
			break;
		}
		esp_err_t err = httpd_resp_send_chunk(req, buf, n);
		if (err != ESP_OK) {
			SPI_MUTEX_LOCK();
			fclose(f);
			SPI_MUTEX_UNLOCK();
			return err;
		}
	}
	SPI_MUTEX_LOCK();
	fclose(f);
	SPI_MUTEX_UNLOCK();
	return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t sdrm_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	char raw[96];
	char decoded[96];
	char vfs[112];

	if (!query_get(req, "file", raw, sizeof(raw))) {
		return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
	}
	if (!url_decode(raw, strlen(raw), decoded, sizeof(decoded))) {
		httpd_resp_set_type(req, "text/html");
		return httpd_resp_send(req, "<html><body>invalid path</body></html>",
				       HTTPD_RESP_USE_STRLEN);
	}
	/* Trim leading slashes */
	const char *rel = decoded;
	while (*rel == '/') {
		rel++;
	}
	if (rel[0] == '\0' || !rel_path_ok(rel)) {
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "text/html");
		return httpd_resp_send(req, "<html><body>invalid path</body></html>",
				       HTTPD_RESP_USE_STRLEN);
	}

	snprintf(vfs, sizeof(vfs), "/sd/%s", rel);

	struct stat st;
	SPI_MUTEX_LOCK();
	int st_rc = stat(vfs, &st);
	SPI_MUTEX_UNLOCK();
	if (st_rc != 0) {
		httpd_resp_set_type(req, "text/html");
		return httpd_resp_send(req, "<html><body>not found</body></html>",
				       HTTPD_RESP_USE_STRLEN);
	}

	bool ok;
	if (S_ISDIR(st.st_mode)) {
		ok = delete_sd_dir_recursive(vfs);
		httpd_resp_set_type(req, "text/html");
		if (ok) {
			return httpd_resp_send(req, "<html><body>ok</body></html>",
					       HTTPD_RESP_USE_STRLEN);
		}
		httpd_resp_set_status(req, "500 Internal Server Error");
		return httpd_resp_send(req, "<html><body>failed to delete folder</body></html>",
				       HTTPD_RESP_USE_STRLEN);
	}

	SPI_MUTEX_LOCK();
	ok = (unlink(vfs) == 0);
	SPI_MUTEX_UNLOCK();
	httpd_resp_set_type(req, "text/html");
	if (ok) {
		return httpd_resp_send(req, "<html><body>ok</body></html>", HTTPD_RESP_USE_STRLEN);
	}
	httpd_resp_set_status(req, "404 Not Found");
	return httpd_resp_send(req, "<html><body>failed to delete</body></html>",
			       HTTPD_RESP_USE_STRLEN);
}

#endif /* FEATURE_SDCARD */

static esp_err_t status_txt_get(httpd_req_t *req)
{
	static const char body[] = "ok\n";
	httpd_resp_set_type(req, "text/plain");
	return httpd_resp_send(req, body, sizeof(body) - 1);
}

static esp_err_t root_get(httpd_req_t *req)
{
	maybe_clear_stale_session(req);
	return send_lfs(req, "/index.html", "text/html", nullptr, true);
}

static esp_err_t index_html_get(httpd_req_t *req)
{
	maybe_clear_stale_session(req);
	return send_lfs(req, "/index.html", "text/html", nullptr, true);
}

static esp_err_t livemap_html_get(httpd_req_t *req)
{
	return send_lfs(req, "/livemap.html", "text/html", nullptr, true);
}

static esp_err_t upd_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	return send_lfs(req, "/upd.html", "text/html", nullptr, true);
}

/* Sketch helpers (RX_FSK.ino) -- return pointer to sketch-owned buffer. */
extern const char *createStatusForm(void);
extern const char *createLiveJson(void);
extern const char *getQRGAsJson(void);
extern const char *createQRGForm(void);
extern Conn *connectors[];
extern const char *createWIFIForm(void);
extern const char *createConfigForm(void);
extern const char *createControlForm(void);
extern const char *httpdHandleQRGPost(void);
extern const char *httpdHandleWIFIPost(void);
extern const char *httpdHandleConfigPost(void);
extern const char *httpdHandleControlPost(void);
extern const char *httpdHandleUpdatePost(void);
extern const char *httpdCreateUpdateForm(int run);
extern const char *httpdCreateKMLLive(void);
extern const char *httpdCreateKMLDynamic(void);
extern const char *httpdCreateGPX(int index);

static esp_err_t send_cstr(httpd_req_t *req, const char *ctype, const char *body)
{
	if (body == nullptr) {
		return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "empty");
	}
	httpd_resp_set_type(req, ctype);
	return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_html_get(httpd_req_t *req)
{
	return send_cstr(req, "text/html", createStatusForm());
}

static esp_err_t live_json_get(httpd_req_t *req)
{
	return send_cstr(req, "application/json", createLiveJson());
}

static esp_err_t qrg_json_get(httpd_req_t *req)
{
	return send_cstr(req, "application/json", getQRGAsJson());
}

/* One chunk per connector, same bytes as the old Async chunked handler.
 * getStatus() is up to ~1 KiB; a single line fits on this stack (httpd stack is 8 KiB). */
static esp_err_t status_json_get(httpd_req_t *req)
{
	httpd_resp_set_type(req, "application/json");
	if (connectors[0] == nullptr) {
		return httpd_resp_send(req, "{}\n", HTTPD_RESP_USE_STRLEN);
	}
	for (int i = 0; connectors[i] != nullptr; i++) {
		char line[1280];
		String name = connectors[i]->getName();
		String st = connectors[i]->getStatus();
		int n = snprintf(line, sizeof(line), "%s\"%s\": \"%s\"\n",
				 i == 0 ? "{" : ",\n", name.c_str(), st.c_str());
		if (n < 0) {
			return ESP_FAIL;
		}
		if ((size_t)n >= sizeof(line)) {
			n = (int)sizeof(line) - 1;
		}
		if (httpd_resp_send_chunk(req, line, (size_t)n) != ESP_OK) {
			return ESP_FAIL;
		}
	}
	if (httpd_resp_send_chunk(req, "}\n", 2) != ESP_OK) {
		return ESP_FAIL;
	}
	return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t recv_post_body(httpd_req_t *req, char *buf, size_t cap)
{
	int remaining = req->content_len;
	if (remaining <= 0 || remaining >= (int)cap) {
		return ESP_ERR_INVALID_SIZE;
	}
	int got = 0;
	while (got < remaining) {
		int n = httpd_req_recv(req, buf + got, (size_t)(remaining - got));
		if (n <= 0) {
			if (n == HTTPD_SOCK_ERR_TIMEOUT) {
				continue;
			}
			return ESP_FAIL;
		}
		got += n;
	}
	buf[got] = '\0';
	return ESP_OK;
}

extern char message[]; /* sketch HTML/POST scratch (RX_FSK.ino) */

static esp_err_t editor_post(httpd_req_t *req, const char *(*apply)(void),
			     const char *(*form)(void), bool need_auth)
{
	if (need_auth && !require_auth(req, 2)) {
		return ESP_OK;
	}
	/* Reuse sketch `message[]` for URL-encoded body; create*Form() overwrites after apply. */
	const size_t cap = 10240 * 3 - 2048;
	esp_err_t err = recv_post_body(req, message, cap);
	if (err != ESP_OK) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
	}
	formParseUrlencodedMutable(message);
	(void)apply();
	return send_cstr(req, "text/html", form());
}

static esp_err_t qrg_html_get(httpd_req_t *req)
{
	/* Legacy: GET is open; POST requires auth. */
	return send_cstr(req, "text/html", createQRGForm());
}

static esp_err_t qrg_html_post(httpd_req_t *req)
{
	return editor_post(req, httpdHandleQRGPost, createQRGForm, true);
}

static esp_err_t wifi_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	return send_cstr(req, "text/html", createWIFIForm());
}

static esp_err_t wifi_html_post(httpd_req_t *req)
{
	return editor_post(req, httpdHandleWIFIPost, createWIFIForm, true);
}

static esp_err_t config_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	return send_cstr(req, "text/html", createConfigForm());
}

static esp_err_t config_html_post(httpd_req_t *req)
{
	return editor_post(req, httpdHandleConfigPost, createConfigForm, true);
}

static esp_err_t control_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	return send_cstr(req, "text/html", createControlForm());
}

static esp_err_t control_html_post(httpd_req_t *req)
{
	return editor_post(req, httpdHandleControlPost, createControlForm, true);
}

static esp_err_t update_html_get(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	return send_cstr(req, "text/html", httpdCreateUpdateForm(0));
}

static esp_err_t update_html_post(httpd_req_t *req)
{
	if (!require_auth(req, 2)) {
		return ESP_OK;
	}
	/* Sets updatePrefix / host and enterMode(ST_UPDATE); reply while OTA starts. */
	static char body[1024];
	esp_err_t err = recv_post_body(req, body, sizeof(body));
	if (err != ESP_OK) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
	}
	formParseUrlencodedMutable(body);
	(void)httpdHandleUpdatePost();
	return send_cstr(req, "text/html", httpdCreateUpdateForm(1));
}

static esp_err_t live_kml_get(httpd_req_t *req)
{
	return send_cstr(req, "application/vnd.google-earth.kml+xml", httpdCreateKMLLive());
}

static esp_err_t dynamic_kml_get(httpd_req_t *req)
{
	return send_cstr(req, "application/vnd.google-earth.kml+xml", httpdCreateKMLDynamic());
}

static esp_err_t options_any(httpd_req_t *req)
{
	httpd_resp_set_status(req, "200 OK");
	return httpd_resp_send(req, nullptr, 0);
}

static bool path_ends_with_ci(const char *path, const char *ext)
{
	size_t n = strlen(path);
	size_t e = strlen(ext);
	if (n < e) {
		return false;
	}
	return strcasecmp(path + n - e, ext) == 0;
}

static bool path_ends_with_gpx(const char *path)
{
	return path_ends_with_ci(path, ".gpx");
}

static bool path_is_direct_static(const char *path)
{
	return path_ends_with_ci(path, ".html") || path_ends_with_ci(path, ".css") ||
	       path_ends_with_ci(path, ".js");
}

static esp_err_t logout_get(httpd_req_t *req)
{
	char session[COOKIE_SIZE];
	if (session_from_cookie_hdr(req, session, sizeof(session))) {
		revokeCookie(session);
	}
	httpd_resp_set_status(req, "302 Found");
	httpd_resp_set_hdr(req, "Location", "/");
	httpd_resp_set_hdr(req, "Set-Cookie", CLEAR_SESSION_COOKIE);
	return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t static_catchall_get(httpd_req_t *req)
{
	char path[128];
	if (!uri_to_lfs_path(req, path, sizeof(path))) {
		return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not Found");
	}
	/* Legacy onNotFound: URLs ending in .gpx -> generated GPX (index after leading '/'). */
	if (path_ends_with_gpx(path)) {
		int index = atoi(path + 1);
		const char *body = httpdCreateGPX(index);
		if (body == nullptr || strcmp(body, "ERROR") == 0) {
			return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad gpx index");
		}
		return send_cstr(req, "application/gpx+xml", body);
	}
	/* Fallback serves public web assets only (no .txt secrets, no templates). */
	if (!path_is_direct_static(path)) {
		return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not Found");
	}
	const char *ctype = content_type_for_path(path);
	const char *cache = path_ends_with_ci(path, ".css") ? "max-age=86400" : "max-age=900";
	return send_lfs(req, path, ctype, cache, false);
}

static const httpd_uri_t uri_status_txt = {
	.uri = "/status.txt",
	.method = HTTP_GET,
	.handler = status_txt_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_root = {
	.uri = "/",
	.method = HTTP_GET,
	.handler = root_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_index = {
	.uri = "/index.html",
	.method = HTTP_GET,
	.handler = index_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_livemap_html = {
	.uri = "/livemap.html",
	.method = HTTP_GET,
	.handler = livemap_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_upd = {
	.uri = "/upd.html",
	.method = HTTP_GET,
	.handler = upd_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_status_html = {
	.uri = "/status.html",
	.method = HTTP_GET,
	.handler = status_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_live_json = {
	.uri = "/live.json",
	.method = HTTP_GET,
	.handler = live_json_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_qrg_json = {
	.uri = "/qrg.json",
	.method = HTTP_GET,
	.handler = qrg_json_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_status_json = {
	.uri = "/status.json",
	.method = HTTP_GET,
	.handler = status_json_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_qrg_html_get = {
	.uri = "/qrg.html",
	.method = HTTP_GET,
	.handler = qrg_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_qrg_html_post = {
	.uri = "/qrg.html",
	.method = HTTP_POST,
	.handler = qrg_html_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_wifi_html_get = {
	.uri = "/wifi.html",
	.method = HTTP_GET,
	.handler = wifi_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_wifi_html_post = {
	.uri = "/wifi.html",
	.method = HTTP_POST,
	.handler = wifi_html_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_config_html_get = {
	.uri = "/config.html",
	.method = HTTP_GET,
	.handler = config_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_config_html_post = {
	.uri = "/config.html",
	.method = HTTP_POST,
	.handler = config_html_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_control_html_get = {
	.uri = "/control.html",
	.method = HTTP_GET,
	.handler = control_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_control_html_post = {
	.uri = "/control.html",
	.method = HTTP_POST,
	.handler = control_html_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_update_html_get = {
	.uri = "/update.html",
	.method = HTTP_GET,
	.handler = update_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_update_html_post = {
	.uri = "/update.html",
	.method = HTTP_POST,
	.handler = update_html_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_live_kml = {
	.uri = "/live.kml",
	.method = HTTP_GET,
	.handler = live_kml_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_dynamic_kml = {
	.uri = "/dynamic.kml",
	.method = HTTP_GET,
	.handler = dynamic_kml_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_edit = {
	.uri = "/edit.html",
	.method = HTTP_GET,
	.handler = edit_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_login_get = {
	.uri = "/login.html",
	.method = HTTP_GET,
	.handler = login_html_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_login_post = {
	.uri = "/login.html",
	.method = HTTP_POST,
	.handler = login_html_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_logout = {
	.uri = "/logout",
	.method = HTTP_GET,
	.handler = logout_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_file_get = {
	.uri = "/file/*",
	.method = HTTP_GET,
	.handler = file_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_file_put = {
	.uri = "/file/*",
	.method = HTTP_PUT,
	.handler = file_put,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_file_post = {
	.uri = "/file/*",
	.method = HTTP_POST,
	.handler = file_post,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_file_post_root = {
	.uri = "/file",
	.method = HTTP_POST,
	.handler = file_post_root,
	.user_ctx = nullptr,
};

#if FEATURE_SDCARD
static const httpd_uri_t uri_files_json = {
	.uri = "/files.json",
	.method = HTTP_GET,
	.handler = files_json_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_sd_file = {
	.uri = "/sd/*",
	.method = HTTP_GET,
	.handler = sd_file_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_sdrm = {
	.uri = "/sdrm.html",
	.method = HTTP_GET,
	.handler = sdrm_html_get,
	.user_ctx = nullptr,
};
#endif

/* Wildcard: match any remaining path (registered after exact URIs). */
static const httpd_uri_t uri_static_any = {
	.uri = "/*",
	.method = HTTP_GET,
	.handler = static_catchall_get,
	.user_ctx = nullptr,
};

static const httpd_uri_t uri_options_any = {
	.uri = "/*",
	.method = HTTP_OPTIONS,
	.handler = options_any,
	.user_ctx = nullptr,
};

void httpd_stop_server(void)
{
	if (s_server == nullptr) {
		return;
	}
	httpd_stop(s_server);
	s_server = nullptr;
	ESP_LOGI(TAG, "HTTP server stopped");
}

void httpd_start_server(void)
{
	httpd_stop_server();

	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.server_port = 80;
	/* Default stack is 4096; SD stream uses a 4 KiB local buf + lwIP -- need headroom. */
	config.stack_size = 8192;
	config.lru_purge_enable = true;
	config.max_uri_handlers = 40;
	config.max_open_sockets = 7;
	config.uri_match_fn = httpd_uri_match_wildcard;

	esp_err_t err = httpd_start(&s_server, &config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
		s_server = nullptr;
		return;
	}

	const httpd_uri_t *handlers[] = {
		&uri_status_txt,   &uri_root,	       &uri_index,
		&uri_livemap_html, &uri_upd,	     &uri_status_html,
		&uri_live_json,	   &uri_qrg_json,      &uri_status_json,     &uri_qrg_html_get,
		&uri_qrg_html_post,&uri_wifi_html_get, &uri_wifi_html_post,  &uri_config_html_get,
		&uri_config_html_post, &uri_control_html_get, &uri_control_html_post,
		&uri_update_html_get, &uri_update_html_post, &uri_live_kml, &uri_dynamic_kml,
		&uri_edit,
		&uri_login_get,	   &uri_login_post,    &uri_logout,
		&uri_file_get,	     &uri_file_put,
		&uri_file_post,	   &uri_file_post_root,
#if FEATURE_SDCARD
		&uri_files_json,   &uri_sd_file,       &uri_sdrm,
#endif
		&uri_static_any,   &uri_options_any,
	};
	for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++) {
		err = httpd_register_uri_handler(s_server, handlers[i]);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "register %s failed: %s", handlers[i]->uri,
				 esp_err_to_name(err));
			httpd_stop_server();
			return;
		}
	}

	ESP_LOGI(TAG, "HTTP server started on :80 (status+/file+/sd)");
}

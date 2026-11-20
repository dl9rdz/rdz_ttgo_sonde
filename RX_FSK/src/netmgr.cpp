/* netmgr.cpp -- Wi-Fi link manager (ESP-IDF). */
#include "netmgr.h"

#include <string.h>
#include <stdlib.h>

#include <LittleFS.h>

#include "Display.h"
#include "logger.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h" /* xTaskGetTickCount, vTaskDelay (Improv scan wait) */
#include "lwip/ip4_addr.h"

#define TAG "netmgr"

#define MAX_WIFI 10
#define SCAN_CAP 16
#define CONNECT_TIMEOUT_MS 10000
#define DISC_GRACE_MS 2000 /* first DISC during connect: pull deadline in by this */
#define DISC_RETRY_DELAY_MS 2000
#define AP_IDLE_TIMEOUT_MS 60000
#define EVT_QUEUE_LEN 16

struct NetEntry {
	const char *ssid;
	const char *pw;
};

enum EvtKind : uint8_t {
	EVT_SCAN_DONE = 1,
	EVT_STA_CONNECTED,
	EVT_STA_DISCONNECTED,
	EVT_GOT_IP,
	EVT_LOST_IP,
	EVT_AP_STA_JOINED,
	EVT_AP_STA_LEFT,
	EVT_CMD_RELOAD,
	EVT_CMD_CONNECT_STA,
	EVT_CMD_SCAN,
	EVT_TICK,
};

struct Evt {
	EvtKind kind;
	int reason;
};

enum Phase : uint8_t {
	PH_IDLE = 0,
	PH_STA_SCAN,
	PH_STA_CONNECT,
	PH_STA_UP,
	PH_AP_ONLY,
	PH_AP_FALLBACK,   /* modes 1/3: APSTA, may scan */
	PH_AP_TIMED,      /* mode 5: AP then return to direct */
	PH_DIRECT,        /* modes 4/5: cycle station entries */
};

struct Target {
	int entry;
	int channel;
	uint8_t bssid[6];
	bool have_bssid;
	int8_t rssi;
};

static NetEntry s_entries[MAX_WIFI];
static int s_n = 0;
static char *s_arena = nullptr;

static int s_mode = 0;
static char s_hostname[32] = {0};
static Phase s_phase = PH_IDLE;
static NetStatus s_status = NetStatus::Down;
static bool s_sta_up = false;
static bool s_ap_up = false;
static bool s_connecting = false;
static bool s_got_disc = false;
static bool s_disc_grace_used = false;
static int s_ap_clients = 0;
static int s_direct_idx = 1;
static int s_direct_ok = -1;
static uint32_t s_connect_t0 = 0;       /* assoc start (wall clock for logs) */
static uint32_t s_connect_deadline = 0; /* when connect attempt times out */
static uint32_t s_ap_idle_t0 = 0;
static uint32_t s_retry_t0 = 0;
static bool s_waiting_retry = false;
static Target s_target;
static bool s_have_target = false;

static esp_netif_t *s_sta_netif = nullptr;
static esp_netif_t *s_ap_netif = nullptr;
static bool s_wifi_inited = false;
static bool s_started = false;

static QueueHandle_t s_evtq = nullptr;
static NetEdge s_pending_edge = NetEdge::None;
static SemaphoreHandle_t s_cred_mu = nullptr;

static void cred_lock()
{
	if (s_cred_mu)
		xSemaphoreTake(s_cred_mu, portMAX_DELAY);
}
static void cred_unlock()
{
	if (s_cred_mu)
		xSemaphoreGive(s_cred_mu);
}

static char s_improv_ssid[33];
static char s_improv_pw[65];
static void (*s_scan_fn)(const char *, int, void *) = nullptr;
static void *s_scan_arg = nullptr;
static SemaphoreHandle_t s_scan_done = nullptr;
/* One-shot: draw scan SSIDs on OLED during startup (modes 1/3). */
static bool s_draw_scan = false;

NetMgr netmgr;

/* ---- credentials -------------------------------------------------------- */

static void clear_entries()
{
	for (int i = 0; i < MAX_WIFI; i++) {
		s_entries[i].ssid = nullptr;
		s_entries[i].pw = nullptr;
	}
	s_n = 0;
	free(s_arena);
	s_arena = nullptr;
}

static void set_default_ap()
{
	clear_entries();
	s_entries[0].ssid = "RDZsonde";
	s_entries[0].pw = "RDZsonde";
	s_n = 1;
}

static int read_pair(File &f, char *ssid, size_t ssid_sz, char *pw, size_t pw_sz)
{
	if (!f.available())
		return -1;
	int n1 = f.readBytesUntil('\n', ssid, ssid_sz - 1);
	ssid[n1] = 0;
	if (n1 > 0 && ssid[n1 - 1] == '\r')
		ssid[--n1] = 0;
	if (!f.available())
		return -1;
	int n2 = f.readBytesUntil('\n', pw, pw_sz - 1);
	pw[n2] = 0;
	if (n2 > 0 && pw[n2 - 1] == '\r')
		pw[--n2] = 0;
	return n1;
}

static bool load_networks()
{
	cred_lock();
	clear_entries();

	File f = LittleFS.open("/networks.txt", "r");
	if (!f) {
		ESP_LOGW(TAG, "networks.txt missing, using default AP");
		set_default_ap();
		cred_unlock();
		return false;
	}

	/* Pass 1: count bytes needed */
	size_t bytes = 0;
	int count = 0;
	char ssidbuf[96];
	char pwbuf[96];
	while (count < MAX_WIFI) {
		int n1 = read_pair(f, ssidbuf, sizeof(ssidbuf), pwbuf, sizeof(pwbuf));
		if (n1 < 0)
			break;
		if (n1 == 0)
			continue;
		bytes += (size_t)n1 + 1 + strlen(pwbuf) + 1;
		count++;
	}
	f.close();

	if (count == 0) {
		set_default_ap();
		cred_unlock();
		return false;
	}

	s_arena = (char *)malloc(bytes);
	if (!s_arena) {
		ESP_LOGE(TAG, "arena alloc failed (%u)", (unsigned)bytes);
		set_default_ap();
		cred_unlock();
		return false;
	}

	/* Pass 2: copy into arena */
	f = LittleFS.open("/networks.txt", "r");
	if (!f) {
		clear_entries();
		set_default_ap();
		cred_unlock();
		return false;
	}
	char *p = s_arena;
	s_n = 0;
	while (s_n < MAX_WIFI) {
		int n1 = read_pair(f, ssidbuf, sizeof(ssidbuf), pwbuf, sizeof(pwbuf));
		if (n1 < 0)
			break;
		if (n1 == 0)
			continue;
		size_t n2 = strlen(pwbuf);
		memcpy(p, ssidbuf, (size_t)n1 + 1);
		s_entries[s_n].ssid = p;
		p += n1 + 1;
		memcpy(p, pwbuf, n2 + 1);
		s_entries[s_n].pw = p;
		p += n2 + 1;
		s_n++;
	}
	f.close();

	ESP_LOGI(TAG, "loaded %d networks (%u bytes)", s_n, (unsigned)bytes);
	for (int i = 0; i < s_n; i++) {
		ESP_LOGI(TAG, "  [%d] %s", i, s_entries[i].ssid);
	}
	cred_unlock();
	return true;
}

/* ---- logging helpers ---------------------------------------------------- */

static uint32_t connect_elapsed_ms()
{
	return (uint32_t)((xTaskGetTickCount() - s_connect_t0) * portTICK_PERIOD_MS);
}

static const char *disc_reason_str(int reason)
{
	switch (reason) {
	case WIFI_REASON_UNSPECIFIED:
		return "UNSPECIFIED";
	case WIFI_REASON_AUTH_EXPIRE:
		return "AUTH_EXPIRE";
	case WIFI_REASON_AUTH_LEAVE:
		return "AUTH_LEAVE";
	case WIFI_REASON_ASSOC_EXPIRE:
		return "ASSOC_EXPIRE";
	case WIFI_REASON_ASSOC_TOOMANY:
		return "ASSOC_TOOMANY";
	case WIFI_REASON_NOT_AUTHED:
		return "NOT_AUTHED";
	case WIFI_REASON_NOT_ASSOCED:
		return "NOT_ASSOCED";
	case WIFI_REASON_ASSOC_LEAVE:
		return "ASSOC_LEAVE";
	case WIFI_REASON_ASSOC_NOT_AUTHED:
		return "ASSOC_NOT_AUTHED";
	case WIFI_REASON_DISASSOC_PWRCAP_BAD:
		return "DISASSOC_PWRCAP_BAD";
	case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
		return "DISASSOC_SUPCHAN_BAD";
	case WIFI_REASON_IE_INVALID:
		return "IE_INVALID";
	case WIFI_REASON_MIC_FAILURE:
		return "MIC_FAILURE";
	case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
		return "4WAY_HANDSHAKE_TIMEOUT";
	case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
		return "GROUP_KEY_UPDATE_TIMEOUT";
	case WIFI_REASON_IE_IN_4WAY_DIFFERS:
		return "IE_IN_4WAY_DIFFERS";
	case WIFI_REASON_GROUP_CIPHER_INVALID:
		return "GROUP_CIPHER_INVALID";
	case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
		return "PAIRWISE_CIPHER_INVALID";
	case WIFI_REASON_AKMP_INVALID:
		return "AKMP_INVALID";
	case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
		return "UNSUPP_RSN_IE_VERSION";
	case WIFI_REASON_INVALID_RSN_IE_CAP:
		return "INVALID_RSN_IE_CAP";
	case WIFI_REASON_802_1X_AUTH_FAILED:
		return "802_1X_AUTH_FAILED";
	case WIFI_REASON_CIPHER_SUITE_REJECTED:
		return "CIPHER_SUITE_REJECTED";
	case WIFI_REASON_BEACON_TIMEOUT:
		return "BEACON_TIMEOUT";
	case WIFI_REASON_NO_AP_FOUND:
		return "NO_AP_FOUND";
	case WIFI_REASON_AUTH_FAIL:
		return "AUTH_FAIL";
	case WIFI_REASON_ASSOC_FAIL:
		return "ASSOC_FAIL";
	case WIFI_REASON_HANDSHAKE_TIMEOUT:
		return "HANDSHAKE_TIMEOUT";
	case WIFI_REASON_CONNECTION_FAIL:
		return "CONNECTION_FAIL";
	case WIFI_REASON_AP_TSF_RESET:
		return "AP_TSF_RESET";
	case WIFI_REASON_ROAMING:
		return "ROAMING";
	default:
		return "?";
	}
}

/* ---- edges / status ----------------------------------------------------- */

static void post_edge(NetEdge e)
{
	/* Same thread as poll(); last edge wins if several happen in one process(). */
	s_pending_edge = e;
}

static void set_status(NetStatus st)
{
	NetStatus prev = s_status;
	s_status = st;
	bool was_up = (prev == NetStatus::Uplink || prev == NetStatus::Ap);
	bool now_up = (st == NetStatus::Uplink || st == NetStatus::Ap);
	if (!was_up && now_up)
		post_edge(NetEdge::ServicesUp);
	else if (was_up && !now_up)
		post_edge(NetEdge::ServicesDown);
}

static void post_evt(EvtKind kind, int reason = 0)
{
	if (!s_evtq)
		return;
	Evt e = { kind, reason };
	xQueueSend(s_evtq, &e, 0);
}

/* ---- IDF Wi-Fi helpers -------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
			       void *data)
{
	(void)arg;
	if (base == WIFI_EVENT) {
		switch (id) {
		case WIFI_EVENT_SCAN_DONE:
			post_evt(EVT_SCAN_DONE);
			break;
		case WIFI_EVENT_STA_CONNECTED:
			post_evt(EVT_STA_CONNECTED);
			break;
		case WIFI_EVENT_STA_DISCONNECTED: {
			wifi_event_sta_disconnected_t *d =
				(wifi_event_sta_disconnected_t *)data;
			post_evt(EVT_STA_DISCONNECTED, d ? d->reason : 0);
			break;
		}
		case WIFI_EVENT_AP_STACONNECTED:
			post_evt(EVT_AP_STA_JOINED);
			break;
		case WIFI_EVENT_AP_STADISCONNECTED:
			post_evt(EVT_AP_STA_LEFT);
			break;
		default:
			break;
		}
	} else if (base == IP_EVENT) {
		if (id == IP_EVENT_STA_GOT_IP)
			post_evt(EVT_GOT_IP);
		else if (id == IP_EVENT_STA_LOST_IP)
			post_evt(EVT_LOST_IP);
	}
}

static void ensure_wifi()
{
	if (s_wifi_inited)
		return;

	ESP_ERROR_CHECK(esp_netif_init());
	esp_err_t err = esp_event_loop_create_default();
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
		ESP_ERROR_CHECK(err);

	s_sta_netif = esp_netif_create_default_wifi_sta();
	s_ap_netif = esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr,
		nullptr));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr,
		nullptr));

	if (s_hostname[0] && s_sta_netif)
		esp_netif_set_hostname(s_sta_netif, s_hostname);

	s_wifi_inited = true;
}

static void set_sta_auth_open()
{
	wifi_config_t cfg = {};
	esp_wifi_get_config(WIFI_IF_STA, &cfg);
	cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
	esp_wifi_set_config(WIFI_IF_STA, &cfg);
}

static void configure_ap_ip()
{
	if (!s_ap_netif)
		return;
	esp_netif_dhcps_stop(s_ap_netif);
	esp_netif_ip_info_t ip = {};
	IP4_ADDR(&ip.ip, 192, 168, 4, 1);
	IP4_ADDR(&ip.gw, 192, 168, 4, 1);
	IP4_ADDR(&ip.netmask, 255, 255, 255, 0);
	esp_netif_set_ip_info(s_ap_netif, &ip);
	esp_netif_dhcps_start(s_ap_netif);
}

static void start_softap(wifi_mode_t mode)
{
	ensure_wifi();
	const char *ssid = (s_n > 0 && s_entries[0].ssid) ? s_entries[0].ssid
							  : "RDZsonde";
	const char *pw = (s_n > 0 && s_entries[0].pw) ? s_entries[0].pw
						      : "RDZsonde";

	wifi_config_t ap = {};
	strncpy((char *)ap.ap.ssid, ssid, sizeof(ap.ap.ssid) - 1);
	ap.ap.ssid_len = strlen(ssid);
	strncpy((char *)ap.ap.password, pw, sizeof(ap.ap.password) - 1);
	ap.ap.channel = 1;
	ap.ap.max_connection = 4;
	ap.ap.authmode =
		(strlen(pw) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

	esp_wifi_set_mode(mode);
	esp_wifi_set_config(WIFI_IF_AP, &ap);
	esp_wifi_start();
	configure_ap_ip();
	if (s_hostname[0] && s_ap_netif)
		esp_netif_set_hostname(s_ap_netif, s_hostname);

	s_ap_up = true;
	s_ap_clients = 0;
	s_ap_idle_t0 = xTaskGetTickCount();
	set_status(NetStatus::Ap);
	ESP_LOGI(TAG, "softAP '%s' up (mode=%d)", ssid, (int)mode);
}

static void stop_softap()
{
	if (!s_ap_up)
		return;
	esp_wifi_set_mode(WIFI_MODE_STA);
	s_ap_up = false;
	s_ap_clients = 0;
	if (s_status == NetStatus::Ap)
		set_status(NetStatus::Down);
	ESP_LOGI(TAG, "softAP down");
}

static void do_sta_connect(int entry, int channel, const uint8_t *bssid,
			   bool have_bssid)
{
	if (entry < 1 || entry >= s_n || !s_entries[entry].ssid)
		return;
	ensure_wifi();
	esp_wifi_set_mode(s_ap_up ? WIFI_MODE_APSTA : WIFI_MODE_STA);
	esp_wifi_start();

	wifi_config_t cfg = {};
	strncpy((char *)cfg.sta.ssid, s_entries[entry].ssid,
		sizeof(cfg.sta.ssid) - 1);
	if (s_entries[entry].pw)
		strncpy((char *)cfg.sta.password, s_entries[entry].pw,
			sizeof(cfg.sta.password) - 1);
	cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
	if (channel > 0)
		cfg.sta.channel = (uint8_t)channel;
	if (have_bssid) {
		memcpy(cfg.sta.bssid, bssid, 6);
		cfg.sta.bssid_set = true;
	}
	esp_wifi_set_config(WIFI_IF_STA, &cfg);
	esp_wifi_connect();

	s_connecting = true;
	s_got_disc = false;
	s_connect_t0 = xTaskGetTickCount();
	s_connect_deadline = s_connect_t0 + pdMS_TO_TICKS(CONNECT_TIMEOUT_MS);
	s_target.entry = entry;
	s_have_target = true;
	if (s_status != NetStatus::Ap)
		set_status(NetStatus::Connecting);
	if (have_bssid && bssid) {
		LOG_I(TAG,
		      "assoc start [%d] %s ch=%d BSSID %02X:%02X:%02X:%02X:%02X:%02X",
		      entry, s_entries[entry].ssid, channel, bssid[0], bssid[1],
		      bssid[2], bssid[3], bssid[4], bssid[5]);
	} else {
		LOG_I(TAG, "assoc start [%d] %s (no BSSID/ch lock)", entry,
		      s_entries[entry].ssid);
	}
}

static void do_sta_disconnect()
{
	esp_wifi_disconnect();
	s_connecting = false;
	s_sta_up = false;
}

static void on_got_ip()
{
	esp_netif_dns_info_t dns = {};
	dns.ip.type = ESP_IPADDR_TYPE_V4;
	dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(8, 8, 8, 8);
	if (s_sta_netif)
		esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_FALLBACK,
				       &dns);

	s_sta_up = true;
	s_connecting = false;
	s_got_disc = false;
	s_disc_grace_used = false;
	s_direct_ok = s_have_target ? s_target.entry : s_direct_ok;
	set_status(NetStatus::Uplink);
	char ipbuf[24] = "?";
	if (s_sta_netif) {
		esp_netif_ip_info_t ip;
		if (esp_netif_get_ip_info(s_sta_netif, &ip) == ESP_OK)
			snprintf(ipbuf, sizeof(ipbuf), IPSTR, IP2STR(&ip.ip));
	}
	LOG_I(TAG, "got IP %s (%u ms)", ipbuf,
	      (unsigned)connect_elapsed_ms());
}

/* ---- scan (stack only) -------------------------------------------------- */

static int find_entry(const char *ssid)
{
	if (!ssid)
		return -1;
	for (int i = 1; i < s_n; i++) {
		if (s_entries[i].ssid && strcmp(s_entries[i].ssid, ssid) == 0)
			return i;
	}
	return -1;
}

static void draw_scan_on_display(const wifi_ap_record_t *rec, uint16_t n)
{
	if (!disp.rdis)
		return;
	uint8_t dispw, disph, dispxs, dispys;
	disp.rdis->getDispSize(&disph, &dispw, &dispys, &dispxs);
	if (dispys == 0)
		return;
	int dwidth = dispw / dispxs;
	int rows = disph / dispys;
	int lastl = (rows > 2) ? (rows - 2) * dispys : 0;
	int line = 0;
	for (uint16_t i = 0; i < n; i++) {
		disp.rdis->drawString(0, dispys * (1 + line),
				      (const char *)rec[i].ssid, dwidth);
		line = (line + 1) % rows;
	}
	if (s_have_target && s_target.entry >= 0 &&
	    s_target.entry < s_n && s_entries[s_target.entry].ssid) {
		disp.rdis->drawString(0, lastl, "Conn:");
		disp.rdis->drawString(6 * dispxs, lastl,
				      s_entries[s_target.entry].ssid);
	}
}

static void handle_scan_done()
{
	uint16_t n = SCAN_CAP;
	wifi_ap_record_t rec[SCAN_CAP];
	esp_err_t err = esp_wifi_scan_get_ap_records(&n, rec);
	if (err != ESP_OK) {
		n = 0;
	}

	LOG_I(TAG, "scan done: %u AP(s)", (unsigned)n);
	for (uint16_t i = 0; i < n; i++) {
		const uint8_t *b = rec[i].bssid;
		LOG_D(TAG,
		      "scan %s RSSI %d, MAC %02X:%02X:%02X:%02X:%02X:%02X, ch %d",
		      (const char *)rec[i].ssid, (int)rec[i].rssi, b[0],
		      b[1], b[2], b[3], b[4], b[5], (int)rec[i].primary);
	}

	if (s_scan_fn) {
		for (uint16_t i = 0; i < n; i++) {
			s_scan_fn((const char *)rec[i].ssid, rec[i].rssi,
				  s_scan_arg);
		}
		s_scan_fn = nullptr;
		s_scan_arg = nullptr;
		if (s_scan_done)
			xSemaphoreGive(s_scan_done);
		return;
	}

	/* Pick strongest configured station entry */
	s_have_target = false;
	int best_rssi = -127;
	for (uint16_t i = 0; i < n; i++) {
		int e = find_entry((const char *)rec[i].ssid);
		if (e < 0)
			continue;
		if (rec[i].rssi <= best_rssi)
			continue;
		best_rssi = rec[i].rssi;
		s_target.entry = e;
		s_target.channel = rec[i].primary;
		memcpy(s_target.bssid, rec[i].bssid, 6);
		s_target.have_bssid = true;
		s_target.rssi = rec[i].rssi;
		s_have_target = true;
	}

	if (s_have_target) {
		const uint8_t *b = s_target.bssid;
		LOG_I(TAG,
		      "scan pick [%d] %s RSSI %d ch %d BSSID %02X:%02X:%02X:%02X:%02X:%02X",
		      s_target.entry, s_entries[s_target.entry].ssid,
		      (int)s_target.rssi, s_target.channel, b[0], b[1], b[2],
		      b[3], b[4], b[5]);
	} else {
		LOG_I(TAG, "scan: no configured network in range");
	}

	if (s_draw_scan) {
		draw_scan_on_display(rec, n);
		s_draw_scan = false;
	}
}

static void start_scan()
{
	ensure_wifi();
	wifi_mode_t mode;
	esp_wifi_get_mode(&mode);
	if (mode == WIFI_MODE_NULL || mode == WIFI_MODE_AP) {
		esp_wifi_set_mode(s_ap_up ? WIFI_MODE_APSTA : WIFI_MODE_STA);
		esp_wifi_start();
	}
	wifi_scan_config_t sc = {};
	sc.show_hidden = true;
	esp_wifi_scan_start(&sc, false);
}

/* ---- policy ------------------------------------------------------------- */

static int next_direct_entry(int from)
{
	if (s_n < 2)
		return -1;
	int i = from;
	for (int k = 0; k < s_n; k++) {
		if (i < 1 || i >= s_n)
			i = 1;
		if (s_entries[i].ssid && s_entries[i].ssid[0])
			return i;
		i++;
	}
	return -1;
}

static void begin_direct_cycle()
{
	s_phase = PH_DIRECT;
	if (s_direct_ok >= 1)
		s_direct_idx = s_direct_ok;
	int e = next_direct_entry(s_direct_idx);
	if (e < 0) {
		set_status(NetStatus::Down);
		return;
	}
	s_direct_idx = e;
	do_sta_connect(e, 0, nullptr, false);
}

static void begin_sta_scan()
{
	s_phase = PH_STA_SCAN;
	s_have_target = false;
	s_disc_grace_used = false;
	set_status(NetStatus::Connecting);
	ensure_wifi();
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	set_sta_auth_open();
	start_scan();
}

static void enter_ap_fallback()
{
	LOG_I(TAG, "AP fallback");
	s_phase = PH_AP_FALLBACK;
	s_connecting = false;
	start_softap(WIFI_MODE_APSTA);
	/* scan when idle (no clients) */
}

static void enter_ap_timed()
{
	s_phase = PH_AP_TIMED;
	s_connecting = false;
	start_softap(WIFI_MODE_AP);
}

static void enter_ap_only()
{
	s_phase = PH_AP_ONLY;
	start_softap(WIFI_MODE_AP);
}

static void on_sta_disconnected(int reason)
{
	LOG_I(TAG, "STA disconnect reason=%d (%s)%s (%u ms)", reason,
	      disc_reason_str(reason),
	      s_connecting ? " during connect" : "",
	      (unsigned)connect_elapsed_ms());
	if (s_connecting) {
		s_got_disc = true;
		/* Keep waiting (no immediate disconnect); once, pull deadline
		 * in so a dead attempt retries sooner. */
		if (!s_disc_grace_used) {
			s_disc_grace_used = true;
			uint32_t now = xTaskGetTickCount();
			uint32_t sooner = now + pdMS_TO_TICKS(DISC_GRACE_MS);
			if (sooner < s_connect_deadline) {
				s_connect_deadline = sooner;
				LOG_I(TAG,
				      "DISC during connect: remaining wait %d ms",
				      DISC_GRACE_MS);
			}
		}
		return;
	}
	if (s_sta_up) {
		s_sta_up = false;
		set_status(NetStatus::Down);
		/* real loss: resume by mode, no softAP */
		if (s_mode == 1 || s_mode == 3)
			begin_sta_scan();
		else if (s_mode == 4 || s_mode == 5)
			begin_direct_cycle();
	}
}

static void on_sta_connected()
{
	if (s_got_disc) {
		s_got_disc = false;
		/* association recovered; wait for IP */
	}
	LOG_I(TAG, "associated, waiting DHCP (%u ms)",
	      (unsigned)connect_elapsed_ms());
}

static void tick_connect_timeout()
{
	uint32_t now = xTaskGetTickCount();

	if (s_waiting_retry) {
		if ((now - s_retry_t0) < pdMS_TO_TICKS(DISC_RETRY_DELAY_MS))
			return;
		s_waiting_retry = false;
		LOG_I(TAG, "connect retry after %d ms pause",
		      DISC_RETRY_DELAY_MS);
		if (s_have_target) {
			do_sta_connect(s_target.entry,
				       s_target.have_bssid ? s_target.channel
							   : 0,
				       s_target.bssid, s_target.have_bssid);
		}
		return;
	}

	if (!s_connecting)
		return;
	if ((int32_t)(now - s_connect_deadline) < 0)
		return;

	LOG_W(TAG, "connect timeout after %u ms (disc_during=%d)",
	      (unsigned)connect_elapsed_ms(), (int)s_got_disc);
	if (s_got_disc && s_have_target) {
		/* retry same SSID after disconnect during attempt */
		do_sta_disconnect();
		s_connecting = false;
		s_waiting_retry = true;
		s_retry_t0 = now;
		LOG_I(TAG, "will retry same target in %d ms",
		      DISC_RETRY_DELAY_MS);
		return;
	}

	do_sta_disconnect();
	s_connecting = false;

	if (s_phase == PH_STA_CONNECT || s_phase == PH_STA_SCAN) {
		/* modes 1/3 failure -> AP fallback */
		enter_ap_fallback();
	} else if (s_phase == PH_DIRECT) {
		s_direct_idx++;
		int e = next_direct_entry(s_direct_idx);
		if (e < 0) {
			s_direct_idx = 1;
			e = next_direct_entry(1);
		}
		if (e >= 0) {
			s_direct_idx = e;
			do_sta_connect(e, 0, nullptr, false);
		}
	}
}

static void tick_ap_fallback()
{
	if (s_phase != PH_AP_FALLBACK || !s_ap_up)
		return;
	if (s_ap_clients > 0)
		return;
	if (s_connecting)
		return;
	/* scan for configured networks */
	s_have_target = false;
	start_scan();
	s_phase = PH_AP_FALLBACK; /* stay; scan done handler continues */
}

static void after_scan_in_fallback()
{
	if (s_phase != PH_AP_FALLBACK)
		return;
	if (s_ap_clients > 0)
		return;
	if (!s_have_target)
		return;
	/* shut AP first, then STA-only connect */
	stop_softap();
	s_phase = PH_STA_CONNECT;
	do_sta_connect(s_target.entry, s_target.channel, s_target.bssid,
		       s_target.have_bssid);
}

static void tick_ap_timed()
{
	if (s_phase != PH_AP_TIMED || !s_ap_up)
		return;
	if (s_ap_clients > 0) {
		s_ap_idle_t0 = xTaskGetTickCount();
		return;
	}
	uint32_t now = xTaskGetTickCount();
	if ((now - s_ap_idle_t0) < pdMS_TO_TICKS(AP_IDLE_TIMEOUT_MS))
		return;
	stop_softap();
	begin_direct_cycle();
}

static void policy_start()
{
	s_disc_grace_used = false;
	s_got_disc = false;
	s_waiting_retry = false;
	switch (s_mode) {
	case 0:
		s_phase = PH_IDLE;
		set_status(NetStatus::Down);
		break;
	case 1:
	case 3:
		s_draw_scan = true;
		begin_sta_scan();
		break;
	case 2:
		enter_ap_only();
		break;
	case 4:
		begin_direct_cycle();
		break;
	case 5:
		enter_ap_timed();
		break;
	default:
		s_phase = PH_IDLE;
		break;
	}
}

static void handle_evt(const Evt &e)
{
	switch (e.kind) {
	case EVT_SCAN_DONE:
		handle_scan_done();
		if (s_phase == PH_STA_SCAN) {
			if (s_have_target) {
				s_phase = PH_STA_CONNECT;
				do_sta_connect(s_target.entry, s_target.channel,
					       s_target.bssid,
					       s_target.have_bssid);
			} else {
				enter_ap_fallback();
			}
		} else if (s_phase == PH_AP_FALLBACK) {
			after_scan_in_fallback();
		}
		break;
	case EVT_STA_CONNECTED:
		on_sta_connected();
		break;
	case EVT_STA_DISCONNECTED:
		on_sta_disconnected(e.reason);
		break;
	case EVT_GOT_IP:
		if (s_ap_up) {
			/* should not happen with current policy; drop AP */
			stop_softap();
		}
		s_phase = PH_STA_UP;
		on_got_ip();
		break;
	case EVT_LOST_IP:
		if (s_sta_up) {
			s_sta_up = false;
			set_status(NetStatus::Down);
			if (s_mode == 1 || s_mode == 3)
				begin_sta_scan();
			else if (s_mode == 4 || s_mode == 5)
				begin_direct_cycle();
		}
		break;
	case EVT_AP_STA_JOINED:
		s_ap_clients++;
		s_ap_idle_t0 = xTaskGetTickCount();
		break;
	case EVT_AP_STA_LEFT:
		if (s_ap_clients > 0)
			s_ap_clients--;
		if (s_ap_clients == 0)
			s_ap_idle_t0 = xTaskGetTickCount();
		break;
	case EVT_CMD_RELOAD:
		load_networks();
		break;
	case EVT_CMD_CONNECT_STA: {
		/* save as entry 1, rewrite, reload, connect */
		char ap_ssid[33] = "RDZsonde";
		char ap_pw[65] = "RDZsonde";
		if (s_n > 0 && s_entries[0].ssid)
			strncpy(ap_ssid, s_entries[0].ssid, 32);
		if (s_n > 0 && s_entries[0].pw)
			strncpy(ap_pw, s_entries[0].pw, 64);

		File f = LittleFS.open("/networks.txt", "w");
		if (f) {
			f.printf("%s\n%s\n", ap_ssid, ap_pw);
			f.printf("%s\n%s\n", s_improv_ssid, s_improv_pw);
			/* keep remaining station entries if any */
			for (int i = 2; i < s_n; i++) {
				if (s_entries[i].ssid && s_entries[i].ssid[0])
					f.printf("%s\n%s\n", s_entries[i].ssid,
						 s_entries[i].pw
							 ? s_entries[i].pw
							 : "");
			}
			f.close();
		}
		load_networks();
		if (s_ap_up)
			stop_softap();
		s_mode = 4;
		s_direct_idx = 1;
		s_direct_ok = 1;
		begin_direct_cycle();
		break;
	}
	case EVT_CMD_SCAN:
		start_scan();
		break;
	case EVT_TICK:
		tick_connect_timeout();
		if (s_phase == PH_AP_FALLBACK && !s_connecting &&
		    s_ap_clients == 0) {
			/* periodic rescan while fallback AP is idle */
			static uint32_t last_scan;
			uint32_t now = xTaskGetTickCount();
			if ((now - last_scan) > pdMS_TO_TICKS(15000)) {
				last_scan = now;
				tick_ap_fallback();
			}
		}
		tick_ap_timed();
		break;
	}
}

/* Drain Wi-Fi events and run timers. Called from poll() on the main loop. */
static void process()
{
	Evt e;
	while (s_evtq && xQueueReceive(s_evtq, &e, 0) == pdTRUE)
		handle_evt(e);
	Evt tick = { EVT_TICK, 0 };
	handle_evt(tick);
}

/* ---- public API --------------------------------------------------------- */

void NetMgr::start(int mode, const char *hostname)
{
	s_mode = mode;
	if (hostname)
		strncpy(s_hostname, hostname, sizeof(s_hostname) - 1);

	if (!s_cred_mu)
		s_cred_mu = xSemaphoreCreateMutex();
	if (!s_evtq)
		s_evtq = xQueueCreate(EVT_QUEUE_LEN, sizeof(Evt));
	if (!s_scan_done)
		s_scan_done = xSemaphoreCreateBinary();

	load_networks();

	if (s_started) {
		if (s_ap_up)
			stop_softap();
		do_sta_disconnect();
		s_sta_up = false;
		s_waiting_retry = false;
		s_status = NetStatus::Down;
	}

	if (mode == 0) {
		s_phase = PH_IDLE;
		s_status = NetStatus::Down;
		s_started = true;
		return;
	}

	ensure_wifi();
	s_started = true;
	policy_start();
}

void NetMgr::reload()
{
	if (!s_cred_mu)
		s_cred_mu = xSemaphoreCreateMutex();
	load_networks();
}

void NetMgr::connectSta(const char *ssid, const char *pw)
{
	strncpy(s_improv_ssid, ssid ? ssid : "", sizeof(s_improv_ssid) - 1);
	strncpy(s_improv_pw, pw ? pw : "", sizeof(s_improv_pw) - 1);
	/* Run on the calling (main) thread via process/poll */
	post_evt(EVT_CMD_CONNECT_STA);
	process();
}

void NetMgr::scan(void (*fn)(const char *ssid, int rssi, void *arg), void *arg)
{
	s_scan_fn = fn;
	s_scan_arg = arg;
	if (s_scan_done)
		xSemaphoreTake(s_scan_done, 0);
	post_evt(EVT_CMD_SCAN);
	/* Pump events on this thread until scan completes (Improv blocks here). */
	uint32_t t0 = xTaskGetTickCount();
	while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(15000)) {
		process();
		if (s_scan_done &&
		    xSemaphoreTake(s_scan_done, 0) == pdTRUE)
			break;
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

NetEdge NetMgr::poll()
{
	process();
	NetEdge e = s_pending_edge;
	s_pending_edge = NetEdge::None;
	return e;
}

NetStatus NetMgr::status() const
{
	return s_status;
}

bool NetMgr::ipv4(char *buf, size_t n) const
{
	if (!buf || n == 0)
		return false;
	buf[0] = 0;
	esp_netif_t *nif = nullptr;
	if (s_status == NetStatus::Uplink)
		nif = s_sta_netif;
	else if (s_status == NetStatus::Ap)
		nif = s_ap_netif;
	if (!nif)
		return false;
	esp_netif_ip_info_t ip;
	if (esp_netif_get_ip_info(nif, &ip) != ESP_OK)
		return false;
	snprintf(buf, n, IPSTR, IP2STR(&ip.ip));
	return ip.ip.addr != 0;
}

bool NetMgr::dns(int index, char *buf, size_t n) const
{
	if (!buf || n == 0 || !s_sta_netif)
		return false;
	buf[0] = 0;
	esp_netif_dns_type_t t = ESP_NETIF_DNS_MAIN;
	if (index == 1)
		t = ESP_NETIF_DNS_BACKUP;
	else if (index >= 2)
		t = ESP_NETIF_DNS_FALLBACK;
	esp_netif_dns_info_t d;
	if (esp_netif_get_dns_info(s_sta_netif, t, &d) != ESP_OK)
		return false;
	if (d.ip.type != ESP_IPADDR_TYPE_V4)
		return false;
	snprintf(buf, n, IPSTR, IP2STR(&d.ip.u_addr.ip4));
	return d.ip.u_addr.ip4.addr != 0;
}

int NetMgr::count() const
{
	return s_n;
}

const char *NetMgr::ssid(int i) const
{
	if (i < 0 || i >= s_n || !s_entries[i].ssid)
		return "";
	return s_entries[i].ssid;
}

const char *NetMgr::password(int i) const
{
	if (i < 0 || i >= s_n || !s_entries[i].pw)
		return "";
	return s_entries[i].pw;
}


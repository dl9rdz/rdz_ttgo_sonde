/* netmgr.h -- Wi-Fi link manager (ESP-IDF). Owns networks.txt and association. */
#ifndef NETMGR_H
#define NETMGR_H

#include <stddef.h>

enum class NetStatus { Down, Connecting, Uplink, Ap };
enum class NetEdge { None, ServicesUp, ServicesDown };

class NetMgr {
public:
	void start(int mode, const char *hostname);
	void reload();
	void connectSta(const char *ssid, const char *pw);
	/* Improv: one-shot enumerate (does not pick a connect target). */
	void scan(void (*fn)(const char *ssid, int rssi, void *arg), void *arg);

	NetEdge poll();

	NetStatus status() const;
	bool ipv4(char *buf, size_t n) const;
	bool dns(int index, char *buf, size_t n) const;

	int count() const;
	const char *ssid(int i) const;
	const char *password(int i) const;
};

extern NetMgr netmgr;

#endif

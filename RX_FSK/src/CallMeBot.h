#ifndef CALL_ME_BOT_H
#define CALL_ME_BOT_H
// Support for WhatApp notification using Call Me Bot service

#include "Sonde.h"
#include <WiFi.h>


class CallMeBot {
	enum com_result {SUCCESS, FAILURE, NOT_READY};

public:

	static void doCallMeBot(WiFiClient* client, SondeInfo* sondeInfo);
    

private:
	static constexpr unsigned long RETRY_INTERVAL = 10; //Retry interval in seconds
	static int sendNotification(WiFiClient* client, SondeInfo* sondeInfo);
	static int getReply(WiFiClient* callbot_client, SondeInfo* sondeInfo);
	static void showStatus(char* str, bool clearScreen, int y, int x);
};

#endif

#include "Display.h"
#include "CallMeBot.h"

extern Sonde sonde;
extern Display disp;

static unsigned long stallStartTime;

void CallMeBot::doCallMeBot(WiFiClient* client, SondeInfo* sondeInfo){
	
	NotificationStatus currentStatus = sondeInfo->notificationStatus;
	NotificationStatus nextStatus    = currentStatus; //By default, stay in current state
	//Serial.printf("doCallMeBot: currentStatus=%d\n", currentStatus);
	switch(currentStatus){
		int res;
		case NOT_NOTIFIED:
			if ((sondeInfo->d.validPos&0x7)!=0x7) break; //Only send a notification if the location is valid
			showStatus("Sending WhatsApp...", true, 0, 0); //Message will typically be immediately overwritten, consider removing
			res=sendNotification(client, sondeInfo);
			if (res==SUCCESS){
				showStatus("Sent.", false, 1, 0); //Message will typically be immediately overwritten, consider removing
				nextStatus = SENT;
			}else{
				stallStartTime = millis();
				nextStatus = NotificationStatus(WAIT_BEFORE_RETRY);
			}
			break;
		
		case SENT:
			res=getReply(client, sondeInfo);
			if (res==SUCCESS){
				showStatus("CallMeBot Done.", true, 0, 0); //Message will typically be immediately overwritten, consider removing
				nextStatus = NotificationStatus(REPLY_OK);
			}else if(res==NOT_READY){
				nextStatus = currentStatus; //Keep current status
			}else{ //FAILURE
				stallStartTime = millis();
				nextStatus = NotificationStatus(WAIT_BEFORE_RETRY);
			}
			break;

		case REPLY_OK:
			//Nothing to do!	
			break;

		case REPLY_NOT_OK:
			stallStartTime = millis();
			nextStatus = NotificationStatus(WAIT_BEFORE_RETRY);
			showStatus("CallMeBot Error!", true, 0, 0); //Will typically be immediately overwritten
			break;
			
		case WAIT_BEFORE_RETRY:
			if (millis() > (stallStartTime+RETRY_INTERVAL*1000)){
				nextStatus = NotificationStatus(NOT_NOTIFIED);
			}
			//else, continue waiting
			break;
			
		default:
			Serial.printf("doCallMeBot: Error! currentStatus=%d\n", currentStatus);
			break;
	}

	//Serial.printf("doCallMeBot: nextStatus=%d\n", nextStatus);
	sondeInfo->notificationStatus = nextStatus;
}




int CallMeBot::sendNotification(WiFiClient* callbot_client, SondeInfo* sondeInfo){
	
	if(!callbot_client->connected()) {
		if(!callbot_client->connect("api.callmebot.com", 80)) {
			Serial.println("Connection to callmebot FAILED");
			return FAILURE;
		}
	}
	char locStr[100];
	//Seems like the server filters out "&", so no zoom control...
	if ((sondeInfo->d.validPos&0x7)==0x7){
		snprintf(locStr,100,"http://maps.google.com/maps?q=%.7f,%.7f\\nAlt:+%.0f",
		sondeInfo->d.lat,
		sondeInfo->d.lon,
		sondeInfo->d.alt);
	}else{
		snprintf(locStr,100,"No+location!");
	}
	
	char req[256];
	snprintf(req, 256, "GET /whatsapp.php?phone=%s&apikey=%s&text=\"[RDZ]+Detected+%s\\n"
    "%s\""
	" HTTP/1.1\r\n"
		"Host: %s\r\n"
		"Accept: application/json\r\n"
		"Cache-Control: no-cache\r\n\r\n",
		sonde.config.callmebot.phone,
		sonde.config.callmebot.apikey,
		sondeInfo->d.ser,
		locStr,
		"api.callmebot.com");
	callbot_client->print(req);
	Serial.print(req);
	
	return SUCCESS;
	
}


int CallMeBot::getReply(WiFiClient* callbot_client, SondeInfo* sondeInfo){

	unsigned long startTime = millis();
	while (callbot_client->available() == 0) {
		if (millis() > (startTime+1000)) {
		  Serial.println("CallBot Client Timeout!");
		  callbot_client->stop();
		  return NOT_READY;
		}
		yield();
	}

	while (callbot_client->available()) {
		// read line till /n
		String line = callbot_client->readStringUntil('\n');
		Serial.print(line);  
		// remove space, to check if the line is end of headers
		line.trim();

		if (!line.length()) {
		  //headers ended
		  //break; // and get the OTA started
		}

		// Check if the HTTP Response is 200
		// else break and Exit Update
		if (line.startsWith("HTTP/1.1")) {
		  if (line.indexOf("200") < 0) {
			Serial.println("Got a non 200 status code from server. Exiting");
			return FAILURE;
		  }else{
			return SUCCESS; 
		  }
		}
	}
	
	return NOT_READY;
}

	
void CallMeBot::showStatus(char* str, bool clearScreen, int y, int x){
	if (clearScreen) sonde.clearDisplay();
	disp.rdis->setFont(FONT_SMALL);
    disp.rdis->drawString(x, y, str);
}
		  
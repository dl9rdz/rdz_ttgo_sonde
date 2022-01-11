#ifndef SH_FREQ_IMPORT_
#define SH_FREQ_IMPORT_H
// Automated frequency import from SondeHub

#include <WiFi.h>

enum ImportState { START, BEFOREID, COPYID, AFTERID, BEFOREKEY, COPYKEY, AFTERKEY, SKIPVAL, BEFORENUMVAL, COPYNUMVAL, BEFORESTRINGVAL, COPYSTRINGVAL, AFTERPAYLOAD, ENDORNEXT, ENDREACHED };

class ShFreqImport {
public:
	// Fetch data from sondehub and populate qrg.txt with result
	// return: 0: ok; 1: failure
	static int shImportSendRequest(WiFiClient *client, float lat, float lon, int dist, int time);

	// return 0: ok, need more data; 1: finished/failure, close connection
	// Asynchronous I/O. Handle data if available
	static int shImportHandleReply(WiFiClient *client);  

private:
	static int stringToStype(const char *type);
	static void setLabel(int idx, char *id, float lat, float lon);
	static void usekeyvalue();
	static int handleChar(char c);

	// add one entry on available slot at or after ppos
	static void populate(char *id, float lat, float lon, float freq, const char *type);
	static void cleanup();
};

#endif

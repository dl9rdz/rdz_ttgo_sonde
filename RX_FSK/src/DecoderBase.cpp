
#include "DecoderBase.h"
#include "Radio.h"
#include "Sonde.h"

#define DECODERBASE_DEBUG 0

#if DECODERBASE_DEBUG
#define DBG(x) x
#else
#define DBG(x)
#endif


int DecoderBase::setup(decoderSetupCfg &setupcfg, uint16_t agcbw, uint16_t rxbw) {
        if(radio.setFSK()!=0) {
                DBG(Serial.println("Setting FSK mode FAILED"));
                return 1;
        }
        if(radio.setBitrate(setupcfg.bitrate)!=0) {
                DBG(Serial.println("Setting bitrate FAILED"));
                return 1;
        }
#if DECODERBASE_DEBUG
        float br = radio.getBitrate();
        Serial.print("Exact bitrate is ");
        Serial.println(br);
#endif
        if(radio.setAFCBandwidth(agcbw)!=0) {
                DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", agcbw));
                return 1;
        }
        if(radio.setRxBandwidth(rxbw)!=0) {
                DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", rxbw));
                return 1;
        }

        if(radio.setRxConf(setupcfg.rx_cfg)!=0) {
                DBG(Serial.println("Setting RX Config FAILED"));
                return 1;
        }
        if(radio.setSyncConf(setupcfg.sync_cfg, setupcfg.sync_len, setupcfg.sync_data)!=0) {
                DBG(Serial.println("Setting SYNC Config FAILED"));
                return 1;
        }
        if(radio.setPreambleDetect(setupcfg.preamble_cfg)!=0) {
                DBG(Serial.println("Setting PreambleDetect FAILED"));
                return 1;
        }
	return 0;
}

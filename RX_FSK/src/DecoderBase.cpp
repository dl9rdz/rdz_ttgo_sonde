
#include "DecoderBase.h"
#include "SX1278FSK.h"
#include "Sonde.h"

#define DECODERBASE_DEBUG 1

#if DECODERBASE_DEBUG
#define DBG(x) x
#else
#define DBG(x)
#endif


int DecoderBase::setup(decoderSetupCfg &setupcfg, uint16_t agcbw, uint16_t rxbw) {
        if(sx1278.setFSK()!=0) {
                DBG(Serial.println("Setting FSK mode FAILED"));
                return 1;
        }
        if(sx1278.setBitrate(setupcfg.bitrate)!=0) {
                DBG(Serial.println("Setting bitrate FAILED"));
                return 1;
        }
#if DECODERBASE_DEBUG
        float br = sx1278.getBitrate();
        Serial.print("Exact bitrate is ");
        Serial.println(br);
#endif
        if(sx1278.setAFCBandwidth(agcbw)!=0) {
                DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(rxbw)!=0) {
                DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", rxbw));
                return 1;
        }

        if(sx1278.setRxConf(setupcfg.rx_cfg)!=0) {
                DBG(Serial.println("Setting RX Config FAILED"));
                return 1;
        }
        if(sx1278.setSyncConf(setupcfg.sync_cfg, setupcfg.sync_len, setupcfg.sync_data)!=0) {
                DBG(Serial.println("Setting SYNC Config FAILED"));
                return 1;
        }
        if(sx1278.setPreambleDetect(setupcfg.preamble_cfg)!=0) {
                DBG(Serial.println("Setting PreambleDetect FAILED"));
                return 1;
        }
	return 0;
}

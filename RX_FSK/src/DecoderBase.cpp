
#include "DecoderBase.h"
#include "Radio.h"
#include "Sonde.h"
#include "logger.h"

#define TAG "DecoderBase"

int DecoderBase::setup(decoderSetupCfg &setupcfg, uint16_t agcbw, uint16_t rxbw) {
        if(radio.setFSK()!=0) {
                LOG_W(TAG, "Setting FSK mode FAILED\n");
                return 1;
        }
        if(radio.setBitrate(setupcfg.bitrate)!=0) {
                LOG_W(TAG, "Setting bitrate FAILED\n");
                return 1;
        }
        float br = radio.getBitrate();
        LOG_D(TAG, "Exact bitrate is %f\n", br);
        if(radio.setAFCBandwidth(agcbw)!=0) {
                LOG_W(TAG, "Setting AFC bandwidth %d Hz FAILED\n", agcbw);
                return 1;
        }
        if(radio.setRxBandwidth(rxbw)!=0) {
                LOG_W(TAG, "Setting RX bandwidth to %d Hz FAILED\n", rxbw);
                return 1;
        }

        if(radio.setRxConf(setupcfg.rx_cfg)!=0) {
                LOG_W(TAG, "Setting RX Config FAILED\n");
                return 1;
        }
        // Re-apply LNA gain (incl. optional LnaBoostHf from config.lnaboost)
        if(radio.setLNAGain(0)!=0) {
                LOG_W(TAG, "Setting LNA gain FAILED\n");
                return 1;
        }
        if(radio.setSyncConf(setupcfg.sync_cfg, setupcfg.sync_len, setupcfg.sync_data)!=0) {
                LOG_W(TAG, "Setting SYNC Config FAILED\n");
                return 1;
        }
        if(radio.setPreambleDetect(setupcfg.preamble_cfg)!=0) {
                LOG_W(TAG, "Setting PreambleDetect FAILED\n");
                return 1;
        }
	return 0;
}

#ifndef Radio_h
#define Radio_h

#include <stdint.h>

// Portable FIFO status bits (mapped from SX1278 REG_IRQ_FLAGS2 for that backend)
#define FIFO_EMPTY          0x01
#define FIFO_FULL           0x02
#define FIFO_OVERRUN        0x04
#define FIFO_PAYLOAD_READY  0x08

class Radio {
public:
	virtual ~Radio() {}

	virtual uint8_t ON() = 0;
	virtual void OFF() = 0;

	virtual uint8_t setFSK() = 0;

	virtual uint8_t setBitrate(float bps) = 0;
	virtual float getBitrate() = 0;

	virtual uint8_t setRxBandwidth(float bps) = 0;
	virtual float getRxBandwidth() = 0;

	virtual uint8_t setAFCBandwidth(float bps) = 0;
	virtual float getAFCBandwidth() = 0;

	virtual uint8_t setFrequency(float freq) = 0;
	virtual float getFrequency() = 0;

	virtual uint8_t setLNAGain(int gain) = 0;
	virtual int getLNAGain() = 0;

	virtual uint8_t setRxConf(uint8_t conf) = 0;
	virtual uint8_t setSyncConf(uint8_t conf, int len, const uint8_t *syncpattern) = 0;
	virtual uint8_t setPreambleDetect(uint8_t conf) = 0;
	virtual uint8_t setPacketConfig(uint8_t conf1, uint8_t conf2) = 0;
	virtual uint8_t setPayloadLength(int len) = 0;

	virtual void clearIRQFlags() = 0;

	virtual int16_t getRSSI() = 0;
	virtual int32_t getFEI() = 0;
	virtual int32_t getAFC() = 0;
	virtual uint16_t getRawAFC() = 0;
	virtual void setRawAFC(uint16_t afc) = 0;

	virtual uint8_t receivePacketTimeout(uint32_t wait, uint8_t *data) = 0;

	// Enter FSK RX; also clears FastHop (PLL_HOP=0)
	virtual void startRX() = 0;

	virtual uint8_t fifoStatus() = 0;
	virtual uint8_t readFifoByte() = 0;

	virtual bool preambleDetected() = 0;

	virtual void scanBegin(float chanbw, uint8_t smooth) = 0;
	virtual void scanSetFrequency(float f) = 0;
	virtual int scanRSSI() = 0;
	virtual void scanEnd() = 0;
};

extern Radio &radio;

#endif

/*
 * Functions for using SX127x in FSK mode (mainly receive)
 * Copyright (C) 2019 Hansi Reiser, dl9rdz
 *
 * Partially based on the SX1278 libraray for managing Semtech modules
 *  Copyright (C) 2015 Wireless Open Source
 *  http://wirelessopensource.com
 *
 *  SPDX-License-Identifier:    LGPL-2.1+
 */

#include "SX1278FSK.h"
#include "SPI.h"
#include <Sonde.h>

SX1278FSK::SX1278FSK()
{
	// Initialize class variables
};




/*
Function: Turns the module ON.
Returns: 0 on success, 1 otherwise
*/
uint8_t SX1278FSK::ON()
{
	uint8_t state = 2;
#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'ON'"));
#endif

	// Powering the module
	pinMode(SX1278_SS, OUTPUT);
	digitalWrite(SX1278_SS, HIGH);

	//Configure the MISO, MOSI, CS, SPCR.
	SPI.begin();
	//Set most significant bit first
	SPI.setBitOrder(MSBFIRST);
	//Divide the clock frequency
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	//Set data mode
	SPI.setDataMode(SPI_MODE0);

	// Set Maximum Over Current Protection
	state = setMaxCurrent(0x1B);
	if( state == 0 )
	{
#if (SX1278FSK_debug_mode > 1)
		Serial.println(F("## Setting ON with maximum current supply ##"));
		Serial.println();
#endif
	}
	else
	{
		return 1;
	}

	// set FSK mode
	state = setFSK();
	return state;
}

/*
Function: Turn the module OFF.
Returns: Nothing
*/
void SX1278FSK::OFF()
{
#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'OFF'"));
#endif

	SPI.end();
	// Powering the module
	pinMode(SX1278_SS,OUTPUT);
	digitalWrite(SX1278_SS,LOW);

#if (SX1278FSK_debug_mode > 1)
	Serial.println(F("## Setting OFF ##"));
	Serial.println();
#endif
}

/*
Function: Reads the indicated register.
Returns: The content of the register
Parameters:
	address: address register to read from
*/
byte SX1278FSK::readRegister(byte address)
{
	byte value = 0x00;

	digitalWrite(SX1278_SS,LOW);

	//delay(1);
	bitClear(address, 7);		// Bit 7 cleared to write in registers
	SPI.transfer(address);
	value = SPI.transfer(0x00);
	digitalWrite(SX1278_SS,HIGH);

#if (SX1278FSK_debug_mode > 1)
	if(address!=0x3F) {
		Serial.print(F("## Reading:  ##\t"));
		Serial.print(F("Register "));
		Serial.print(address, HEX);
		Serial.print(F(":  "));
		Serial.print(value, HEX);
		Serial.println();
	}
#endif

	return value;
}

/*
Function: Writes on the indicated register.
Returns: Nothing
Parameters:
	address: address register to write in
	data: value to write in the register
*/
void SX1278FSK::writeRegister(byte address, byte data)
{
	digitalWrite(SX1278_SS,LOW);

	//delay(1);
	bitSet(address, 7);			// Bit 7 set to read from registers
	SPI.transfer(address);
	SPI.transfer(data);
	digitalWrite(SX1278_SS,HIGH);

#if (SX1278FSK_debug_mode > 1)
	Serial.print(F("## Writing:  ##\t"));
	Serial.print(F("Register "));
	bitClear(address, 7);
	Serial.print(address, HEX);
	Serial.print(F(":  "));
	Serial.print(data, HEX);
	Serial.println();
#endif

}

/*
 * Function: Clears the IRQ flags
 * 
 * Configuration registers are accessed through the SPI interface. 
 * Registers are readable in all device mode including Sleep. However, they 
 * should be written only in Sleep and Stand-by modes.
 * 
 * Returns: Nothing
 */
void SX1278FSK::clearIRQFlags()
{
	byte st0;

	// Save the previous status
	st0 = readRegister(REG_OP_MODE);		
	// Stdby mode to write in registers
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	
	// FSK mode flags1 register
	writeRegister(REG_IRQ_FLAGS1, 0xFF);
	// FSK mode flags2 register 
	writeRegister(REG_IRQ_FLAGS2, 0xFF);
	// Getting back to previous status
	if(st0 != FSK_STANDBY_MODE)  {
		writeRegister(REG_OP_MODE, st0);
	}
#if (SX1278FSK_debug_mode > 1)
	Serial.println(F("## FSK flags cleared ##"));
#endif
}

/*
Function: Sets the module in FSK mode.
Returns:   Integer that determines if there has been any error
	state = 2  --> The command has not been executed
	state = 1  --> There has been an error while executing the command
	state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278FSK::setFSK()
{
	uint8_t state = 2;
	byte st0;
	byte config1;

#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setFSK'"));
#endif

	writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
	// If we are in LORA mode, above line activate Sleep mode, but does not change mode to FSK
	// as mode change is only allowed in sleep mode. Next line changes to FSK
	writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);

	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode

	//writeRegister(REG_FIFO_THRESH, 0x80);	// condition to start packet tx
	//config1 = readRegister(REG_SYNC_CONFIG);
	//config1 = config1 & B00111111;
	//writeRegister(REG_SYNC_CONFIG,config1);

	delay(100);

	st0 = readRegister(REG_OP_MODE);	// Reading config mode
	if( st0 == FSK_STANDBY_MODE )
	{ // FSK mode
		state = 0;
#if (SX1278FSK_debug_mode > 1)
		Serial.println(F("## FSK set with success ##"));
		Serial.println();
#endif
	} else { // LoRa mode
		state = 1;
		Serial.println( st0 );
#if (SX1278FSK_debug_mode > 1)
		Serial.println(F("** There has been an error while setting FSK **"));
		Serial.println();
#endif
	}
	return state;
}


/* Function: Sets FSK bitrate
 * Returns: 0 for success, >0 in case of error
 * Parameters: bps: requested bitrate
 * (raw data rate, for Mancester encoding, the effective bitrate is bps/2)
*/

uint8_t SX1278FSK::setBitrate(float bps)
{
	// TODO: Check if FSK mode is active

	// check if bitrate is allowed allowed bitrate
	if((bps < 1200) || (bps > 300000)) {
	  return 1;
	}

	// set mode to FSK STANDBY
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE); 

	// set bit rate
	uint16_t bitRate = (SX127X_CRYSTAL_FREQ * 1.0) / bps;
	writeRegister(REG_BITRATE_MSB, (bitRate & 0xFF00) >> 8);
	writeRegister(REG_BITRATE_LSB, (bitRate & 0x00FF));

	// also set fractional part
	uint16_t fracRate = (SX127X_CRYSTAL_FREQ * 16.0) / bps - bitRate * 16 + 0.5;
	writeRegister(REG_BIT_RATE_FRAC, fracRate&0x0F);
	return 0;
}
/* Function: Gets configured bitrate 
 * Returns bitrate in bit/second
 */
float SX1278FSK::getBitrate()
{
	uint8_t fmsb = readRegister(REG_BITRATE_MSB);
	uint8_t flsb = readRegister(REG_BITRATE_LSB);
	uint8_t ffrac = readRegister(REG_BIT_RATE_FRAC) & 0x0F;
	return SX127X_CRYSTAL_FREQ / ( (fmsb<<8) + flsb + ffrac / 16.0 );
}

//typedef struct rxbwset { float bw; uint8_t mant; uint8_t rxp; } st_rxbwsettings;

uint8_t SX1278FSK::setRxBandwidth(float bw)
{
	// TODO: Check if in FSK mode
	//
	if(bw<2600 || bw>250000) { return 1; /* invalid */ }

	uint8_t rxbwexp = 1;
	bw = SX127X_CRYSTAL_FREQ / bw / 8;
	while(bw>31) { rxbwexp++; bw/=2.0; }
	uint8_t rxbwmant = bw<17?0 : bw<21? 1:2;

	// set mode to FSK STANDBY
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE); 

	writeRegister(REG_RX_BW, rxbwexp | (rxbwmant<<3));
	return 0;
}

float SX1278FSK::getRxBandwidth()
{
	uint8_t rxbw = readRegister(REG_RX_BW);
	uint8_t rxbwexp = rxbw&0x07;
	uint8_t rxbwmant = (rxbw>>3)&0x03;
	rxbwmant = 16 + 4*rxbwmant;
	return SX127X_CRYSTAL_FREQ / ( rxbwmant << (rxbwexp+2));
}

uint8_t SX1278FSK::setAFCBandwidth(float bw)
{
	// TODO: Check if in FSK mode
	//
	if(bw<2600 || bw>250000) { return 1; /* invalid */ }

	uint8_t rxbwexp = 1;
	bw = SX127X_CRYSTAL_FREQ / bw / 8;
	while(bw>31) { rxbwexp++; bw/=2.0; }
	uint8_t rxbwmant = bw<17?0 : bw<21? 1:2;

	// set mode to FSK STANDBY
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE); 

	writeRegister(REG_AFC_BW, rxbwexp | (rxbwmant<<3));
	return 0;
}

float SX1278FSK::getAFCBandwidth()
{
	uint8_t rxbw = readRegister(REG_AFC_BW);
	uint8_t rxbwexp = rxbw&0x07;
	uint8_t rxbwmant = (rxbw>>3)&0x03;
	rxbwmant = 16 + 4*rxbwmant;
	return SX127X_CRYSTAL_FREQ / ( rxbwmant << (rxbwexp+2));
}


uint8_t SX1278FSK::setFrequency(float freq) {

	// set mode to FSK STANDBY 
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
	freq += sonde.config.freqofs;  // manual frequency correction

	uint32_t frf = freq * 1.0 * (1<<19) / SX127X_CRYSTAL_FREQ;
	writeRegister(REG_FRF_MSB, (frf&0xff0000)>>16);
	writeRegister(REG_FRF_MID, (frf&0x00ff00)>>8);
	writeRegister(REG_FRF_LSB, (frf&0x0000ff));
	return 0;
}

float SX1278FSK::getFrequency() {
	uint8_t fmsb = readRegister(REG_FRF_MSB);
	uint8_t fmid = readRegister(REG_FRF_MID);
	uint8_t flsb = readRegister(REG_FRF_LSB);
	return ((fmsb<<16)|(fmid<<8)|flsb) * 1.0 / (1<<19) * SX127X_CRYSTAL_FREQ;
}


static int gaintab[]={-999,0,-6,-12,-24,-36,-48,-999};
int SX1278FSK::getLNAGain() {
	int gain = (readRegister(REG_LNA)>>5)&0x07;
	return gaintab[gain];
}
uint8_t SX1278FSK::setLNAGain(int gain) {
	uint8_t g=1;
	while(gain<gaintab[g] && g<6) {g++; }
	writeRegister(REG_LNA, g<<5);
	return 0;
}

uint8_t SX1278FSK::getRxConf() {
	return readRegister(REG_RX_CONFIG);
}
uint8_t SX1278FSK::setRxConf(uint8_t conf) {
	writeRegister(REG_RX_CONFIG, conf);
	return 0;
}

uint8_t SX1278FSK::setSyncConf(uint8_t conf, int len, const uint8_t *syncpattern) {
	int res=0;
	writeRegister(REG_SYNC_CONFIG, conf);
	if(len>8) return 1;
	for(int i=0; i<len; i++) {
		writeRegister(REG_SYNC_VALUE1+i, syncpattern[i]);
	}
	return res;
}

uint8_t SX1278FSK::getSyncConf() {
	return sx1278.readRegister(REG_SYNC_CONFIG);
}

uint8_t SX1278FSK::setPreambleDetect(uint8_t conf) {
	sx1278.writeRegister(REG_PREAMBLE_DETECT, conf);
	return 0;
}

uint8_t SX1278FSK::getPreambleDetect() {
	return sx1278.readRegister(REG_PREAMBLE_DETECT);
}

uint8_t SX1278FSK::setPacketConfig(uint8_t conf1, uint8_t conf2)
{
	uint8_t ret=0;
	sx1278.writeRegister(REG_PACKET_CONFIG1, conf1);
	sx1278.writeRegister(REG_PACKET_CONFIG2, conf2);
	return ret;
};
uint16_t SX1278FSK::getPacketConfig() {
	uint8_t c1 = sx1278.readRegister(REG_PACKET_CONFIG1);
	uint8_t c2 = sx1278.readRegister(REG_PACKET_CONFIG2);
	return (c2<<8)|c1;
}

/*
Function: Gets the preamble length from the module.
Returns: preamble length
*/
uint16_t SX1278FSK::getPreambleLength()
{
	uint16_t p_length;

#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getPreambleLength'"));
#endif

	p_length = readRegister(REG_PREAMBLE_MSB_FSK);
	p_length = (p_length<<8) | readRegister(REG_PREAMBLE_LSB_FSK);
#if (SX1278FSK_debug_mode > 1)
		Serial.print(F("## Preamble length configured is "));
		Serial.print(p_length, HEX);
		Serial.print(F(" ##"));
		Serial.println();
#endif
	return p_length;
}

/*
Function: Sets the preamble length in the module
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
Parameters:
l: length value to set as preamble length.
*/
uint8_t SX1278FSK::setPreambleLength(uint16_t l)
{
	byte st0;
	int8_t state = 2;

#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setPreambleLength'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
	// Storing MSB preamble length in FSK mode
	writeRegister(REG_PREAMBLE_MSB_FSK, l>>8);
	writeRegister(REG_PREAMBLE_LSB_FSK, l&0xFF);

	state = 0;
#if (SX1278FSK_debug_mode > 1)
	Serial.print(F("## Preamble length "));
	Serial.print(l, HEX);
	Serial.println(F(" has been successfully set ##"));
	Serial.println();
#endif

	if(st0 != FSK_STANDBY_MODE) {
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	}
	return state;
}

/*
Function: Gets the payload length from the module.
Returns: configured length; -1 on error
*/
int SX1278FSK::getPayloadLength()
{	
	int length;
#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getPayloadLength'"));
#endif
	length = readRegister(REG_PAYLOAD_LENGTH_FSK);

#if (SX1278FSK_debug_mode > 1)
	Serial.print(F("## Payload length configured is "));
	Serial.print(length);
	Serial.println(F(" ##"));
#endif
	return length;
}

/*
Function: Sets the payload length from the module.
Returns: 0 for ok, otherwise error
// TODO: Larger than 255 bytes?
*/
uint8_t SX1278FSK::setPayloadLength(int len)
{	
#if (SX1278FSK_debug_mode > 1)
	Serial.print(F("Starting 'setPayloadLength'"));
	Serial.println(len);
#endif
	uint8_t conf2 = readRegister(REG_PACKET_CONFIG2);
	conf2 = (conf2 & 0xF8) | ( (len>>8)&0x7 );
	writeRegister(REG_PACKET_CONFIG2, conf2);
	writeRegister(REG_PAYLOAD_LENGTH_FSK, len&0xFF);
	return 0;
}

/*
Function: Gets the current value of RSSI.
Returns: RSSI value
*/
int16_t SX1278FSK::getRSSI()
{
	int16_t RSSI;
	//int rssi_mean = 0;
	int total = 1;

	/// FSK mode
	// get mean value of RSSI
	for(int i = 0; i < total; i++)
	{
		RSSI = readRegister(REG_RSSI_VALUE_FSK);
		//rssi_mean += _RSSI;
	}
	//rssi_mean = rssi_mean / total;	
	//RSSI = rssi_mean;

#if (SX1278FSK_debug_mode > 0)
	Serial.print(F("## RSSI value is "));
	Serial.print(RSSI);
	Serial.println(F(" ##"));
#endif
	return RSSI;
}

/*
Function: Gets the current value of FEI (frequency error indication)
Returns: FEI value in Hz
*/
int32_t SX1278FSK::getFEI()
{
	int32_t FEI;
	int16_t regval = (readRegister(REG_FEI_MSB)<<8) | readRegister(REG_FEI_LSB);
	//Serial.printf("feireg: %04x\n", regval);
	FEI = (int32_t)(regval * SX127X_FSTEP);
	return FEI;
}
/*
Function: Gets the current value of AFC (automated frequency correction)
Returns: AFC value in Hz
*/
int32_t SX1278FSK::getAFC()
{
	int32_t AFC;
	int16_t regval = (readRegister(REG_AFC_MSB)<<8) | readRegister(REG_AFC_LSB);
	//Serial.printf("afcreg: %04x\n", regval);
	AFC = (int32_t)(regval * SX127X_FSTEP);
	return AFC;
}

/*
Function: Gets the current supply limit of the power amplifier, protecting battery chemistries.
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
Parameters:
rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
int SX1278FSK::getMaxCurrent()
{
	int value;

#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getMaxCurrent'"));
#endif

	value = readRegister(REG_OCP);

	// extract only the OcpTrim value from the OCP register
	value &= B00011111;

	if( value <= 15 ) {
		value = (45 + (5 * value));
	} else if( value <= 27 ) {
		value = (-30 + (10 * value));
	} else {
		value = 240;		
	}
#if (SX1278FSK_debug_mode > 1)
	Serial.print(F("## Maximum current supply configured is "));
	Serial.print(value, DEC);
	Serial.println(F(" mA ##"));
	Serial.println();
#endif
	return value;
}

/*
Function: Limits the current supply of the power amplifier, protecting battery chemistries.
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
state = -1 --> Forbidden parameter value for this function
Parameters:
rate: value to compute the maximum current supply. Range: 0x00 to 0x1B. The 
Maximum current is:
Imax = 45+5*OcpTrim [mA] 	if OcpTrim <= 15 (120 mA) /
Imax = -30+10*OcpTrim [mA] 	if 15 < OcpTrim <= 27 (130 to 240 mA)
Imax = 240mA 				for higher settings
*/
int8_t SX1278FSK::setMaxCurrent(uint8_t rate)
{
	int8_t state = 2;
	byte st0;

#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'setMaxCurrent'"));
#endif

	// Maximum rate value = 0x1B, because maximum current supply = 240 mA
	if (rate > 0x1B)
	{
		state = -1;
#if (SX1278FSK_debug_mode > 1)
		Serial.print(F("** Maximum current supply is 240 mA, "));
		Serial.println(F("so maximum parameter value must be 27 (DEC) or 0x1B (HEX) **"));
		Serial.println();
#endif
	}
	else
	{
		// Enable Over Current Protection
		rate |= B00100000;

		state = 1;
		st0 = readRegister(REG_OP_MODE);	// Save the previous status
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Set FSK Standby mode to write in registers
		writeRegister(REG_OCP, rate);		// Modifying maximum current supply
		if(st0 != FSK_STANDBY_MODE) {
			writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
		}
		state = 0;
	}
	return state;
}


/*
Function: Configures the module to receive information.
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278FSK::receive()
{
	uint8_t state = 1;
#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'receive'"));
#endif
	// TODO: Is there anything else to be done?
	//
	writeRegister(REG_OP_MODE, FSK_RX_MODE);  
	state = 0;
#if (SX1278FSK_debug_mode > 1)
		Serial.println(F("## Receiving FSK mode activated with success ##"));
#endif
	return state;
}

// ugly. shouldn't be here in a nice software design
extern int hasKeyPress();

/*
Function: Configures the module to receive a packet
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
Parameters:
	wait: timeout in ms
	data: memory where to place received data
*/
uint8_t SX1278FSK::receivePacketTimeout(uint32_t wait, byte *data)
{
	int di=0;
	uint8_t state = 2;
	unsigned long previous;
	byte value = 0x00;
#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'receivePacketTimeout'"));
#endif
	// set RX mode
	state = receive();
	if(state != 0) { return state; }

#if (SX1278FSK_debug_mode > 0)
	Serial.println(F("RX mode sucessfully activated"));
#endif
	previous = millis();
	/// FSK mode
	value = readRegister(REG_IRQ_FLAGS2);
	byte ready=0;
	// while not yet done or FIFO not yet empty
	while( (!ready || bitRead(value,6)==0) && (millis() - previous < wait) )
	{
		if( bitRead(value,2)==1 ) ready=1;
		if( bitRead(value, 6) == 0 ) { // FIFO not empty
			data[di++] = readRegister(REG_FIFO);
			// It's a bit of a hack.... get RSSI and AFC (a) at beginning of packet and
			// for RS41 after about 0.5 sec. It might be more logical to put this decoder-specific
			// code into RS41.cpp instead of this file... (maybe TODO?)
			
			if(di==1 || di==290 ) {
				int rssi=getRSSI();
				int afc=getAFC();
				Serial.printf("Test(%d): RSSI=%d", rxtask.currentSonde, rssi/2);
				Serial.print("Test: AFC="); Serial.println(afc);
				sonde.sondeList[rxtask.currentSonde].rssi = rssi;
				sonde.sondeList[rxtask.currentSonde].afc = afc;
				if(rxtask.receiveResult==0xFFFF)
					rxtask.receiveResult = RX_UPDATERSSI;
				//sonde.si()->rssi = rssi;
				//sonde.si()->afc = afc;
			}
			if(di>520) {
				// TODO
				Serial.println("TOO MUCH DATA");
				break;
			}
			previous = millis(); // reset timeout after receiving data
		} else {
			delay(10);
		}
		value = readRegister(REG_IRQ_FLAGS2);
	}
	if( !ready || bitRead(value, 6)==0) {
#if 1&&(SX1278FSK_debug_mode > 0)
		Serial.println(F("** The timeout has expired **"));
		Serial.println();
#endif
		sonde.si()->rssi = getRSSI();
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
		return 1;  // TIMEOUT
	}

#if (SX1278FSK_debug_mode > 0)
	Serial.println(F("## Packet received:"));
	for(unsigned int i = 0; i < di; i++)
	{
		Serial.print(data[i], HEX);		// Printing payload
		Serial.print("|");
	}
	Serial.println(F(" ##"));
#endif
	state = 0;
	// Initializing flags	
	clearIRQFlags();	

	return state;
}


#if 0
/*
Function: It gets the temperature from the measurement block module.
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278FSK::getTemp()
{
	byte st0;
	uint8_t state = 2;

#if (SX1278FSK_debug_mode > 1)
	Serial.println();
	Serial.println(F("Starting 'getTemp'"));
#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if( _modem == LORA )
	{ // Allowing access to FSK registers while in LoRa standby mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
	}

	state = 1;
	// Saving temperature value
	_temp = readRegister(REG_TEMP);
	if( _temp & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		_temp = ( ( ~_temp + 1 ) & 0xFF );
	}
	else
	{
		// Divide by 4
		_temp = ( _temp & 0xFF );
	}


#if (SX1278FSK_debug_mode > 1)
	Serial.print(F("## Temperature is: "));
	Serial.print(_temp);
	Serial.println(F(" ##"));
	Serial.println();
#endif

	if( _modem == LORA )
	{
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	}

	state = 0;
	return state;
}

/*
Function: It prints the registers related to RX
Returns: Integer that determines if there has been any error
state = 2  --> The command has not been executed
state = 1  --> There has been an error while executing the command
state = 0  --> The command has been executed with no errors
*/
void SX1278FSK::showRxRegisters()
{	
	Serial.println(F("\n--- Show RX register ---"));

	// variable
	byte reg;	

	for(int i = 0x00; i < 0x80; i++)
	{
		reg = readRegister(i);	
		Serial.print(F("Reg 0x"));
		Serial.print(i, HEX);
		Serial.print(F(":"));
		Serial.print(reg, HEX);
		Serial.println();
		delay(100);
	}

	Serial.println(F("------------------------"));

}
#endif

SX1278FSK sx1278 = SX1278FSK();

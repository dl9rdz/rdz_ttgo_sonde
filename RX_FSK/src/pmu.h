#include <inttypes.h>
#include <Wire.h>

#define AXP192_SLAVE_ADDRESS 0x34

enum { TYPE_NONE=-1, TYPE_UNKNOWN=0, TYPE_AXP192, TYPE_AXP2101 };

class PMU {
protected:
    PMU(TwoWire &wire) : _wire(wire) { };

public:
    TwoWire &_wire;
    static PMU *getInstance(TwoWire &wire);
    int type;

    static int readRegisterWire(TwoWire &wire, uint8_t reg);
    int readRegister(uint8_t reg);
    uint16_t readRegisters_8_4(uint8_t reghi, uint8_t reglo);
    uint16_t readRegisters_8_5(uint8_t reghi, uint8_t reglo);
    int writeRegister(uint8_t reg, uint8_t val);
    int getRegisterBit(uint8_t register, uint8_t bit);
    int setRegisterBit(uint8_t register, uint8_t bit);
    int clearRegisterBit(uint8_t register, uint8_t bit);

    int handleIRQ();

    virtual int init();
    virtual void disableAllIRQ();
    virtual void enableIRQ();
    virtual int getIrqKeyStatus();

    virtual int isBatteryConnected();
    virtual int isVbusIn();
    virtual int isCharging();
    virtual float getBattVoltage();
    virtual float getBattDischargeCurrent();
    virtual float getBattChargeCurrent();
    virtual float getAcinVoltage();
    virtual float getAcinCurrent();
    virtual float getVbusVoltage();
    virtual float getVbusCurrent();
    virtual float getTemperature();
};

/* Interface */
class AXP192PMU : public PMU {
public:
    AXP192PMU(TwoWire &wire) : PMU(wire) { type = TYPE_AXP192; };
    int init();
    void disableAllIRQ();
    void enableIRQ();
    int getIrqKeyStatus();

    int isBatteryConnected();
    int isVbusIn();
    int isCharging();
    float getBattVoltage();
    float getBattDischargeCurrent();
    float getBattChargeCurrent();
    float getAcinVoltage();
    float getAcinCurrent();
    float getVbusVoltage();
    float getVbusCurrent();
    float getTemperature();

protected:
    void _enableIRQ(uint8_t addr, uint8_t mask);

    int setVoltageReg(uint8_t reg, uint8_t regval);
    int setDC1(uint16_t millivolt);
    int setDC2(uint16_t millivolt);
    int setDC3(uint16_t millivolt);
    int setLDO2(uint16_t millivolt);
    int setLDOio(uint16_t millivolt);

    int enableDC1(bool onff = true);
    int enableDC3(bool onoff = true);
    int enableLDO2(bool onoff = true);
    int enableLDO3(bool onoff = true);
    int enableDC2(bool onoff = true);
    int enableEXTEN(bool onoff = true);

    int enableADC(uint8_t channels);
}; 

class AXP2101PMU : public PMU {
public:
    AXP2101PMU(TwoWire &wire) : PMU(wire) { };
    int init();
    void disableAllIRQ();
    void enableIRQ();
    int getIrqKeyStatus();

    int isBatteryConnected();
    int isVbusIn();
    int isCharging();
    float getBattVoltage();
    float getBattDischargeCurrent();
    float getBattChargeCurrent();
    float getAcinVoltage();
    float getAcinCurrent();
    float getVbusVoltage();
    float getVbusCurrent();
    float getTemperature();

protected:
    void _enableIRQ(uint8_t addr, uint8_t mask);

    int setVBACKUP(uint16_t millivolt);
    int setDCDC1(uint16_t millivolt);
    int setALDO2(uint16_t millivolt);
    int setALDO3(uint16_t millivolt);

};

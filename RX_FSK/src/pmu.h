#include <inttypes.h>
#include <Wire.h>

#define AXP192_SLAVE_ADDRESS 0x34

enum { TYPE_NONE=-1, TYPE_UNKNOWN=0, TYPE_AXP192, TYPE_AXP2101 };

typedef enum {
    CHG_LED_OFF,
    CHG_LED_BLINK_1HZ,
    CHG_LED_BLINK_4HZ,
    CHG_LED_ON,
    CHG_LED_CTRL_CHG_ON,    // The charging indicator is controlled by the charger, charging == ON
    CHG_LED_CTRL_CHG_BLINK, // The charging indicator is controlled by the charger, charging == BLINK
} chg_led_mode_t;

typedef enum {
    AXP2101_CHG_TRI_STATE,   //tri_charge
    AXP2101_CHG_PRE_STATE,   //pre_charge
    AXP2101_CHG_CC_STATE,    //constant charge
    AXP2101_CHG_CV_STATE,    //constant voltage
    AXP2101_CHG_DONE_STATE,  //charge done
    AXP2101_CHG_STOP_STATE,  //not chargin
} axp2101_chg_status_t;

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
    uint16_t readRegisters_5_8(uint8_t reghi, uint8_t reglo);
    uint16_t readRegisters_6_8(uint8_t reghi, uint8_t reglo);
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
    virtual int getChargerStatus();
    virtual int getBatteryPercent();
    virtual void setChargingLedMode(uint8_t mode);
    virtual float getBattVoltage();
    virtual float getBattDischargeCurrent();
    virtual float getBattChargeCurrent();
    virtual float getAcinVoltage();
    virtual float getAcinCurrent();
    virtual float getVbusVoltage();
    virtual float getVbusCurrent();
    virtual float getTemperature();
    virtual float getSystemVoltage();
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
    int getChargerStatus();
    int getBatteryPercent();
    float getBattVoltage();
    float getBattDischargeCurrent();
    float getBattChargeCurrent();
    float getAcinVoltage();
    float getAcinCurrent();
    float getVbusVoltage();
    float getVbusCurrent();
    float getTemperature();
    float getSystemVoltage();

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

    void setChargingLedMode(uint8_t mode);
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
    int getChargerStatus();
    int getBatteryPercent();
    float getBattVoltage();
    float getBattDischargeCurrent();
    float getBattChargeCurrent();
    float getAcinVoltage();
    float getAcinCurrent();
    float getVbusVoltage();
    float getVbusCurrent();
    float getTemperature();
    float getSystemVoltage();

protected:
    void _enableIRQ(uint8_t addr, uint8_t mask);

    int setVBACKUP(uint16_t millivolt);
    int setDCDC1(uint16_t millivolt);
    int setALDO2(uint16_t millivolt);
    int setALDO3(uint16_t millivolt);

    void setChargingLedMode(uint8_t mode);

};

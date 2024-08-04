#include <stdio.h>
#include <Wire.h>
#include "pmu.h"
#include "Sonde.h"

// 0: cleared; 1: set; 2: do not check, also query state of axp via i2c on each loop
uint8_t pmu_irq = 0;
#define PMU_IRQ             35

#define AXP192_VMIN 1800
#define AXP192_VSTEP 100


#define AXP192_IC_TYPE                          (0x03)

#define AXP192_DC_MIN 700
#define AXP192_DC_STEPS 25

#define AXP192_LDO_MIN                     (1800)
#define AXP192_LDO_STEPS                   (100)

#define AXP192_VOLTREG_DC1

// some registers:
#define AXP192_STATUS                           (0x00)
#define AXP192_MODE_CHGSTATUS                   (0x01)

// Power voltage control register
#define AXP192_DC2OUT_VOL                       (0x23)
#define AXP192_DC1OUT_VOL                       (0x26)
#define AXP192_DC3OUT_VOL                       (0x27)
#define AXP192_LDO23OUT_VOL                     (0x28)
#define AXP192_GPIO0_VOL                        (0x91)

// Power enable registers
#define AXP192_LDO23_DC123_EXT_CTL              (0x12)

// ADC control
#define AXP192_ADC_EN1                          (0x82)

// ADC results
#define AXP192_BAT_AVERVOL_H8                   (0x78)
#define AXP192_BAT_AVERVOL_L4                   (0x79)
#define AXP192_BAT_AVERCHGCUR_H8                (0x7A)
#define AXP192_BAT_AVERCHGCUR_L4                (0x7B)
#define AXP192_BAT_AVERCHGCUR_L5                (0x7B)
#define AXP192_ACIN_VOL_H8                      (0x56)
#define AXP192_ACIN_VOL_L4                      (0x57)
#define AXP192_ACIN_CUR_H8                      (0x58)
#define AXP192_ACIN_CUR_L4                      (0x59)
#define AXP192_VBUS_VOL_H8                      (0x5A)
#define AXP192_VBUS_VOL_L4                      (0x5B)
#define AXP192_VBUS_CUR_H8                      (0x5C)
#define AXP192_VBUS_CUR_L4                      (0x5D)
#define AXP192_INTERNAL_TEMP_H8                 (0x5E)
#define AXP192_INTERNAL_TEMP_L4                 (0x5F)
#define AXP192_TS_IN_H8                         (0x62)
#define AXP192_TS_IN_L4                         (0x63)
#define AXP192_GPIO0_VOL_ADC_H8                 (0x64)
#define AXP192_GPIO0_VOL_ADC_L4                 (0x65)
#define AXP192_GPIO1_VOL_ADC_H8                 (0x66)
#define AXP192_GPIO1_VOL_ADC_L4                 (0x67)
#define AXP192_BAT_AVERDISCHGCUR_H8             (0x7C)
#define AXP192_BAT_AVERDISCHGCUR_L5             (0x7D)


// Interrupt enable
#define AXP192_INTEN1                           (0x40)
#define AXP192_INTEN2                           (0x41)
#define AXP192_INTEN3                           (0x42)
#define AXP192_INTEN4                           (0x43)
#define AXP192_INTEN5                           (0x4A)

// Int clear.
#define AXP192_INTSTS1                          (0x44)
#define AXP192_INTSTS2                          (0x45)
#define AXP192_INTSTS3                          (0x46)
#define AXP192_INTSTS4                          (0x47)
#define AXP192_INTSTS5                          (0x4D)

extern SemaphoreHandle_t axpSemaphore;

/////////////////////////////////////////////////////////////////////////////////////
/// High-level functions 
PMU *PMU::getInstance(TwoWire &wire) {
    PMU *pmu = NULL;
    // Check if there is some AXP192 or AXP2101 present
    uint8_t chipid = readRegisterWire(wire, AXP192_IC_TYPE);
    // AXP192: 0x03  AXP2101:  0x4A
    if(chipid==0x03) {
        pmu = new AXP192PMU(wire);
    }
    else if (chipid==0x4A) { 
        pmu = new AXP2101PMU(wire);
    }
    return pmu;
}


int PMU::readRegisterWire(TwoWire &wire, uint8_t reg) {
    wire.beginTransmission(AXP192_SLAVE_ADDRESS);
    wire.write(reg);
    if (wire.endTransmission() != 0) {
        return -1;
    }
    wire.requestFrom(AXP192_SLAVE_ADDRESS, 1U);
    return wire.read();
}
int PMU::readRegister(uint8_t reg) {
    return readRegisterWire(_wire, reg);
}
uint16_t PMU::readRegisters_8_4(uint8_t regh, uint8_t regl)
{
    uint8_t hi = readRegister(regh);
    uint8_t lo = readRegister(regl);
    return (hi << 4) | (lo & 0x0F);
}

uint16_t PMU::readRegisters_8_5(uint8_t regh, uint8_t regl)
{
    uint8_t hi = readRegister(regh);
    uint8_t lo = readRegister(regl);
    return (hi << 5) | (lo & 0x1F);
}

int PMU::writeRegister(uint8_t reg, uint8_t val) {
    _wire.beginTransmission(AXP192_SLAVE_ADDRESS);
    _wire.write(reg);
    _wire.write(val);
    return (_wire.endTransmission() == 0) ? 0 : -1;
}
int PMU::getRegisterBit(uint8_t reg, uint8_t bit) {
    int val = readRegister(reg);
    if (val == -1) { return -1; }
    return (val >> bit) & 0x01;
}
int PMU::setRegisterBit(uint8_t reg, uint8_t bit) {
    int val = readRegister(reg);
    if (val == -1) { return -1; }
    return writeRegister(reg, (val | (1<<bit)));
}
int PMU::clearRegisterBit(uint8_t reg, uint8_t bit) {
    int val = readRegister(reg);
    if (val == -1) { return -1; }
    return writeRegister(reg, (val & ( ~(1<<bit))));
}

// Returns if there was a keypress, using the following enum defined in RX_FSK.ini:
enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

int PMU::handleIRQ() { 
   if (pmu_irq) {
      Serial.println("PMU_IRQ is set\n");
   } else {
      return -1;
   }
   int keypress = -1;
   xSemaphoreTake( axpSemaphore, portMAX_DELAY );
   keypress = getIrqKeyStatus();
   if(keypress) { Serial.printf("Keypress: %d (%s)", keypress, keypress==KP_SHORT?"short":"mid"); }
   if (pmu_irq != 2) {
       pmu_irq = 0;
   }
   xSemaphoreGive( axpSemaphore );
   return keypress;
}

int AXP192PMU::init() {
    // Initialize AXP192, for T-BEAM v1.1 or M5Stack

    // LDO2: LoRa VCC on T-BEAM, PERI_VDD on M5Core2 (LCD)
    setLDO2(3300);
    enableLDO2();
    if(sonde.config.type == TYPE_M5_CORE2) {
        // Display backlight (LCD_BL) on M5 Core2
        setDC3(3300);
        enableDC3();
        pmu_irq = 2;  // IRQ pin not connected on Core2
        // Set GPIO0 VDO to 3.3V (as is done by original M5Stack software)
        // (default value 2.8V did not have the expected effect :))
        setLDOio(3300);
        // ADC configuration: Enable monitoring of AC [bits 4,5 in enable register]
        uint8_t val = readRegister(AXP192_ADC_EN1);
        writeRegister(AXP192_ADC_EN1, val | (1 << 4) | (1 << 5) );
    } else {
        // T-Beam specific
        // GPS power on T-Beam (its the  buzzer on M5 Core2, so only enable for T-Beam)
        enableLDO3();
        // ADC configuration: Enable monitoring of USB [bits 2,3 in enable register]
        uint8_t val = readRegister(AXP192_ADC_EN1);
        writeRegister(AXP192_ADC_EN1, val | (1 << 4) | (1 << 5) );
    }
    // Common configuration for T-Beam and M5 Core2
    // DCDC2: M5Core: Unused, T-Beam: Unused, so set to disabled!! (was enabled in previous versions)
    enableDC2(false);

    // EXTEN: M5Core2: 5V Boost enable; T-Beam EXTEN
    enableEXTEN();

    // DCDC1: M5Core: MCU_VDD, T-Beam 1.1: "VCC_2.5V" == 3V3-Pin on pin header on board
    setDC1(3300);
    enableDC1();

    // ADC configuration: Enable monitor batt current [bit 6 in eable register]
    uint8_t val = readRegister(AXP192_ADC_EN1);
    writeRegister(AXP192_ADC_EN1, val | (1 << 6) );

    if (pmu_irq != 2) {
        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
           pmu_irq = 1;
        }, FALLING);
    }
    return 0;
}



////////////////////////////////////////////////////////////
/// Helper functions

int AXP192PMU::getIrqKeyStatus() {
    int status = readRegister(AXP192_INTSTS3);

    // Also clear IRQ status
    writeRegister(AXP192_INTSTS1, 0xFF);
    writeRegister(AXP192_INTSTS2, 0xFF);
    writeRegister(AXP192_INTSTS3, 0xFF);
    writeRegister(AXP192_INTSTS4, 0xFF);
    writeRegister(AXP192_INTSTS5, 0xFF);

    //
    if ( status & 0x01 ) return KP_MID;
    if ( status & 0x02 ) return KP_SHORT;
    return KP_NONE;
}


void AXP192PMU::disableAllIRQ() {
    writeRegister(AXP192_INTEN1, 0);
    writeRegister(AXP192_INTEN2, 0);
    writeRegister(AXP192_INTEN3, 0);
    writeRegister(AXP192_INTEN4, 0);
    writeRegister(AXP192_INTEN5, 0);
}

void AXP192PMU::_enableIRQ(uint8_t addr, uint8_t mask) {
    int data = readRegister(addr);
    writeRegister(addr, data | mask);
}

// we want KP_SHORT and KP_LONG interrupts...
// IRQ4, in reg 0x42h, Bit 16+17
void AXP192PMU::enableIRQ() {
   //_enableIRQ( AXP192_INTEN1, mask&0xFF );
   //_enableIRQ( AXP192_INTEN2, mask>>8 );
   _enableIRQ( AXP192_INTEN3, 0x03 );
   //_enableIRQ( AXP192_INTEN4, mask>>24 );
   //_enableIRQ( AXP192_INTEN5, mask>>32 );
}

// Functions for setting voltage output levels
int AXP192PMU::setVoltageReg(uint8_t reg, uint8_t regval) {
    int val = readRegister(reg);
    if (val==-1) return -1;
    val &= 0x80;
    val |= regval;
    return writeRegister(reg, val);
}

int AXP192PMU::setDC1(uint16_t millivolt) {
    return setVoltageReg(AXP192_DC1OUT_VOL, (millivolt-AXP192_DC_MIN)/AXP192_DC_STEPS );
}
int AXP192PMU::setDC2(uint16_t millivolt) {
    return setVoltageReg(AXP192_DC2OUT_VOL, (millivolt-AXP192_DC_MIN)/AXP192_DC_STEPS );
}
int AXP192PMU::setDC3(uint16_t millivolt) {
    return setVoltageReg(AXP192_DC3OUT_VOL , (millivolt-AXP192_DC_MIN)/AXP192_DC_STEPS );
}
int AXP192PMU::setLDO2(uint16_t millivolt) {
    return setVoltageReg(AXP192_LDO23OUT_VOL, (millivolt-AXP192_LDO_MIN)/AXP192_LDO_STEPS);
}
int AXP192PMU::setLDOio(uint16_t millivolt) {
    return setVoltageReg(AXP192_GPIO0_VOL, (millivolt-AXP192_LDO_MIN)/AXP192_LDO_STEPS);
}


// LDO23_DC123_EXT_CTL
// 0:DC-DC1, 1:DC-DC3, 2:LDO2, 3:LDO3, 4:DC-DC2, 6:EXTEN
int AXP192PMU::enableDC1(bool onoff) {
    return onoff ? setRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 0) : clearRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 0);
}
int AXP192PMU::enableDC3(bool onoff) {
    return onoff ? setRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 1) : clearRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 1);
}
int AXP192PMU::enableLDO2(bool onoff) {
    return onoff ? setRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 2) : clearRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 2);
}
int AXP192PMU::enableLDO3(bool onoff) {
    return onoff ? setRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 3) : clearRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 3);
}
int AXP192PMU::enableDC2(bool onoff) {
    return onoff ? setRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 4) : clearRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 4);
}
int AXP192PMU::enableEXTEN(bool onoff) {
    return onoff ? setRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 6) : clearRegisterBit(AXP192_LDO23_DC123_EXT_CTL, 6);
}

int AXP192PMU::enableADC(uint8_t channels) {
    uint8_t val = readRegister(AXP192_ADC_EN1);
    return writeRegister(AXP192_ADC_EN1, val | channels );
}

int AXP192PMU::isBatteryConnected() {
    return getRegisterBit(AXP192_MODE_CHGSTATUS, 5);
}
int AXP192PMU::isVbusIn() {
    return getRegisterBit(AXP192_STATUS, 5);
}
int AXP192PMU::isCharging() {
    return getRegisterBit(AXP192_MODE_CHGSTATUS, 6);
}

#define AXP192_BATT_VOLTAGE_STEP                (1.1F)
float AXP192PMU::getBattVoltage() {
    return readRegisters_8_4(AXP192_BAT_AVERVOL_H8, AXP192_BAT_AVERVOL_L4) * AXP192_BATT_VOLTAGE_STEP;
}

#define AXP192_BATT_DISCHARGE_CUR_STEP          (0.5F)
float AXP192PMU::getBattDischargeCurrent() {
    return readRegisters_8_5(AXP192_BAT_AVERDISCHGCUR_H8, AXP192_BAT_AVERDISCHGCUR_L5) * AXP192_BATT_DISCHARGE_CUR_STEP;
}

#define AXP192_BATT_CHARGE_CUR_STEP             (0.5F)
float AXP192PMU::getBattChargeCurrent() {
    return readRegisters_8_5(AXP192_BAT_AVERCHGCUR_H8, AXP192_BAT_AVERCHGCUR_L5) * AXP192_BATT_CHARGE_CUR_STEP;
}

#define AXP192_ACIN_VOLTAGE_STEP                (1.7F)
float AXP192PMU::getAcinVoltage() {
    return readRegisters_8_4(AXP192_ACIN_VOL_H8, AXP192_ACIN_VOL_L4) * AXP192_ACIN_VOLTAGE_STEP;
}

#define AXP192_ACIN_CUR_STEP                    (0.625F)
float AXP192PMU::getAcinCurrent() {
        return readRegisters_8_4(AXP192_ACIN_CUR_H8, AXP192_ACIN_CUR_L4) * AXP192_ACIN_CUR_STEP;
}

#define AXP192_VBUS_VOLTAGE_STEP                (1.7F)
float AXP192PMU::getVbusVoltage() {
    return readRegisters_8_4(AXP192_VBUS_VOL_H8, AXP192_VBUS_VOL_L4) * AXP192_VBUS_VOLTAGE_STEP;
}

#define AXP192_VBUS_CUR_STEP                    (0.375F)
float AXP192PMU::getVbusCurrent() {
    return readRegisters_8_4(AXP192_VBUS_CUR_H8, AXP192_VBUS_CUR_L4) * AXP192_VBUS_CUR_STEP;
}

#define AXP192_INTERNAL_TEMP_STEP               (0.1F)
float AXP192PMU::getTemperature() {
    return readRegisters_8_4(AXP192_INTERNAL_TEMP_H8, AXP192_INTERNAL_TEMP_L4) * AXP192_INTERNAL_TEMP_STEP - 144.7;
}

//////////////////////////////////////////////////////////////////

/////// Functions for AXP2101

// Registers
#define AXP2101_PMU_STATUS1			 (0x00)
#define AXP2101_PMU_STATUS2			 (0x01)
#define AXP2101_CHARGE_GAUGE_WDT_CTRL            (0x18)
#define AXP2101_BTN_BAT_CHG_VOL_SET              (0x6A)
#define AXP2101_DC_ONOFF_DVM_CTRL                (0x80)
#define AXP2101_LDO_ONOFF_CTRL0                  (0x90)
#define AXP2101_LDO_ONOFF_CTRL1                  (0x91)

// Not the right name....
#define AXP2101_LDO_VOL0_CTRL                    (0x82)

#define AXP2101_LDO_VOL1_CTRL                    (0x93)
#define AXP2101_LDO_VOL2_CTRL                    (0x94)

#define AXP2101_ADC_CHANNEL_CTRL		 (0x30)
#define AXP2101_PMU_ADC0			 (0x34)
#define AXP2101_PMU_ADC1			 (0x36)
#define AXP2101_PMU_ADC2			 (0x38)

// vterm_cfg: Bit 2:0, 4.2V = 011 (3)
#define AXP2101_CHG_V_CFG		         (0x64)
// ICC_CFG: Bit 4:0: constant current charge current limit, 25*N mA (N<=8), 200+100*(N-8) (N>8)
#define AXP2101_ICC_CFG				 (0x62)

// Interrupt enable
#define AXP2101_INTEN1                           (0x40)
#define AXP2101_INTEN2                           (0x41)
#define AXP2101_INTEN3                           (0x42)

// Interrupt status
#define AXP2101_INTSTS1                          (0x48)
#define AXP2101_INTSTS2                          (0x49)
#define AXP2101_INTSTS3                          (0x4A)

// Constants
#define AXP2101_ALDO_VOL_MIN                     (500)
#define AXP2101_ALDO_VOL_STEPS                   (100)

#define AXP2101_BTN_VOL_MIN                      (2600)
#define AXP2101_BTN_VOL_STEPS                    (100)

// 200 + 100*(11-8) = 500
#define AXP2101_CHG_CUR_500MA			 (0x0B)
#define AXP2101_CHG_VOL_4V2			 (3)


// return 0: ok, -1: error
int AXP2101PMU::init() {
    // Initialize AXP2101, for T-BEAM v1.2

    // Hard-coded for now, disable DC2/3/4/5 ALDO1,4 BLDO1/2 DLDO1/2
    int val = readRegister(AXP2101_DC_ONOFF_DVM_CTRL);
    writeRegister(AXP2101_DC_ONOFF_DVM_CTRL, val & (~0x1E));  // clear Bit 1,2,3,4 (DC2/3/4/5)   

    // clear bit 0 (aldo1), 3 (aldo4), 4,5(bldo1/2), 7 (dldo1)
    val = readRegister(AXP2101_LDO_ONOFF_CTRL0);
    writeRegister(AXP2101_LDO_ONOFF_CTRL0, val & (~0xB9));

    // clear bit 0 (dldo2)
    val = readRegister(AXP2101_LDO_ONOFF_CTRL1);
    writeRegister(AXP2101_LDO_ONOFF_CTRL1, val & (~0x01));

    // Set PowerVDD to 3100mV (GNSS RTC) -- reg 6A [MS412FE data sheet: charge volt 2.8-3.3; standard value 3.1]
    val =  readRegister(AXP2101_BTN_BAT_CHG_VOL_SET);
    if (val == -1) return -1;
    val  &= 0xF8;
    val |= (3100 - AXP2101_BTN_VOL_MIN) / AXP2101_BTN_VOL_STEPS;
    writeRegister(AXP2101_BTN_BAT_CHG_VOL_SET, val);

    setRegisterBit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 2);

    // ESP32 VDD 3300mV
    // No need to set, automatically open , Don't close it
    val = readRegister(AXP2101_LDO_VOL0_CTRL);
    Serial.printf("VDD: %x\n", val);

    // LoRa VDD 3300mV on ALDO2
    val =  readRegister(AXP2101_LDO_VOL1_CTRL);
    if (val == -1) return -1;
    val &= 0xE0;
    val |= (3300 - AXP2101_ALDO_VOL_MIN) / AXP2101_ALDO_VOL_STEPS;
    writeRegister(AXP2101_LDO_VOL1_CTRL, val);
    setRegisterBit(AXP2101_LDO_ONOFF_CTRL0, 1);

    // GNSS VDD 3300mV on ALDO3
    val =  readRegister(AXP2101_LDO_VOL2_CTRL);
    if (val == -1) return -1;
    val &= 0xE0;
    val |= (3300 - AXP2101_ALDO_VOL_MIN) / AXP2101_ALDO_VOL_STEPS;
    writeRegister(AXP2101_LDO_VOL2_CTRL, val);
    setRegisterBit(AXP2101_LDO_ONOFF_CTRL0, 2);

    if (pmu_irq != 2) {
        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
           pmu_irq = 1;
        }, FALLING);
    }

    // Set charging configuration: 500mA, 4.2V cut off

    // Set constant current charge limit to 500mA (reguster 0x62)
    // Data sheep (7.3.3.) tells that default value is 1.024 A (which should be fine??)
    // Data sheet (register table) tells that default value is "{EFUSE,0b,EFUSE}", whatever that is
    // Let's set this to 500 mA manually to be sure
    val = readRegister(AXP2101_ICC_CFG);
    if (val == -1) return -1;
    val &= 0xE0;
    writeRegister(AXP2101_ICC_CFG, val | AXP2101_CHG_CUR_500MA);

#if 0
    // Set cut-off voltage to 4.2V (register 0x63)
    // This is the default value, so setting it should not be needed.
    val = readRegister(AXP2101_CHG_V_CFG);
    if (val == -1) return -1;
    val &= 0xFC;
    writeRegister(AXP2101_CHG_V_CFG, val | AXP2101_CHG_VOL_4V2);
#endif

    // Disable TS measurement, enable vsys, vbus, vbat measurement
    // Disable TS is important for T-Beam 1.2 (no TS thermistor), otherwise it will not charge.
    writeRegister(AXP2101_ADC_CHANNEL_CTRL, 0x0d);

    // Clear all IRQ
    getIrqKeyStatus();

    // precharge current (reg 0x61): default value (0101b) should be fine
    // Termination current (125mA default value) should be fine as well
#if 0
    // Just some debug code
    Serial.println("All good \n");

    for(int i=0; i<0x80; i++) {
        val = readRegister(i);
        Serial.printf("Reg %x: %x (%d)\n", i, val, val);
    }
#endif

    return 0;
}

void AXP2101PMU::disableAllIRQ() {
    writeRegister(AXP2101_INTEN1, 0);
    writeRegister(AXP2101_INTEN2, 0);
    writeRegister(AXP2101_INTEN3, 0);
}   

void AXP2101PMU::_enableIRQ(uint8_t addr, uint8_t mask) {
    int data = readRegister(addr);              
    writeRegister(addr, data | mask);           
}

// we want KP_SHORT and KP_LONG interrupts...
// IRQen1, in req 0x41h, Bit 2(long)+3(short) (10+11 global)
void AXP2101PMU::enableIRQ() {
   //_enableIRQ( AXP2101_INTEN1, mask&0xFF );
   _enableIRQ( AXP2101_INTEN2, 0x0C );
   //_enableIRQ( AXP2101_INTEN3, 0x03 );
}

int AXP2101PMU::getIrqKeyStatus()  { 
    int status = readRegister(AXP2101_INTSTS2);
   
    // Also clear IRQ status
    writeRegister(AXP2101_INTSTS1, 0xFF);
    writeRegister(AXP2101_INTSTS2, 0xFF);
    writeRegister(AXP2101_INTSTS3, 0xFF);

    // 
    if ( status & 0x04 ) return KP_MID;
    if ( status & 0x08 ) return KP_SHORT; 
    return KP_NONE;
}

int AXP2101PMU::isBatteryConnected() { 
	int val = readRegister(AXP2101_PMU_STATUS1);
	// PMU status1, bit 3: battery present
	return (val & 0x08) ? 1 : 0;
}
int AXP2101PMU::isVbusIn() { return -1; }
int AXP2101PMU::isCharging() { 
	int val = readRegister(AXP2101_PMU_STATUS2);	
	// PMU status2, bit 6:5 == 01 => charge (00: standby, 10: discharge)
	return (val & 0x60) == 0x20 ? 1 : 0;
}
float AXP2101PMU::getBattVoltage() {   // returns mV
	int hi = readRegister(AXP2101_PMU_ADC0) & 0x3F;
	int lo = readRegister(AXP2101_PMU_ADC0+1);
	return (float)((hi<<8) | lo );
}

float AXP2101PMU::getBattDischargeCurrent() { return -1; }
float AXP2101PMU::getBattChargeCurrent() { return -1; }
float AXP2101PMU::getAcinVoltage() { return -1; }
float AXP2101PMU::getAcinCurrent() { return -1; }
float AXP2101PMU::getVbusVoltage() { 
	int hi = readRegister(AXP2101_PMU_ADC2) & 0x3F;
	int lo = readRegister(AXP2101_PMU_ADC2+1);
	Serial.printf("vbus: %d %d\n", hi, lo);
	float vbus = (float)((hi<<8) | lo );
	if(vbus>7000) return -1;
	return vbus;
}
float AXP2101PMU::getVbusCurrent() { return -1; }
float AXP2101PMU::getTemperature() { return -1; }



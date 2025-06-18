#include "SIM_config.h"

#if AP_SIM_INA3221_ENABLED

#include "SIM_I2CDevice.h"

#include <AP_Common/Bitmask.h>

/*

Testing:

param set BATT_MONITOR 30
reboot
param set BATT_I2C_ADDR 0x42
param set BATT_I2C_BUS 1
param set BATT_CHANNEL 2
reboot

*/

namespace SITL {

class INA3221 : public I2CDevice
{
public:

    INA3221();

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    void update(const class Aircraft &aircraft) override;

private:

    static const uint16_t MANUFACTURER_ID = 0b0101010001001001; // from datasheet p25
    static const uint16_t DIE_ID = 0b0011001000100000; // from datasheet p25

    // All 16-bit INA3221 registers are two 8-bit bytes via the I2C
    // interface. Table 4 shows a register map for the INA3221.
    union Registers {
        uint16_t word[256];
        struct PACKED ByName {
            union {
                uint16_t word;
                struct PACKED {
                    uint16_t mode : 3;
                    uint16_t shunt_voltage_conversiontime : 3;
                    uint16_t bus_voltage_conversiontime : 3;
                    uint16_t averaging_mode : 3;
                    uint16_t ch1_enable : 1;
                    uint16_t ch2_enable : 1;
                    uint16_t ch3_enable : 1;
                    uint16_t reset : 1;
                } bits;
            } configuration;
            uint16_t Channel_1_Shunt_Voltage;
            uint16_t Channel_1_Bus_Voltage;
            uint16_t Channel_2_Shunt_Voltage;
            uint16_t Channel_2_Bus_Voltage;
            uint16_t Channel_3_Shunt_Voltage;
            uint16_t Channel_3_Bus_Voltage;
            uint16_t Channel_1_CriticalAlertLimit;
            uint16_t Channel_1_WarningAlertLimit;
            uint16_t Channel_2_CriticalAlertLimit;
            uint16_t Channel_2_WarningAlertLimit;
            uint16_t Channel_3_CriticalAlertLimit;
            uint16_t Channel_3_WarningAlertLimit;
            uint16_t Shunt_VoltageSum;
            uint16_t Shunt_VoltageSumLimit;
            uint16_t Mask_Enable;
            uint16_t Power_ValidUpperLimit;
            uint16_t Power_ValidLowerLimit;
            uint16_t unused[236];
            uint16_t ManufacturerID;
            uint16_t Die_ID;
        } byname;
    } registers;

    // 256 2-byte registers:
    assert_storage_size<Registers::ByName, 512> assert_storage_size_registers_reg;

    Bitmask<256> writable_registers;

    void reset();
};

} // namespace SITL

#endif  // AP_SIM_INA3221_ENABLED

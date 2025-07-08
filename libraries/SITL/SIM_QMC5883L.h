#pragma once

#include "SIM_config.h"

#if AP_SIM_COMPASS_QMC5883L_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class QMC5883L : public I2CDevice
{
public:

    QMC5883L();

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    union Registers {
        uint8_t byte[256];
        struct PACKED ByName {
            uint8_t DATA_OUTPUT_X_low;
            uint8_t DATA_OUTPUT_X_high;
            uint8_t DATA_OUTPUT_Y_low;
            uint8_t DATA_OUTPUT_Y_high;
            uint8_t DATA_OUTPUT_Z_low;
            uint8_t DATA_OUTPUT_Z_high;
            uint8_t REG_STATUS;
            uint8_t unused1[2]; // unused
            uint8_t REG_CONF1;
            uint8_t REG_CONF2;
            uint8_t ZEROX_ZEROB; // magic register number from driver
            uint8_t ZEROX_ZEROC; // magic register from driver should always be 0x01
            uint8_t REG_ID;
            uint8_t unused2[242];
        } byname;
    } registers;

    // 256 1-byte registers:
    assert_storage_size<Registers::ByName, 256> assert_storage_size_registers_reg UNUSED_PRIVATE_MEMBER;

    Bitmask<256> writable_registers;

    void reset();
};

} // namespace SITL

#endif  // AP_SIM_COMPASS_QMC5883L_ENABLED

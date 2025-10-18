#include "SIM_config.h"

#if AP_SIM_RF_LIGHTWAREI2C_LEGACY16BIT_ENABLED

#include "SIM_RF_LightWareI2C_Legacy16Bit.h"

#include <stdio.h>

void SITL::LightWareI2C_Legacy16Bit::init()
{
    add_register("RANGE", LightWareI2C_Legacy16BitDevReg::RANGE, I2CRegisters::RegMode::RDONLY);
    set_register(LightWareI2C_Legacy16BitDevReg::RANGE, (uint16_t)0U);

    add_register("LOST_SIGNAL_TIMEOUT_READ", LightWareI2C_Legacy16BitDevReg::LOST_SIGNAL_TIMEOUT_READ, I2CRegisters::RegMode::RDONLY);
    set_register(LightWareI2C_Legacy16BitDevReg::LOST_SIGNAL_TIMEOUT_READ, (uint16_t)0U);

    add_register("LOST_SIGNAL_TIMEOUT_WRITE", LightWareI2C_Legacy16BitDevReg::LOST_SIGNAL_TIMEOUT_WRITE, I2CRegisters::RegMode::WRONLY);
    set_register(LightWareI2C_Legacy16BitDevReg::LOST_SIGNAL_TIMEOUT_WRITE, (uint16_t)0U);
}

int SITL::LightWareI2C_Legacy16Bit::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    // so this is a bit crap.  The driver first tries to write to a
    // register on an sf20 device and if nothing interesting happens
    // tries the legacy driver (which we play with).  We have to
    // ignore the probe for the sf20:
    if (data->nmsgs == 1) {
        if (data->msgs[0].len > 6) {
            return -1;
        }
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_addr = data->msgs[0].buf[0];
        if (reg_addr == 0x3f) {
            return -1;
        }
    }

    return I2CRegisters_16Bit::rdwr(data);
}

#endif  // AP_SIM_RF_LIGHTWAREI2C_LEGACY16BIT_ENABLED

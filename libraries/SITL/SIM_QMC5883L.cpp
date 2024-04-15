#include "SIM_config.h"

#if AP_SIM_COMPASS_QMC5883L_ENABLED

#include "SIM_QMC5883L.h"

// FIXME: read the datasheet :-)

#include <SITL/SITL.h>
#include <SITL/SIM_Aircraft.h>

#include <stdio.h>

#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A

#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_ODR_100HZ (0x02 << 2)

#define QMC5883L_OSR_512 (0x00 << 6)

#define QMC5883L_RNG_8G (0x01 << 4)

SITL::QMC5883L::QMC5883L()
{
    writable_registers.set(0);
    writable_registers.set(0x0b);
    writable_registers.set(0x20);
    writable_registers.set(0x21);
    writable_registers.set(QMC5883L_REG_CONF1);
    writable_registers.set(QMC5883L_REG_CONF2);

    reset();
}

void SITL::QMC5883L::reset()
{
    memset(registers.byte, 0, ARRAY_SIZE(registers.byte));
    registers.byname.ZEROX_ZEROC = 0x01;
    registers.byname.REG_STATUS = 0x0;
    registers.byname.REG_ID = 0xFF;
}


int SITL::QMC5883L::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_addr = data->msgs[0].buf[0];
        for (uint8_t i=0; i<data->msgs[1].len; i++) {
            const uint8_t register_value = registers.byte[reg_addr+i];
            data->msgs[1].buf[i] = register_value;

            // FIXME: is this really how the status data-ready bit works?
            if (reg_addr == 0x05) {  // that's the last data register...
                registers.byname.REG_STATUS &= ~0x04;
            }
        }
        return 0;
    }

    if (data->nmsgs == 1) {
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_addr = data->msgs[0].buf[0];
        if (!writable_registers.get(reg_addr)) {
            AP_HAL::panic("Register 0x%02x is not writable!", reg_addr);
        }
        const uint8_t register_value = data->msgs[0].buf[1];
        ::fprintf(stderr, "Setting register (0x%x) to 0x%x\n", reg_addr, register_value);
        registers.byte[reg_addr] = register_value;
        return 0;
    }

    return -1;
};

void SITL::QMC5883L::update(const Aircraft &aircraft)
{

    // FIXME: swipe stuff from AP_Compass_SITL here.

    // FIXME: somehow decide to use the simulated compass offsets etc
    // from SITL

    Vector3f f = AP::sitl()->state.bodyMagField;

    // Vector3<int16_t> field
    // int16_t str_x = 123;
    // int16_t str_y = -56;
    // int16_t str_z = 1;

    f.rotate_inverse(ROTATION_ROLL_180_YAW_270);

    f.x = -f.x;
    f.z = -f.z;

    f.x *= 3;
    f.y *= 3;
    f.z *= 3;

    Vector3<int16_t> k {
        k.x = htole16((int16_t)f.x),
        k.y = htole16((int16_t)f.y),
        k.z = htole16((int16_t)f.z)
    };

    if (registers.byname.REG_CONF1 & QMC5883L_MODE_CONTINUOUS) {
        // FIXME: clock according to configuration here
        registers.byname.REG_STATUS |= 0x04;
        registers.byname.DATA_OUTPUT_X_low = k.x & 0xFF;
        registers.byname.DATA_OUTPUT_X_high = k.x >> 8;
        registers.byname.DATA_OUTPUT_Y_low = k.y & 0xFF;
        registers.byname.DATA_OUTPUT_Y_high = k.y >> 8;
        registers.byname.DATA_OUTPUT_Z_low = k.z & 0xFF;
        registers.byname.DATA_OUTPUT_Z_high = k.z >> 8;
    }
}

#endif  // AP_SIM_COMPASS_QMC5883L_ENABLED

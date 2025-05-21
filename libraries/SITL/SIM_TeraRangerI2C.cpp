#include "SIM_config.h"

#if AP_SIM_TERARANGERI2C_ENABLED

#include "SIM_TeraRangerI2C.h"

using namespace SITL;

int TeraRangerI2C::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // read/write message
        const uint8_t reg_base_addr = data->msgs[0].buf[0];
        if (reg_base_addr == 0) {
            AP_HAL::panic("Should not get combed read/write for TRIGGER_READING");
        }
    }

    if (data->msgs[0].flags == I2C_M_RD) {
        if (reading_start_us != 0) {
            if (data->nmsgs != 1) {
                AP_HAL::panic("Unexpected number of i2c messages");
            }
            const auto &msg = data->msgs[0];
            if (reading_start_us == 0) {
                AP_HAL::panic("Attempt to read sample without requesting it");
            }
            const uint32_t now_us = MAX(AP_HAL::micros(), 1U);
            if (now_us - reading_start_us < 500) {
                AP_HAL::panic("Attempt to read sensor before data would be ready");
            }
            const uint16_t reading = MIN(rangefinder_range*1000, 65535);
            msg.buf[0] = reading >> 8;
            msg.buf[1] = reading & 0xff;
            msg.buf[2] = crc_crc8(msg.buf, 2);
            reading_start_us = 0;
            return 0;
        }
    } else if (data->msgs[0].flags == 0) {  // write request
        const uint8_t reg_base_addr = data->msgs[0].buf[0];
        if (reg_base_addr == 0) {
            if (reading_start_us != 0) {
                AP_HAL::panic("Requesting sample without reading previous one?");
            }
            reading_start_us = MAX(AP_HAL::micros(), 1U);
            return 0;
        }
    } else {
        AP_HAL::panic("Bad flags in i2c transfer %02x", data->msgs[0].flags);
    }

    return I2CRegisters_8Bit::rdwr(data);
}

#endif  // AP_SIM_TERARANGERI2C_ENABLED

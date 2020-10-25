#include "SIM_MaxSonarI2CXL.h"

#include <SITL/SITL.h>
#include <SITL/SIM_Aircraft.h>

#include <stdio.h>

int SITL::MaxSonarI2CXL::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    const uint32_t now = AP_HAL::millis();

    struct I2C::i2c_msg &msg = data->msgs[0];
    if (msg.flags == I2C_M_RD) {
        // driver is attempting to receive reading...
        if (now - cmd_take_reading_received_ms < 20) {
            // not sure we ought to be returning -1 here - what does
            // the real device do?  return stale data?  garbage data?
            return -1;
        }
        if (msg.len != 2) {
            AP_HAL::panic("Unxpected message length (%u)", msg.len);
        }

        const uint16_t range_cm = rangefinder_range * 100;

        msg.buf[0] = range_cm >> 8;
        msg.buf[1] = range_cm & 0xff;

        return 0;
    }

    const uint8_t CMD_TAKE_READING  = 0x51;
    const uint8_t cmd = msg.buf[0];
    if (cmd != CMD_TAKE_READING) {
        AP_HAL::panic("Unknown command (%u)", cmd);
    }
    cmd_take_reading_received_ms = now;

    return 0;
};

void SITL::MaxSonarI2CXL::update(const Aircraft &aircraft)
{
    rangefinder_range = aircraft.rangefinder_range();
}

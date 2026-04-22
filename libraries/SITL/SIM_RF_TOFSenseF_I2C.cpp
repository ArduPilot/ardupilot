#include "SIM_config.h"

#if AP_SIM_RF_TOFSENSEF_I2C_ENABLED

#include "SIM_RF_TOFSenseF_I2C.h"

#include <stdio.h>

#include <GCS_MAVLink/GCS.h>

void SITL::TOFSenseF_I2C::update(const class Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (last_update_ms - now_ms < 50) {
        return;
    }
    last_update_ms = now_ms;

    range = aircraft.rangefinder_range();
}

int SITL::TOFSenseF_I2C::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        abort();
    }

    switch (data->msgs[0].flags) {
    case I2C_RDWR:
        return SITL::TOFSenseF_I2C::take_reading(data);
    case I2C_M_RD:
        return SITL::TOFSenseF_I2C::read_data(data);
    default:
        break;
    }
    abort();
}

int SITL::TOFSenseF_I2C::take_reading(I2C::i2c_rdwr_ioctl_data *&data)
{
    auto msg = data->msgs[0];
    if (msg.flags != I2C_RDWR) {
        abort();
    }

    if (msg.len != 2) {
        abort();
    }

    if (msg.buf[0] != 0x24) {  // get distance
        abort();
    }

    if (msg.buf[1] != 0x28) {  // get signal status
        abort();
    }

    reading_requested = true;

    return 0;
}

int SITL::TOFSenseF_I2C::read_data(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (!reading_requested) {
        abort();
    }

    auto msg = data->msgs[0];

    if (msg.flags != I2C_M_RD) {
        abort();
    }

    if (msg.len != 8) {
        abort();
    }

    put_le32_ptr(&msg.buf[0], uint32_t(range*1000));  // m -> mm

    // 1 here means healthy, 2 here is current signal strength
    const uint32_t signal_strength_and_status = (2U<<16 | 1);
    put_le32_ptr(&msg.buf[4], signal_strength_and_status);

    return 0;
}

#endif  // AP_SIM_RF_TOFSENSEF_I2C_ENABLED

#pragma once

#include "SIM_I2CDevice.h"

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SIM_AIRSPEEDDLVR_ENABLED
#define AP_SIM_AIRSPEEDDLVR_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_SIM_AIRSPEEDDLVR_ENABLED

namespace SITL {

class Airspeed_DLVR : public I2CDevice
{
public:

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

private:

    float pressure;
    float temperature;

    // time we last updated the measurements (simulated internal
    // workings of sensor)
    uint32_t last_update_ms;

    // time we last responded to an i2c request for data:
    uint32_t last_sent_ms;
};

} // namespace SITL

#endif  // AP_SIM_AIRSPEEDDLVR_ENABLED

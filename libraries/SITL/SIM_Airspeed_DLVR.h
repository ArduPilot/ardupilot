#pragma once

#include "SIM_config.h"

#if AP_SIM_AIRSPEED_DLVR_ENABLED

#include "SIM_I2CDevice.h"

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

#endif  // AP_SIM_AIRSPEED_DLVR_ENABLED

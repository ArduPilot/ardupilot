#pragma once

#include "SIM_I2CDevice.h"

namespace SITL {

class MaxSonarI2CXL : public I2CDevice
{
public:

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    uint32_t cmd_take_reading_received_ms;

    float rangefinder_range;
};

} // namespace SITL

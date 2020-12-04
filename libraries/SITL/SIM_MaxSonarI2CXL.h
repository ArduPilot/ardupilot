#pragma once

#include "SIM_I2CDevice.h"

namespace SITL {

class MaxSonarI2CXL : public I2CDevice, public I2CCommandResponseDevice
{
public:

    // methods to make I2CCommandResponseDevice happy:
    uint8_t command_take_reading() const override { return 0x51; }
    uint16_t reading() const override {
        return rangefinder_range * 100;
    };

    void update(const class Aircraft &aircraft) override {
        rangefinder_range = aircraft.rangefinder_range();
    }

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CCommandResponseDevice::rdwr(data);
    }


private:

    float rangefinder_range;
};

} // namespace SITL

#pragma once

#include "SIM_I2CDevice.h"

// https://www.eagletreesystems.com/Manuals/microsensor-i2c.pdf

namespace SITL {

class EagleTreeAirspeedv3 : public I2CDevice, public I2CCommandResponseDevice
{
public:

    // methods to make I2CCommandResponseDevice happy:
    uint8_t command_take_reading() const override { return 0x07; }
    uint16_t reading() const override {
        const float airspeed_kph = airspeed / 1000 * 3600;
        return airspeed_kph;  // TODO: work out endianness
    };

    void update(const class Aircraft &aircraft) override {
        airspeed = aircraft.get_airspeed_pitot();
    }

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CCommandResponseDevice::rdwr(data);
    }

private:

    float airspeed;
};

} // namespace SITL

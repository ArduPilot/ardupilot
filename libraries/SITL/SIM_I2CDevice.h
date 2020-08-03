#pragma once

#include "SIM_I2C.h"

#include <SITL/SIM_Aircraft.h>

namespace SITL {

class I2CDevice {
public:
    virtual void update(const class Aircraft &aircraft) { }

    virtual int rdwr(I2C::i2c_rdwr_ioctl_data *&data) = 0;
};

} // namespace SITL

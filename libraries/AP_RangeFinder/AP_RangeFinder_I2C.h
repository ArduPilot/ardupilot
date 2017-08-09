#pragma once

#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_I2C : public AP_RangeFinder_Backend
{

public:

protected:

    virtual uint8_t addr() const = 0;
    virtual bool probe() = 0;

    // constructor
    AP_RangeFinder_I2C(RangeFinder::RangeFinder_State &_state) :
        AP_RangeFinder_Backend(_state)
        { }

    bool probe_buses();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

private:

};

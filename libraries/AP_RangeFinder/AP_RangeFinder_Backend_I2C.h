#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_Backend_I2C : public AP_RangeFinder_Backend
{
public:

protected:

    // constructor
    AP_RangeFinder_Backend_I2C(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::I2CDevice &_dev)
        : AP_RangeFinder_Backend(_state, _params)
        , dev(_dev)
        { }

    // configures the rangefinder, deletes it if unable to init it.
    static AP_RangeFinder_Backend_I2C *configure(AP_RangeFinder_Backend_I2C *);

    AP_HAL::I2CDevice &dev;

    virtual bool init() = 0;
};

#endif  // AP_RANGEFINDER_ENABLED

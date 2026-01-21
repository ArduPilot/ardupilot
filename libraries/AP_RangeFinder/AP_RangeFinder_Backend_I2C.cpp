#include "AP_RangeFinder_Backend_I2C.h"

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

AP_RangeFinder_Backend_I2C *AP_RangeFinder_Backend_I2C::configure(AP_RangeFinder_Backend_I2C *sensor)
{
    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

#endif  // AP_RANGEFINDER_ENABLED

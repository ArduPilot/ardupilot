#include "AP_RangeFinder_I2C.h"
#include <utility>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

bool AP_RangeFinder_I2C::probe_buses()
{
    uint8_t _addr = addr();
    for (uint8_t i=0; i<2; i++) {
        _dev = std::move(hal.i2c_mgr->get_device(i, _addr));
        if (probe()) {
            return true;
        }
    }
    return false;
}

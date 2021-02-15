/*
* AP_PM_SNGCJA5.h
 *
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_PM_SNGCJA5 {
    public:
        // init - initialize sensor library
        void init();

        // retrieve latest sensor data - returns true if new data is available
        void update();

    private:
        AP_HAL::OwnPtr<AP_HAL::Device> dev;

        void read_frames(void);

        uint32_t _last_read_ms;

        // init(bus) - attempt to initialise sensor on bus
        bool init(int8_t bus);
};

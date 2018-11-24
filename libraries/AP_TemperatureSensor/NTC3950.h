#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class NTC3950 {
public:

    bool init(void);
    float temperature(void) { return _temperature; } // temperature in degrees C
    bool healthy(void) { // do we have a valid temperature reading?
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        return true;
#endif
        return _healthy;
    }
    void update_reading(void);

    AP_HAL::OwnPtr<AP_HAL::AnalogSource> _analog_source;

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    float _temperature = 42.42; // degrees C
#else
    float _temperature; // degrees C
#endif
    bool _healthy;
};

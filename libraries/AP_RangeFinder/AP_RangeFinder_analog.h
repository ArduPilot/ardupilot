/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RangeFinder_analog_H__
#define __AP_RangeFinder_analog_H__

#include "RangeFinder.h"
#include <Filter.h>

class AP_RangeFinder_analog
{
public:
    // constructor
    AP_RangeFinder_analog();

    // initialise the rangefinder
    void Init(void *adc);
    
    // return distance in centimeters
    float distance_cm(void);

    // return raw voltage. Used for calibration
    float voltage(void);    

    // return true if the sonar is in the configured range
    bool in_range(void);

    // return true if enabled
    bool enabled(void) { return (bool)_enabled.get(); }

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::AnalogSource *_source;
    AP_Int8  _pin;
    AP_Int8  _stop_pin;
    AP_Int16 _settle_time_ms;
    AP_Float _scaling;
    AP_Float _offset;
    AP_Int8  _function;
    AP_Int16 _min_distance_cm;
    AP_Int16 _max_distance_cm;
    AP_Int8 _enabled;
};
#endif // __AP_RangeFinder_analog_H__


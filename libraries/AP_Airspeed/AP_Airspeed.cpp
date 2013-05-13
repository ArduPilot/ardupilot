/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *   APM_Airspeed.cpp - airspeed (pitot) driver
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License
 *   as published by the Free Software Foundation; either version 2.1
 *   of the License, or (at your option) any later version.
 */

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Airspeed.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] PROGMEM = {

    // @Param: ENABLE
    // @DisplayName: Airspeed enable
    // @Description: enable airspeed sensor
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, AP_Airspeed, _enable, 1),

    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    AP_GROUPINFO("USE",    1, AP_Airspeed, _use, 0),

    // @Param: OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    AP_GROUPINFO("OFFSET", 2, AP_Airspeed, _offset, 0),

    // @Param: RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    AP_GROUPINFO("RATIO",  3, AP_Airspeed, _ratio, 1.9936f),

    AP_GROUPEND
};

/*
  this scaling factor converts from the old system where we used a 
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

// calibrate the airspeed. This must be called at least once before
// the get_airspeed() interface can be used
void AP_Airspeed::calibrate()
{
    float sum = 0;
    uint8_t c;
    if (!_enable) {
        return;
    }
    _source->voltage_average_ratiometric();
    for (c = 0; c < 10; c++) {
        hal.scheduler->delay(100);
        sum += _source->voltage_average_ratiometric() * SCALING_OLD_CALIBRATION;
    }
    float raw = sum/c;
    _offset.set_and_save(raw);
    _airspeed = 0;
}

// read the airspeed sensor
void AP_Airspeed::read(void)
{
    float airspeed_pressure;
    if (!_enable) {
        return;
    }
    float raw               = _source->voltage_average_ratiometric() * SCALING_OLD_CALIBRATION;
    airspeed_pressure       = max(raw - _offset, 0);
    float new_airspeed      = sqrtf(airspeed_pressure * _ratio);
    _airspeed               = 0.7f * _airspeed  +  0.3f * new_airspeed;
}

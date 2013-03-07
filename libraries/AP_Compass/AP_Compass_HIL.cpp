/*
 *       AP_Compass_HIL.cpp - Arduino Library for HIL model of HMC5843 I2C Magnetometer
 *       Code by James Goppert. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and / or
 *               modify it under the terms of the GNU Lesser General Public
 *               License as published by the Free Software Foundation; either
 *               version 2.1 of the License, or (at your option) any later version.
 */


#include <AP_HAL.h>
#include "AP_Compass_HIL.h"

extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_HIL::read()
{
    // get offsets
    Vector3f ofs = _offset.get();

    // apply motor compensation
    if(_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset = _motor_compensation.get() * _thr_or_curr;
    }else{
        _motor_offset.x = 0;
        _motor_offset.y = 0;
        _motor_offset.z = 0;
    }

    // return last values provided by setHIL function
    mag_x = _hil_mag.x + ofs.x + _motor_offset.x;
    mag_y = _hil_mag.y + ofs.y + _motor_offset.y;
    mag_z = _hil_mag.z + ofs.z + _motor_offset.z;

    // values set by setHIL function
    last_update = hal.scheduler->micros();      // record time of update
    return true;
}

// Update raw magnetometer values from HIL data
//
void AP_Compass_HIL::setHIL(float _mag_x, float _mag_y, float _mag_z)
{
    _hil_mag.x = _mag_x;
    _hil_mag.y = _mag_y;
    _hil_mag.z = _mag_z;
    healthy = true;
}

void AP_Compass_HIL::accumulate(void)
{
	// nothing to do
}

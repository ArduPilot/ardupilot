/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

// constructor
AP_Compass_HIL::AP_Compass_HIL() : Compass() 
{
    product_id = AP_COMPASS_TYPE_HIL;
    _setup_earth_field();
}

// setup _Bearth
void AP_Compass_HIL::_setup_earth_field(void)
{
    // assume a earth field strength of 400
    _Bearth(400, 0, 0);
	
    // rotate _Bearth for inclination and declination. -66 degrees
    // is the inclination in Canberra, Australia
    Matrix3f R;
    R.from_euler(0, ToRad(66), _declination.get());
    _Bearth = R * _Bearth;
}

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

#define MAG_OFS_X 5.0
#define MAG_OFS_Y 13.0
#define MAG_OFS_Z -18.0

// Update raw magnetometer values from HIL data
//
void AP_Compass_HIL::setHIL(float roll, float pitch, float yaw)
{
    Matrix3f R;

    // create a rotation matrix for the given attitude
    R.from_euler(roll, pitch, yaw);

    if (_last_declination != _declination.get()) {
        _setup_earth_field();
        _last_declination = _declination.get();
    }

    // convert the earth frame magnetic vector to body frame, and
    // apply the offsets
    _hil_mag = R.mul_transpose(_Bearth);
    _hil_mag -= Vector3f(MAG_OFS_X, MAG_OFS_Y, MAG_OFS_Z);

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _hil_mag.rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _hil_mag.rotate((enum Rotation)_orientation.get());

    // and add in AHRS_ORIENTATION setting
    _hil_mag.rotate(_board_orientation);

    healthy = true;
}

void AP_Compass_HIL::accumulate(void)
{
    // nothing to do
}

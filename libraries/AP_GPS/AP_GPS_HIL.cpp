// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  Hardware in the loop gps class.
//	Code by James Goppert
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//

#include <AP_HAL.h>
#include "AP_GPS_HIL.h"

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_HIL::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
    idleTimeout = 1200;
}

bool AP_GPS_HIL::read(void)
{
    bool result = _updated;

    // return true once for each update pushed in
    _updated = false;
    return result;
}

void AP_GPS_HIL::setHIL(uint32_t _time, float _latitude, float _longitude, float _altitude,
                        float _ground_speed, float _ground_course, float _speed_3d, uint8_t _num_sats)
{
    time                        = _time;
    latitude            = _latitude*1.0e7;
    longitude           = _longitude*1.0e7;
    altitude            = _altitude*1.0e2;
    ground_speed        = _ground_speed*1.0e2;
    ground_course       = _ground_course*1.0e2;
    speed_3d            = _speed_3d*1.0e2;
    num_sats            = _num_sats;
    fix                         = true;
    new_data            = true;
    _updated            = true;
}


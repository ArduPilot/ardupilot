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

#include "AP_GPS_HIL.h"
#include "WProgram.h"

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_HIL::AP_GPS_HIL(Stream *s) : GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_HIL::init(void)
{	
}

void AP_GPS_HIL::update(void)
{
}

int AP_GPS_HIL::status(void)
{
    return 2; // gps locked
    // TODO: show as locked after first packet received
}

void AP_GPS_HIL::setHIL(long _time, long _latitude, long _longitude, long _altitude,
    long _ground_speed, long _ground_course, long _speed_3d, uint8_t _num_sats)
{
    time = _time;
    latitude = _latitude*1e7;
    longitude = _longitude*1e7;
    altitude = _altitude*1e2;
    ground_speed = _ground_speed*1e2;
    ground_course = _ground_course*1e2;
    speed_3d = _speed_3d*1e2;
    num_sats = _num_sats;
    new_data = true;
    fix = true;
    valid_read = true;
}


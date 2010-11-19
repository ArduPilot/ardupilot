// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  Hardware in the loop gps class.
//  Code by James Goppert
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//
#ifndef AP_GPS_HIL_h
#define AP_GPS_HIL_h

#include <GPS.h>

class AP_GPS_HIL : public GPS {
public:
	AP_GPS_HIL(Stream *s);
	void init(void);
	void update(void);
    int status(void);
    void setHIL(long time, long latitude, long longitude, long altitude,
            long ground_speed, long ground_course, long speed_3d, uint8_t num_sats);
};

#endif	// AP_GPS_HIL_H

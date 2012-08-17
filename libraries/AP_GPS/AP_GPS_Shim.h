// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  Shim GPS driver, for use when the actual GPS data is coming from somewhere else.
//  Code by Mike Smith
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//
#ifndef AP_GPS_Shim_h
#define AP_GPS_Shim_h

#include "GPS.h"

class AP_GPS_Shim : public GPS
{
public:
    AP_GPS_Shim() : GPS(NULL) {
    }

    virtual void        init(enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE) {
    };
    virtual bool        read(void) {
        bool        updated = _updated;
        _updated = false;
        return updated;
    }

    /// Set-and-mark-updated macro for the public member variables; each instance
    /// defines a member function set_<variable>(<type>)
    ///
#define __GPS_SHIM_SET(__type, __name) void set_ ## __name(__type v) { __name = v; _updated = true; }
    __GPS_SHIM_SET(uint32_t, time);
    __GPS_SHIM_SET(long, latitude);
    __GPS_SHIM_SET(long, longitude);
    __GPS_SHIM_SET(long, altitude);
    __GPS_SHIM_SET(long, ground_speed);
    __GPS_SHIM_SET(long, ground_course);
    __GPS_SHIM_SET(long, speed_3d);
    __GPS_SHIM_SET(int, hdop);
#undef __GPS_SHIM_SET

private:
    bool        _updated;               ///< set anytime a member is updated, cleared when read() returns true
};

#endif  // AP_GPS_HIL_H

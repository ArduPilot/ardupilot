// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/// @file	limits.cpp
/// @brief	Imposes limits on location (geofence), altitude and other parameters.
/// Each limit breach will trigger an action or set of actions to recover.
/// Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#include <AP_Limit_GPSLock.h>

const AP_Param::GroupInfo AP_Limit_GPSLock::var_info[] PROGMEM = {
    // @Param: GPSLCK_ON
    // @DisplayName: Enable gpslock
    // @Description: Setting this to Enabled(1) will enable the gpslock.
    // Setting this to Disabled(0) will disable the gpslock
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GPSLCK_ON",   0,      AP_Limit_GPSLock,       _enabled, 0),

    // @Param: GPSLCK_REQ
    // @DisplayName: Require gpslock
    // @Description: Setting this to Enabled(1) will make being inside the
    // gpslock a required check before arming the vehicle.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GPSLCK_REQ",  1,      AP_Limit_GPSLock,       _required, 0),
    AP_GROUPEND

};

AP_Limit_GPSLock::AP_Limit_GPSLock(GPS *&gps) :
    AP_Limit_Module(AP_LIMITS_GPSLOCK),     // enabled and required
    _gps(gps)
{
}


bool AP_Limit_GPSLock::triggered() {
    _triggered = false;     // reset trigger before checking

    if (!_gps || !_gps->fix) {
        _triggered = true;
    }

    return _triggered;
}


// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/// @file	limits.cpp
/// @brief	Imposes limits on location (geofence), altitude and other parameters
/// Each limit breach will trigger an action or set of actions to recover.
//  Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#include "AP_Limit_Altitude.h"

const AP_Param::GroupInfo AP_Limit_Altitude::var_info[] PROGMEM = {
    // @Param: ALT_ON
    // @DisplayName: Enable altitude
    // @Description: Setting this to Enabled(1) will enable the altitude. Setting this to Disabled(0) will disable the altitude
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ALT_ON",  0,      AP_Limit_Altitude,      _enabled, 0),

    // @Param: ALT_REQD
    // @DisplayName: Require altitude
    // @Description: Setting this to Enabled(1) will make being inside the altitude a required check before arming the vehicle.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ALT_REQ", 1,      AP_Limit_Altitude,      _required, 0),

    // @Param: ALT_MIN
    // @DisplayName: Minimum Altitude
    // @Description: Minimum Altitude. Zero to disable. Sets a "floor" that your vehicle will try to stay above. IF the vehicle is crossing the threshold at speed, it will take a while to recover, so give yourself enough room. Caution: minimum altitude limits can cause unexpected behaviour, such as inability to land, or sudden takeoff. Read the wiki instructions before setting.
    // @Units: Meters
    // @Range: 0 250000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 2,      AP_Limit_Altitude,      _min_alt, 0),

    // @Param: ALT_MAX
    // @DisplayName: Maximum Altitude
    // @Description: Maximum Altitude. Zero to disable. Sets a "ceiling" that your vehicle will try to stay below. IF the vehicle is crossing the threshold at speed, it will take a while to recover.
    // @Units: Meters
    // @Range: 0 250000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MAX", 3,      AP_Limit_Altitude,      _max_alt, 0),
    AP_GROUPEND
};

AP_Limit_Altitude::AP_Limit_Altitude(struct Location *current_loc) :
    AP_Limit_Module(AP_LIMITS_ALTITUDE)     // enabled and required
{
    _current_loc = current_loc;
}

bool AP_Limit_Altitude::triggered()
{
    _triggered = false;     // reset trigger before checking

    // _max_alt is zero if disabled
    // convert _max_alt to centimeters to compare to actual altitude
    if (_max_alt > 0  && _current_loc->alt > _max_alt*100 ) {
        _triggered = true;
    }

    // _min_alt is zero if disabled
    // convert _min_alt to centimeters to compare to actual altitude
    if (_min_alt > 0 && _current_loc->alt < _min_alt*100 ) {
        _triggered = true;
    }
    return _triggered;
}

AP_Int32 AP_Limit_Altitude::max_alt() {
    return _max_alt;
}

AP_Int32 AP_Limit_Altitude::min_alt() {
    return _min_alt;
}


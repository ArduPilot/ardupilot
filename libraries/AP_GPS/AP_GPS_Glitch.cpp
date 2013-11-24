// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-



#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include "AP_GPS_Glitch.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo GPS_Glitch::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: GPS Glitch protection enable/disable
    // @Description: Allows you to enable (1) or disable (0) gps glitch protection
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  GPS_Glitch,   _enabled,   1),

    // @Param: RADIUS
    // @DisplayName: GPS glitch protection radius within which all new positions are accepted
    // @Description: GPS glitch protection radius within which all new positions are accepted
    // @Units: cm
    // @Range: 100 2000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("RADIUS",      1,  GPS_Glitch,   _radius_cm,   GPS_GLITCH_RADIUS_CM),

    // @Param: ACCEL
    // @DisplayName: GPS glitch protection's max vehicle acceleration assumption
    // @Description: GPS glitch protection's max vehicle acceleration assumption
    // @Units: cm/s/s
    // @Range: 100 2000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ACCEL",   2,  GPS_Glitch,   _accel_max_cmss,   GPS_GLITCH_ACCEL_MAX_CMSS),

    AP_GROUPEND
};

// constuctor
GPS_Glitch::GPS_Glitch(GPS*& gps) :
    _gps(gps)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// check_position - returns true if gps position is acceptable, false if not
void GPS_Glitch::check_position()
{
    uint32_t now = hal.scheduler->millis(); // current system time
    float sane_dt;                  // time since last sane gps reading
    float accel_based_distance;     // movement based on max acceleration
    Location curr_pos;              // our current position estimate
    Location gps_pos;               // gps reported position
    float distance_cm;              // distance from gps to current position estimate in cm
    bool all_ok;                    // true if the new gps position passes sanity checks

    // exit immediately if we don't have gps lock
    if (_gps == NULL || _gps->status() != GPS::GPS_OK_FIX_3D) {
        _flags.glitching = true;
        return;
    }

    // if not initialised or disabled update last good position and exit
    if (!_flags.initialised || !_enabled) {
        _last_good_update = now;
        _last_good_lat = _gps->latitude;
        _last_good_lon = _gps->longitude;
        _last_good_vel.x = _gps->velocity_north();
        _last_good_vel.y = _gps->velocity_east();
        _flags.initialised = true;
        _flags.glitching = false;
        return;
    }

    // calculate time since last sane gps reading in ms
    sane_dt = (now - _last_good_update) / 1000.0f;

    // project forward our position from last known velocity
    curr_pos.lat = _last_good_lat;
    curr_pos.lng = _last_good_lon;
    location_offset(curr_pos, _last_good_vel.x * sane_dt, _last_good_vel.y * sane_dt);

    // calculate distance from recent gps position to current estimate
    gps_pos.lat = _gps->latitude;
    gps_pos.lng = _gps->longitude;
    distance_cm = get_distance_cm(curr_pos, gps_pos);

    // all ok if within a given hardcoded radius
    if (distance_cm <= _radius_cm) {
        all_ok = true;
    }else{
        // or if within the maximum distance we could have moved based on our acceleration
        accel_based_distance = 0.5f * _accel_max_cmss * sane_dt * sane_dt;
        all_ok = (distance_cm <= accel_based_distance);
    }

    // store updates to gps position
    if (all_ok) {
        // position is acceptable
        _last_good_update = now;
        _last_good_lat = _gps->latitude;
        _last_good_lon = _gps->longitude;
        _last_good_vel.x = _gps->velocity_north();
        _last_good_vel.y = _gps->velocity_east();
    }
    
    // update glitching flag
    _flags.glitching = !all_ok;
}


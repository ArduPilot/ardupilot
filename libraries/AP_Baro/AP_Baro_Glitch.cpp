// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-



#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include "AP_Baro_Glitch.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo Baro_Glitch::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Baro Glitch protection enable/disable
    // @Description: Allows you to enable (1) or disable (0) baro glitch protection
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  Baro_Glitch,   _enabled,   1),

    // @Param: DIST
    // @DisplayName: Baro glitch protection distance within which alt update is immediately accepted
    // @Description: Baro glitch protection distance within which alt update is immediately accepted
    // @Units: cm
    // @Range: 100 2000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("DIST",  1,  Baro_Glitch,   _dist_ok_cm,   BARO_GLITCH_DISTANCE_OK_CM),

    // @Param: ACCEL
    // @DisplayName: Baro glitch protection's max vehicle acceleration assumption
    // @Description: Baro glitch protection's max vehicle acceleration assumption
    // @Units: cm/s/s
    // @Range: 100 2000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ACCEL",   2,  Baro_Glitch,    _accel_max_cmss,   BARO_GLITCH_ACCEL_MAX_CMSS),

    AP_GROUPEND
};

// constuctor
Baro_Glitch::Baro_Glitch(AP_Baro &baro) :
    _baro(baro),
    _last_good_update(0),
    _last_good_alt(0),
    _last_good_vel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.initialised = 0;
    _flags.glitching = 0;
}

// check_alt - checks latest baro altitude against last know alt, velocity and maximum acceleration and updates glitching flag
void Baro_Glitch::check_alt()
{
    uint32_t now = hal.scheduler->millis(); // current system time
    float sane_dt;                  // time since last sane baro reading
    float accel_based_distance;     // movement based on max acceleration
    int32_t alt_projected;          // altitude estimate projected from previous iteration
    int32_t distance_cm;            // distance from baro alt to current alt estimate in cm
    bool all_ok;                    // true if the new baro alt passes sanity checks

    // exit immediately if baro is unhealthy
    if (!_baro.healthy()) {
        _flags.glitching = true;
        return;
    }

    // if not initialised or disabled update last good alt and exit
    if (!_flags.initialised || !_enabled) {
        _last_good_update = now;
        _last_good_alt = _baro.get_altitude();
        _last_good_vel = _baro.get_climb_rate();
        _flags.initialised = true;
        _flags.glitching = false;
        return;
    }

    // calculate time since last sane baro reading in ms
    sane_dt = (now - _last_good_update) / 1000.0f;

    // estimate our alt from last known alt and velocity
    alt_projected = _last_good_alt + (_last_good_vel * sane_dt);

    // calculate distance from recent baro alt to current estimate
    int32_t baro_alt = _baro.get_altitude() * 100.0f;

    // baro may have become unhealthy when calculating altitude
    if (!_baro.healthy()) {
        _flags.glitching = true;
        return;
    }

    // calculte distance from projected distance
    distance_cm = labs(alt_projected - baro_alt);

    // all ok if within a given hardcoded radius
    if (distance_cm <= _dist_ok_cm) {
        all_ok = true;
    }else{
        // or if within the maximum distance we could have moved based on our acceleration
        accel_based_distance = 0.5f * _accel_max_cmss * sane_dt * sane_dt;
        all_ok = (distance_cm <= accel_based_distance);
    }

    // store updates to baro position
    if (all_ok) {
        // position is acceptable
        _last_good_update = now;
        _last_good_alt = baro_alt;
        _last_good_vel = _baro.get_climb_rate();
    }

    // update glitching flag
    _flags.glitching = !all_ok;
}


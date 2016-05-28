// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GPS_Glitch.h
/// @brief	GPS Glitch protection

#ifndef __GPS_GLITCH_H__
#define __GPS_GLITCH_H__

#include <AP_HAL.h>

#include <inttypes.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_GPS.h>

#define GPS_GLITCH_ACCEL_MAX_CMSS   1000.0f // vehicle can accelerate at up to 5m/s/s in any direction
#define GPS_GLITCH_RADIUS_CM        200.0f  // gps movement within 2m of current position are always ok

/// @class	GPS_Glitch
/// @brief	GPS Glitch protection class
class GPS_Glitch
{
public:
    // constructor
	GPS_Glitch(const AP_GPS &gps);

    // check_position - checks latest gps position against last know position, velocity and maximum acceleration and updates glitching flag
    void    check_position();

    // enable - enable or disable gps sanity checking
    void    enable(bool true_or_false) { _enabled = true_or_false; }

    // enabled - returns true if glitch detection is enabled
    bool    enabled() const { return _enabled; }

    // glitching - returns true if we are experiencing a glitch
    bool    glitching() const { return _flags.glitching; }

    // last_good_update - returns system time of the last good update
    uint32_t last_good_update() const { return _last_good_update; }

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    /// external dependencies
    const AP_GPS &_gps;               // reference to gps

    /// structure for holding flags
    struct GPS_Glitch_flags {
        uint8_t initialised : 1; // 1 if we have received at least one good gps lock
        uint8_t glitching   : 1; // 1 if we are experiencing a gps glitch
    } _flags;

    // parameters
    AP_Int8     _enabled;           // top level enable/disable control
    AP_Float    _radius_cm;         // radius in cm within which all new positions from GPS are accepted
    AP_Float    _accel_max_cmss;    // vehicles maximum acceleration in cm/s/s

    // gps sanity check variables
    uint32_t    _last_good_update;  // system time of last gps update that passed checks
    int32_t     _last_good_lat;     // last good latitude reported by the gps
    int32_t     _last_good_lon;     // last good longitude reported by the gps
    Vector2f    _last_good_vel;     // last good velocity reported by the gps in cm/s in lat and lon directions
};

#endif // __GPS_H__

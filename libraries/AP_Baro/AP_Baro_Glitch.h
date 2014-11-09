// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Baro_Glitch.h
/// @brief	Barometer Glitch protection

#ifndef __BARO_GLITCH_H__
#define __BARO_GLITCH_H__

#include <AP_HAL.h>

#include <inttypes.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Baro.h>

#define BARO_GLITCH_ACCEL_MAX_CMSS  1500.0f // vehicle can accelerate at up to 15m/s/s vertically
#define BARO_GLITCH_DISTANCE_OK_CM  500.0f  // baro movement within 5m of current position is always ok

/// @class	Baro_Glitch
/// @brief	Baro Glitch protection class
class Baro_Glitch
{
public:
    // constructor
	Baro_Glitch(AP_Baro &baro);

    // check_alt - checks latest baro altitude against last know alt, velocity and maximum acceleration and updates glitching flag
    void    check_alt();

    // enable - enable or disable baro sanity checking
    void    enable(bool true_or_false) { _enabled = true_or_false; }

    // enabled - returns true if glitch detection is enabled
    bool    enabled() const { return _enabled; }

    // glitching - returns true if we are experiencing a glitch
    bool    glitching() const { return _flags.glitching; }

    // reset - resets glitch detection to start from current baro alt
    //      should be called after barometer altitude is reset during arming
    void    reset() { _flags.initialised = false; }

    // last_good_update - returns system time of the last good update
    uint32_t last_good_update() const { return _last_good_update; }

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    /// external dependencies
    AP_Baro     &_baro;             // reference to barometer

    /// structure for holding flags
    struct Baro_Glitch_flags {
        uint8_t initialised : 1;    // 1 if we have received at least one good baro alt
        uint8_t glitching   : 1;    // 1 if we are experiencing a baro glitch
    } _flags;

    // parameters
    AP_Int8     _enabled;           // top level enable/disable control
    AP_Float    _dist_ok_cm;        // distance in cm within which all new positions from Baro are accepted
    AP_Float    _accel_max_cmss;    // vehicles maximum acceleration in cm/s/s

    // baro sanity check variables
    uint32_t    _last_good_update;  // system time of last baro update that passed checks
    int32_t     _last_good_alt;     // last good altitude reported by the baro
    float       _last_good_vel;     // last good velocity reported by the baro in cm/s
};

#endif // __BARO_GLITCH_H__

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_Sprayer.h
/// @brief	Crop sprayer library

#ifndef AC_SPRAYER_H
#define AC_SPRAYER_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_InertialNav.h>     // Inertial Navigation library

#define AC_SPRAYER_DEFAULT_PUMP_RATE        0       // default quantity of spray per meter travelled
#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    // default speed of spinner (higher means spray is throw further horizontally
#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     // we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     // delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    // shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AC_Sprayer {

public:
    /// Constructor
    AC_Sprayer(AP_InertialNav* inav);

    /// enable - allows sprayer to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool true_false);

    /// enabled - returns true if fence is enabled
    bool enabled() const { return _enabled; }

    /// To-Do: add function to decode pilot input from channel 6 tuning knob

    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    void set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }

    /// update - adjusts servo positions based on speed and requested quantity
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    // pointers to other objects we depend upon
    AP_InertialNav* _inav;

    // parameters
    AP_Int8         _enabled;               // top level enable/disable control
    AP_Float        _pump_pct_1ms;          // desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
    AP_Int16        _spinner_pwm;           // pwm rate of spinner
    AP_Float        _speed_min;             // minimum speed in cm/s above which the sprayer will be started

    // internal variables
    bool            _spraying;              // true if we are currently spraying
    uint32_t        _speed_over_min_time;   // time at which we reached speed minimum
    uint32_t        _speed_under_min_time;  // time at which we fell below speed minimum
};

#endif /* AC_SPRAYER_H */

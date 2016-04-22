// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Parachute.h
/// @brief	Parachute release library
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Relay/AP_Relay.h>

#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_0       0
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_1       1
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_2       2
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_3       3
#define AP_PARACHUTE_TRIGGER_TYPE_SERVO         10

#define AP_PARACHUTE_RELEASE_DELAY_MS           500    // delay in milliseconds between call to release() and when servo or relay actually moves.  Allows for warning to user
#define AP_PARACHUTE_RELEASE_DURATION_MS       2000    // when parachute is released, servo or relay stay at their released position/value for 2000ms (2seconds)

#define AP_PARACHUTE_SERVO_ON_PWM_DEFAULT      1300    // default PWM value to move servo to when shutter is activated
#define AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT     1100    // default PWM value to move servo to when shutter is deactivated

#define AP_PARACHUTE_ALT_MIN_DEFAULT            10     // default min altitude the vehicle should have before parachute is released

/// @class	AP_Parachute
/// @brief	Class managing the release of a parachute
class AP_Parachute {

public:

    /// Constructor
    AP_Parachute(AP_Relay& relay) :
        _relay(relay),
        _release_time(0),
        _release_initiated(false),
        _release_in_progress(false),
        _released(false)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    /// enabled - enable or disable parachute release
    void enabled(bool on_off);

    /// enabled - returns true if parachute release is enabled
    bool enabled() const { return _enabled; }

    /// release - release parachute
    void release();

    /// released - true if the parachute has been released (or release is in progress)
    bool released() const { return _released; }
    
    /// release_initiated - true if the parachute release sequence has been initiated (may wait before actual release)
    bool release_initiated() const { return _release_initiated; }
    
    /// update - shuts off the trigger should be called at about 10hz
    void update();

    /// alt_min - returns the min altitude above home the vehicle should have before parachute is released
    ///   0 = altitude check disabled
    int16_t alt_min() const { return _alt_min; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _release_type;  // 0:Servo,1:Relay
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated
    AP_Int16    _alt_min;       // min altitude the vehicle should have before parachute is released
    AP_Int16    _delay_ms;      // delay before chute release for motors to stop

    // internal variables
    AP_Relay   &_relay;         // pointer to relay object from the base class Relay.
    uint32_t    _release_time;  // system time that parachute is ordered to be released (actual release will happen 0.5 seconds later)
    bool        _release_initiated:1;    // true if the parachute release initiated (may still be waiting for engine to be suppressed etc.)
    bool        _release_in_progress:1;  // true if the parachute release is in progress
    bool        _released:1;             // true if the parachute has been released
};

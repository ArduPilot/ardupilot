// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Parachute.h
/// @brief	Parachute release library
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Vehicle/AP_Vehicle.h>

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
#define AP_PARACHUTE_ALT_MAX_DEFAULT            50     // default max altitude above which the parachute should not be released

#define AP_PARACHUTE_AUTO_ON_DEFAULT            0      // automatic emergency parachute release is off by default
#define AP_PARACHUTE_AUTO_ALT_DEFAULT           60     // altitude threshold above home at which to deploy parachute automatically
#define AP_PARACHUTE_AUTO_SINK_DEFAULT          8      // sink rate in m/s at which to deploy parachute automatically

/// @class	AP_Parachute
/// @brief	Class managing the release of a parachute
class AP_Parachute {

public:

    /// Constructor
    AP_Parachute(AP_Relay& relay) :
        _relay(relay),
        _release_time(0),
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
    
    /// update - shuts off the trigger should be called at about 10hz
    void update();

    /// alt_min - returns the min altitude above home the vehicle should have before parachute is released
    ///   0 = altitude check disabled
    int16_t alt_min() const { return _alt_min; }

    /// alt_max - returns the max altitude above home, above which the parachute should not be released
    int16_t alt_max() const { return _alt_max; }

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    /// the time when control of aircraft was continuously lost
    void control_loss_ms(uint32_t time);
    uint32_t control_loss_ms() const { return _control_loss_ms; }

    int16_t auto_alt() const { return _auto_alt; }
    float auto_sink() const { return _auto_sink; }

    /// auto_enabled - returns true if parachute automatic emergency release is enabled
    bool auto_enabled() const { return enabled() && _auto_enabled; }
#endif

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _release_type;  // 0:Servo,1:Relay
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated
    AP_Int16    _alt_min;       // min altitude the vehicle should have before parachute is released
    AP_Int16    _alt_max;       // max altitude above which the parachute should not be released
    AP_Int16    _delay_ms;      // delay before chute release for motors to stop
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    AP_Int8     _auto_enabled;  // 1 if automatic emergency parachute release is enabled
    AP_Int16    _auto_alt;      // altitude threshold above home at which to deploy parachute automatically (if enabled)
    AP_Float    _auto_sink;     // sink rate in m/s at which to deploy parachute automatically (if enabled)
#endif

    // internal variables
    AP_Relay   &_relay;         // pointer to relay object from the base class Relay.
    uint32_t    _release_time;  // system time that parachute is ordered to be released (actual release will happen 0.5 seconds later)
    bool        _release_in_progress:1;  // true if the parachute release is in progress
    bool        _released:1;    // true if the parachute has been released
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    uint32_t    _control_loss_ms;  // automatic parachute deployment check, start of continuously lost control
#endif
};

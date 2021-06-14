/// @file   AP_Parachute.h
/// @brief  Parachute release library
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Relay/AP_Relay.h>
#include <GCS_MAVLink/GCS.h>


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

#define AP_PARACHUTE_CRITICAL_SINK_DEFAULT      0    // default critical sink speed in m/s to trigger emergency parachute

#ifndef HAL_PARACHUTE_ENABLED
// default to parachute enabled to match previous configs
#define HAL_PARACHUTE_ENABLED 1
#endif

#if HAL_PARACHUTE_ENABLED

/// @class  AP_Parachute
/// @brief  Class managing the release of a parachute
class AP_Parachute {

public:
    /// Constructor
    AP_Parachute(AP_Relay &relay)
        : _relay(relay)
    {
        // setup parameter defaults
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (_singleton != nullptr) {
            AP_HAL::panic("Parachute must be singleton");
        }
#endif
        _singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_Parachute(const AP_Parachute &other) = delete;
    AP_Parachute &operator=(const AP_Parachute&) = delete;

    /// enabled - enable or disable parachute release
    void enabled(bool on_off);

    /// enabled - returns true if parachute release is enabled
    bool enabled() const { return _enabled; }

    MAV_RESULT handle_cmd(const mavlink_command_long_t &packet);

    enum release_reason {
        SINK_RATE = 0,
        ACCEL_FALLING = 1,
        CONTROL_LOSS = 2,
        MISSION_ITEM = 3,
        MANUAL = 4,
    };

    /// release - release parachute
    void release(release_reason reason);

    /// release_initiated - true if the parachute release sequence has been initiated (may wait before actual release)
    bool release_initiated() const { return _release_initiated; }

    /// release_in_progress - true if the parachute release sequence is in progress
    bool release_in_progress() const { return _release_in_progress; }

    /// update - shuts off the trigger should be called at about 10hz
    void update();

    /// alt_min - returns the min altitude above home the vehicle should have before parachute is released
    ///   0 = altitude check disabled
    int16_t alt_min() const { return _alt_min; }

    float max_ang_err() const { return _ang_error_max.get(); }

    float max_rp_ang() const { return _sb_rp_ang_max.get(); }

    /// set_is_flying - accessor to the is_flying flag
    void set_is_flying(const bool is_flying) { _is_flying = is_flying; }

    // update - set vehicle sink rate and earth frame Z accel
    void update(const float sink_rate, const float accel, const bool upper_throttle_limit);

    // trigger parachute release thresholds
    void check();

    static const struct AP_Param::GroupInfo        var_info[];

    // get singleton instance
    static AP_Parachute *get_singleton() { return _singleton; }

private:

    // send user command long updating the parchute status
    void send_msg();

    // Set servo or relay to off position
    void release_off();

    // Structure to lookup for release reasons
    struct LookupTable{
       release_reason option;
       const char *announcement;
    };
    static const LookupTable lookuptable[];
    const char *string_for_release(release_reason reason) const;

    enum OPTIONS {
        DONT_DEPLOY_LANDING_GEAR = 1U << 0,
        DONT_DISARM = 1U << 1,
        NOTIFY_ONLY = 1U << 2,
    };

    static AP_Parachute *_singleton;
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _release_type;  // 0:Servo,1:Relay
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated
    AP_Int16    _alt_min;       // min altitude the vehicle should have before parachute is released
    AP_Int16    _delay_ms;      // delay before chute release for motors to stop
    AP_Float    _critical_sink;      // critical sink rate to trigger emergency parachute with throttle saturated
    AP_Float    _min_accel;          // critical earth frame Z acceleration
    AP_Int32    _options;            // bitmask of options
    AP_Int32    _cancel_delay;       // ms delay to allow a cancel message between trigger and release
    AP_Float    _ang_error_max;       // Maximum roll/pitch error when aircraft not in standby
    AP_Float    _sb_rp_ang_max;       // Maximum absolute roll/pitch angle when aircraft in standby
    AP_Float    _abs_critical_sink;   // critical sink rate to trigger emergency parachute, no throttle check

    // internal variables
    AP_Relay   &_relay;         // pointer to relay object from the base class Relay.
    uint32_t    _release_time;  // system time that parachute is ordered to be released (actual release will happen 0.5 seconds later)
    bool        _release_initiated:1;    // true if the parachute release initiated (may still be waiting for engine to be suppressed etc.)
    bool        _release_setup:1;        // true if parchute release has been setup (vehicle disarmed, landing gear deployed)
    bool        _release_in_progress:1;  // true if the parachute release is in progress
    bool        _is_flying:1;            // true if the vehicle is flying
    uint32_t    _sink_time_ms;           // system time that the vehicle exceeded critical sink rate, with throttle threshold
    uint32_t    _fall_time_ms;           // system time that the vehicle stated falling lower faster _min_accel
    uint8_t     _release_reasons;        // bitmask of the current reasons to release
    uint32_t    _cancel_timeout_ms;
    uint32_t    _last_msg_send_ms;
    uint32_t    _abs_sink_time_ms;       // system time that the vehicle exceeded critical sink rate

};

namespace AP {
    AP_Parachute *parachute();
};

#endif // HAL_PARACHUTE_ENABLED

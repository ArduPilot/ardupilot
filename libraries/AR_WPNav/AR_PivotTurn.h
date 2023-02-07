#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>

/*
 * Pivot Turn controller for skid-steering rovers and boats
 *
 * How-to-Use:
 *    1. call "enable(true)" once for skid-steering vehicles to enable this controller
 *    2. before vehicle starts towards a waypoint call "check_activation" and provide the earth-frame heading to the waypoint
 *       this may change the controller's internal state to "active"
 *    3. on each main loop iteration call "active()" to see if this controller thinks it is controllering the vehicle
 *    4. call "get_turn_rate_rads()" to retrieve the desired turn rate towards the next waypoint
 *    5. pass above turn rate into the rate controller, set speed controller's speed to zero
 *    6. this controller's "active" state will change to false once it has completed the pivot turn
 */

class AR_PivotTurn {
public:

    // constructor
    AR_PivotTurn(AR_AttitudeControl& atc);

    // enable or disable pivot controller
    void enable(bool enable_pivot);

    // true if this controller is controlling vehicle
    bool active() const;

    // checks if pivot turns should be activated or deactivated
    // force_active should be true if the caller wishes to trigger the start of a pivot turn regardless of the heading error
    void check_activation(float desired_heading_deg, bool force_active = false);

    // check if pivot turn would be activated given an expected change in yaw in degrees
    // note this does not actually active the pivot turn.  To activate use the check_activation method
    bool would_activate(float yaw_change_deg) const WARN_IF_UNUSED;

    // forcibly deactivate this controller
    void deactivate() { _active = false; };

    // get turn rate (in rad/sec)
    // desired heading should be the heading towards the next waypoint in degrees
    // dt should be the time since the last call in seconds
    float get_turn_rate_rads(float desired_heading_deg, float dt);

    // accessors for parameter values
    float get_rate_max() const { return _rate_max; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // return post-turn delay duration in milliseconds
    uint32_t get_delay_duration_ms() const;

    // parameters
    AP_Int16 _angle;                // minimum angle error (in degrees) that leads to pivot turn
    AP_Int16 _rate_max;             // maximum turn rate (in degrees) during pivot turn
    AP_Float _delay;                // waiting time (in seconds) after pivot turn completes

    // references
    AR_AttitudeControl& _atc;       // rover attitude control library

    // local variables
    bool _enabled;                  // true if vehicle can pivot
    bool _active;                   // true if vehicle is currently pivoting
    uint32_t _delay_start_ms;       // system time when post-turn delay started
};

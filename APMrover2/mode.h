#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>  // for MAV_SEVERITY
#include "defines.h"

#define MODE_NEXT_HEADING_UNKNOWN   99999.0f    // used to indicate to set_desired_location method that next leg's heading is unknown

class Mode
{
public:

    // Constructor
    Mode();

    // enter this mode, returns false if we failed to enter
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual uint32_t mode_number() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    //
    // attributes of the mode
    //

    // return if in non-manual mode : AUTO, GUIDED, RTL
    virtual bool is_autopilot_mode() const { return false; }

    // returns true if steering is directly controlled by RC
    virtual bool manual_steering() const { return false; }

    // returns true if the throttle is controlled automatically
    virtual bool auto_throttle() { return is_autopilot_mode(); }

    // return true if throttle should be supressed in event of a
    // FAILSAFE_EVENT_THROTTLE
    virtual bool failsafe_throttle_suppress() const { return true; }

    //
    // attributes for mavlink system status reporting
    //

    // returns true if any RC input is used
    virtual bool has_manual_input() const { return false; }

    // true if heading is controlled
    virtual bool attitude_stabilized() const { return true; }

    //
    // navigation methods
    //

    // return distance (in meters) to destination
    virtual float get_distance_to_destination() const { return 0.0f; }

    // set desired location and speed (used in RTL, Guided, Auto)
    //   next_leg_bearing_cd should be heading to the following waypoint (used to slow the vehicle in order to make the turn)
    virtual void set_desired_location(const struct Location& destination, float next_leg_bearing_cd = MODE_NEXT_HEADING_UNKNOWN);

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    virtual bool reached_destination() { return true; }

    // set desired heading and speed - supported in Auto and Guided modes
    virtual void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed);

    // get speed error in m/s, returns zero for modes that do not control speed
    float speed_error() { return _speed_error; }

    // Navigation control variables
    // The instantaneous desired lateral acceleration in m/s/s
    float lateral_acceleration;

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // calculate steering angle given a desired lateral acceleration
    void calc_nav_steer(bool reversed = false);

    // calculate desired lateral acceleration
    void calc_lateral_acceleration(const struct Location &origin, const struct Location &destination, bool reversed = false);

    // calculates the amount of throttle that should be output based
    // on things like proximity to corners and current speed
    virtual void calc_throttle(float target_speed, bool nudge_allowed = true);

    // performs a controlled stop. returns true once vehicle has stopped
    bool stop_vehicle();

    // estimate maximum vehicle speed (in m/s)
    float calc_speed_max(float cruise_speed, float cruise_throttle);

    // calculate pilot input to nudge speed up or down
    //  target_speed should be in meters/sec
    //  cruise_speed is vehicle's cruising speed, cruise_throttle is the throttle (from -1 to +1) that achieves the cruising speed
    //  return value is a new speed (in m/s) which up to the projected maximum speed based on the cruise speed and cruise throttle
    float calc_speed_nudge(float target_speed, float cruise_speed, float cruise_throttle);

    // calculated a reduced speed(in m/s) based on yaw error and lateral acceleration and/or distance to a waypoint
    // should be called after calc_lateral_acceleration and before calc_throttle
    // relies on these internal members being updated: lateral_acceleration, _yaw_error_cd, _distance_to_destination
    float calc_reduced_speed_for_turn_or_distance(float desired_speed);

    // references to avoid code churn:
    class AP_AHRS &ahrs;
    class Parameters &g;
    class ParametersG2 &g2;
    class RC_Channel *&channel_steer;
    class RC_Channel *&channel_throttle;
    class AP_Mission &mission;
    class AR_AttitudeControl &attitude_control;


    // private members for waypoint navigation
    Location _origin;           // origin Location (vehicle will travel from the origin to the destination)
    Location _destination;      // destination Location when in Guided_WP
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;  // true once the vehicle has reached the destination
    float _desired_yaw_cd;      // desired yaw in centi-degrees
    float _yaw_error_cd;        // error between desired yaw and actual yaw in centi-degrees
    float _desired_speed;       // desired speed in m/s
    float _desired_speed_final; // desired speed in m/s when we reach the destination
    float _speed_error;         // ground speed error in m/s
};


class ModeAuto : public Mode
{
public:

    uint32_t mode_number() const override { return AUTO; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void calc_throttle(float target_speed, bool nudge_allowed = true);

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }
    bool failsafe_throttle_suppress() const override { return false; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

    // set desired location, heading and speed
    // set stay_active_at_dest if the vehicle should attempt to maintain it's position at the destination (mostly for boats)
    void set_desired_location(const struct Location& destination, float next_leg_bearing_cd = MODE_NEXT_HEADING_UNKNOWN, bool stay_active_at_dest = false);
    bool reached_destination() override;

    // heading and speed control
    void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed) override;
    bool reached_heading();

    // execute the mission in reverse (i.e. backing up)
    void set_reversed(bool value);

protected:

    bool _enter() override;
    void _exit() override;

    enum AutoSubMode {
        Auto_WP,                // drive to a given location
        Auto_HeadingAndSpeed    // turn to a given heading
    } _submode;

private:

    bool check_trigger(void);

    // this is set to true when auto has been triggered to start
    bool auto_triggered;

    bool _reached_heading;      // true when vehicle has reached desired heading in TurnToHeading sub mode
    bool _stay_active_at_dest;  // true when we should actively maintain position even after reaching the destination
    bool _reversed;             // execute the mission by backing up
};


class ModeGuided : public Mode
{
public:

    uint32_t mode_number() const override { return GUIDED; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }
    bool failsafe_throttle_suppress() const override { return false; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // set desired location, heading and speed
    void set_desired_location(const struct Location& destination);
    void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed) override;

    // set desired heading-delta, turn-rate and speed
    void set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed);
    void set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed);

protected:

    enum GuidedMode {
        Guided_WP,
        Guided_HeadingAndSpeed,
        Guided_TurnRateAndSpeed
    };

    bool _enter() override;

    GuidedMode _guided_mode;    // stores which GUIDED mode the vehicle is in

    // attitude control
    bool have_attitude_target;  // true if we have a valid attitude target
    uint32_t _des_att_time_ms;  // system time last call to set_desired_attitude was made (used for timeout)
    float _desired_yaw_rate_cds;// target turn rate centi-degrees per second
};


class ModeHold : public Mode
{
public:

    uint32_t mode_number() const override { return HOLD; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool attitude_stabilized() const override { return false; }
};


class ModeManual : public Mode
{
public:

    uint32_t mode_number() const override { return MANUAL; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool manual_steering() const override { return true; }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
};


class ModeRTL : public Mode
{
public:

    uint32_t mode_number() const override { return RTL; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }
    bool failsafe_throttle_suppress() const override { return false; }

    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() override { return _reached_destination; }

protected:

    bool _enter() override;
};


class ModeSteering : public Mode
{
public:

    uint32_t mode_number() const override { return STEERING; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
};

class ModeInitializing : public Mode
{
public:

    uint32_t mode_number() const override { return INITIALISING; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
};

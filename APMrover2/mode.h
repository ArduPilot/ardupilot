#pragma once

#include <stdint.h>

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>

#include "defines.h"

#define MODE_NEXT_HEADING_UNKNOWN   99999.0f    // used to indicate to set_desired_location method that next leg's heading is unknown

// pre-define ModeRTL so Auto can appear higher in this file
class ModeRTL;

class Mode
{
public:

    // Auto Pilot modes
    // ----------------
    enum Number {
        MANUAL       = 0,
        ACRO         = 1,
        STEERING     = 3,
        HOLD         = 4,
        LOITER       = 5,
        FOLLOW       = 6,
        SIMPLE       = 7,
        AUTO         = 10,
        RTL          = 11,
        SMART_RTL    = 12,
        GUIDED       = 15,
        INITIALISING = 16
    };

    // Constructor
    Mode();

    // enter this mode, returns false if we failed to enter
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual uint32_t mode_number() const = 0;

    // returns short text name (up to 4 bytes)
    virtual const char *name4() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    //
    // attributes of the mode
    //

    // return if in non-manual mode : Auto, Guided, RTL, SmartRTL
    virtual bool is_autopilot_mode() const { return false; }

    // returns true if vehicle can be armed or disarmed from the transmitter in this mode
    virtual bool allows_arming_from_transmitter() { return !is_autopilot_mode(); }

    //
    // attributes for mavlink system status reporting
    //

    // returns true if any RC input is used
    virtual bool has_manual_input() const { return false; }

    // true if heading is controlled
    virtual bool attitude_stabilized() const { return true; }

    // true if mode requires position and/or velocity estimate
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }

    //
    // navigation methods
    //

    // return distance (in meters) to destination
    virtual float get_distance_to_destination() const { return 0.0f; }

    // set desired location and speed (used in RTL, Guided, Auto)
    //   next_leg_bearing_cd should be heading to the following waypoint (used to slow the vehicle in order to make the turn)
    virtual void set_desired_location(const struct Location& destination, float next_leg_bearing_cd = MODE_NEXT_HEADING_UNKNOWN);

    // set desired location as offset from the EKF origin, return true on success
    bool set_desired_location_NED(const Vector3f& destination, float next_leg_bearing_cd = MODE_NEXT_HEADING_UNKNOWN);

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    virtual bool reached_destination() { return true; }

    // set desired heading and speed - supported in Auto and Guided modes
    virtual void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed);

    // get speed error in m/s, returns zero for modes that do not control speed
    float speed_error() const { return _speed_error; }

    // get default speed for this mode (held in CRUISE_SPEED, WP_SPEED or RTL_SPEED)
    // rtl argument should be true if called from RTL or SmartRTL modes (handled here to avoid duplication)
    float get_speed_default(bool rtl = false) const;

    // set desired speed in m/s
    bool set_desired_speed(float speed);

    // restore desired speed to default from parameter values (CRUISE_SPEED or WP_SPEED)
    // rtl argument should be true if called from RTL or SmartRTL modes (handled here to avoid duplication)
    void set_desired_speed_to_default(bool rtl = false);

    // execute the mission in reverse (i.e. backing up)
    void set_reversed(bool value);

    // handle tacking request (from auxiliary switch) in sailboats
    virtual void handle_tack_request();

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
    // steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
    // throttle_out is in the range -100 ~ +100
    void get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out);

    // decode pilot input steering and return steering_out and speed_out (in m/s)
    void get_pilot_desired_steering_and_speed(float &steering_out, float &speed_out);

    // decode pilot lateral movement input and return in lateral_out argument
    void get_pilot_desired_lateral(float &lateral_out);

    // decode pilot's input and return heading_out (in cd) and speed_out (in m/s)
    void get_pilot_desired_heading_and_speed(float &heading_out, float &speed_out);

    // calculate steering output to drive along line from origin to destination waypoint
    void calc_steering_to_waypoint(const struct Location &origin, const struct Location &destination, bool reversed = false);

    // calculate steering angle given a desired lateral acceleration
    void calc_steering_from_lateral_acceleration(float lat_accel, bool reversed = false);

    // calculate steering output to drive towards desired heading
    // rate_max is a maximum turn rate in deg/s.  set to zero to use default turn rate limits
    void calc_steering_to_heading(float desired_heading_cd, float rate_max_degs = 0.0f);

    // calculates the amount of throttle that should be output based
    // on things like proximity to corners and current speed
    virtual void calc_throttle(float target_speed, bool nudge_allowed, bool avoidance_enabled);

    // performs a controlled stop. returns true once vehicle has stopped
    bool stop_vehicle();

    // estimate maximum vehicle speed (in m/s)
    // cruise_speed is in m/s, cruise_throttle should be in the range -1 to +1
    float calc_speed_max(float cruise_speed, float cruise_throttle) const;

    // calculate pilot input to nudge speed up or down
    //  target_speed should be in meters/sec
    //  cruise_speed is vehicle's cruising speed, cruise_throttle is the throttle (from -1 to +1) that achieves the cruising speed
    //  return value is a new speed (in m/s) which up to the projected maximum speed based on the cruise speed and cruise throttle
    float calc_speed_nudge(float target_speed, float cruise_speed, float cruise_throttle);

    // calculated a reduced speed(in m/s) based on yaw error and lateral acceleration and/or distance to a waypoint
    // should be called after calc_steering_to_waypoint and before calc_throttle
    // relies on these internal members being updated: lateral_acceleration, _yaw_error_cd, _distance_to_destination
    float calc_reduced_speed_for_turn_or_distance(float desired_speed);

    // calculate vehicle stopping location using current location, velocity and maximum acceleration
    void calc_stopping_location(Location& stopping_loc);

protected:

    // decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
    // steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
    // throttle_out is in the range -100 ~ +100
    void get_pilot_input(float &steering_out, float &throttle_out);

    // references to avoid code churn:
    class AP_AHRS &ahrs;
    class Parameters &g;
    class ParametersG2 &g2;
    class RC_Channel *&channel_steer;
    class RC_Channel *&channel_throttle;
    class RC_Channel *&channel_lateral;
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
    uint32_t last_steer_to_wp_ms;   // system time of last call to calc_steering_to_waypoint
    bool _reversed;             // execute the mission by backing up
};


class ModeAcro : public Mode
{
public:

    uint32_t mode_number() const override { return ACRO; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }

    // acro mode requires a velocity estimate for non skid-steer rovers
    bool requires_position() const override { return false; }
    bool requires_velocity() const override;

    // sailboats in acro mode support user manually initiating tacking from transmitter
    void handle_tack_request() override;
};


class ModeAuto : public Mode
{
public:

    // constructor
    ModeAuto(ModeRTL& mode_rtl);

    uint32_t mode_number() const override { return AUTO; }
    const char *name4() const override { return "AUTO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void calc_throttle(float target_speed, bool nudge_allowed, bool avoidance_enabled);

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // set desired location, heading and speed
    void set_desired_location(const struct Location& destination, float next_leg_bearing_cd = MODE_NEXT_HEADING_UNKNOWN);
    bool reached_destination() override;

    // heading and speed control
    void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed) override;
    bool reached_heading();

    // start RTL (within auto)
    void start_RTL();

protected:

    bool _enter() override;
    void _exit() override;

    enum AutoSubMode {
        Auto_WP,                // drive to a given location
        Auto_HeadingAndSpeed,   // turn to a given heading
        Auto_RTL                // perform RTL within auto mode
    } _submode;

private:

    bool check_trigger(void);

    // references
    ModeRTL& _mode_rtl;

    // this is set to true when auto has been triggered to start
    bool auto_triggered;

    bool _reached_heading;      // true when vehicle has reached desired heading in TurnToHeading sub mode
};


class ModeGuided : public Mode
{
public:

    uint32_t mode_number() const override { return GUIDED; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

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
    const char *name4() const override { return "HOLD"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool attitude_stabilized() const override { return false; }

    // hold mode does not require position or velocity estimate
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }
};

class ModeLoiter : public Mode
{
public:

    uint32_t mode_number() const override { return LOITER; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

protected:

    bool _enter() override;
};

class ModeManual : public Mode
{
public:

    uint32_t mode_number() const override { return MANUAL; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

    // manual mode does not require position or velocity estimate
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }

protected:

    void _exit() override;
};


class ModeRTL : public Mode
{
public:

    uint32_t mode_number() const override { return RTL; }
    const char *name4() const override { return "RTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() override { return _reached_destination; }

protected:

    bool _enter() override;
};

class ModeSmartRTL : public Mode
{
public:

    uint32_t mode_number() const override { return SMART_RTL; }
    const char *name4() const override { return "SRTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() override { return smart_rtl_state == SmartRTL_StopAtHome; }

    // save current position for use by the smart_rtl flight mode
    void save_position();

protected:

    // Safe RTL states
    enum SmartRTLState {
        SmartRTL_WaitForPathCleanup,
        SmartRTL_PathFollow,
        SmartRTL_StopAtHome,
        SmartRTL_Failure
    } smart_rtl_state;

    bool _enter() override;
    bool _load_point;
};


class ModeSteering : public Mode
{
public:

    uint32_t mode_number() const override { return STEERING; }
    const char *name4() const override { return "STER"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }

    // steering requires velocity but not position
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return true; }
};

class ModeInitializing : public Mode
{
public:

    uint32_t mode_number() const override { return INITIALISING; }
    const char *name4() const override { return "INIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
};

class ModeFollow : public Mode
{
public:

    uint32_t mode_number() const override { return FOLLOW; }
    const char *name4() const override { return "FOLL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeSimple : public Mode
{
public:

    uint32_t mode_number() const override { return SIMPLE; }
    const char *name4() const override { return "SMPL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void init_heading();

private:

    // simple type enum used for SIMPLE_TYPE parameter
    enum simple_type {
        Simple_InitialHeading = 0,
        Simple_CardinalDirections = 1,
    };

    float _initial_heading_cd;  // vehicle heading (in centi-degrees) at moment vehicle was armed
    float _desired_heading_cd;  // latest desired heading (in centi-degrees) from pilot
};


#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/SCurve.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <APM_Control/AR_PosControl.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include "AR_PivotTurn.h"

class AR_WPNav {
public:

    // constructor
    AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control);

    // initialise waypoint controller.  speed_max should be set to the maximum speed in m/s (or left at zero to use the default speed)
    void init(float speed_max = 0);

    // update navigation
    virtual void update(float dt);

    // get or set maximum speed in m/s
    // if set_speed_max is called in rapid succession changes in speed may be delayed by up to 0.5sec
    float get_speed_max() const { return _base_speed_max; }
    bool set_speed_max(float speed_max);

    // set speed nudge in m/s.  this will have no effect unless nudge_speed_max > speed_max
    // nudge_speed_max should always be positive regardless of whether the vehicle is travelling forward or reversing
    void set_nudge_speed_max(float nudge_speed_max);

    // execute the mission in reverse (i.e. drive backwards to destination)
    bool get_reversed() const { return _reversed; }
    void set_reversed(bool reversed) { _reversed = reversed; }

    // get navigation outputs for speed (in m/s) and turn rate (in rad/sec)
    float get_speed() const { return _desired_speed_limited; }
    float get_turn_rate_rads() const { return _desired_turn_rate_rads; }

    // get desired lateral acceleration (for reporting purposes only because will be zero during pivot turns)
    float get_lat_accel() const { return _desired_lat_accel; }

    // set desired location and (optionally) next_destination
    // next_destination should be provided if known to allow smooth cornering
    virtual bool set_desired_location(const Location &destination, Location next_destination = Location()) WARN_IF_UNUSED;

    // set desired location to a reasonable stopping point, return true on success
    bool set_desired_location_to_stopping_location()  WARN_IF_UNUSED;

    // set desired location as offset from the EKF origin, return true on success
    bool set_desired_location_NED(const Vector3f& destination) WARN_IF_UNUSED;
    bool set_desired_location_NED(const Vector3f &destination, const Vector3f &next_destination) WARN_IF_UNUSED;

    // set desired location but expect the destination to be updated again in the near future
    // position controller input shaping will be used for navigation instead of scurves
    // Note: object avoidance is not supported if this method is used
    bool set_desired_location_expect_fast_update(const Location &destination) WARN_IF_UNUSED;

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    virtual bool reached_destination() const { return _reached_destination; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const { return _distance_to_destination; }

    // return true if destination is valid
    bool is_destination_valid() const { return _orig_and_dest_valid; }

    // get current destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_destination() const { return _destination; }

    // return heading (in centi-degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing_cd() const { return _wp_bearing_cd; }
    float nav_bearing_cd() const { return _desired_heading_cd; }
    float crosstrack_error() const { return _cross_track_error; }

    // get object avoidance adjusted origin. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    virtual const Location &get_oa_origin() const { return _origin; }

    // get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    virtual const Location &get_oa_destination() const { return get_destination(); }

    // return the heading (in centi-degrees) to the next waypoint accounting for OA, (used by sailboats)
    virtual float oa_wp_bearing_cd() const { return wp_bearing_cd(); }

    // settor to allow vehicle code to provide turn related param values to this library (should be updated regularly)
    void set_turn_params(float turn_radius, bool pivot_possible);

    // enable speeding up position target to catch-up with vehicles travelling faster than WP_SPEED
    // designed to support sailboats that do not have precise speed control
    // only supported when using SCurves and not when using position controller input shaping
    void enable_overspeed(bool enable) { _overspeed_enabled = enable; }

    // accessors for parameter values
    float get_default_speed() const { return _speed_max; }
    float get_radius() const { return _radius; }
    float get_pivot_rate() const { return _pivot.get_rate_max(); }

    // calculate stopping location using current position and attitude controller provided maximum deceleration
    // returns true on success, false on failure
    bool get_stopping_location(Location& stopping_loc) WARN_IF_UNUSED;

    // is_fast_waypoint returns true if vehicle will not stop at destination (e.g. set_desired_location provided a next_destination)
    bool is_fast_waypoint() const { return _fast_waypoint; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // true if update has been called recently
    bool is_active() const;

    // move target location along track from origin to destination using SCurves navigation
    void advance_wp_target_along_track(const Location &current_loc, float dt);

    // update psc input shaping navigation controller
    void update_psc_input_shaping(float dt);

    // update distance and bearing from vehicle's current position to destination
    void update_distance_and_bearing_to_destination();

    // calculate steering and speed to drive along line from origin to destination waypoint
    void update_steering_and_speed(const Location &current_loc, float dt);

    // calculate the crosstrack error (does not rely on L1 controller)
    float calc_crosstrack_error(const Location& current_loc) const;

    // calculate yaw change at next waypoint in degrees
    // returns zero if the angle cannot be calculated because some points are on top of others
    float get_corner_angle(const Location& loc1, const Location& loc2, const Location& loc3) const;

    // helper function to initialise position controller if it hasn't been called recently
    // this should be called before updating the position controller with new targets but after the EKF has a good position estimate
    void init_pos_control_if_necessary();

    // set origin and destination to stopping point
    bool set_origin_and_destination_to_stopping_point();

    // check for changes in _base_speed_max or _nudge_speed_max
    // updates position controller limits and recalculate scurve path if required
    void update_speed_max();

    // parameters
    AP_Float _speed_max;            // target speed between waypoints in m/s
    AP_Float _radius;               // distance in meters from a waypoint when we consider the waypoint has been reached
    AR_PivotTurn _pivot;            // pivot turn controller
    AP_Float _accel_max;            // max acceleration.  If zero then attitude controller's specified max accel is used
    AP_Float _jerk_max;             // max jerk (change in acceleration).  If zero then value is same as accel_max

    // references
    AR_AttitudeControl& _atc;       // rover attitude control library
    AR_PosControl &_pos_control;    // rover position control library

    // scurve
    SCurve _scurve_prev_leg;        // previous scurve trajectory used to blend with current scurve trajectory
    SCurve _scurve_this_leg;        // current scurve trajectory
    SCurve _scurve_next_leg;        // next scurve trajectory used to blend with current scurve trajectory
    bool _fast_waypoint;            // true if vehicle will stop at the next waypoint
    bool _pivot_at_next_wp;         // true if vehicle should pivot at next waypoint
    bool _overspeed_enabled;        // if true scurve's position target will speedup to catch vehicles travelling faster than WP_SPEED
    float _track_scalar_dt;         // time scaler to ensure scurve target doesn't get too far ahead of vehicle

    // variables held in vehicle code (for now)
    float _turn_radius;             // vehicle turn radius in meters

    // variables for navigation
    uint32_t _last_update_ms;       // system time of last call to update
    Location _origin;               // origin Location (vehicle will travel from the origin to the destination)
    Location _destination;          // destination Location when in Guided_WP
    bool _orig_and_dest_valid;      // true if the origin and destination have been set
    bool _reversed;                 // execute the mission by backing up
    enum class NavControllerType {
        NAV_SCURVE = 0,             // scurves used for navigation
        NAV_PSC_INPUT_SHAPING       // position controller input shaping used for navigation
    } _nav_control_type;            // navigation controller that should be used to travel from _origin to _destination

    // speed_max handling
    float _base_speed_max;          // speed max (in m/s) derived from parameters or passed into init
    float _nudge_speed_max;         // "nudge" speed max (in m/s) normally from the pilot.  has no effect if less than _base_speed_max.  always positive.
    uint32_t _last_speed_update_ms; // system time that speed_max was last update.  used to ensure speed_max is not update too quickly

    // main outputs from navigation library
    float _desired_speed_limited;   // desired speed (above) but accel/decel limited
    float _desired_turn_rate_rads;  // desired turn-rate in rad/sec (negative is counter clockwise, positive is clockwise)
    float _desired_lat_accel;       // desired lateral acceleration (for reporting only)
    float _desired_heading_cd;      // desired heading (back towards line between origin and destination)
    float _wp_bearing_cd;           // heading to waypoint in centi-degrees
    float _cross_track_error;       // cross track error (in meters).  distance from current position to closest point on line between origin and destination

    // variables for reporting
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;      // true once the vehicle has reached the destination
};

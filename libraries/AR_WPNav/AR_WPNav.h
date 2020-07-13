#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>

const float AR_WPNAV_HEADING_UNKNOWN = 99999.0f; // used to indicate to set_desired_location method that next leg's heading is unknown

class AR_WPNav {
public:

    // constructor
    AR_WPNav(AR_AttitudeControl& atc, AP_Navigation& nav_controller);

    // update navigation
    void update(float dt);

    // return desired speed
    float get_desired_speed() const { return _desired_speed; }

    // set desired speed in m/s
    void set_desired_speed(float speed) { _desired_speed = MAX(speed, 0.0f); }

    // restore desired speed to default from parameter value
    void set_desired_speed_to_default() { _desired_speed = _speed_max; }

    // execute the mission in reverse (i.e. drive backwards to destination)
    bool get_reversed() const { return _reversed; }
    void set_reversed(bool reversed) { _reversed = reversed; }

    // get navigation outputs for speed (in m/s) and turn rate (in rad/sec)
    float get_speed() const { return _desired_speed_limited; }
    float get_turn_rate_rads() const { return _desired_turn_rate_rads; }

    // get desired lateral acceleration (for reporting purposes only because will be zero during pivot turns)
    float get_lat_accel() const { return _desired_lat_accel; }

    // set desired location
    // next_leg_bearing_cd should be heading to the following waypoint (used to slow the vehicle in order to make the turn)
    bool set_desired_location(const struct Location& destination, float next_leg_bearing_cd = AR_WPNAV_HEADING_UNKNOWN)  WARN_IF_UNUSED;

    // set desired location to a reasonable stopping point, return true on success
    bool set_desired_location_to_stopping_location()  WARN_IF_UNUSED;

    // set desired location as offset from the EKF origin, return true on success
    bool set_desired_location_NED(const Vector3f& destination, float next_leg_bearing_cd = AR_WPNAV_HEADING_UNKNOWN) WARN_IF_UNUSED;

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    bool reached_destination() const { return _reached_destination; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const { return _distance_to_destination; }

    // return true if destination is valid
    bool is_destination_valid() const { return _orig_and_dest_valid; }

    // get current destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_destination() { return _destination; }

    // get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_oa_destination() { return _oa_destination; }

    // return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing_cd() const { return _wp_bearing_cd; }
    float nav_bearing_cd() const { return _desired_heading_cd; }
    float crosstrack_error() const { return _cross_track_error; }

    // return the heading (in centi-degrees) to the next waypoint accounting for OA, (used by sailboats)
    float oa_wp_bearing_cd() const { return _oa_wp_bearing_cd; }

    // settor to allow vehicle code to provide turn related param values to this library (should be updated regularly)
    void set_turn_params(float turn_max_g, float turn_radius, bool pivot_possible);

    // accessors for parameter values
    float get_default_speed() const { return _speed_max; }
    float get_radius() const { return _radius; }
    float get_pivot_rate() const { return _pivot_rate; }

    // calculate stopping location using current position and attitude controller provided maximum deceleration
    // returns true on success, false on failure
    bool get_stopping_location(Location& stopping_loc) WARN_IF_UNUSED;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // true if update has been called recently
    bool is_active() const;

    // update distance and bearing from vehicle's current position to destination
    void update_distance_and_bearing_to_destination();

    // calculate steering output to drive along line from origin to destination waypoint
    // relies on update_distance_and_bearing_to_destination being called first
    void update_steering(const Location& current_loc, float current_speed);

    // calculated desired speed(in m/s) based on yaw error and lateral acceleration and/or distance to a waypoint
    // relies on update_distance_and_bearing_to_destination and update_steering being run so these internal members
    // have been updated: _wp_bearing_cd, _cross_track_error, _distance_to_destination
    void update_desired_speed(float dt);

    // returns true if vehicle should pivot turn at next waypoint
    bool use_pivot_steering_at_next_WP(float yaw_error_cd) const;

    // returns true if vehicle should pivot immediately (because heading error is too large)
    bool use_pivot_steering(float yaw_error_cd);

    // adjust speed to ensure it does not fall below value held in SPEED_MIN
    void apply_speed_min(float &desired_speed);

private:

    // parameters
    AP_Float _speed_max;            // target speed between waypoints in m/s
    AP_Float _speed_min;            // target speed minimum in m/s.  Vehicle will not slow below this speed for corners
    AP_Float _radius;               // distance in meters from a waypoint when we consider the waypoint has been reached
    AP_Float _overshoot;            // maximum horizontal overshoot in meters
    AP_Int16 _pivot_angle;          // angle error that leads to pivot turn
    AP_Int16 _pivot_rate;           // desired turn rate during pivot turns in deg/sec

    // references
    AR_AttitudeControl& _atc;       // rover attitude control library
    AP_Navigation& _nav_controller; // navigation controller (aka L1 controller)

    // variables held in vehicle code (for now)
    float _turn_max_mss;            // lateral acceleration maximum in m/s/s
    float _turn_radius;             // vehicle turn radius in meters
    bool _pivot_possible;           // true if vehicle can pivot
    bool _pivot_active;             // true if vehicle is currently pivoting

    // variables for navigation
    uint32_t _last_update_ms;       // system time of last call to update
    Location _origin;               // origin Location (vehicle will travel from the origin to the destination)
    Location _destination;          // destination Location when in Guided_WP
    bool _orig_and_dest_valid;      // true if the origin and destination have been set
    bool _reversed;                 // execute the mission by backing up
    float _desired_speed_final;     // desired speed in m/s when we reach the destination

    // main outputs from navigation library
    float _desired_speed;           // desired speed in m/s
    float _desired_speed_limited;   // desired speed (above) but accel/decel limited and reduced to keep vehicle within _overshoot of line
    float _desired_turn_rate_rads;  // desired turn-rate in rad/sec (negative is counter clockwise, positive is clockwise)
    float _desired_lat_accel;       // desired lateral acceleration (for reporting only)
    float _desired_heading_cd;      // desired heading (back towards line between origin and destination)
    float _wp_bearing_cd;           // heading to waypoint in centi-degrees
    float _cross_track_error;       // cross track error (in meters).  distance from current position to closest point on line between origin and destination

    // variables for reporting
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;      // true once the vehicle has reached the destination

    // object avoidance variables
    bool _oa_active;                // true if we should use alternative destination to avoid obstacles
    Location _oa_origin;            // intermediate origin during avoidance
    Location _oa_destination;       // intermediate destination during avoidance
    float _oa_distance_to_destination; // OA (object avoidance) distance from vehicle to _oa_destination in meters
    float _oa_wp_bearing_cd;        // OA adjusted heading to _oa_destination in centi-degrees
};

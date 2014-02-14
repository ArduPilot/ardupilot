/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_WPNAV_H
#define AC_WPNAV_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_PosControl.h>      // Position control library

// loiter maximum velocities and accelerations
#define WPNAV_ACCELERATION              100.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_ACCELERATION_MIN           50.0f      // minimum acceleration in cm/s/s - used for sanity checking _wp_accel parameter
#define WPNAV_ACCEL_MAX                 980.0f      // max acceleration in cm/s/s that the loiter velocity controller will ask from the lower accel controller.
                                                    // should be 1.5 times larger than WPNAV_ACCELERATION.
                                                    // max acceleration = max lean angle * 980 * pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s

#define WPNAV_LOITER_SPEED              500.0f      // maximum default loiter speed in cm/s
#define WPNAV_LOITER_ACCEL_MAX          250.0f      // maximum acceleration in loiter mode
#define WPNAV_LOITER_ACCEL_MIN           25.0f      // minimum acceleration in loiter mode
#define WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR 200.0f      // maximum speed used to correct position error (i.e. not including feed forward)

#define MAX_LEAN_ANGLE                  4500        // default maximum lean angle

#define WPNAV_WP_SPEED                  500.0f      // default horizontal speed betwen waypoints in cm/s
#define WPNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm

#define WPNAV_WP_SPEED_UP               250.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             150.0f      // default maximum descent velocity

#define WPNAV_ALT_HOLD_P                1.0f        // default throttle controller's altitude hold's P gain.
#define WPNAV_ALT_HOLD_ACCEL_MAX        250.0f      // hard coded copy of throttle controller's maximum acceleration in cm/s.  To-Do: remove duplication with throttle controller definition

#define WPNAV_MIN_LEASH_LENGTH          100.0f      // minimum leash lengths in cm

class AC_WPNav
{
public:

    /// Constructor
    AC_WPNav(const AP_InertialNav* inav, const AP_AHRS* ahrs, AC_PosControl& pos_control);

    ///
    /// loiter controller
    ///

    /// set_loiter_target in cm from home
    void set_loiter_target(const Vector3f& position);

    /// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
    void init_loiter_target();

    /// set_loiter_velocity - allows main code to pass the maximum velocity for loiter
    void set_loiter_velocity(float velocity_cms) { _loiter_speed_cms = velocity_cms; };

    /// calculate_loiter_leash_length - calculates the maximum distance in cm that the target position may be from the current location
    void calculate_loiter_leash_length();

    /// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
    void set_pilot_desired_acceleration(float control_roll, float control_pitch);

    /// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
    void get_loiter_stopping_point_xy(Vector3f& stopping_point) const;

    /// get_loiter_distance_to_target - get horizontal distance to loiter target in cm
    float get_loiter_distance_to_target() const { return _pos_control.get_distance_to_target(); }

    /// get_loiter_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_loiter_bearing_to_target() const;

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_loiter();

    ///
    /// waypoint controller
    ///

    /// set_horizontal_velocity - allows main code to pass target horizontal velocity for wp navigation
    void set_horizontal_velocity(float velocity_cms) { _wp_speed_cms = velocity_cms; };

    /// get_horizontal_velocity - allows main code to retrieve target horizontal velocity for wp navigation
    float get_horizontal_velocity() { return _wp_speed_cms; };

    /// get_climb_velocity - returns target climb speed in cm/s during missions
    float get_climb_velocity() const { return _wp_speed_up_cms; };

    /// get_descent_velocity - returns target descent speed in cm/s during missions.  Note: always positive
    float get_descent_velocity() const { return _wp_speed_down_cms; };

    /// get_wp_radius - access for waypoint radius in cm
    float get_wp_radius() const { return _wp_radius_cm; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration() const { return _wp_accel_cms.get(); }

    /// get_wp_destination waypoint using position vector (distance from home in cm)
    const Vector3f &get_wp_destination() const { return _destination; }

    /// set_wp_destination waypoint using position vector (distance from home in cm)
    void set_wp_destination(const Vector3f& destination);

    /// set_wp_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    void set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination);

    /// get_wp_stopping_point_xy - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_xy(Vector3f& stopping_point) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    float get_wp_distance_to_destination();

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_wp_bearing_to_destination();

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_wp_destination() const { return _flags.reached_destination; }

    /// set_fast_waypoint - set to true to ignore the waypoint radius and consider the waypoint 'reached' the moment the intermediate point reaches it
    void set_fast_waypoint(bool fast) { _flags.fast_waypoint = fast; }

    /// update_wp - update waypoint controller
    void update_wpnav();

    /// calculate_wp_leash_length - calculates track speed, acceleration and leash lengths for waypoint controller
    void calculate_wp_leash_length();

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); };
    int32_t get_pitch() const { return _pos_control.get_pitch(); };

    /// get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    float get_desired_alt() const { return _pos_control.get_alt_target(); }

    /// set_desired_alt - set desired altitude (in cm above home)
    void set_desired_alt(float desired_alt) { _pos_control.set_alt_target(desired_alt); }
    /// advance_wp_target_along_track - move target location along track from origin to destination
    void advance_wp_target_along_track(float dt);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
    } _flags;

    /// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
    ///		updated velocity sent directly to position controller
    void calc_loiter_desired_velocity(float nav_dt);

    /// get_bearing_cd - return bearing in centi-degrees between two positions
    float get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const;

    // references to inertial nav and ahrs libraries
    const AP_InertialNav* const _inav;
    const AP_AHRS*        const _ahrs;
    AC_PosControl&              _pos_control;

    // parameters
    AP_Float    _loiter_speed_cms;      // maximum horizontal speed in cm/s while in loiter
    AP_Float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // climb speed target in cm/s
    AP_Float    _wp_speed_down_cms;     // descent speed target in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cms;          // acceleration in cm/s/s during missions

    // loiter controller internal variables
    uint32_t    _loiter_last_update;    // time of last update_loiter call
    uint8_t     _loiter_step;           // used to decide which portion of loiter controller to run during this iteration
    int16_t     _pilot_accel_fwd_cms; 	// pilot's desired acceleration forward (body-frame)
    int16_t     _pilot_accel_rgt_cms;   // pilot's desired acceleration right (body-frame)
    float       _loiter_accel_cms;      // loiter's acceleration in cm/s/s

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    uint8_t     _wp_step;               // used to decide which portion of wpnav controller to run during this iteration
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track
};
#endif	// AC_WPNAV_H

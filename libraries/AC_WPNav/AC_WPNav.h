/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_WPNAV_H
#define AC_WPNAV_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_InertialNav.h>     // Inertial Navigation library

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
    AC_WPNav(const AP_InertialNav* inav, const AP_AHRS* ahrs, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_rate_lat, AC_PID* pid_rate_lon);

    ///
    /// simple loiter controller
    ///

    /// get_loiter_target - get loiter target as position vector (from home in cm)
    const Vector3f &get_loiter_target() const { return _target; }

    /// set_loiter_target in cm from home
    void set_loiter_target(const Vector3f& position);

    /// init_loiter_target - set initial loiter target based on current position and velocity
    void init_loiter_target(const Vector3f& position, const Vector3f& velocity);

    /// move_loiter_target - move destination using pilot input
    void move_loiter_target(float control_roll, float control_pitch, float dt);

    /// get_distance_to_target - get horizontal distance to loiter target in cm
    float get_distance_to_target() const;

    /// get_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_bearing_to_target() const;

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_loiter();

    /// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
    void get_stopping_point(const Vector3f& position, const Vector3f& velocity, Vector3f &target) const;

    ///
    /// waypoint controller
    ///

    /// get_destination waypoint using position vector (distance from home in cm)
    const Vector3f &get_destination() const { return _destination; }

    /// set_destination waypoint using position vector (distance from home in cm)
    void set_destination(const Vector3f& destination);

    /// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    void set_origin_and_destination(const Vector3f& origin, const Vector3f& destination);

    /// advance_target_along_track - move target location along track from origin to destination
    void advance_target_along_track(float dt);

    /// get_distance_to_destination - get horizontal distance to destination in cm
    float get_distance_to_destination();

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_bearing_to_destination();

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_destination() const { return _flags.reached_destination; }

    /// set_fast_waypoint - set to true to ignore the waypoint radius and consider the waypoint 'reached' the moment the intermediate point reaches it
    void set_fast_waypoint(bool fast) { _flags.fast_waypoint = fast; }

    /// update_wp - update waypoint controller
    void update_wpnav();

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_desired_roll() const { return _desired_roll; };
    int32_t get_desired_pitch() const { return _desired_pitch; };

    /// get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    float get_desired_alt() const { return _target.z; }

    /// set_desired_alt - set desired altitude (in cm above home)
    void set_desired_alt(float desired_alt) { _target.z = desired_alt; }

    /// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
    void set_cos_sin_yaw(float cos_yaw, float sin_yaw, float cos_pitch) {
        _cos_yaw = cos_yaw;
        _sin_yaw = sin_yaw;
        _cos_pitch = cos_pitch;
    }

    /// set_althold_kP - pass in alt hold controller's P gain
    void set_althold_kP(float kP) { if(kP>0.0f) _althold_kP = kP; }

    /// set_horizontal_velocity - allows main code to pass target horizontal velocity for wp navigation
    void set_horizontal_velocity(float velocity_cms) { _wp_speed_cms = velocity_cms; };

    /// get_horizontal_velocity - allows main code to retrieve target horizontal velocity for wp navigation
    float get_horizontal_velocity() { return _wp_speed_cms; };

    /// get_climb_velocity - returns target climb speed in cm/s during missions
    float get_climb_velocity() const { return _wp_speed_up_cms; };

    /// get_descent_velocity - returns target descent speed in cm/s during missions.  Note: always positive
    float get_descent_velocity() const { return _wp_speed_down_cms; };

    /// get_waypoint_radius - access for waypoint radius in cm
    float get_waypoint_radius() const { return _wp_radius_cm; }

    /// get_waypoint_acceleration - returns acceleration in cm/s/s during missions
    float get_waypoint_acceleration() const { return _wp_accel_cms.get(); }

    /// set_lean_angle_max - limits maximum lean angle
    void set_lean_angle_max(int16_t angle_cd) { if (angle_cd >= 1000 && angle_cd <= 8000) {_lean_angle_max_cd = angle_cd;} }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
    } _flags;

    /// translate_loiter_target_movements - consumes adjustments created by move_loiter_target
    void translate_loiter_target_movements(float nav_dt);

    /// get_loiter_position_to_velocity - loiter position controller
    ///     converts desired position held in _target vector to desired velocity
    void get_loiter_position_to_velocity(float dt, float max_speed_cms);

    /// get_loiter_velocity_to_acceleration - loiter velocity controller
    ///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    void get_loiter_velocity_to_acceleration(float vel_lat_cms, float vel_lon_cms, float dt);

    /// get_loiter_acceleration_to_lean_angles - loiter acceleration controller
    ///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
    void get_loiter_acceleration_to_lean_angles(float accel_lat_cmss, float accel_lon_cmss);

    /// get_bearing_cd - return bearing in centi-degrees between two positions
    float get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const;

    /// reset_I - clears I terms from loiter PID controller
    void reset_I();

    /// calculate_loiter_leash_length - calculates the maximum distance in cm that the target position may be from the current location
    void calculate_loiter_leash_length();

    /// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
    ///    set climb param to true if track climbs vertically, false if descending
    void calculate_wp_leash_length(bool climb);

    // references to inertial nav and ahrs libraries
    const AP_InertialNav* const _inav;
    const AP_AHRS*        const _ahrs;

    // pointers to pid controllers
    APM_PI*		const _pid_pos_lat;
    APM_PI*		const _pid_pos_lon;
    AC_PID*		const _pid_rate_lat;
    AC_PID*		const _pid_rate_lon;

    // parameters
    AP_Float    _loiter_speed_cms;      // maximum horizontal speed in cm/s while in loiter
    AP_Float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // climb speed target in cm/s
    AP_Float    _wp_speed_down_cms;     // descent speed target in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cms;          // acceleration in cm/s/s during missions
    uint8_t     _loiter_step;           // used to decide which portion of loiter controller to run during this iteration
    uint8_t     _wpnav_step;            // used to decide which portion of wpnav controller to run during this iteration
    uint32_t	_loiter_last_update;    // time of last update_loiter call
    uint32_t	_wpnav_last_update;     // time of last update_wpnav call
    float       _loiter_dt;             // time difference since last loiter call
    float       _wpnav_dt;              // time difference since last loiter call
    float       _cos_yaw;               // short-cut to save on calcs required to convert roll-pitch frame to lat-lon frame
    float       _sin_yaw;
    float       _cos_pitch;
    float       _althold_kP;            // alt hold's P gain

    // output from controller
    int32_t     _desired_roll;          // fed to stabilize controllers at 50hz
    int32_t     _desired_pitch;         // fed to stabilize controllers at 50hz

    // loiter controller internal variables
    Vector3f    _target;   		        // loiter's target location in cm from home
    int16_t     _pilot_vel_forward_cms; // pilot's desired velocity forward (body-frame)
    int16_t     _pilot_vel_right_cms;   // pilot's desired velocity right (body-frame)
    Vector3f    _target_vel;            // pilot's latest desired velocity in earth-frame
    Vector3f    _vel_last;              // previous iterations velocity in cm/s
    float       _loiter_leash;          // loiter's horizontal leash length in cm.  used to stop the pilot from pushing the target location too far from the current location
    float       _loiter_accel_cms;      // loiter's acceleration in cm/s/s
    int16_t     _lean_angle_max_cd;     // maximum lean angle in centi-degrees

    // waypoint controller internal variables
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _distance_to_target;    // distance to loiter target
    float       _wp_leash_xy;           // horizontal leash length in cm
    float       _wp_leash_z;            // horizontal leash length in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track

public:
    // for logging purposes
    Vector2f dist_error;                // distance error calculated by loiter controller
    Vector2f desired_vel;               // loiter controller desired velocity
    Vector2f desired_accel;             // the resulting desired acceleration

    // To-Do: add split of fast (100hz for accel->angle) and slow (10hz for navigation)
    /// update - run the loiter and wpnav controllers - should be called at 100hz
    //void update_100hz(void);
    /// update - run the loiter and wpnav controllers - should be called at 10hz
    //void update_10hz(void);
};
#endif	// AC_WPNAV_H

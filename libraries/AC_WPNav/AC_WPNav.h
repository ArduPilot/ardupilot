/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_WPNAV_H
#define AC_WPNAV_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_InertialNav.h>     // Inertial Navigation library

// loiter maximum velocities and accelerations
#define MAX_LOITER_POS_VELOCITY         750         // should be 1.5 ~ 2.0 times the pilot input's max velocity
#define MAX_LOITER_POS_ACCEL            250
#define MAX_LOITER_VEL_ACCEL            400         // should be 1.5 times larger than MAX_LOITER_POS_ACCEL
#define MAX_LOITER_POS_VEL_VELOCITY     1000
#define WPINAV_MAX_POS_ERROR            2000.0f     // maximum distance (in cm) that the desired track can stray from our current location.
#define WP_SPEED                        500         // default horizontal speed betwen waypoints in cm/s

class AC_WPNav
{
public:

    /// Constructor
    AC_WPNav(AP_InertialNav* inav, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_rate_lat, AC_PID* pid_rate_lon);

    ///
    /// simple loiter controller
    ///

    /// get_loiter_target - get loiter target as position vector (from home in cm)
    Vector3f get_loiter_target() { return _target; }

    /// set_loiter_target in cm from home
    void set_loiter_target(const Vector3f& position) { _target = position; }

    /// move_loiter_target - move destination using forward and right velocities in cm/s
    void move_loiter_target(int16_t vel_forward_cms, int16_t vel_right_cms, float dt);

    /// get_distance_to_target - get horizontal distance to loiter target in cm
    float get_distance_to_target();

    /// get_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_bearing_to_target();

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_loiter();

    ///
    /// waypoint navigation
    ///

    /// set_destination waypoint using position vector (distance from home in cm)
    void set_destination(const Vector3f& destination);

    /// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
    void set_origin_and_destination(const Vector3f& origin, const Vector3f& destination);

    /// advance_target_along_track - move target location along track from origin to destination
    void advance_target_along_track(float velocity_cms, float dt);

    /// get_distance_to_destination - get horizontal distance to destination in cm
    float get_distance_to_destination();

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_bearing_to_destination();

    /// update_wp - update waypoint controller
    void update_wpnav();

    ///
    /// shared methods
    ///

    /// get desired roll, pitch and altitude which should be fed into stabilize controllers
    int32_t get_desired_roll() { return _desired_roll; };
    int32_t get_desired_pitch() { return _desired_pitch; };

    /// desired altitude (in cm) that should be fed into altitude hold controller.  only valid when navigating between waypoints
    int32_t get_desired_altitude() { return _desired_altitude; };

    /// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
    void set_cos_sin_yaw(float cos_yaw, float sin_yaw, float cos_roll) {
        _cos_yaw = cos_yaw;
        _sin_yaw = sin_yaw;
        _cos_roll = cos_roll;
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// get_loiter_pos_lat_lon - loiter position controller
    ///     converts desired position provided as distance from home in lat/lon directions to desired velocity
    void get_loiter_pos_lat_lon(int32_t target_lat_from_home, int32_t target_lon_from_home, float dt);

    /// get_loiter_vel_lat_lon - loiter velocity controller
    ///    converts desired velocities in lat/lon frame to accelerations in lat/lon frame
    void get_loiter_vel_lat_lon(int16_t vel_lat, int16_t vel_lon, float dt);

    /// get_loiter_accel_lat_lon - loiter acceration controller
    ///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
    void get_loiter_accel_lat_lon(int16_t accel_lat, int16_t accel_lon);

    /// get_bearing_cd - return bearing in centi-degrees between two positions
    float get_bearing_cd(const Vector3f origin, const Vector3f destination);

    /// reset_I - clears I terms from loiter PID controller
    void reset_I();

    // pointers to inertial nav library
    AP_InertialNav*	_inav;

    // pointers to pid controllers
    APM_PI*		_pid_pos_lat;
    APM_PI*		_pid_pos_lon;
    AC_PID*		_pid_rate_lat;
    AC_PID*		_pid_rate_lon;

    // parameters
    AP_Float    _speed_cms;         // default horizontal speed in cm/s
    uint32_t	_last_update;		// time of last update call
    float       _cos_yaw;           // short-cut to save on calcs required to convert roll-pitch frame to lat-lon frame
    float       _sin_yaw;
    float       _cos_roll;

    // output from controller
    int32_t     _desired_roll;          // fed to stabilize controllers at 50hz
    int32_t     _desired_pitch;         // fed to stabilize controllers at 50hz
    int32_t     _desired_altitude;      // fed to alt hold controller at 50hz

    // internal variables
    Vector3f    _target;   		        // loiter's target location in cm from home
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    Vector3f    _pos_delta;             // position difference between origin and destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _distance_to_target;    // distance to loiter target

    // To-Do: add split of fast (100hz for accel->angle) and slow (10hz for navigation)
    //float       _desired_accel_fwd;     // updated from loiter controller at 10hz, consumed by accel->angle controller at 50hz
    //float       _desired_accel_rgt;
    /// update - run the loiter and wpnav controllers - should be called at 100hz
    //void update_100hz(void);
    /// update - run the loiter and wpnav controllers - should be called at 10hz
    //void update_10hz(void);
};
#endif	// AC_WPNAV_H

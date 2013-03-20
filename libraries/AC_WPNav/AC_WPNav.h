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

// possible states
#define WPNAV_STATE_INACTIVE    0
#define WPNAV_STATE_LOITER 	    1
#define WPNAV_STATE_WPNAV       2

class AC_WPNav
{
public:

    /// Constructor
    AC_WPNav(AP_InertialNav* inav, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_lat_rate, AC_PID* pid_lon_rate);

    ///
    /// simple loiter controller
    ///

    /// set_loiter_target in cm from home
    void set_loiter_target(const Vector3f& position);

    /// move_loiter_target - move destination using forward and right velocities in cm/s
    void move_loiter_target(int16_t vel_forward_cms, int16_t vel_right_cms, float dt);

    ///
    /// waypoint navigation
    ///

    /// set_destination with distance from home in cm
    void set_destination(const Vector3f& destination);

    /// set_origin_and_destination - set origin and destination using lat/lon coordinates
    void set_origin_and_destination(const Vector3f& origin, const Vector3f& destination);

    /// advance_target_along_track - move target location along track from origin to destination
    void advance_target_along_track(float velocity_cms, float dt);

    ///
    /// shared methods
    ///

    /// update - run the loiter and wpnav controllers - should be called at 10hz
    void update(void);

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

    /// waypoint controller
    /// get_wpinav_pos - wpinav position controller with desired position held in wpinav_destination
    void get_wpinav_pos(float dt);

    // pointers to inertial nav library
    AP_InertialNav*	_inav;

    // pointers to pid controllers
    APM_PI*		_pid_pos_lat;
    APM_PI*		_pid_pos_lon;
    AC_PID*		_pid_rate_lat;
    AC_PID*		_pid_rate_lon;

    // parameters
    AP_Float    _speed_cms;         // default horizontal speed in cm/s
    uint8_t		_state;				// records whether we are loitering or navigating towards a waypoint
    uint32_t	_last_update;		// time of last update call
    float       _cos_yaw;           // short-cut to save on calcs required to convert roll-pitch frame to lat-lon frame
    float       _sin_yaw;
    float       _cos_roll;

    // output from controller
    int32_t     _desired_roll;
    int32_t     _desired_pitch;
    int32_t     _desired_altitude;

    Vector3f    _target;   		        // loiter's target location in cm from home
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    Vector3f    _wpinav_target;         // the intermediate target location in cm from home
    Vector3f    _pos_delta;             // position difference between origin and destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
};
#endif	// AC_WPNAV_H

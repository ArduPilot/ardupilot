#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>

#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to avoid hitting fence

// bit masks for enabled fence types.
#define AC_AVOID_DISABLED               0       // avoidance disabled
#define AC_AVOID_STOP_AT_FENCE          1       // stop at fence
#define AC_AVOID_USE_PROXIMITY_SENSOR   2       // stop based on proximity sensor output
#define AC_AVOID_ALL                    3       // use fence and promiximity sensor

// definitions for non-GPS avoidance
#define AC_AVOID_NONGPS_DIST_MAX_DEFAULT    10.0f   // objects over 10m away are ignored (default value for DIST_MAX parameter)
#define AC_AVOID_ANGLE_MAX_PERCENT          0.75f   // object avoidance max lean angle as a percentage (expressed in 0 ~ 1 range) of total vehicle max lean angle

#define AC_AVOID_UPWARD_MARGIN_M            2.0f    // stop 2m before objects above the vehicle

/*
 * This class prevents the vehicle from leaving a polygon fence in
 * 2 dimensions by limiting velocity (adjust_velocity).
 */
class AC_Avoid {
public:

    /// Constructor
    AC_Avoid(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AC_Fence& fence, const AP_Proximity& proximity);

    /*
     * Adjusts the desired velocity so that the vehicle can stop
     * before the fence/object.
     * Note: Vector3f version is for convenience and only adjusts x and y axis
     */
    void adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel);
    void adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel);

    // adjust vertical climb rate so vehicle does not break the vertical fence
    void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms);

    // adjust roll-pitch to push vehicle away from objects
    // roll and pitch value are in centi-degrees
    // angle_max is the user defined maximum lean angle for the vehicle in centi-degrees
    void adjust_roll_pitch(float &roll, float &pitch, float angle_max);

    // enable/disable proximity based avoidance
    void proximity_avoidance_enable(bool on_off) { _proximity_enabled = on_off; }
    bool proximity_avoidance_enabled() { return _proximity_enabled; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    /*
     * Adjusts the desired velocity for the circular fence.
     */
    void adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel);

    /*
     * Adjusts the desired velocity for the polygon fence.
     */
    void adjust_velocity_polygon_fence(float kP, float accel_cmss, Vector2f &desired_vel);

    /*
     * Adjusts the desired velocity based on output from the proximity sensor
     */
    void adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel);

    /*
     * Adjusts the desired velocity given an array of boundary points
     *   earth_frame should be true if boundary is in earth-frame, false for body-frame
     */
    void adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel, const Vector2f* boundary, uint16_t num_points, bool earth_frame);

    /*
     * Limits the component of desired_vel in the direction of the unit vector
     * limit_direction to be at most the maximum speed permitted by the limit_distance.
     *
     * Uses velocity adjustment idea from Randy's second email on this thread:
     * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
     */
    void limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel, const Vector2f& limit_direction, float limit_distance) const;

    /*
     * Gets the current position or altitude, relative to home (not relative to EKF origin) in cm
     */
    Vector2f get_position() const;
    float get_alt_above_home() const;

    /*
     * Computes the speed such that the stopping distance
     * of the vehicle will be exactly the input distance.
     */
    float get_max_speed(float kP, float accel_cmss, float distance) const;

    /*
     * Computes distance required to stop, given current speed.
     */
    float get_stopping_distance(float kP, float accel_cmss, float speed) const;

    /*
     * Gets the fence margin in cm
     */
    float get_margin() const { return _fence.get_margin() * 100.0f; }

    /*
     * methods for avoidance in non-GPS flight modes
     */

    // convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
    float distance_to_lean_pct(float dist_m);

    // returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
    void get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative);

    // external references
    const AP_AHRS& _ahrs;
    const AP_InertialNav& _inav;
    const AC_Fence& _fence;
    const AP_Proximity& _proximity;

    // parameters
    AP_Int8 _enabled;
    AP_Int16 _angle_max;        // maximum lean angle to avoid obstacles (only used in non-GPS flight modes)
    AP_Float _dist_max;         // distance (in meters) from object at which obstacle avoidance will begin in non-GPS modes

    bool _proximity_enabled = true; // true if proximity sensor based avoidance is enabled (used to allow pilot to enable/disable)
};

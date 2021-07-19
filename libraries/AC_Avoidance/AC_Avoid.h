#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller

#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to avoid hitting fence

// bit masks for enabled fence types.
#define AC_AVOID_DISABLED               0       // avoidance disabled
#define AC_AVOID_STOP_AT_FENCE          1       // stop at fence
#define AC_AVOID_USE_PROXIMITY_SENSOR   2       // stop based on proximity sensor output
#define AC_AVOID_STOP_AT_BEACON_FENCE   4       // stop based on beacon perimeter
#define AC_AVOID_DEFAULT                (AC_AVOID_STOP_AT_FENCE | AC_AVOID_USE_PROXIMITY_SENSOR)

// definitions for non-GPS avoidance
#define AC_AVOID_NONGPS_DIST_MAX_DEFAULT    5.0f    // objects over 5m away are ignored (default value for DIST_MAX parameter)
#define AC_AVOID_ANGLE_MAX_PERCENT          0.75f   // object avoidance max lean angle as a percentage (expressed in 0 ~ 1 range) of total vehicle max lean angle

#define AC_AVOID_ACTIVE_LIMIT_TIMEOUT_MS    500     // if limiting is active if last limit is happend in the last x ms
#define AC_AVOID_ACCEL_TIMEOUT_MS           200     // stored velocity used to calculate acceleration will be reset if avoidance is active after this many ms

/*
 * This class prevents the vehicle from leaving a polygon fence or hitting proximity-based obstacles
 * Additionally the vehicle may back up if the margin to obstacle is breached
 */
class AC_Avoid {
public:
    AC_Avoid();

    /* Do not allow copies */
    AC_Avoid(const AC_Avoid &other) = delete;
    AC_Avoid &operator=(const AC_Avoid&) = delete;

    // get singleton instance
    static AC_Avoid *get_singleton() {
        return _singleton;
    }

    // return true if any avoidance feature is enabled
    bool enabled() const { return _enabled != AC_AVOID_DISABLED; }

    // Adjusts the desired velocity so that the vehicle can stop
    // before the fence/object.
    // kP, accel_cmss are for the horizontal axis
    // kP_z, accel_cmss_z are for vertical axis
    void adjust_velocity(Vector3f &desired_vel_cms, bool &backing_up, float kP, float accel_cmss, float kP_z, float accel_cmss_z, float dt);
    void adjust_velocity(Vector3f &desired_vel_cms, float kP, float accel_cmss, float kP_z, float accel_cmss_z, float dt) {
        bool backing_up = false;
        adjust_velocity(desired_vel_cms, backing_up, kP, accel_cmss, kP_z, accel_cmss_z, dt);
    }

    // This method limits velocity and calculates backaway velocity from various supported fences
    // Also limits vertical velocity using adjust_velocity_z method
    void adjust_velocity_fence(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt);

    // adjust desired horizontal speed so that the vehicle stops before the fence or object
    // accel (maximum acceleration/deceleration) is in m/s/s
    // heading is in radians
    // speed is in m/s
    // kP should be zero for linear response, non-zero for non-linear response
    // dt is the time since the last call in seconds
    void adjust_speed(float kP, float accel, float heading, float &speed, float dt);

    // adjust vertical climb rate so vehicle does not break the vertical fence
    void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float& backup_speed, float dt);
    void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt) {
        float backup_speed = 0.0f;
        adjust_velocity_z(kP, accel_cmss, climb_rate_cms, backup_speed, dt);
        if (!is_zero(backup_speed)) {
            climb_rate_cms = MIN(climb_rate_cms, backup_speed);
        }
    }
    

    // adjust roll-pitch to push vehicle away from objects
    // roll and pitch value are in centi-degrees
    // angle_max is the user defined maximum lean angle for the vehicle in centi-degrees
    void adjust_roll_pitch(float &roll, float &pitch, float angle_max);

    // enable/disable proximity based avoidance
    void proximity_avoidance_enable(bool on_off) { _proximity_enabled = on_off; }
    bool proximity_avoidance_enabled() const { return _proximity_enabled; }
    void proximity_alt_avoidance_enable(bool on_off) { _proximity_alt_enabled = on_off; }

    // helper functions

    // Limits the component of desired_vel_cms in the direction of the unit vector
    // limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
    // uses velocity adjustment idea from Randy's second email on this thread:
    //   https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
    void limit_velocity_2D(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt);
    
    // Note: This method is used to limit velocity horizontally and vertically given a 3D desired velocity vector 
    // Limits the component of desired_vel_cms in the direction of the obstacle_vector based on the passed value of "margin"
    void limit_velocity_3D(float kP, float accel_cmss, Vector3f &desired_vel_cms, const Vector3f& limit_direction, float limit_distance_cm, float kP_z, float accel_cmss_z ,float dt);
    
     // compute the speed such that the stopping distance of the vehicle will
     // be exactly the input distance.
     // kP should be non-zero for Copter which has a non-linear response
    float get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const;

    // return margin (in meters) that the vehicle should stay from objects
    float get_margin() const { return _margin; }

    // return minimum alt (in meters) above which avoidance will be active
    float get_min_alt() const { return _alt_min; }

    // return true if limiting is active
    bool limits_active() const {return (AP_HAL::millis() - _last_limit_time) < AC_AVOID_ACTIVE_LIMIT_TIMEOUT_MS;};

    static const struct AP_Param::GroupInfo var_info[];

private:
    // behaviour types (see BEHAVE parameter)
    enum BehaviourType {
        BEHAVIOR_SLIDE = 0,
        BEHAVIOR_STOP = 1
    };

    /*
     * Limit acceleration so that change of velocity output by avoidance library is controlled
     * This helps reduce jerks and sudden movements in the vehicle
     */
    void limit_accel(const Vector3f &original_vel, Vector3f &modified_vel, float dt);

    /*
     * Adjusts the desired velocity for the circular fence.
     */
    void adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * Adjusts the desired velocity for inclusion and exclusion polygon fences
     */
    void adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * Adjusts the desired velocity for the inclusion and exclusion circles
     */
    void adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);
    void adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * Adjusts the desired velocity for the beacon fence.
     */
    void adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * Adjusts the desired velocity based on output from the proximity sensor
     */
    void adjust_velocity_proximity(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt);

    /*
     * Adjusts the desired velocity given an array of boundary points
     * The boundary must be in Earth Frame
     * margin is the distance (in meters) that the vehicle should stop short of the polygon
     * stay_inside should be true for fences, false for exclusion polygons
     */
    void adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, const Vector2f* boundary, uint16_t num_points, float margin, float dt, bool stay_inside);

    /*
     * Computes distance required to stop, given current speed.
     */
    float get_stopping_distance(float kP, float accel_cmss, float speed_cms) const;

   /*
    * Compute the back away velocity required to avoid breaching margin
    * INPUT: This method requires the breach in margin distance (back_distance_cm), direction towards the breach (limit_direction)
    *        It then calculates the desired backup velocity and passes it on to "find_max_quadrant_velocity" method to distribute the velocity vector into respective quadrants
    * OUTPUT: The method then outputs four velocities (quad1/2/3/4_back_vel_cms), which correspond to the final desired backup velocity in each quadrant
    */
    void calc_backup_velocity_2D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &qua2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cm, Vector2f limit_direction, float dt);
    
    /*
    * Compute the back away velocity required to avoid breaching margin, including vertical component
    * min_z_vel is <= 0, and stores the greatest velocity in the downwards direction
    * max_z_vel is >= 0, and stores the greatest velocity in the upwards direction
    * eventually max_z_vel + min_z_vel will give the final desired Z backaway velocity
    */
    void calc_backup_velocity_3D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cms, Vector3f limit_direction, float kp_z, float accel_cmss_z, float back_distance_z, float& min_z_vel, float& max_z_vel, float dt);
   
   /*
    * Calculate maximum velocity vector that can be formed in each quadrant 
    * This method takes the desired backup velocity, and four other velocities corresponding to each quadrant
    * The desired velocity is then fit into one of the 4 quadrant velocities as per the sign of its components
    * This ensures that we have multiple backup velocities, we can get the maximum of all of those velocities in each quadrant
    */
    void find_max_quadrant_velocity(Vector2f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel);

    /*
    * Calculate maximum velocity vector that can be formed in each quadrant and separately store max & min of vertical components
    */
    void find_max_quadrant_velocity_3D(Vector3f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel, float &max_z_vel, float &min_z_vel);

    /*
     * methods for avoidance in non-GPS flight modes
     */

    // convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
    float distance_to_lean_pct(float dist_m);

    // returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
    void get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative);

    // Logging function
    void Write_SimpleAvoidance(const uint8_t state, const Vector3f& desired_vel, const Vector3f& modified_vel, const bool back_up) const;

    // parameters
    AP_Int8 _enabled;
    AP_Int16 _angle_max;        // maximum lean angle to avoid obstacles (only used in non-GPS flight modes)
    AP_Float _dist_max;         // distance (in meters) from object at which obstacle avoidance will begin in non-GPS modes
    AP_Float _margin;           // vehicle will attempt to stay this distance (in meters) from objects while in GPS modes
    AP_Int8 _behavior;          // avoidance behaviour (slide or stop)
    AP_Float _backup_speed_max; // Maximum speed that will be used to back away (in m/s)
    AP_Float _alt_min;          // alt below which Proximity based avoidance is turned off
    AP_Float _accel_max;        // maximum accelration while simple avoidance is active
    AP_Float _backup_deadzone;  // distance beyond AVOID_MARGIN parameter, after which vehicle will backaway from obstacles

    bool _proximity_enabled = true; // true if proximity sensor based avoidance is enabled (used to allow pilot to enable/disable)
    bool _proximity_alt_enabled = true; // true if proximity sensor based avoidance is enabled based on altitude
    uint32_t _last_limit_time;      // the last time a limit was active
    uint32_t _last_log_ms;          // the last time simple avoidance was logged
    Vector3f _prev_avoid_vel;       // copy of avoidance adjusted velocity

    static AC_Avoid *_singleton;
};

namespace AP {
    AC_Avoid *ac_avoid();
};

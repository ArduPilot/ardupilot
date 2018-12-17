#pragma once

/// @file    AP_LQR_Control.h
/// @brief   LQR Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Written by Akshath Singhal 2018
 *
 */

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>

class AP_LQR_Control : public AP_Navigation {
public:
    AP_LQR_Control(const AP_SpdHgtControl *spdHgtControl)
        : _spdHgtControl(spdHgtControl)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_LQR_Control(const AP_LQR_Control &other) = delete;
    AP_LQR_Control &operator=(const AP_LQR_Control&) = delete;

    /* see AP_Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(void) const;
    float lateral_acceleration(void) const;

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const;

    float crosstrack_error(void) const { return _crosstrack_error; }

    int32_t target_bearing_cd(void) const;
    float turn_distance(float wp_radius) const;
    float turn_distance(float wp_radius, float turn_angle) const;
    float loiter_radius(const float loiter_radius) const;
    void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min = 0.0f);
    void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction);
    void update_heading_hold(int32_t navigation_heading_cd);
    void update_level_flight(void);
    bool reached_loiter_target(void);
    
    void set_data_is_stale(void)  {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const {
        return _data_is_stale;
    }

    // this supports the NAVLQR_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) {
        _reverse = reverse;
    }

private:

    // pointer to the SpdHgtControl object
    const AP_SpdHgtControl *_spdHgtControl;

    // lateral acceration in m/s required to fly to the
    // reference point (+ve to right)
    float _latAccDem;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // bearing error angle (radians) +ve to left of track
    float _bearing_error;

    // crosstrack error in meters
    float _crosstrack_error;

    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

    float _nav_bearing;


    /* 
    Trajectory tracking LQR parameters
    */
    
    // maximum permissible cross-track
    AP_Int16 _max_xtrack;
    
    // q2 multiplicative factor
    AP_Int16 _q2_val;

    // q1 multiplicative factor
    AP_Int16 _q1_val;

    uint32_t _last_update_waypoint_us;
    bool _data_is_stale = true;

    AP_Float _loiter_bank_limit;

    bool _reverse = false;
    float get_yaw_rad();
    float get_yaw_sensor_cd();
    float get_gs_angle_cd();
};

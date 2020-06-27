#pragma once

/// @file    AC_WPNav_Heli.h
/// @brief   ArduCopter waypoint navigation library for traditional helicopters

#include "AC_WPNav.h"
#include <AP_L1_Control/AP_L1_Control_Heli.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl_Heli.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library

// default values
#define WPNAV_HELI_MIN_L1_NAV_SPEED                     550.0f
#define WPNAV_HELI_MIN_L1_NAV_CMD_SPEED                 700.0f

class AC_WPNav_Heli : public AC_WPNav {
public:
    AC_WPNav_Heli( const AP_InertialNav& inav,
                   AP_AHRS_View& ahrs,
                   AC_PosControl& pos_control,
                   const AC_AttitudeControl& attitude_control,
                   const AP_L1_Control_Heli& L1_controller,
                   const AP_SpdHgtControl_Heli& helispdhgtctrl,
                   AP_Mission& mission);
    bool use_l1_navigation();

    /// wp_and_spline_init - initialise straight line and spline waypoint controllers
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init() override;

    /// set_wp_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    bool set_wp_destination(const Location_Class& destination) override;

    /// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain
    bool set_wp_destination(const Vector3f& destination, bool terrain_alt = false) override;

    /// set the L1 navigation controller origin and destination
    void set_L1_wp_origin_and_destination(const Location_Class& destination);

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    bool update_l1_wpnav();

    /// advance_wp_target_along_track - move target location along track from origin to destination
    bool advance_l1_wp_target_along_track(float dt);

    /// reached l1 destination
    bool reached_l1_destination() const { return _reached_l1_destination; }

    /// get L1 desired roll, pitch which should be fed into stabilize controllers
    int32_t get_l1_roll() const { return _L1_controller.nav_roll_cd(); }
    int32_t get_l1_pitch() const { return _helispdhgtctrl.get_pitch(); }
    int32_t get_angle_total() { return _L1_controller.get_angle_total(); }

    // get desired yaw rate to coordinate turns
    int32_t get_yaw_rate() const { return _L1_controller.turn_rate_cds(); }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // References to external libraries
    AP_L1_Control_Heli _L1_controller;
    AP_SpdHgtControl_Heli _helispdhgtctrl;
    AP_Mission& _mission;

    Location_Class _prev_WP_loc;
    Location_Class _next_WP_loc;

private:

    
    // parameters

    // L1 controller variables
    AP_Int8     _l1_nav_use;
    AP_Float    _loiter_radius;
    bool        _reached_l1_destination;
    bool        _stopping_at_waypoint;
    uint32_t    _wp_last_l1_update;
};

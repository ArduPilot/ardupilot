/// @file    AP_NPFG.h
/// @brief   Non-Linear path following guidance

#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_Param/AP_Param.h>
#include <AP_TECS/AP_TECS.h>

#include "AP_NPFG/npfg.h"

class AP_NPFG : public  AP_Navigation {
public:
    AP_NPFG(AP_AHRS &ahrs, const AP_TECS *tecs);

    // do not permit copies
    CLASS_NO_COPY(AP_NPFG);

    // return the desired roll angle in centi-degrees to move towards
    // the target waypoint
    int32_t nav_roll_cd(void) const override;

    // return the desired lateral acceleration in m/s/s to move towards
    // the target waypoint
    float lateral_acceleration(void) const override;

    // note: all centi-degree values returned in AP_Navigation should
    // be wrapped at -18000 to 18000 in centi-degrees.

    // return the tracking bearing that the navigation controller is
    // using in centi-degrees. This is used to display an arrow on
    // ground stations showing the effect of the cross-tracking in the
    // controller
    int32_t nav_bearing_cd(void) const override;

    // return the difference between the vehicles current course and
    // the nav_bearing_cd() in centi-degrees. A positive value means
    // the vehicle is tracking too far to the left of the correct
    // bearing.
    int32_t bearing_error_cd(void) const override;

    // return the target bearing in centi-degrees. This is the bearing
    // from the vehicles current position to the target waypoint. This
    // should be calculated in the update_*() functions below.
    int32_t target_bearing_cd(void) const override;

    // return the crosstrack error in meters. This is the distance in
    // the X-Y plane that we are off the desired track
    float crosstrack_error(void) const override;
    float crosstrack_error_integrator(void) const override;

    // return the distance in meters at which a turn should commence
    // to allow the vehicle to neatly move to the next track in the
    // mission when approaching a waypoint. Assumes 90 degree turn
    float turn_distance(float wp_radius) const override;

    // return the distance in meters at which a turn should commence
    // to allow the vehicle to neatly move to the next track in the
    // mission when approaching a waypoint
    float turn_distance(float wp_radius, float turn_angle) const override;

    // return the target loiter radius for the current location that
    // will not cause excessive airframe loading
    float loiter_radius(const float radius) const override;

    // update the internal state of the navigation controller, given
    // the previous and next waypoints. This is the step function for
    // navigation control for path following between two points.  This
    // function is called at regular intervals (typically 10Hz). The
    // main flight code will call an output function (such as
    // nav_roll_cd()) after this function to ask for the new required
    // navigation attitude/steering.
    void update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min = 0.0f) override;

    // update the internal state of the navigation controller for when
    // the vehicle has been commanded to circle about a point.  This
    // is the step function for navigation control for circling.  This
    // function is called at regular intervals (typically 10Hz). The
    // main flight code will call an output function (such as
    // nav_roll_cd()) after this function to ask for the new required
    // navigation attitude/steering.
    void update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) override;

    // update the internal state of the navigation controller, given a
    // fixed heading. This is the step function for navigation control
    // for a fixed heading.  This function is called at regular
    // intervals (typically 10Hz). The main flight code will call an
    // output function (such as nav_roll_cd()) after this function to
    // ask for the new required navigation attitude/steering.
    void update_heading_hold(int32_t navigation_heading_cd) override;

    // update the internal state of the navigation controller for
    // level flight on the current heading. This is the step function
    // for navigation control for level flight.  This function is
    // called at regular intervals (typically 10Hz). The main flight
    // code will call an output function (such as nav_roll_cd()) after
    // this function to ask for the new required navigation
    // attitude/steering.
    void update_level_flight(void) override;

    // update the internal state of the navigation controller when
    // the vehicle has been commanded with a path following setpoint, which
    // includes the closest point on the path, the unit tangent to the path,
    // and the curvature. This is the step function for navigation control when
    // path following. This function is called at regular intervals
    // (typically 10Hz). The main flight code will call an output function
    // (such as nav_roll_cd()) after this function to ask for the new required
    // navigation attitude/steering.
    void update_path(const class Location &position_on_path, Vector2f unit_path_tangent, float path_curvature, int8_t direction) override;

    // return true if we have reached the target loiter location. This
    // may be a fuzzy decision, depending on internal navigation
    // parameters. For example the controller may return true only
    // when on the circular path around the waypoint, and not when
    // tracking towards the center. This function is only valid when
    // the update_loiter() method is used
    bool reached_loiter_target(void) override;

    // notify Navigation controller that a new waypoint has just been
    // processed. This means that until we handle an update_XXX() function
    // the data is stale with old navigation information.
    void set_data_is_stale(void) override;

    // return true if a new waypoint has been processed by mission
    // controller but the navigation controller still has old stale data
    // from previous waypoint navigation handling. This gets cleared on
    // every update_XXXXXX() call.
    bool data_is_stale(void) const override;

    void set_reverse(bool reverse) override;

    // store the NPFG_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    void update_parameters();

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    // pointer to the SpdHgtControl object
    const AP_TECS *_tecs;

    // nonlinear path following guidance implementation
    NPFG _npfg;

    // parameters
    AP_Int8 _npfg_en_period_lb;
    AP_Int8 _npfg_en_period_ub;
    AP_Int8 _npfg_en_track_keeping;
    AP_Int8 _npfg_en_min_gsp;
    AP_Int8 _npfg_en_wind_reg;
    AP_Float _npfg_period;
    AP_Float _npfg_damping;
    AP_Float _npfg_track_keeping_gsp_max;
    AP_Float _npfg_roll_time_const;
    AP_Float _npfg_switch_distance_multiplier;
    AP_Float _npfg_period_safety_factor;

    // AP_Float _param_fw_gnd_spd_min;
    // AP_Float _param_fw_r_lim;
    // AP_Float _param_fw_pn_r_slew_max;

    uint32_t _last_update_ms;
};

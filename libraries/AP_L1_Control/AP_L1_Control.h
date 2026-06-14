#pragma once

/// @file    AP_L1_Control.h
/// @brief   L1 Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by Brandon Jones 2013
 *
 *  Modified by Paul Riseborough 2013 to provide:
 *  - Explicit control over frequency and damping
 *  - Explicit control over track capture angle
 *  - Ability to use loiter radius smaller than L1 length
 */

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_Common/Location.h>
#include <AP_Vehicle/AP_FixedWing.h>

class AP_L1_Control : public AP_Navigation {
public:
    AP_L1_Control(AP_AHRS & ahrs, const AP_TECS *tecs,
                  const AP_FixedWing &aparm, const uint32_t log_bitmask)
        : _ahrs(ahrs)
        , _tecs(tecs)
        , _aparm(aparm)
        , _log_bitmask(log_bitmask)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_L1_Control);

    /* see AP_Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(void) const override;
    float lateral_acceleration(void) const override;

    // Status flags for roll-to-lateral-acceleration gain updates.
    enum log_RLAG_Flags : uint16_t {
        RLAG_OK                 = 0,
        RLAG_DISABLED           = (1U<<0),
        RLAG_FIRST_CALL         = (1U<<1),
        RLAG_BAD_DT             = (1U<<2),
        RLAG_LOW_GS             = (1U<<3),
        RLAG_LOW_DEMAND         = (1U<<4),
        RLAG_MODEL_TOO_SMALL    = (1U<<5),
        RLAG_ROLL_REVERSAL      = (1U<<6),
        RLAG_DIR_MISMATCH       = (1U<<7),
    };

    // return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const override;

    // return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const override;

    float crosstrack_error_m(void) const override { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const override { return _L1_xtrack_i; }

    int32_t target_bearing_cd(void) const override;
    float turn_distance(float wp_radius) const override;
    float turn_distance(float wp_radius, float turn_angle) const override;
    float loiter_radius (const float loiter_radius) const override;
    void update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min = 0.0f) override;
    void update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) override;
    void update_heading_hold(int32_t navigation_heading_cd) override;
    void update_level_flight(void) override;
    bool reached_loiter_target(void) override;

    // set the default NAVL1_PERIOD
    void set_default_period(float period) {
        _L1_period.set_default(period);
    }

    void set_data_is_stale(void) override {
        _data_is_stale = true;
    }
    bool data_is_stale(void) const override {
        return _data_is_stale;
    }

    // this supports the NAVl1_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void set_reverse(bool reverse) override {
        _reverse = reverse;
    }

private:
    // reference to the AHRS object
    AP_AHRS &_ahrs;

    // pointer to the SpdHgtControl object
    const AP_TECS *_tecs;

    // reference to the fixed-wing parameters object
    const AP_FixedWing &_aparm;

    // logging bitmask
    const uint32_t _log_bitmask;

    // lateral acceleration in m/s required to fly to the
    // L1 reference point (+ve to right)
    float _latAccDem;

    // L1 tracking distance in meters which is dynamically updated
    float _L1_dist;

    // Status which is true when the vehicle has started circling the WP
    bool _WPcircle;

    // bearing angle (radians) to L1 point
    float _nav_bearing;

    // bearing error angle (radians) +ve to left of track
    float _bearing_error;

    // crosstrack error in meters
    float _crosstrack_error;

    // target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

    // L1 tracking loop period (sec)
    AP_Float _L1_period;
    // L1 tracking loop damping ratio
    AP_Float _L1_damping;

    // previous value of cross-track velocity
    float _last_Nu;

    // prevent indecision in waypoint tracking
    void _prevent_indecision(float &Nu);

    // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
    // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
    float _L1_xtrack_i = 0;
    AP_Float _L1_xtrack_i_gain;
    float _L1_xtrack_i_gain_prev = 0;
    uint32_t _last_update_waypoint_us;
    bool _data_is_stale = true;

    AP_Float _loiter_bank_limit;

    // Adaptive correction of roll-to-lateral-acceleration effectiveness.
    // Improves turns and circle tracking when the coordinated-turn model is
    // imperfect.
    AP_Float _lat_acc_k_tc;
    float _lat_acc_k = 1.0f;
    float _lat_acc_k_tc_prev;
    uint32_t _last_lat_acc_update_us;
    void _update_lat_acc_gain(const Vector2f &groundspeed);

    // remember reached_loiter_target decision
    struct {
        uint32_t reached_loiter_target_ms;
        float radius;
        int8_t direction;
        Location center_WP;
    } _last_loiter;

    bool _reverse = false;
    float get_yaw() const;
    int32_t get_yaw_sensor() const;
};

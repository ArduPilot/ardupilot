#include "AP_NPFG.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of parameters
const AP_Param::GroupInfo AP_NPFG::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: NPFG control period.
    // @Description: Period in seconds of NPFG controller.
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("PERIOD", 0, AP_NPFG, _npfg_period, 10.0f),

    // @Param: DAMPING
    // @DisplayName: NPFG control damping ratio.
    // @Description: Damping ratio for NPFG controller.
    // @User: Advanced
    AP_GROUPINFO("DAMPING", 1, AP_NPFG, _npfg_damping, 0.707f),

    // @Param: LB_PERIOD
    // @DisplayName: NPFG enable control period lower bound constraints. 
    // @Description: Enable automatic lower bounding of the user set controller period.
    // @User: Advanced
    AP_GROUPINFO("LB_PERIOD", 2, AP_NPFG, _npfg_en_period_lb, 1),

    // @Param: UB_PERIOD
    // @DisplayName: NPFG enable control period upper bound constraints. 
    // @Description: Enable automatic adaptation of the user set controller period for track keeping performance.
    // @User: Advanced
    AP_GROUPINFO("UB_PERIOD", 3, AP_NPFG, _npfg_en_period_ub, 1),

    // @Param: TRACK_KEEP
    // @DisplayName: NPFG enable track keeping.
    // @Description: The airspeed reference is incremented to return to the track and sustain zero ground velocity until excess wind subsides.
    // @User: Advanced
    AP_GROUPINFO("TRACK_KEEP", 4, AP_NPFG, _npfg_en_track_keeping, 0),

    // @Param: EN_MIN_GSP
    // @DisplayName: NPFG enable min ground speed.
    // @Description: The airspeed reference is incremented to sustain a user defined minimum forward ground speed.
    // @User: Advanced
    AP_GROUPINFO("EN_MIN_GSP", 5, AP_NPFG, _npfg_en_min_gsp, 1),

    // @Param: WIND_REG
    // @DisplayName: NPFG enable excess wind regulation.
    // @Description: The airspeed reference is incremented to regulate the excess wind, but not overcome it. Disabling this parameter disables all other excess wind handling options, using only the nominal airspeed for reference.
    // @User: Advanced
    AP_GROUPINFO("WIND_REG", 6, AP_NPFG, _npfg_en_wind_reg, 1),

    // @Param: GSP_MAX_TK
    // @DisplayName: NPFG track keeping max, min.
    // @Description: Maximum, minimum forward ground speed demand from track keeping logic.
    // @User: Advanced
    AP_GROUPINFO("GSP_MAX_TK", 7, AP_NPFG, _npfg_track_keeping_gsp_max, 5.0f),

    // @Param: ROLL_TC
    // @DisplayName: NPFG roll time constant.
    // @Description: Autopilot roll response time constant.
    // @User: Advanced
    AP_GROUPINFO("ROLL_TC", 8, AP_NPFG, _npfg_roll_time_const, 0.0f),

    // @Param: SW_DST_MLT
    // @DisplayName: NPFG switch distance multiplier.
    // @Description: A value multiplied by the track error boundary resulting in a lower switch distance as the bearing angle changes quadratically (instead of linearly as in L1), the time constant (automatically calculated for on track stability) proportional track error boundary typically over estimates the required switching distance.
    // @User: Advanced
    AP_GROUPINFO("SW_DST_MLT", 9, AP_NPFG, _npfg_switch_distance_multiplier, 0.32f),

    // @Param: PERIOD_SF
    // @DisplayName: NPFG period safety factor.
    // @Description: Multiplied by the minimum period for conservative lower bound.
    // @User: Advanced
    AP_GROUPINFO("PERIOD_SF", 10, AP_NPFG, _npfg_period_safety_factor, 1.5f),

    AP_GROUPEND
};

AP_NPFG::AP_NPFG(AP_AHRS &ahrs, const AP_TECS *tecs)
    : _ahrs(ahrs)
    , _tecs(tecs)
{
    update_parameters();
}

int32_t AP_NPFG::nav_roll_cd(void) const {
    return 0;
}

float AP_NPFG::lateral_acceleration(void) const {
    return 0.0;
}

int32_t AP_NPFG::nav_bearing_cd(void) const {
    return 0;
}

int32_t AP_NPFG::bearing_error_cd(void) const {
    return 0;
}

int32_t AP_NPFG::target_bearing_cd(void) const {
    return 0;
}

float AP_NPFG::crosstrack_error(void) const {
    return 0.0;
}

float AP_NPFG::crosstrack_error_integrator(void) const {
    return 0.0;
}

float AP_NPFG::turn_distance(float wp_radius) const {
    return 0.0;
}

float AP_NPFG::turn_distance(float wp_radius, float turn_angle) const {
    return 0.0;
}

float AP_NPFG::loiter_radius(const float radius) const {
    return 0.0;
}

void AP_NPFG::update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min) {
}

void AP_NPFG::update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) {
    // check for parameter updates
    update_parameters();

    //! @todo - obtain current position from _ahrs in local cartesian coords
    Vector2f curr_pos_local;

    //! @todo - obtain ground vel from _ahrs in local cartesian coords
    Vector2f ground_vel;

    //! @todo - obtain wind vel from _ahrs in local cartesian coords
    Vector2f wind_vel;

    //! @todo - unit path tangent is the normalised setpoint velocity
    Vector2f unit_path_tangent;

    //! @todo - the setpoint position in local cartesian coords
    Vector2f position_on_path;

    //! @todo calculate from loiter radius and direction
    //! @todo check loiter radius is not zero or infinite
    float path_curvature = -1.0 * float(loiter_direction) / radius;

    _npfg.guideToPath(curr_pos_local, ground_vel, wind_vel,
        unit_path_tangent, position_on_path, path_curvature);
}

void AP_NPFG::update_heading_hold(int32_t navigation_heading_cd) {
}

void AP_NPFG::update_level_flight(void) {
}

bool AP_NPFG::reached_loiter_target(void) {
    return false;
}

void AP_NPFG::set_data_is_stale(void) {
}

bool AP_NPFG::data_is_stale(void) const {
    return false;
}

void AP_NPFG::set_reverse(bool reverse) {
}

void AP_NPFG::update_parameters() {

    //! @todo populate all parameters

    _npfg.setPeriod(_npfg_period);
    _npfg.setDamping(_npfg_damping);
    _npfg.enablePeriodLB(_npfg_en_period_lb);
    _npfg.enablePeriodUB(_npfg_en_period_ub);
    _npfg.enableMinGroundSpeed(_npfg_en_min_gsp);
    _npfg.enableTrackKeeping(_npfg_en_track_keeping);
    _npfg.enableWindExcessRegulation(_npfg_en_wind_reg);
    // _npfg.setMinGroundSpeed(_param_fw_gnd_spd_min);
    _npfg.setMaxTrackKeepingMinGroundSpeed(_npfg_track_keeping_gsp_max);
    _npfg.setRollTimeConst(_npfg_roll_time_const);
    _npfg.setSwitchDistanceMultiplier(_npfg_switch_distance_multiplier);
    // _npfg.setRollLimit(radians(_param_fw_r_lim));
    // _npfg.setRollSlewRate(radians(_param_fw_pn_r_slew_max));
    _npfg.setPeriodSafetyFactor(_npfg_period_safety_factor);
}

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
    AP_GROUPINFO("TRACK_KEEP", 4, AP_NPFG, _npfg_en_track_keeping, 1),

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

    // // @Param: FW_GND_SPD_MIN
    // // @DisplayName: NPFG forward ground speed minimum (m/s).
    // // @Description: NPFG forward ground speed minimum (m/s).
    // // @User: Advanced
    // AP_GROUPINFO("FW_GND_SPD_MIN", 11, AP_NPFG, _param_fw_gnd_spd_min, 15.0f),

    // // @Param: FW_R_LIM
    // // @DisplayName: NPFG forward roll limit (deg).
    // // @Description: NPFG forward roll limit (deg).
    // // @User: Advanced
    // AP_GROUPINFO("FW_R_LIM", 12, AP_NPFG, _param_fw_r_lim, 30.0f),

    // // @Param: FW_PN_R_SLEW_MAX
    // // @DisplayName: NPFG forward roll slew limit (deg/s).
    // // @Description: NPFG forward roll slew limit (deg/s).
    // // @User: Advanced
    // AP_GROUPINFO("FW_PN_R_SLEW_MAX", 13, AP_NPFG, _param_fw_pn_r_slew_max, 0.0f),

    AP_GROUPEND
};

AP_NPFG::AP_NPFG(AP_AHRS &ahrs, const AP_TECS *tecs)
    : _ahrs(ahrs)
    , _tecs(tecs)
{
    update_parameters();
}

int32_t AP_NPFG::nav_roll_cd(void) const {
    float roll_setpoint_rad = _npfg.getRollSetpoint();
    int32_t roll_setpoint_cd = int32_t(100 * degrees(roll_setpoint_rad));
    return roll_setpoint_cd;
}

float AP_NPFG::lateral_acceleration(void) const {
    return _npfg.getLateralAccel();
}

int32_t AP_NPFG::nav_bearing_cd(void) const {
    float bearing_rad = _npfg.getBearing();
    int32_t bearing_cd = int32_t(100 * degrees(bearing_rad));
    return bearing_cd;
}

int32_t AP_NPFG::bearing_error_cd(void) const {
    // not implemented
    return 0;
}

int32_t AP_NPFG::target_bearing_cd(void) const {
    return 0;
}

float AP_NPFG::crosstrack_error(void) const {
    return _npfg.getTrackError();
}

float AP_NPFG::crosstrack_error_integrator(void) const {
    // not implemented
    return 0.0;
}

float AP_NPFG::turn_distance(float wp_radius) const {
    // not implemented
    return 0.0;
}

float AP_NPFG::turn_distance(float wp_radius, float turn_angle) const {
    // not implemented
    return 0.0;
}

float AP_NPFG::loiter_radius(const float radius) const {
    // not implemented
    return 0.0;
}

void AP_NPFG::update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min) {
    // not implemented
}

void AP_NPFG::update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) {
    // not implemented
}

void AP_NPFG::update_heading_hold(int32_t navigation_heading_cd) {
    // not implemented
}

void AP_NPFG::update_level_flight(void) {
    // not implemented
}

void AP_NPFG::update_path(const class Location &position_on_path, Vector2f unit_path_tangent, float path_curvature, int8_t direction) {
    update_parameters();

    // calculate control interval
    uint32_t now_ms = AP_HAL::millis();
    uint32_t control_interval_ms = now_ms - _last_update_ms;
    _last_update_ms = now_ms;

    const float control_interval_s = float(control_interval_ms) * 1.0E-3;

    // update control time step for slew rates  
    _npfg.setDt(control_interval_s);

    //! @todo(srmainwaring) remove hardcoding
    _npfg.setAirspeedNom(25.0f);
    _npfg.setAirspeedMax(35.0f);

    // get current position and velocity
    Location current_loc;
    if (_ahrs.get_location(current_loc) == false) {
        //! @todo(srmainwaring) if no GPS loc available, maintain last nav/target_bearing
        // _data_is_stale = true;
        // return;
    }

    // current position from _ahrs in local cartesian coords (m)
    Vector2f curr_pos_local_cm;
    if (current_loc.get_vector_xy_from_origin_NE(curr_pos_local_cm) == false) {
        //! @todo(srmainwaring) add error handling
    }
    Vector2f curr_pos_local(0.01 * curr_pos_local_cm.x, 0.01 * curr_pos_local_cm.y);

    // ground vel from _ahrs in local cartesian coords (m/s)
    Vector2f ground_vel = _ahrs.groundspeed_vector();

    // wind estimate from _ahrs in local cartesian coords (m/s)
    Vector3f wind_estimate = _ahrs.wind_estimate();
    Vector2f wind_vel(wind_estimate.x, wind_estimate.y);

    // setpoint position in local cartesian coords (m)
    Vector2f position_on_path_ned_cm;
    if (position_on_path.get_vector_xy_from_origin_NE(position_on_path_ned_cm) == false) {
        //! @todo(srmainwaring) add error handling
    }
    Vector2f position_on_path_ned(0.01 * position_on_path_ned_cm.x, 0.01 * position_on_path_ned_cm.y);

    _npfg.guideToPath(curr_pos_local, ground_vel, wind_vel,
        unit_path_tangent, position_on_path_ned,
        path_curvature * float(direction));
}

bool AP_NPFG::reached_loiter_target(void) {
    // not implemented
    return false;
}

void AP_NPFG::set_data_is_stale(void) {
    // not implemented
}

bool AP_NPFG::data_is_stale(void) const {
    // not implemented
    return false;
}

void AP_NPFG::set_reverse(bool reverse) {
    // not implemented
}

void AP_NPFG::update_parameters() {

    //! @todo populate all parameters

    _npfg.setPeriod(_npfg_period);
    _npfg.setDamping(_npfg_damping);
    _npfg.enablePeriodLB(_npfg_en_period_lb);
    _npfg.enablePeriodUB(_npfg_en_period_ub);
    _npfg.enableMinGroundSpeed(_npfg_en_min_gsp);
    _npfg.enableTrackKeeping(_npfg_en_track_keeping);
    _npfg.enableWindExcessRegulation(false /*_npfg_en_wind_reg*/);
    _npfg.setMinGroundSpeed(0.0 /*_param_fw_gnd_spd_min*/);
    _npfg.setMaxTrackKeepingMinGroundSpeed(_npfg_track_keeping_gsp_max);
    _npfg.setRollTimeConst(_npfg_roll_time_const);
    _npfg.setSwitchDistanceMultiplier(_npfg_switch_distance_multiplier);
    _npfg.setRollLimit(radians(30.0 /*_param_fw_r_lim*/));
    _npfg.setRollSlewRate(radians(90.0 /*_param_fw_pn_r_slew_max*/));
    _npfg.setPeriodSafetyFactor(_npfg_period_safety_factor);
}

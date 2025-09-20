#include "Rover.h"

#if MODE_DOCK_ENABLED

const AP_Param::GroupInfo ModeDock::var_info[] = {
    // @Param: _SPEED
    // @DisplayName: Dock mode speed
    // @Description: Vehicle speed limit in dock mode
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_SPEED", 1, ModeDock, speed, 0.0f),

    // @Param: _DIR
    // @DisplayName: Dock mode direction of approach
    // @Description: Compass direction in which vehicle should approach towards dock. -1 value represents unset parameter
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_DIR", 2, ModeDock, desired_dir, -1.00f),

    // @Param: _HDG_CORR_EN
    // @DisplayName: Dock mode heading correction enable/disable
    // @Description: When enabled, the autopilot modifies the path to approach the target head-on along desired line of approach in dock mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("_HDG_CORR_EN", 3, ModeDock, hdg_corr_enable, 0),

    // @Param: _HDG_CORR_WT
    // @DisplayName: Dock mode heading correction weight
    // @Description: This value describes how aggressively vehicle tries to correct its heading to be on desired line of approach
    // @Range: 0.00 0.90
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_HDG_CORR_WT", 4, ModeDock, hdg_corr_weight, 0.75f),

    // @Param: _STOP_DIST
    // @DisplayName: Distance from docking target when we should stop
    // @Description: The vehicle starts stopping when it is this distance away from docking target
    // @Units: m
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_STOP_DIST", 5, ModeDock, stopping_dist, 0.30f),

    AP_GROUPEND
};

ModeDock::ModeDock(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define AR_DOCK_ACCEL_MAX              20.0    // acceleration used when user has specified no acceleration limit

// initialize dock mode
bool ModeDock::_enter()
{
    // refuse to enter the mode if dock is not in sight
    if (!rover.precland.enabled() || !rover.precland.target_acquired()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Dock: target not acquired");
        return false;
    }

    if (hdg_corr_enable && is_negative(desired_dir)) {
        // DOCK_DIR is required for heading correction
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Dock: Set DOCK_DIR or disable heading correction");
        return false;
    }

    // set speed limit to dock_speed param if available
    // otherwise the vehicle uses wp_nav default speed limit
    const float speed_max = is_positive(speed) ? speed : g2.wp_nav.get_default_speed();
    float atc_accel_max = MIN(g2.attitude_control.get_accel_max(), g2.attitude_control.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        // accel_max of zero means no limit so use maximum acceleration
        atc_accel_max = AR_DOCK_ACCEL_MAX;
    }
    const float accel_max = is_positive(g2.wp_nav.get_default_accel()) ? MIN(g2.wp_nav.get_default_accel(), atc_accel_max) : atc_accel_max;
    const float jerk_max = is_positive(g2.wp_nav.get_default_jerk()) ? g2.wp_nav.get_default_jerk() : accel_max;

    // initialise position controller
    g2.pos_control.set_limits(speed_max, accel_max, g2.attitude_control.get_turn_lat_accel_max(), jerk_max);
    g2.pos_control.init();

    // set the position controller reversed in case the camera is mounted on vehicle's back
    g2.pos_control.set_reversed(rover.precland.get_orient() == 4);

    // construct unit vector in the desired direction from where we want the vehicle to approach the dock
    // this helps to dock the vehicle head-on even when we enter the dock mode at an angle towards the dock
    _desired_heading_NE = Vector2f{cosf(radians(desired_dir)), sinf(radians(desired_dir))};

    _docking_complete = false;

    return true;
}

void ModeDock::update()
{
    // if docking is complete, rovers stop and boats loiter
    if (_docking_complete) {
        // rovers stop, boats loiter 
        // note that loiter update must be called after successful initialisation on mode loiter
        if (_loitering) {
            // mode loiter must be initialised before calling update method
            rover.mode_loiter.update();
        } else {
            stop_vehicle();
        }
        return;
    }

    const bool real_dock_in_sight = rover.precland.get_target_position_m(_dock_pos_rel_origin_m);
    Vector2f dock_pos_rel_vehicle_m;
    if (!calc_dock_pos_rel_vehicle_NE_m(dock_pos_rel_vehicle_m)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    _distance_to_destination = dock_pos_rel_vehicle_m.length();

    // we force the vehicle to use real dock as target when we are too close to the dock
    // note that heading correction does not work when we force real target
    const bool force_real_target = _distance_to_destination < _force_real_target_limit_m;

    // if we are close enough to the dock or the target is not in sight when we strictly require it
    // we mark the docking to be completed so that the vehicle stops
    if (_distance_to_destination <= stopping_dist || (force_real_target && !real_dock_in_sight)) {
        _docking_complete = true;

        // send a one time notification to GCS
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Dock: Docking complete");

        // initialise mode loiter if it is a boat
        if (rover.is_boat()) {
            // if we fail to enter, we set _loitering to false
            _loitering = rover.mode_loiter.enter();
        }
        return;
    }

    Vector2f target_m = _dock_pos_rel_origin_m.tofloat();

    // ***** HEADING CORRECTION *****
    // to make to vehicle dock from a given direction we simulate a virtual moving target on the line of approach
    // this target is always at DOCK_HDG_COR_WT times the distance from dock to vehicle (along the line of approach)
    // For e.g., if the dock is 100 m away form dock, DOCK_HDG_COR_WT is 0.75
    // then the position target is 75 m from the dock, i.e., 25 m from the vehicle
    // as the vehicle tries to reach this target, this target appears to move towards the dock and at last it is sandwiched b/w dock and vehicle
    // since this target is moving along desired direction of approach, the vehicle also comes on that line while following it
    if  (!force_real_target && hdg_corr_enable) {
        const float correction_vec_mag = hdg_corr_weight * dock_pos_rel_vehicle_m.projected(_desired_heading_NE).length();
        target_m = _dock_pos_rel_origin_m.tofloat() - _desired_heading_NE * correction_vec_mag;
    }

    const Vector2p target_pos { target_m.topostype() };
    g2.pos_control.input_pos_target(target_pos, rover.G_Dt);
    g2.pos_control.update(rover.G_Dt);

    // get desired speed and turn rate from pos_control
    float desired_speed = g2.pos_control.get_desired_speed();
    float desired_turn_rate = g2.pos_control.get_desired_turn_rate_rads();

    // slow down the vehicle as we approach the dock
    desired_speed = apply_slowdown(desired_speed);

    // run steering and throttle controllers
    calc_steering_from_turn_rate(desired_turn_rate);
    calc_throttle(desired_speed, true);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: DOCK
// @Description: Dock mode target information
// @Field: TimeUS: Time since system startup
// @Field: DockX: Docking Station position, X-Axis
// @Field: DockY: Docking Station position, Y-Axis
// @Field: DockDist: Distance to docking station
// @Field: TPosX: Current Position Target, X-Axis
// @Field: TPosY: Current Position Target, Y-Axis
// @Field: DSpd: Desired speed
// @Field: DTrnRt: Desired Turn Rate

        AP::logger().WriteStreaming(
            "DOCK",
            "TimeUS,DockX,DockY,DockDist,TPosX,TPosY,DSpd,DTrnRt",
            "smmmmmnE",
            "F0000000",
            "Qfffffff",
            AP_HAL::micros64(),
            (float)_dock_pos_rel_origin_m.x,
            (float)_dock_pos_rel_origin_m.y,
            _distance_to_destination,
            target_m.x,
            target_m.y,
            desired_speed,
            desired_turn_rate);
#endif
}

float ModeDock::apply_slowdown(float desired_speed)
{
    const float dock_speed_slowdown_lmt = 0.5f;

    // no slowdown for speed below dock_speed_slowdown_lmt
    if (fabsf(desired_speed) < dock_speed_slowdown_lmt) {
        return desired_speed;
    }

    Vector3f target_vec_rel_vehicle_ned_m;
    if(!calc_dock_pos_rel_vehicle_NE_m(target_vec_rel_vehicle_ned_m.xy())) {
        return desired_speed;
    }

    const Matrix3f &body_to_ned = AP::ahrs().get_rotation_body_to_ned();
    Vector3f target_vec_body = body_to_ned.mul_transpose(target_vec_rel_vehicle_ned_m);
    const float target_error_m = fabsf(target_vec_body.y);
    float error_ratio = target_error_m / _acceptable_pos_error_m;
    error_ratio = constrain_float(error_ratio, 0.0f, 1.0f);

    const float dock_slow_dist_max_m = 15.0f;
    const float dock_slow_dist_min_m = 5.0f;
    // approach slowdown is not applied when the vehicle is more than dock_slow_dist_max meters away
    // within dock_slow_dist_max and dock_slow_dist_min the weight of the slowdown increases linearly
    // once the vehicle reaches dock_slow_dist_min the slowdown weight becomes 1
    float slowdown_weight = 1 - (target_vec_body.x * 0.01f - dock_slow_dist_min_m) / (dock_slow_dist_max_m - dock_slow_dist_min_m);
    slowdown_weight = constrain_float(slowdown_weight, 0.0f, 1.0f);
    
    desired_speed = MAX(dock_speed_slowdown_lmt, fabsf(desired_speed) * (1 - error_ratio * slowdown_weight));

    // restrict speed to avoid going beyond stopping distance
    desired_speed = MIN(desired_speed, safe_sqrt(2 * fabsf(_distance_to_destination - stopping_dist) * g2.pos_control.get_accel_max()));

    // we worked on absolute value of speed before
    // make it negative again if reversed
    if (g2.pos_control.get_reversed()) {
        desired_speed *= -1;
    }

    return desired_speed;
}

// calculate position of dock relative to the vehicle
// we need this method here because there can be a window during heading correction when we might lose the target
// during that window precland won't be able to give us this vector
// we can calculate it based on most recent value from precland because the dock is assumed stationary wrt ekf origin
bool ModeDock::calc_dock_pos_rel_vehicle_NE_m(Vector2f &dock_pos_rel_vehicle_m) const {
    Vector2f current_pos_m;
    if (!AP::ahrs().get_relative_position_NE_origin_float(current_pos_m)) {
        return false;
    }
 
    dock_pos_rel_vehicle_m = _dock_pos_rel_origin_m.tofloat() - current_pos_m;
    return true;
}
#endif // MODE_DOCK_ENABLED

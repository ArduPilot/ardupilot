#include "Copter.h"

#if MODE_ZIGZAG_ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_M 3.0
#define ZIGZAG_LINE_INFINITY -1

const AP_Param::GroupInfo ModeZigZag::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: ZigZag auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeZigZag, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if HAL_SPRAYER_ENABLED
    // @Param: SPRAYER
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPRAYER", 2, ModeZigZag, _spray_enabled, 0),
#endif // HAL_SPRAYER_ENABLED

    // @Param: WP_DELAY
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("WP_DELAY", 3, ModeZigZag, _wp_delay_s, 0),

    // @Param: SIDE_DIST
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("SIDE_DIST", 4, ModeZigZag, _side_dist_m, 4),

    // @Param: DIRECTION
    // @DisplayName: Sideways direction in ZigZag auto
    // @Description: The direction to move sideways in ZigZag mode
    // @Values: 0:forward, 1:right, 2:backward, 3:left
    // @User: Advanced
    AP_GROUPINFO("DIRECTION", 5, ModeZigZag, _direction, 0),

    // @Param: LINE_NUM
    // @DisplayName: Total number of lines
    // @Description: Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways
    // @Range: -1 32767
    // @User: Advanced
    AP_GROUPINFO("LINE_NUM", 6, ModeZigZag, _line_num, 0),

    AP_GROUPEND
};

ModeZigZag::ModeZigZag(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise zigzag controller
bool ModeZigZag::init(bool ignore_checks)
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // initialise waypoint state
    stage = STORING_POINTS;
    dest_A_ne_m.zero();
    dest_B_ne_m.zero();

    // initialize zigzag auto
    init_auto();

    return true;
}

// perform cleanup required when leaving zigzag mode
void ModeZigZag::exit()
{
    // The sprayer will stop if the flight mode is changed from ZigZag to other
    spray(false);
}

// run the zigzag controller
// should be called at 100hz or more
void ModeZigZag::run()
{
    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // set the direction and the total number of lines
    zigzag_direction = (Direction)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, INT16_MAX);

    // auto control
    if (stage == AUTO) {
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // vehicle should be under manual control when disarmed or landed
            return_to_manual_control(false);
        } else if (reached_destination()) {
            // if vehicle has reached destination switch to manual control or moving to A or B
            AP_Notify::events.waypoint_complete = 1;
            if (is_auto) {
                if (line_num == ZIGZAG_LINE_INFINITY || line_count < line_num) {
                    if (auto_stage == AutoState::SIDEWAYS) {
                        save_or_move_to_destination((ab_dest_stored == Destination::A) ? Destination::B : Destination::A);
                    } else {
                        // spray off
                        spray(false);
                        move_to_side();
                    }
                } else {
                    init_auto();
                    return_to_manual_control(true);
                }
            } else {
                return_to_manual_control(true);
            }
        } else {
            auto_control();
        }
    }

    // manual control
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        manual_control();
    }
}

// save current position as A or B.  If both A and B have been saved move to the one specified
void ModeZigZag::save_or_move_to_destination(Destination ab_dest)
{
    // get current position as an offset from EKF origin
    const Vector2p curr_pos_ned_m = pos_control->get_pos_desired_NED_m().xy();

    // handle state machine changes
    switch (stage) {

        case STORING_POINTS:
            if (ab_dest == Destination::A) {
                // store point A
                dest_A_ne_m = curr_pos_ned_m;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point A stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_A);
            } else {
                // store point B
                dest_B_ne_m = curr_pos_ned_m;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point B stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_B);
            }
            // if both A and B have been stored advance state
            if (!dest_A_ne_m.is_zero() && !dest_B_ne_m.is_zero() && !is_zero((dest_B_ne_m - dest_A_ne_m).length_squared())) {
                stage = MANUAL_REGAIN;
                spray(false);
            } else if (!dest_A_ne_m.is_zero() || !dest_B_ne_m.is_zero()) {
                // if only A or B have been stored, spray on
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            Vector3p next_dest_ned_m;
            bool is_terrain_alt;
            if (calculate_next_dest_m(ab_dest, stage == AUTO, next_dest_ned_m, is_terrain_alt)) {
                wp_nav->wp_and_spline_init_m();
                if (wp_nav->set_wp_destination_NED_m(next_dest_ned_m, is_terrain_alt)) {
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;
                    ab_dest_stored = ab_dest;
                    // spray on while moving to A or B
                    spray(true);
                    reach_wp_time_ms = 0;
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), (ab_dest == Destination::A) ? "A" : "B");
                    } else {
                        line_count++;
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s (line %d/%d)", name(), (ab_dest == Destination::A) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

void ModeZigZag::move_to_side()
{
    if (!dest_A_ne_m.is_zero() && !dest_B_ne_m.is_zero() && !is_zero((dest_B_ne_m - dest_A_ne_m).length_squared())) {
        Vector3p next_dest_ned_m;
        bool is_terrain_alt;
        if (calculate_side_dest_m(next_dest_ned_m, is_terrain_alt)) {
            wp_nav->wp_and_spline_init_m();
            if (wp_nav->set_wp_destination_NED_m(next_dest_ned_m, is_terrain_alt)) {
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;
                current_dest_ned_m = next_dest_ned_m;
                current_is_terr_alt = is_terrain_alt;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    }
}

// return manual control to the pilot
void ModeZigZag::return_to_manual_control(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        spray(false);
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3p& wp_dest_ned_m = wp_nav->get_wp_destination_NED_m();
            loiter_nav->init_target_m(wp_dest_ned_m.xy());
#if AP_RANGEFINDER_ENABLED
            if (copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy()) {
                copter.surface_tracking.external_init();
            }
#endif
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: manual control", name());
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A_ne_m or dest_B_ne_m
void ModeZigZag::auto_control()
{
    // process pilot's yaw input
    const float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(wp_nav->get_roll_rad(), wp_nav->get_pitch_rad(), target_yaw_rate_rads);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

// manual_control - process manual control
void ModeZigZag::manual_control()
{
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_ms = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    float target_roll_rad, target_pitch_rad;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // get pilot's desired yaw rate
    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // get pilot desired climb rate
    target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    // make sure the climb rate is in the given range, prevent floating point errors
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    // althold state machine
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(loiter_nav->get_roll_rad(), loiter_nav->get_pitch_rad(), target_yaw_rate_rads);
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01, 0.0, 10.0));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(loiter_nav->get_roll_rad(), loiter_nav->get_pitch_rad(), target_yaw_rate_rads);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads);
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(loiter_nav->get_roll_rad(), loiter_nav->get_pitch_rad(), target_yaw_rate_rads);

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();
}

// return true if vehicle is within a small area around the destination
bool ModeZigZag::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination_m() > ZIGZAG_WP_RADIUS_M) {
        return false;
    }

    // wait at time which is set in zigzag_wp_delay
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) >= (uint16_t)constrain_int16(_wp_delay_s, 0, 127) * 1000);
}

// calculate next destination according to vector A-B and current position
// use_wpnav_alt should be true if waypoint controller's altitude target should be used, false for position control or current altitude target
// is_terrain_alt is returned as true if the next_dest_ned_m.z is relative to the terrain surface
bool ModeZigZag::calculate_next_dest_m(Destination ab_dest, bool use_wpnav_alt, Vector3p& next_dest_ned_m, bool& is_terrain_alt) const
{
    // define start_pos_ne_m as either destination A or B
    Vector2p start_pos_ne_m = (ab_dest == Destination::A) ? dest_A_ne_m : dest_B_ne_m;

    // calculate vector from A to B
    Vector2f AB_diff_ne_m = (dest_B_ne_m - dest_A_ne_m).tofloat();

    // check distance between A and B
    if (is_zero(AB_diff_ne_m.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos_ne_m
    const Vector2p curr_pos_ne_m = pos_control->get_pos_desired_NED_m().xy();
    Vector2p veh_to_start_pos_ne_m = (curr_pos_ne_m - start_pos_ne_m);

    // lengthen AB_diff_ne_m so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    float scalar = 1.0f;
    if (veh_to_start_pos_ne_m.length_squared() > AB_diff_ne_m.length_squared()) {
        scalar = veh_to_start_pos_ne_m.length() / AB_diff_ne_m.length();
    }

    // create a line perpendicular to AB but originating at start_pos_ne_m
    Vector2p perp1 = start_pos_ne_m + Vector2p(-AB_diff_ne_m[1] * scalar, AB_diff_ne_m[0] * scalar);
    Vector2p perp2 = start_pos_ne_m + Vector2p(AB_diff_ne_m[1] * scalar, -AB_diff_ne_m[0] * scalar);

    // find the closest point on the perpendicular line
    const Vector2p closest2d_ne_m = Vector2p::closest_point(curr_pos_ne_m, perp1, perp2);
    next_dest_ned_m.x = closest2d_ne_m.x;
    next_dest_ned_m.y = closest2d_ne_m.y;

    if (use_wpnav_alt) {
        // get altitude target from waypoint controller
        is_terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest_ned_m.z = wp_nav->get_wp_destination_NED_m().z;
    } else {
        is_terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
        next_dest_ned_m.z = pos_control->get_pos_desired_NED_m().z;
        if (!is_terrain_alt) {
            next_dest_ned_m.z += pos_control->get_pos_terrain_D_m();
        }
    }

    return true;
}

// calculate side destination according to vertical vector A-B and current position
// is_terrain_alt is returned as true if the next_dest_ned_m.z is relative to the terrain surfaces
bool ModeZigZag::calculate_side_dest_m(Vector3p& next_dest_ned_m, bool& is_terrain_alt) const
{
    // calculate vector from A to B
    Vector2f AB_diff_ne_m = (dest_B_ne_m - dest_A_ne_m).tofloat();

    // calculate a vertical right or left vector for AB from the current yaw direction
    Vector2f AB_side_ne_m;
    if (zigzag_direction == Direction::RIGHT || zigzag_direction == Direction::LEFT) {
        float yaw_ab_sign = (-ahrs.sin_yaw() * AB_diff_ne_m[1]) + (ahrs.cos_yaw() * -AB_diff_ne_m[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::RIGHT ? 1 : -1))) {
            AB_side_ne_m = Vector2f(AB_diff_ne_m[1], -AB_diff_ne_m[0]);
        } else {
            AB_side_ne_m = Vector2f(-AB_diff_ne_m[1], AB_diff_ne_m[0]);
        }
    } else {
        float yaw_ab_sign = (ahrs.cos_yaw() * AB_diff_ne_m[1]) + (ahrs.sin_yaw() * -AB_diff_ne_m[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::FORWARD ? 1 : -1))) {
            AB_side_ne_m = Vector2f(AB_diff_ne_m[1], -AB_diff_ne_m[0]);
        } else {
            AB_side_ne_m = Vector2f(-AB_diff_ne_m[1], AB_diff_ne_m[0]);
        }
    }

    // check distance the vertical vector between A and B
    float AB_side_ne_m_length = AB_side_ne_m.length();
    if (is_zero(AB_side_ne_m_length)) {
        return false;
    }

    // adjust AB_side_ne_m length to zigzag_side_dist
    float scalar = constrain_float(_side_dist_m, 0.1, 100.0) / AB_side_ne_m_length;

    // get distance from vehicle to start_pos_ne_m
    const Vector3p curr_pos_ned_m = pos_control->get_pos_desired_NED_m();
    next_dest_ned_m.xy() = curr_pos_ned_m.xy() + (AB_side_ne_m.topostype() * scalar);

    // if we have a downward facing range finder then use terrain altitude targets
    is_terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    next_dest_ned_m.z = curr_pos_ned_m.z;

    return true;
}

// run zigzag auto feature which is automate both AB and sideways
void ModeZigZag::run_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    // make sure both A and B point are registered and not when moving to A or B
    if (stage != MANUAL_REGAIN) {
        return;
    }

    is_auto = true;
    // resume if zigzag auto is suspended
    if (is_suspended && line_count <= line_num) {
        // resume the stage when it was suspended
        if (auto_stage == AutoState::AB_MOVING) {
            line_count--;
            save_or_move_to_destination(ab_dest_stored);
        } else if (auto_stage == AutoState::SIDEWAYS) {
            wp_nav->wp_and_spline_init_m();
            if (wp_nav->set_wp_destination_NED_m(current_dest_ned_m, current_is_terr_alt)) {
                stage = AUTO;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    } else {
        move_to_side();
    }
}

// suspend zigzag auto
void ModeZigZag::suspend_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    if (auto_stage != AutoState::MANUAL) {
        is_suspended = true;
        return_to_manual_control(true);
    }
}

// initialize zigzag auto
void ModeZigZag::init_auto()
{
    is_auto = false;
    auto_stage = AutoState::MANUAL;
    line_count = 0;
    is_suspended = false;
}

// spray on / off
void ModeZigZag::spray(bool b)
{
#if HAL_SPRAYER_ENABLED
    if (_spray_enabled) {
        copter.sprayer.run(b);
    }
#endif
}

float ModeZigZag::wp_distance_m() const
{
    return is_auto ? wp_nav->get_wp_distance_to_destination_m() : 0.0f;
}
float ModeZigZag::wp_bearing_deg() const
{
    return is_auto ? degrees(wp_nav->get_wp_bearing_to_destination_rad()) : 0;
}
float ModeZigZag::crosstrack_error_m() const
{
    return is_auto ? wp_nav->crosstrack_error_m() : 0;
}

#endif // MODE_ZIGZAG_ENABLED

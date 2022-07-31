#include "Copter.h"

#if MODE_ZIGZAG_ENABLED == ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_CM 300
#define ZIGZAG_LINE_INFINITY -1

const AP_Param::GroupInfo ModeZigZag::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: ZigZag auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeZigZag, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if SPRAYER_ENABLED == ENABLED
    // @Param: SPRAYER
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPRAYER", 2, ModeZigZag, _spray_enabled, 0),
#endif // SPRAYER_ENABLED == ENABLED

    // @Param: WP_DELAY
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("WP_DELAY", 3, ModeZigZag, _wp_delay, 0),

    // @Param: SIDE_DIST
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("SIDE_DIST", 4, ModeZigZag, _side_dist, 4),

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
    if (!copter.failsafe.radio) {
        // apply simple mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise waypoint state
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();

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
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // set the direction and the total number of lines
    zigzag_direction = (Direction)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, 32767);

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
    const Vector2f curr_pos {inertial_nav.get_position_xy_cm()};

    // handle state machine changes
    switch (stage) {

        case STORING_POINTS:
            if (ab_dest == Destination::A) {
                // store point A
                dest_A = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point A stored");
                AP::logger().Write_Event(LogEvent::ZIGZAG_STORE_A);
            } else {
                // store point B
                dest_B = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point B stored");
                AP::logger().Write_Event(LogEvent::ZIGZAG_STORE_B);
            }
            // if both A and B have been stored advance state
            if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
                stage = MANUAL_REGAIN;
                spray(false);
            } else if (!dest_A.is_zero() || !dest_B.is_zero()) {
                // if only A or B have been stored, spray on
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            Vector3f next_dest;
            bool terr_alt;
            if (calculate_next_dest(ab_dest, stage == AUTO, next_dest, terr_alt)) {
                wp_nav->wp_and_spline_init();
                if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;
                    ab_dest_stored = ab_dest;
                    // spray on while moving to A or B
                    spray(true);
                    reach_wp_time_ms = 0;
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", (ab_dest == Destination::A) ? "A" : "B");
                    } else {
                        line_count++;
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s (line %d/%d)", (ab_dest == Destination::A) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

void ModeZigZag::move_to_side()
{
    if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
        Vector3f next_dest;
        bool terr_alt;
        if (calculate_side_dest(next_dest, terr_alt)) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", dir[(uint8_t)zigzag_direction]);
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
            const Vector3f& wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest.xy());
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: manual control");
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
void ModeZigZag::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

// manual_control - process manual control
void ModeZigZag::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // althold state machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

// return true if vehicle is within a small area around the destination
bool ModeZigZag::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > ZIGZAG_WP_RADIUS_CM) {
        return false;
    }

    // wait at time which is set in zigzag_wp_delay
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) >= (uint16_t)constrain_int16(_wp_delay, 0, 127) * 1000);
}

// calculate next destination according to vector A-B and current position
// use_wpnav_alt should be true if waypoint controller's altitude target should be used, false for position control or current altitude target
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZag::calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const
{
    // define start_pos as either destination A or B
    Vector2f start_pos = (ab_dest == Destination::A) ? dest_A : dest_B;

    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    if (is_zero(AB_diff.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    Vector2f veh_to_start_pos = curr_pos2d - start_pos;

    // lengthen AB_diff so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff.length_squared()) {
        scalar = veh_to_start_pos.length() / AB_diff.length();
    }

    // create a line perpendicular to AB but originating at start_pos
    Vector2f perp1 = start_pos + Vector2f(-AB_diff[1] * scalar, AB_diff[0] * scalar);
    Vector2f perp2 = start_pos + Vector2f(AB_diff[1] * scalar, -AB_diff[0] * scalar);

    // find the closest point on the perpendicular line
    const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
    next_dest.x = closest2d.x;
    next_dest.y = closest2d.y;

    if (use_wpnav_alt) {
        // get altitude target from waypoint controller
        terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest.z = wp_nav->get_wp_destination().z;
    } else {
        // if we have a downward facing range finder then use terrain altitude targets
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
        if (terrain_alt) {
            if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
                next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
            }
        } else {
            next_dest.z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
        }
    }

    return true;
}

// calculate side destination according to vertical vector A-B and current position
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZag::calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const
{
    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // calculate a vertical right or left vector for AB from the current yaw direction
    Vector2f AB_side;
    if (zigzag_direction == Direction::RIGHT || zigzag_direction == Direction::LEFT) {
        float yaw_ab_sign = (-ahrs.sin_yaw() * AB_diff[1]) + (ahrs.cos_yaw() * -AB_diff[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::RIGHT ? 1 : -1))) {
            AB_side = Vector2f(AB_diff[1], -AB_diff[0]);
        } else {
            AB_side = Vector2f(-AB_diff[1], AB_diff[0]);
        }
    } else {
        float yaw_ab_sign = (ahrs.cos_yaw() * AB_diff[1]) + (ahrs.sin_yaw() * -AB_diff[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::FORWARD ? 1 : -1))) {
            AB_side = Vector2f(AB_diff[1], -AB_diff[0]);
        } else {
            AB_side = Vector2f(-AB_diff[1], AB_diff[0]);
        }
    }

    // check distance the vertical vector between A and B
    if (is_zero(AB_side.length_squared())) {
        return false;
    }

    // adjust AB_side length to zigzag_side_dist
    float scalar = constrain_float(_side_dist, 0.1f, 100.0f) * 100 / safe_sqrt(AB_side.length_squared());

    // get distance from vehicle to start_pos
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    next_dest.x = curr_pos2d.x + (AB_side.x * scalar);
    next_dest.y = curr_pos2d.y + (AB_side.y * scalar);

    // if we have a downward facing range finder then use terrain altitude targets
    terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    if (terrain_alt) {
        if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
            next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
        }
    } else {
        next_dest.z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
    }

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
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(current_dest, current_terr_alt)) {
                stage = AUTO;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", dir[(uint8_t)zigzag_direction]);
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
#if SPRAYER_ENABLED == ENABLED
    if (_spray_enabled) {
        copter.sprayer.run(b);
    }
#endif
}

#endif // MODE_ZIGZAG_ENABLED == ENABLED

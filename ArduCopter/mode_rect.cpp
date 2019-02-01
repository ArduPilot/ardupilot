#include "Copter.h"

#if MODE_RECT_ENABLED == ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_CM 300 // Ronny später umbenennen

// initialise rect controller
bool Copter::ModeRect::init(bool ignore_checks)
{
    if (!copter.position_ok() && !ignore_checks) { 
        return false;
    }

    // initialize's loiter position and velocity on xy-axes from current pos and velocity
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise position_z and desired velocity_z
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise waypoint state
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();

    return true;
}

// run the rect controller
// should be called at 100hz or more
void Copter::ModeRect::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // auto control
    if (stage == AUTO) {
        // if vehicle has reached destination switch to manual control
        if (reached_destination()) {
            AP_Notify::events.waypoint_complete = 1;
            stage = MANUAL_REGAIN;
            loiter_nav->init_target(wp_nav->get_wp_destination());
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

// save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
void Copter::ModeRect::save_or_move_to_destination(uint8_t dest_num)
{
    // sanity check
    if (dest_num > 1) {
        return;
    }

    // get current position as an offset from EKF origin
    const Vector3f curr_pos = inertial_nav.get_position();

    // handle state machine changes
    switch (stage) {

        case STORING_POINTS:
            if (dest_num == 0) {
                // store point A
                dest_A.x = curr_pos.x;
                dest_A.y = curr_pos.y;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point A stored");
                copter.Log_Write_Event(DATA_ZIGZAG_STORE_A);
            } else {
                // store point B
                dest_B.x = curr_pos.x;
                dest_B.y = curr_pos.y;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point B stored");
                copter.Log_Write_Event(DATA_ZIGZAG_STORE_B);
            }
            // if both A and B have been stored advance state
            if (!dest_A.is_zero() && !dest_B.is_zero() && is_positive((dest_B - dest_A).length_squared())) {
                stage = MANUAL_REGAIN;
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            Vector3f next_dest;
            if (calculate_next_dest(dest_num, next_dest)) {
                // initialise waypoint controller
                wp_nav->wp_and_spline_init();
                if (wp_nav->set_wp_destination(next_dest, false)) {
                    stage = AUTO;
                    reach_wp_time_ms = 0;
                    if (dest_num == 0) {
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to A");
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to B");
                    }
                }
            }
            break;
    }
}

// return manual control to the pilot
void Copter::ModeRect::return_to_manual_control()
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: manual control");
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
void Copter::ModeRect::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wp_nav should have already updated its alt target)
    pos_control->update_z_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);        
}

// manual_control - process manual control
void Copter::ModeRect::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run loiter controller
    loiter_nav->update();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

    // adjust climb rate using rangefinder
    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

    // adjusts target up or down using a climb rate
    pos_control->update_z_controller();
}

// return true if vehicle is within a small area around the destination
bool Copter::ModeRect::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > ZIGZAG_WP_RADIUS_CM) {
        return false;
    }

    // wait at least one second
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) > 1000);
}

// calculate next destination according to vector A-B and current position
bool Copter::ModeRect::calculate_next_dest(uint8_t dest_num, Vector3f& next_dest) const
{
    // sanity check dest_num
    if (dest_num > 1) {
        return false;
    }

    // define start_pos as either A or B depending upon dest_num
    Vector2f start_pos = dest_num == 0 ? dest_A : dest_B;

    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    if (!is_positive(AB_diff.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos
    const Vector3f curr_pos = inertial_nav.get_position();
    const Vector2f curr_pos2d = Vector2f(curr_pos.x, curr_pos.y);
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
    next_dest.z = pos_control->is_active_z() ? pos_control->get_alt_target() : curr_pos.z;

    return true;
}

#endif // MODE_RECT_ENABLED == ENABLED

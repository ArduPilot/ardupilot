#include "Rover.h"

/*
    check for triggering of start of auto mode
*/
bool Rover::auto_check_trigger(void) {
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AUTO triggered off");
        auto_triggered = false;
        return false;
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }

    if (g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Triggered AUTO with pin");
        auto_triggered = true;
        return true;
    }

    if (!is_zero(g.auto_kickstart)) {
        const float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Triggered AUTO xaccel=%.1f", static_cast<double>(xaccel));
            auto_triggered = true;
            return true;
        }
    }

    return false;
}

/*
    work out if we are going to use pivot steering
*/
bool Rover::use_pivot_steering(void)
{
    // check cases where we clearly cannot use pivot steering
    if (control_mode < AUTO || !g2.motors.have_skid_steering() || g.pivot_turn_angle <= 0) {
        pivot_steering_active = false;
        return false;
    }

    // calc bearing error
    const int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;

    // if error is larger than pivot_turn_angle start pivot steering
    if (bearing_error > g.pivot_turn_angle) {
        pivot_steering_active = true;
        return true;
    }

    // if within 10 degrees of the target heading, exit pivot steering
    if (bearing_error < 10) {
        pivot_steering_active = false;
        return false;
    }

    // by default stay in
    return pivot_steering_active;
}

/*
  test if we are loitering AND should be stopped at a waypoint
*/
bool Rover::in_stationary_loiter()
{
    // Confirm we are in AUTO mode and need to loiter for a time period
    if ((loiter_start_time > 0) && (control_mode == AUTO)) {
        // Check if active loiter is enabled AND we are outside the waypoint loiter radius
        // then the vehicle still needs to move so return false
        if (active_loiter && (wp_distance > g.waypoint_radius)) {
            return false;
        }
        return true;
    }

    return false;
}

/*
    calculate the throtte for auto-throttle modes
*/
void Rover::calc_throttle(float target_speed) {
    // If not autostarting OR we are loitering at a waypoint
    // then set the throttle to minimum
    if (!auto_check_trigger() || in_stationary_loiter()) {
        g2.motors.set_throttle(g.throttle_min.get());
        // Stop rotation in case of loitering and skid steering
        if (g2.motors.have_skid_steering()) {
            g2.motors.set_steering(0.0f);
        }
        return;
    }

    const float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    const int throttle_target = throttle_base + throttle_nudge;

    /*
        reduce target speed in proportion to turning rate, up to the
        SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0f, 1.0f);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    const int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    const float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0.0f, 1.0f);
    const float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio * 0.01f;

    float reduction = 1.0f - steer_rate * speed_turn_reduction;

    if (control_mode >= AUTO && guided_mode != Guided_Velocity && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        const float reduction2 = 1.0f - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }

    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed;

    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100.0f) / 100.0f);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        g2.motors.set_throttle(constrain_int16(-throttle, -g.throttle_max, -g.throttle_min));
    } else {
        g2.motors.set_throttle(constrain_int16(throttle, g.throttle_min, g.throttle_max));
    }

    if (!in_reverse && g.braking_percent != 0 && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        const float brake_gain = constrain_float(((-groundspeed_error)-g.braking_speederr)/g.braking_speederr, 0.0f, 1.0f);
        const int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        g2.motors.set_throttle(constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min));

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        set_reverse(true);
    }

    if (guided_mode != Guided_Velocity) {
        if (use_pivot_steering()) {
            // In Guided Velocity, only the steering input is used to calculate the pivot turn.
            g2.motors.set_throttle(0.0f);
        }
    }
}

/*****************************************
    Calculate desired turn angles (in medium freq loop)
 *****************************************/

void Rover::calc_lateral_acceleration() {
    switch (control_mode) {
    case AUTO:
        // If we have reached the waypoint previously navigate
        // back to it from our current position
        if (previously_reached_wp && (loiter_duration > 0)) {
            nav_controller->update_waypoint(current_loc, next_WP);
        } else {
            nav_controller->update_waypoint(prev_WP, next_WP);
        }
        break;

    case RTL:
    case GUIDED:
    case STEERING:
        nav_controller->update_waypoint(current_loc, next_WP);
        break;
    default:
        return;
    }

    // Calculate the required turn of the wheels

    // negative error = left turn
    // positive error = right turn
    lateral_acceleration = nav_controller->lateral_acceleration();
    if (use_pivot_steering()) {
        const int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (bearing_error > 0) {
            lateral_acceleration = g.turn_max_g * GRAVITY_MSS;
        } else {
            lateral_acceleration = -g.turn_max_g * GRAVITY_MSS;
        }
    }
}

/*
    calculate steering angle given lateral_acceleration
*/
void Rover::calc_nav_steer() {
    // check to see if the rover is loitering
    if (in_stationary_loiter()) {
        g2.motors.set_steering(0.0f);
        return;
    }

    // add in obstacle avoidance
    if (!in_reverse) {
        lateral_acceleration += (obstacle.turn_angle/45.0f) * g.turn_max_g;
    }

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g * GRAVITY_MSS, g.turn_max_g * GRAVITY_MSS);

    g2.motors.set_steering(steerController.get_steering_out_lat_accel(lateral_acceleration));
}

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void) {
    // Apply slew rate limit on non Manual modes
    if (control_mode == MANUAL || control_mode == LEARNING) {
        g2.motors.slew_limit_throttle(false);
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f);
        }
    } else {
        g2.motors.slew_limit_throttle(true);
    }

    // send output signals to motors
    g2.motors.output(arming.is_armed() && hal.util->get_soft_armed(), G_Dt);
}

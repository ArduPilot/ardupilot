// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*****************************************
    Throttle slew limit
*****************************************/
void Rover::throttle_slew_limit(int16_t last_throttle) {
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate && last_throttle != 0) {
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}

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
        gcs_send_text(MAV_SEVERITY_WARNING, "AUTO triggered off");
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
        gcs_send_text(MAV_SEVERITY_WARNING, "Triggered AUTO with pin");
        auto_triggered = true;
        return true;
    }

    if (!is_zero(g.auto_kickstart)) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Triggered AUTO xaccel=%.1f", (double)xaccel);
            auto_triggered = true;
            return true;
        }
    }

    return false;
}

/*
    work out if we are going to use pivot steering
*/
bool Rover::use_pivot_steering(void) {
    if (control_mode >= AUTO && g.skid_steer_out && g.pivot_turn_angle != 0) {
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (abs(bearing_error) > g.pivot_turn_angle) {
            return true;
        }
    }
    return false;
}


/*
    calculate the throtte for auto-throttle modes
*/
void Rover::calc_throttle(float target_speed) {
    // If not autostarting OR we are loitering at a waypoint
    // then set the throttle to minimum
    if (!auto_check_trigger() || ((loiter_time > 0) && (control_mode == AUTO))) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    int throttle_target = throttle_base + throttle_nudge;

    /*
        reduce target speed in proportion to turning rate, up to the
        SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0f, 1.0f);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0, 1);
    float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio * 0.01f;

    float reduction = 1.0f - steer_rate*speed_turn_reduction;

    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0f - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }

    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed;

    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        channel_throttle->servo_out = constrain_int16(-throttle, -g.throttle_max, -g.throttle_min);
    } else {
        channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min, g.throttle_max);
    }

    if (!in_reverse && g.braking_percent != 0 && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        float brake_gain = constrain_float(((-groundspeed_error)-g.braking_speederr)/g.braking_speederr, 0, 1);
        int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        channel_throttle->servo_out = constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min);

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        set_reverse(true);
    }

    if (use_pivot_steering()) {
        channel_throttle->servo_out = 0;
    }
}

/*****************************************
    Calculate desired turn angles (in medium freq loop)
 *****************************************/

void Rover::calc_lateral_acceleration() {
    switch (control_mode) {
    case AUTO:
        nav_controller->update_waypoint(prev_WP, next_WP);
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
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (bearing_error > 0) {
            lateral_acceleration = g.turn_max_g*GRAVITY_MSS;
        } else {
            lateral_acceleration = -g.turn_max_g*GRAVITY_MSS;
        }
    }
}

/*
    calculate steering angle given lateral_acceleration
*/
void Rover::calc_nav_steer() {
    // check to see if the rover is loitering
    if ((loiter_time > 0) && (control_mode == AUTO)) {
        channel_steer->servo_out = 0;
        return;
    }

    // add in obstacle avoidance
    lateral_acceleration += (obstacle.turn_angle/45.0f) * g.turn_max_g;

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g*GRAVITY_MSS, g.turn_max_g*GRAVITY_MSS);

    channel_steer->servo_out = steerController.get_steering_out_lat_accel(lateral_acceleration);
}

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void) {
    static int16_t last_throttle;

    // support a separate steering channel
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering, channel_steer->pwm_to_angle_dz(0));

    if (control_mode == MANUAL || control_mode == LEARNING) {
        // do a direct pass through of radio values
        channel_steer->radio_out       = channel_steer->read();
        channel_throttle->radio_out    = channel_throttle->read();
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            // suppress throttle if in failsafe and manual
            channel_throttle->radio_out = channel_throttle->radio_trim;
        }
    } else {
        channel_steer->calc_pwm();
        if (in_reverse) {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out,
                                          -g.throttle_max,
                                          -g.throttle_min);
        } else {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out,
                                          g.throttle_min.get(),
                                          g.throttle_max.get());
        }

        if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE) && control_mode < AUTO) {
            // suppress throttle if in failsafe
            channel_throttle->servo_out = 0;
        }

        if (!hal.util->get_soft_armed()) {
            channel_throttle->servo_out = 0;
        }

        // convert 0 to 100% into PWM
        channel_throttle->calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);
    }

    // record last throttle before we apply skid steering
    last_throttle = channel_throttle->radio_out;

    if (g.skid_steer_out) {
        // convert the two radio_out values to skid steering values
        /*
            mixing rule:
            steering = motor1 - motor2
            throttle = 0.5*(motor1 + motor2)
            motor1 = throttle + 0.5*steering
            motor2 = throttle - 0.5*steering
        */
        float steering_scaled = channel_steer->norm_output();
        float throttle_scaled = channel_throttle->norm_output();
        float motor1 = throttle_scaled + 0.5f*steering_scaled;
        float motor2 = throttle_scaled - 0.5f*steering_scaled;
        channel_steer->servo_out = 4500*motor1;
        channel_throttle->servo_out = 100*motor2;
        channel_steer->calc_pwm();
        channel_throttle->calc_pwm();
    }

    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) {
        case AP_Arming::NO:
            //keep existing behavior: do nothing to radio_out
            //(don't disarm throttle channel even if AP_Arming class is)
            break;

        case AP_Arming::YES_ZERO_PWM:
            channel_throttle->radio_out = 0;
            break;

        case AP_Arming::YES_MIN_PWM:
        default:
            channel_throttle->radio_out = channel_throttle->radio_trim;
            break;
        }
    }

#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
    // send values to the PWM timers for output
    // ----------------------------------------
    channel_steer->output();
    channel_throttle->output();
    RC_Channel_aux::output_ch_all();
#endif
}



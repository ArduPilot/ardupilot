// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
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
static bool auto_check_trigger(void)
{
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("AUTO triggered off"));
        auto_triggered = false;
        return false; 
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && g.auto_kickstart == 0.0f) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }
 
    if (g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Triggered AUTO with pin"));
        auto_triggered = true;
        return true;            
    }

    if (g.auto_kickstart != 0.0f) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"), xaccel);
            auto_triggered = true;
            return true;            
        }
    }

    return false;   
}


/*
  calculate the throtte for auto-throttle modes
 */
static void calc_throttle(float target_speed)
{  
    if (!auto_check_trigger()) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    if (target_speed <= 0) {
        // cope with zero requested speed
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    int throttle_target = g.throttle_cruise + throttle_nudge;  

    /*
      reduce target speed in proportion to turning rate, up to the
      SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0, 1.0);
    float reduction = 1.0 - steer_rate*(100 - g.speed_turn_gain)*0.01;
    
    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0 - (100-g.speed_turn_gain)*0.01*((g.speed_turn_dist - wp_distance)/g.speed_turn_dist);
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }
    
    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = target_speed - ground_speed; 
    
    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min.get(), g.throttle_max.get());
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

static void calc_lateral_acceleration()
{
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
}

/*
  calculate steering angle given lateral_acceleration
 */
static void calc_nav_steer()
{
    float speed = g_gps->ground_speed_cm * 0.01f;

    if (speed < 1.0f) {
        // gps speed isn't very accurate at low speed
        speed = 1.0f;
    }

    // add in obstacle avoidance
    lateral_acceleration += (obstacle.turn_angle/45.0f) * g.turn_max_g;

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g*GRAVITY_MSS, g.turn_max_g*GRAVITY_MSS);

    channel_steer->servo_out = steerController.get_steering_out(lateral_acceleration);
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = channel_throttle->radio_out;

	if ((control_mode == MANUAL || control_mode == LEARNING) &&
        (g.skid_steer_out == g.skid_steer_in)) {
        // do a direct pass through of radio values
        channel_steer->radio_out       = channel_steer->read();
        channel_throttle->radio_out    = channel_throttle->read();
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            // suppress throttle if in failsafe and manual
            channel_throttle->radio_out = channel_throttle->radio_trim;
        }
	} else {       
        channel_steer->calc_pwm();
		channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                       g.throttle_min.get(), 
                                                       g.throttle_max.get());

        if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE) && control_mode < AUTO) {
            // suppress throttle if in failsafe
            channel_throttle->servo_out = 0;
        }

        // convert 0 to 100% into PWM
        channel_throttle->calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);

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
            float motor1 = throttle_scaled + 0.5*steering_scaled;
            float motor2 = throttle_scaled - 0.5*steering_scaled;
            channel_steer->servo_out = 4500*motor1;
            channel_throttle->servo_out = 100*motor2;
            channel_steer->calc_pwm();
            channel_throttle->calc_pwm();
        }
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    channel_steer->output(); 
    channel_throttle->output();

	// Route configurable aux. functions to their respective servos
	g.rc_2.output_ch(CH_2);
	g.rc_4.output_ch(CH_4);
	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_9.output_ch(CH_9);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_12.output_ch(CH_12);
 #endif

#endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) {

    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
        hal.rcout->write(1, 1400);
        mavlink_delay(400);
        hal.rcout->write(1, 1600);
        mavlink_delay(200);
        hal.rcout->write(1, 1500);
#endif
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}


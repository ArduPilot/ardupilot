#include "Plane.h"

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

/*
  allow for runtime change of control channel ordering
 */
void Plane::set_control_channels(void)
{
    if (g.rudder_only) {
        // in rudder only mode the roll and rudder channels are the
        // same.
        channel_roll = RC_Channels::rc_channel(rcmap.yaw()-1);
    } else {
        channel_roll = RC_Channels::rc_channel(rcmap.roll()-1);
    }
    channel_pitch    = RC_Channels::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channels::rc_channel(rcmap.throttle()-1);
    channel_rudder   = RC_Channels::rc_channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(SERVO_MAX);
    channel_pitch->set_angle(SERVO_MAX);
    channel_rudder->set_angle(SERVO_MAX);
    if (!have_reverse_thrust()) {
        // normal operation
        channel_throttle->set_range(100);
    } else {
        // reverse thrust
        if (have_reverse_throttle_rc_option) {
            // when we have a reverse throttle RC option setup we use throttle
            // as a range, and rely on the RC switch to get reverse thrust
            channel_throttle->set_range(100);
        } else {
            channel_throttle->set_angle(100);
        }
        SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleLeft, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);
    }

    if (!arming.is_armed() && arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, have_reverse_thrust()?SRV_Channel::SRV_CHANNEL_LIMIT_TRIM:SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    }

    if (!quadplane.enable) {
        // setup correct scaling for ESCs like the UAVCAN PX4ESC which
        // take a proportion of speed. For quadplanes we use AP_Motors
        // scaling
        g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
    }
}

/*
  initialise RC input channels
 */
void Plane::init_rc_in()
{
    // set rc dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_rudder->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
}

/*
  initialise RC output for main channels. This is done early to allow
  for BRD_SAFETYENABLE=0 and early servo control
 */
void Plane::init_rc_out_main()
{
    /*
      change throttle trim to minimum throttle. This prevents a
      configuration error where the user sets CH3_TRIM incorrectly and
      the motor may start on power up
     */
    if (!have_reverse_thrust()) {
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle);
    }

    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    
    // setup PX4 to output the min throttle when safety off if arming
    // is setup for min on disarm
    if (arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, have_reverse_thrust()?SRV_Channel::SRV_CHANNEL_LIMIT_TRIM:SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    }
}

/*
  initialise RC output channels for aux channels
 */
void Plane::init_rc_out_aux()
{
    SRV_Channels::enable_aux_servos();

    SRV_Channels::cork();
    
    servos_output();
    
    // setup PWM values to send if the FMU firmware dies
    // allows any VTOL motors to shut off
    SRV_Channels::setup_failsafe_trim_all_non_motors();
}

/*
  check for pilot input on rudder stick for arming/disarming
*/
void Plane::rudder_arm_disarm_check()
{
    AP_Arming::ArmingRudder arming_rudder = arming.get_rudder_arming_type();

    if (arming_rudder == AP_Arming::ARMING_RUDDER_DISABLED) {
        //parameter disallows rudder arming/disabling
        return;
    }

    // if throttle is not down, then pilot cannot rudder arm/disarm
    if (get_throttle_input() != 0){
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual throttle mode and not in CRUISE or FBWB
    // modes then disallow rudder arming/disarming
    if (auto_throttle_mode &&
        (control_mode != &mode_cruise && control_mode != &mode_fbwb)) {
        rudder_arm_timer = 0;
        return;      
    }

	if (!arming.is_armed()) {
		// when not armed, full right rudder starts arming counter
		if (channel_rudder->get_control_in() > 4000) {
			uint32_t now = millis();

			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {

				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {
				//time to arm!
				arm_motors(AP_Arming::RUDDER);
				rudder_arm_timer = 0;
			}
		} else {
			// not at full right rudder
			rudder_arm_timer = 0;
		}
	} else if ((arming_rudder == AP_Arming::ARMING_RUDDER_ARMDISARM) && !is_flying()) {
		// when armed and not flying, full left rudder starts disarming counter
		if (channel_rudder->get_control_in() < -4000) {
			uint32_t now = millis();

			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {
				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {
				//time to disarm!
				disarm_motors();
				rudder_arm_timer = 0;
			}
		} else {
			// not at full left rudder
			rudder_arm_timer = 0;
		}
	}
}

void Plane::read_radio()
{
    if (!rc().read_input()) {
        control_failsafe();
        return;
    }

    if(!failsafe.rc_failsafe)
    {
        failsafe.AFS_last_valid_rc_ms = millis();
    }

    failsafe.last_valid_rc_ms = millis();

    if (control_mode == &mode_training) {
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        channel_roll->recompute_pwm_no_deadzone();
        channel_pitch->recompute_pwm_no_deadzone();
        channel_throttle->recompute_pwm_no_deadzone();
        channel_rudder->recompute_pwm_no_deadzone();
    }

    control_failsafe();

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input());

    if (g.throttle_nudge && SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > 50 && geofence_stickmixing()) {
        float nudge = (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) - 50) * 0.02f;
        if (ahrs.airspeed_sensor_enabled()) {
            airspeed_nudge_cm = (aparm.airspeed_max * 100 - aparm.airspeed_cruise_cm) * nudge;
        } else {
            throttle_nudge = (aparm.throttle_max - aparm.throttle_cruise) * nudge;
        }
    } else {
        airspeed_nudge_cm = 0;
        throttle_nudge = 0;
    }

    rudder_arm_disarm_check();

    // potentially swap inputs for tailsitters
    quadplane.tailsitter_check_input();

    // check for transmitter tuning changes
    tuning.check_input(control_mode->mode_number());
}

int16_t Plane::rudder_input(void)
{
    if (g.rudder_only != 0) {
        // in rudder only mode we discard rudder input and get target
        // attitude from the roll channel.
        return 0;
    }

    if ((g2.flight_options & FlightOptions::DIRECT_RUDDER_ONLY) &&
        !(control_mode == MANUAL || control_mode == STABILIZE || control_mode == ACRO)) {
        // the user does not want any input except in these modes
        return 0;
    }

    if (stick_mixing_enabled()) {
        return channel_rudder->get_control_in();
    }

    return 0;
    
}

void Plane::control_failsafe()
{
    if (millis() - failsafe.last_valid_rc_ms > 1000 || rc_failsafe_active()) {
        // we do not have valid RC input. Set all primary channel
        // control inputs to the trim value and throttle to min
        channel_roll->set_radio_in(channel_roll->get_radio_trim());
        channel_pitch->set_radio_in(channel_pitch->get_radio_trim());
        channel_rudder->set_radio_in(channel_rudder->get_radio_trim());

        // note that we don't set channel_throttle->radio_in to radio_trim,
        // as that would cause throttle failsafe to not activate
        channel_roll->set_control_in(0);
        channel_pitch->set_control_in(0);
        channel_rudder->set_control_in(0);
        channel_throttle->set_control_in(0);
    }

    if(g.throttle_fs_enabled == 0)
        return;

    if (g.throttle_fs_enabled) {
        if (rc_failsafe_active()) {
            // we detect a failsafe from radio
            // throttle has dropped below the mark
            failsafe.throttle_counter++;
            if (failsafe.throttle_counter == 10) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe on");
                failsafe.rc_failsafe = true;
                AP_Notify::flags.failsafe_radio = true;
            }
            if (failsafe.throttle_counter > 10) {
                failsafe.throttle_counter = 10;
            }

        }else if(failsafe.throttle_counter > 0) {
            // we are no longer in failsafe condition
            // but we need to recover quickly
            failsafe.throttle_counter--;
            if (failsafe.throttle_counter > 3) {
                failsafe.throttle_counter = 3;
            }
            if (failsafe.throttle_counter == 1) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe off");
            } else if(failsafe.throttle_counter == 0) {
                failsafe.rc_failsafe = false;
                AP_Notify::flags.failsafe_radio = false;
            }
        }
    }
}

bool Plane::trim_radio()
{
    if (failsafe.rc_failsafe) {
        // can't trim if we don't have valid input
        return false;
    }

    int16_t trim_roll_range = (channel_roll->get_radio_max() - channel_roll->get_radio_min())/5;
    int16_t trim_pitch_range = (channel_pitch->get_radio_max() - channel_pitch->get_radio_min())/5;
    if (channel_roll->get_radio_in() < channel_roll->get_radio_min()+trim_roll_range ||
        channel_roll->get_radio_in() > channel_roll->get_radio_max()-trim_roll_range ||
        channel_pitch->get_radio_in() < channel_pitch->get_radio_min()+trim_pitch_range ||
        channel_pitch->get_radio_in() > channel_pitch->get_radio_max()-trim_pitch_range) {
        // don't trim for extreme values - if we attempt to trim so
        // there is less than 20 percent range left then assume the
        // sticks are not properly centered. This also prevents
        // problems with starting APM with the TX off
        return false;
    }

    // trim main surfaces
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_aileron);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevator);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_rudder);

    // trim elevons
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevon_left);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevon_right);

    // trim vtail
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_vtail_left);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_vtail_right);
    
    if (SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) == 0) {
        // trim differential spoilers if no rudder input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft2);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight2);
    }

    if (SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto) == 0 &&
        SRV_Channels::get_output_scaled(SRV_Channel::k_flap) == 0) {
        // trim flaperons if no flap input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_left);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_right);
    }

    // now save input trims, as these have been moved to the outputs
    channel_roll->set_and_save_trim();
    channel_pitch->set_and_save_trim();
    channel_rudder->set_and_save_trim();

    return true;
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
 */
bool Plane::rc_failsafe_active(void) const
{
    if (!g.throttle_fs_enabled) {
        return false;
    }
    if (millis() - failsafe.last_valid_rc_ms > 1000) {
        // we haven't had a valid RC frame for 1 seconds
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->get_radio_in() >= g.throttle_fs_value;
    }
    return channel_throttle->get_radio_in() <= g.throttle_fs_value;
}

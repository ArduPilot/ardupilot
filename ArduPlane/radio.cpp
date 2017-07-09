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
    if (aparm.throttle_min >= 0) {
        // normal operation
        channel_throttle->set_range(100);
    } else {
        // reverse thrust
        channel_throttle->set_angle(100);
        SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleLeft, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);
    }

    if (!arming.is_armed() && arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, aparm.throttle_min<0?SRV_Channel::SRV_CHANNEL_LIMIT_TRIM:SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    }

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed
    g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
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
    if (aparm.throttle_min >= 0) {
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle);
    }

    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    
    // setup PX4 to output the min throttle when safety off if arming
    // is setup for min on disarm
    if (arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, aparm.throttle_min<0?SRV_Channel::SRV_CHANNEL_LIMIT_TRIM:SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    }
}

/*
  initialise RC output channels for aux channels
 */
void Plane::init_rc_out_aux()
{
    update_aux();
    SRV_Channels::enable_aux_servos();

    hal.rcout->cork();
    
    // Initialization of servo outputs
    SRV_Channels::output_trim_all();

    servos_output();
    
    // setup PWM values to send if the FMU firmware dies
    SRV_Channels::setup_failsafe_trim_all();  
}

/*
  check for pilot input on rudder stick for arming/disarming
*/
void Plane::rudder_arm_disarm_check()
{
    AP_Arming_Plane::ArmingRudder arming_rudder = arming.rudder_arming();

    if (arming_rudder == AP_Arming_Plane::ARMING_RUDDER_DISABLED) {
        //parameter disallows rudder arming/disabling
        return;
    }

    // if throttle is not down, then pilot cannot rudder arm/disarm
    if (channel_throttle->get_control_in() != 0){
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual throttle mode and not in CRUISE or FBWB
    // modes then disallow rudder arming/disarming
    if (auto_throttle_mode &&
        (control_mode != CRUISE && control_mode != FLY_BY_WIRE_B)) {
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
	} else if (arming_rudder == AP_Arming_Plane::ARMING_RUDDER_ARMDISARM && !is_flying()) {
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
    if (!hal.rcin->new_input()) {
        control_failsafe(channel_throttle->get_radio_in());
        return;
    }

    if(!failsafe.ch3_failsafe)
    {
        failsafe.AFS_last_valid_rc_ms = millis();
    }

    failsafe.last_valid_rc_ms = millis();

    elevon.ch1_temp = channel_roll->read();
    elevon.ch2_temp = channel_pitch->read();
    uint16_t pwm_roll, pwm_pitch;

    pwm_roll = elevon.ch1_temp;
    pwm_pitch = elevon.ch2_temp;

    RC_Channels::set_pwm_all();
    
    if (control_mode == TRAINING) {
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        channel_roll->set_pwm_no_deadzone(pwm_roll);
        channel_pitch->set_pwm_no_deadzone(pwm_pitch);
        channel_throttle->set_pwm_no_deadzone(channel_throttle->read());
        channel_rudder->set_pwm_no_deadzone(channel_rudder->read());
    } else {
        channel_roll->set_pwm(pwm_roll);
        channel_pitch->set_pwm(pwm_pitch);
    }

    control_failsafe(channel_throttle->get_radio_in());

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, channel_throttle->get_control_in());

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

    if (g.rudder_only != 0) {
        // in rudder only mode we discard rudder input and get target
        // attitude from the roll channel.
        rudder_input = 0;
    } else if (stick_mixing_enabled()) {
        rudder_input = channel_rudder->get_control_in();
    } else {
        // no stick mixing
        rudder_input = 0;
    }

    // potentially swap inputs for tailsitters
    quadplane.tailsitter_check_input();
    
    // check for transmitter tuning changes
    tuning.check_input(control_mode);
}

void Plane::control_failsafe(uint16_t pwm)
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
            failsafe.ch3_counter++;
            if (failsafe.ch3_counter == 10) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe on %u", (unsigned)pwm);
                failsafe.ch3_failsafe = true;
                AP_Notify::flags.failsafe_radio = true;
            }
            if (failsafe.ch3_counter > 10) {
                failsafe.ch3_counter = 10;
            }

        }else if(failsafe.ch3_counter > 0) {
            // we are no longer in failsafe condition
            // but we need to recover quickly
            failsafe.ch3_counter--;
            if (failsafe.ch3_counter > 3) {
                failsafe.ch3_counter = 3;
            }
            if (failsafe.ch3_counter == 1) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe off %u", (unsigned)pwm);
            } else if(failsafe.ch3_counter == 0) {
                failsafe.ch3_failsafe = false;
                AP_Notify::flags.failsafe_radio = false;
            }
        }
    }
}

void Plane::trim_control_surfaces()
{
    read_radio();
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
        return;
    }

    // trim ailerons if not used as old elevons
    if (g.elevon_output == MIXING_DISABLED) {
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_aileron);
    }

    // trim elevator if not used as old elevons or vtail
    if (g.elevon_output == MIXING_DISABLED && g.vtail_output == MIXING_DISABLED) {
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevator);
    }

    // trim rudder if not used as old vtail
    if (g.vtail_output == MIXING_DISABLED) {
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_rudder);
    }

    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_aileron_with_input);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevator_with_input);

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
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight1);
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
}

void Plane::trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }

    trim_control_surfaces();
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
 */
bool Plane::rc_failsafe_active(void)
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

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

    // update flap and airbrake channel assignment
    channel_flap     = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP);
    channel_airbrake = rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRBRAKE);

#if HAL_QUADPLANE_ENABLED
    // update manual forward throttle channel assignment
    quadplane.rc_fwd_thr_ch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FWD_THR);
#endif

    bool set_throttle_esc_scaling = true;
#if HAL_QUADPLANE_ENABLED
    set_throttle_esc_scaling = !quadplane.enable;
#endif
    if (set_throttle_esc_scaling) {
        // setup correct scaling for ESCs like the UAVCAN ESCs which
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
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight);
    }

    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::TRIM);

}

/*
  initialise RC output channels for aux channels
 */
void Plane::init_rc_out_aux()
{
    SRV_Channels::enable_aux_servos();

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
				arming.arm(AP_Arming::Method::RUDDER);
				rudder_arm_timer = 0;
			}
		} else {
			// not at full right rudder
			rudder_arm_timer = 0;
		}
	} else {
		// full left rudder starts disarming counter
		if (channel_rudder->get_control_in() < -4000) {
			uint32_t now = millis();

			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {
				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {
				//time to disarm!
				arming.disarm(AP_Arming::Method::RUDDER);
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

    if (!failsafe.rc_failsafe)
    {
        failsafe.AFS_last_valid_rc_ms = millis();
    }

    if (rc_throttle_value_ok()) {
        failsafe.last_valid_rc_ms = millis();
    }

    control_failsafe();

#if AP_FENCE_ENABLED
    const bool stickmixing = fence_stickmixing();
#else
    const bool stickmixing = true;
#endif
    airspeed_nudge_cm = 0;
    throttle_nudge = 0;
    if (g.throttle_nudge
        && channel_throttle->get_control_in() > 50
        && stickmixing) {
        float nudge = (channel_throttle->get_control_in() - 50) * 0.02f;
        if (ahrs.airspeed_sensor_enabled()) {
            airspeed_nudge_cm = (aparm.airspeed_max * 100 - aparm.airspeed_cruise_cm) * nudge;
        } else {
            throttle_nudge = (aparm.throttle_max - aparm.throttle_cruise) * nudge;
        }
    }

    rudder_arm_disarm_check();

#if HAL_QUADPLANE_ENABLED
    // potentially swap inputs for tailsitters
    quadplane.tailsitter.check_input();
#endif

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
        !(control_mode == &mode_manual || control_mode == &mode_stabilize || control_mode == &mode_acro)) {
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
    if (rc_failsafe_active()) {
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

        airspeed_nudge_cm = 0;
        throttle_nudge = 0;

        switch (control_mode->mode_number()) {
#if HAL_QUADPLANE_ENABLED
            case Mode::Number::QSTABILIZE:
            case Mode::Number::QHOVER:
            case Mode::Number::QLOITER:
            case Mode::Number::QLAND: // throttle is ignored, but reset anyways
            case Mode::Number::QRTL:  // throttle is ignored, but reset anyways
            case Mode::Number::QACRO:
#if QAUTOTUNE_ENABLED
            case Mode::Number::QAUTOTUNE:
#endif
                if (quadplane.available() && quadplane.motors->get_desired_spool_state() > AP_Motors::DesiredSpoolState::GROUND_IDLE) {
                    // set half throttle to avoid descending at maximum rate, still has a slight descent due to throttle deadzone
                    channel_throttle->set_control_in(channel_throttle->get_range() / 2);
                    break;
                }
                FALLTHROUGH;
#endif
            default:
                channel_throttle->set_control_in(0);
                break;
        }
    }

    if (ThrFailsafe(g.throttle_fs_enabled.get()) != ThrFailsafe::Enabled) {
        return;
    }

    if (rc_failsafe_active()) {
        // we detect a failsafe from radio
        // throttle has dropped below the mark
        failsafe.throttle_counter++;
        if (failsafe.throttle_counter == 10) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe %s", "on");
            failsafe.rc_failsafe = true;
            AP_Notify::flags.failsafe_radio = true;
        }
        if (failsafe.throttle_counter > 10) {
            failsafe.throttle_counter = 10;
        }
    } else if(failsafe.throttle_counter > 0) {
        // we are no longer in failsafe condition
        // but we need to recover quickly
        failsafe.throttle_counter--;
        if (failsafe.throttle_counter > 3) {
            failsafe.throttle_counter = 3;
        }
        if (failsafe.throttle_counter == 1) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe %s", "off");
        } else if(failsafe.throttle_counter == 0) {
            failsafe.rc_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
        }
    }
}

void Plane::trim_radio()
{
    if (failsafe.rc_failsafe) {
        // can't trim if we don't have valid input
        return;
    }

    if (plane.control_mode != &mode_manual) {
        gcs().send_text(MAV_SEVERITY_ERROR, "trim failed, not in manual mode");
        return;
    }

    if (labs(channel_roll->get_control_in()) > (channel_roll->get_range() * 0.2) ||
            labs(channel_pitch->get_control_in()) > (channel_pitch->get_range() * 0.2)) {
        // don't trim for extreme values - if we attempt to trim
        // more than 20 percent range left then assume the
        // sticks are not properly centered. This also prevents
        // problems with starting APM with the TX off
        gcs().send_text(MAV_SEVERITY_ERROR, "trim failed, large roll and pitch input");
        return;
    }

    if (degrees(ahrs.get_gyro().length()) > 30.0) {
        // rotating more than 30 deg/second
        gcs().send_text(MAV_SEVERITY_ERROR, "trim failed, large movement");
        return;
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
    
    if (is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_rudder))) {
        // trim differential spoilers if no rudder input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft2);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight2);
    }

    if (is_zero(SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto)) &&
        is_zero(SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap))) {
        // trim flaperons if no flap input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_left);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_right);
    }

    // now save input trims, as these have been moved to the outputs
    channel_roll->set_and_save_trim();
    channel_pitch->set_and_save_trim();
    channel_rudder->set_and_save_trim();

    gcs().send_text(MAV_SEVERITY_NOTICE, "trim complete");
}

/*
  check if throttle value is within allowed range
 */
bool Plane::rc_throttle_value_ok(void) const
{
    if (ThrFailsafe(g.throttle_fs_enabled.get()) == ThrFailsafe::Disabled) {
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->get_radio_in() < g.throttle_fs_value;
    }
    return channel_throttle->get_radio_in() > g.throttle_fs_value;
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
 */
bool Plane::rc_failsafe_active(void) const
{
    if (!rc_throttle_value_ok()) {
        return true;
    }
    if (millis() - failsafe.last_valid_rc_ms > 1000) {
        // we haven't had a valid RC frame for 1 seconds
        return true;
    }
    return false;
}

/*
  expo handling for MANUAL, ACRO and TRAINING modes
 */
static float channel_expo(RC_Channel *chan, int8_t expo, bool use_dz)
{
    if (chan == nullptr) {
        return 0;
    }
    float rin = use_dz? chan->get_control_in() : chan->get_control_in_zero_dz();
    return SERVO_MAX * expo_curve(constrain_float(expo*0.01, 0, 1), rin/SERVO_MAX);
}

float Plane::roll_in_expo(bool use_dz) const
{
    return channel_expo(channel_roll, g2.man_expo_roll, use_dz);
}

float Plane::pitch_in_expo(bool use_dz) const
{
    return channel_expo(channel_pitch, g2.man_expo_pitch, use_dz);
}

float Plane::rudder_in_expo(bool use_dz) const
{
    return channel_expo(channel_rudder, g2.man_expo_rudder, use_dz);
}

bool Plane::throttle_at_zero(void) const
{
/* true if throttle stick is at idle position...if throttle trim has been moved
   to center stick area in conjunction with sprung throttle, cannot use in_trim, must use rc_min
*/
    if (((!(g2.flight_options & FlightOptions::CENTER_THROTTLE_TRIM) && channel_throttle->in_trim_dz()) ||
        (g2.flight_options & FlightOptions::CENTER_THROTTLE_TRIM && channel_throttle->in_min_dz()))) {
        return true;
    }
    return false;
}

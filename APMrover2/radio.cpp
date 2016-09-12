// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*
  allow for runtime change of control channel ordering
 */
void Rover::set_control_channels(void)
{
    channel_steer    = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_learn    = RC_Channel::rc_channel(g.learn_channel-1);

	// set rc channel ranges
	channel_steer->set_angle(SERVO_MAX);
    channel_throttle->set_angle(100);

    // For a rover safety is TRIM throttle
    if (!arming.is_armed() && arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), channel_throttle->get_radio_trim());
        if (g.skid_steer_out) {
            hal.rcout->set_safety_pwm(1UL<<(rcmap.roll()-1),  channel_steer->get_radio_trim());
        }
    }

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed. 
    hal.rcout->set_esc_scaling(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
}

void Rover::init_rc_in()
{
	// set rc dead zones
	channel_steer->set_default_dead_zone(30);
	channel_throttle->set_default_dead_zone(30);

	//set auxiliary ranges
    update_aux();
}

void Rover::init_rc_out()
{
    RC_Channel::rc_channel(CH_1)->enable_out();
    RC_Channel::rc_channel(CH_3)->enable_out();

    if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
        channel_throttle->enable_out();
        if (g.skid_steer_out) {
            channel_steer->enable_out();
        }
    }

    RC_Channel::output_trim_all();    

    // setup PWM values to send if the FMU firmware dies
    RC_Channel::setup_failsafe_trim_all();  

    // output throttle trim when safety off if arming
    // is setup for min on disarm.  MIN is from plane where MIN is effectively no throttle.
    // For Rover's no throttle means TRIM as rovers can go backwards i.e. MIN throttle is
    // full speed backward.
    if (arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1),  channel_throttle->get_radio_trim());
        if (g.skid_steer_out) {
            hal.rcout->set_safety_pwm(1UL<<(rcmap.roll()-1),  channel_steer->get_radio_trim());
        }
    }
}

/*
  check for driver input on rudder/steering stick for arming/disarming
*/
void Rover::rudder_arm_disarm_check()
{
    // In Rover we need to check that its set to the throttle trim and within the DZ
    // if throttle is not within trim dz, then pilot cannot rudder arm/disarm
    if (!channel_throttle->in_trim_dz()) {
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual throttle mode then disallow rudder arming/disarming
    if (auto_throttle_mode) {
        rudder_arm_timer = 0;
        return;
    }

	if (!arming.is_armed()) {
		// when not armed, full right rudder starts arming counter
		if (channel_steer->get_control_in() > 4000) {
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
	} else if (!motor_active() & !g.skid_steer_out) {
		// when armed and motor not active (not moving), full left rudder starts disarming counter
        // This is disabled for skid steering otherwise when tring to turn a skid steering rover around
        // the rover would disarm
		if (channel_steer->get_control_in() < -4000) {
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

void Rover::read_radio()
{
    if (!hal.rcin->new_input()) {
        control_failsafe(channel_throttle->get_radio_in());
        return;
    }

    failsafe.last_valid_rc_ms = AP_HAL::millis();

    RC_Channel::set_pwm_all();

	control_failsafe(channel_throttle->get_radio_in());

	channel_throttle->set_servo_out(channel_throttle->get_control_in());

    // Check if the throttle value is above 50% and we need to nudge
    // Make sure its above 50% in the direction we are travelling
	if ((abs(channel_throttle->get_servo_out()) > 50) &&
        (((channel_throttle->get_servo_out() < 0) && in_reverse) ||
         ((channel_throttle->get_servo_out() > 0) && !in_reverse))) {
            throttle_nudge = (g.throttle_max - g.throttle_cruise) *
                ((fabsf(channel_throttle->norm_input())-0.5f) / 0.5f);
	} else {
		throttle_nudge = 0;
	}

    if (g.skid_steer_in) {
        // convert the two radio_in values from skid steering values
        /*
          mixing rule:
          steering = motor1 - motor2
          throttle = 0.5*(motor1 + motor2)
          motor1 = throttle + 0.5*steering
          motor2 = throttle - 0.5*steering
        */          

        float motor1 = channel_steer->norm_input();
        float motor2 = channel_throttle->norm_input();
        float steering_scaled = motor1 - motor2;
        float throttle_scaled = 0.5f*(motor1 + motor2);
        int16_t steer = channel_steer->get_radio_trim();
        int16_t thr   = channel_throttle->get_radio_trim();
        if (steering_scaled > 0.0f) {
            steer += steering_scaled*(channel_steer->get_radio_max()-channel_steer->get_radio_trim());
        } else {
            steer += steering_scaled*(channel_steer->get_radio_trim()-channel_steer->get_radio_min());
        }
        if (throttle_scaled > 0.0f) {
            thr += throttle_scaled*(channel_throttle->get_radio_max()-channel_throttle->get_radio_trim());
        } else {
            thr += throttle_scaled*(channel_throttle->get_radio_trim()-channel_throttle->get_radio_min());
        }
        channel_steer->set_pwm(steer);
        channel_throttle->set_pwm(thr);
    }

    rudder_arm_disarm_check();

}

void Rover::control_failsafe(uint16_t pwm)
{
	if (!g.fs_throttle_enabled) {
        // no throttle failsafe
		return;
    }

	// Check for failsafe condition based on loss of GCS control
	if (rc_override_active) {
        failsafe_trigger(FAILSAFE_EVENT_RC, (millis() - failsafe.rc_override_timer) > 1500);
	} else if (g.fs_throttle_enabled) {
        bool failed = pwm < (uint16_t)g.fs_throttle_value;
        if (AP_HAL::millis() - failsafe.last_valid_rc_ms > 2000) {
            failed = true;
        }
        failsafe_trigger(FAILSAFE_EVENT_THROTTLE, failed);
	}
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
 */
bool Rover::throttle_failsafe_active(void)
{
    if (!g.fs_throttle_enabled) {
        return false;
    }
    if (millis() - failsafe.last_valid_rc_ms > 1000) {
        // we haven't had a valid RC frame for 1 seconds
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->get_radio_in() >= g.fs_throttle_value;
    }
    return channel_throttle->get_radio_in() <= g.fs_throttle_value;
}

void Rover::trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
    if (channel_steer->get_radio_in() > 1400) {
		channel_steer->set_radio_trim(channel_steer->get_radio_in());
        // save to eeprom
        channel_steer->save_eeprom();
    }
}

void Rover::trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}
    trim_control_surfaces();
}

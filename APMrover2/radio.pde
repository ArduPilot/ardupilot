// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  allow for runtime change of control channel ordering
 */
static void set_control_channels(void)
{
    channel_steer    = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_learn    = RC_Channel::rc_channel(g.learn_channel-1);

	// set rc channel ranges
	channel_steer->set_angle(SERVO_MAX);
	channel_throttle->set_angle(100);

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed. 
    hal.rcout->set_esc_scaling(channel_throttle->radio_min, channel_throttle->radio_max);
}

static void init_rc_in()
{
	// set rc dead zones
	channel_steer->set_default_dead_zone(30);
	channel_throttle->set_default_dead_zone(30);

	//set auxiliary ranges
    update_aux();
}

static void init_rc_out()
{
    RC_Channel::rc_channel(CH_1)->enable_out();
    RC_Channel::rc_channel(CH_3)->enable_out();
    RC_Channel::output_trim_all();    

    // setup PWM values to send if the FMU firmware dies
    RC_Channel::setup_failsafe_trim_all();  
}

static void read_radio()
{
    if (!hal.rcin->new_input()) {
        control_failsafe(channel_throttle->radio_in);
        return;
    }

    failsafe.last_valid_rc_ms = hal.scheduler->millis();

    RC_Channel::set_pwm_all();

	control_failsafe(channel_throttle->radio_in);

	channel_throttle->servo_out = channel_throttle->control_in;

	if (abs(channel_throttle->servo_out) > 50) {
        throttle_nudge = (g.throttle_max - g.throttle_cruise) * ((fabsf(channel_throttle->norm_input())-0.5) / 0.5);
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
        int16_t steer = channel_steer->radio_trim;
        int16_t thr   = channel_throttle->radio_trim;
        if (steering_scaled > 0.0f) {
            steer += steering_scaled*(channel_steer->radio_max-channel_steer->radio_trim);
        } else {
            steer += steering_scaled*(channel_steer->radio_trim-channel_steer->radio_min);
        }
        if (throttle_scaled > 0.0f) {
            thr += throttle_scaled*(channel_throttle->radio_max-channel_throttle->radio_trim);
        } else {
            thr += throttle_scaled*(channel_throttle->radio_trim-channel_throttle->radio_min);
        }
        channel_steer->set_pwm(steer);
        channel_throttle->set_pwm(thr);
    }
}

static void control_failsafe(uint16_t pwm)
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
        if (hal.scheduler->millis() - failsafe.last_valid_rc_ms > 2000) {
            failed = true;
        }
        failsafe_trigger(FAILSAFE_EVENT_THROTTLE, failed);
	}
}

static void trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
    if (channel_steer->radio_in > 1400) {
		channel_steer->radio_trim = channel_steer->radio_in;
        // save to eeprom
        channel_steer->save_eeprom();
    }
}

static void trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}
    trim_control_surfaces();
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

/*
  allow for runtime change of control channel ordering
 */
static void set_control_channels(void)
{
    channel_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_rudder   = RC_Channel::rc_channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(SERVO_MAX);
    channel_pitch->set_angle(SERVO_MAX);
    channel_rudder->set_angle(SERVO_MAX);
    channel_throttle->set_range(0, 100);

    if (!arming.is_armed() && arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), channel_throttle->radio_min);
    }
}

/*
  initialise RC input channels
 */
static void init_rc_in()
{
    // set rc dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_rudder->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);

    update_aux();
}

/*
  initialise RC output channels
 */
static void init_rc_out()
{
    channel_roll->enable_out();
    channel_pitch->enable_out();
    if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
        channel_throttle->enable_out();
    }
    channel_rudder->enable_out();
    enable_aux_servos();

    // Initialization of servo outputs
    for (uint8_t i=0; i<8; i++) {
        RC_Channel::rc_channel(i)->output_trim();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_9,   g.rc_9.radio_trim);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_10,  g.rc_10.radio_trim);
    servo_write(CH_11,  g.rc_11.radio_trim);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_12,  g.rc_12.radio_trim);
#endif

    // setup PX4 to output the min throttle when safety off if arming
    // is setup for min on disarm
    if (arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), channel_throttle->radio_min);
    }
}

// check for pilot input on rudder stick for arming
static void rudder_arm_check() 
{
    //TODO: ensure rudder arming disallowed during radio calibration

    //TODO: waggle ailerons and rudder and beep after rudder arming
    
    static uint32_t rudder_arm_timer;

    if (arming.is_armed()) {
        //already armed, no need to run remainder of this function
        rudder_arm_timer = 0;
        return;
    } 

    if (! arming.rudder_arming_enabled()) {
        //parameter disallows rudder arming
        return;
    }

    //if throttle is not down, then pilot cannot rudder arm
    if (g.rc_3.control_in > 0) {
        rudder_arm_timer = 0;
        return;
    }

    //if not in a 'manual' mode then disallow rudder arming
    if (auto_throttle_mode ) {
        rudder_arm_timer = 0;
        return;      
    }

    // full right rudder starts arming counter
    if (g.rc_4.control_in > 4000) {
        uint32_t now = millis();

        if (rudder_arm_timer == 0 || 
            now - rudder_arm_timer < 3000) {

            if (rudder_arm_timer == 0) rudder_arm_timer = now;
        } else {
            //time to arm!
            if (arming.arm(AP_Arming::RUDDER)) {
                channel_throttle->enable_out();                        
                //only log if arming was successful
                Log_Arm_Disarm();
            }                
        }
    } else { 
        // not at full right rudder
        rudder_arm_timer = 0;
    }
}

static void read_radio()
{
    if (!hal.rcin->valid_channels()) {
        control_failsafe(channel_throttle->radio_in);
        return;
    }

    failsafe.last_valid_rc_ms = hal.scheduler->millis();

    elevon.ch1_temp = channel_roll->read();
    elevon.ch2_temp = channel_pitch->read();
    uint16_t pwm_roll, pwm_pitch;

    if (g.mix_mode == 0) {
        pwm_roll = elevon.ch1_temp;
        pwm_pitch = elevon.ch2_temp;
    }else{
        pwm_roll = BOOL_TO_SIGN(g.reverse_elevons) * (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int16_t(elevon.ch2_temp - elevon.trim2) - BOOL_TO_SIGN(g.reverse_ch1_elevon) * int16_t(elevon.ch1_temp - elevon.trim1)) / 2 + 1500;
        pwm_pitch = (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int16_t(elevon.ch2_temp - elevon.trim2) + BOOL_TO_SIGN(g.reverse_ch1_elevon) * int16_t(elevon.ch1_temp - elevon.trim1)) / 2 + 1500;
    }
    
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
        channel_throttle->set_pwm(channel_throttle->read());
        channel_rudder->set_pwm(channel_rudder->read());
    }

    g.rc_5.set_pwm(hal.rcin->read(CH_5));
    g.rc_6.set_pwm(hal.rcin->read(CH_6));
    g.rc_7.set_pwm(hal.rcin->read(CH_7));
    g.rc_8.set_pwm(hal.rcin->read(CH_8));

    control_failsafe(channel_throttle->radio_in);

    channel_throttle->servo_out = channel_throttle->control_in;

    if (g.throttle_nudge && channel_throttle->servo_out > 50) {
        float nudge = (channel_throttle->servo_out - 50) * 0.02f;
        if (airspeed.use()) {
            airspeed_nudge_cm = (aparm.airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
        } else {
            throttle_nudge = (aparm.throttle_max - aparm.throttle_cruise) * nudge;
        }
    } else {
        airspeed_nudge_cm = 0;
        throttle_nudge = 0;
    }

    rudder_arm_check();
}

static void control_failsafe(uint16_t pwm)
{
    if(g.throttle_fs_enabled == 0)
        return;

    // Check for failsafe condition based on loss of GCS control
    if (failsafe.rc_override_active) {
        if (millis() - failsafe.last_heartbeat_ms > g.short_fs_timeout*1000) {
            failsafe.ch3_failsafe = true;
            AP_Notify::flags.failsafe_radio = true;
        } else {
            failsafe.ch3_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
        }

        //Check for failsafe and debounce funky reads
    } else if (g.throttle_fs_enabled) {
        if (throttle_failsafe_level()) {
            // we detect a failsafe from radio
            // throttle has dropped below the mark
            failsafe.ch3_counter++;
            if (failsafe.ch3_counter == 10) {
                gcs_send_text_fmt(PSTR("MSG FS ON %u"), (unsigned)pwm);
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
                gcs_send_text_fmt(PSTR("MSG FS OFF %u"), (unsigned)pwm);
            } else if(failsafe.ch3_counter == 0) {
                failsafe.ch3_failsafe = false;
                AP_Notify::flags.failsafe_radio = false;
            }
        }
    }
}

static void trim_control_surfaces()
{
    read_radio();
    int16_t trim_roll_range = (channel_roll->radio_max - channel_roll->radio_min)/5;
    int16_t trim_pitch_range = (channel_pitch->radio_max - channel_pitch->radio_min)/5;
    if (channel_roll->radio_in < channel_roll->radio_min+trim_roll_range ||
        channel_roll->radio_in > channel_roll->radio_max-trim_roll_range ||
        channel_pitch->radio_in < channel_pitch->radio_min+trim_pitch_range ||
        channel_pitch->radio_in > channel_pitch->radio_max-trim_pitch_range) {
        // don't trim for extreme values - if we attempt to trim so
        // there is less than 20 percent range left then assume the
        // sticks are not properly centered. This also prevents
        // problems with starting APM with the TX off
        return;
    }

    // Store control surface trim values
    // ---------------------------------
    if(g.mix_mode == 0) {
        if (channel_roll->radio_in != 0) {
            channel_roll->radio_trim = channel_roll->radio_in;
        }
        if (channel_pitch->radio_in != 0) {
            channel_pitch->radio_trim = channel_pitch->radio_in;
        }

        // the secondary aileron/elevator is trimmed only if it has a
        // corresponding transmitter input channel, which k_aileron
        // doesn't have
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_aileron_with_input);
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_elevator_with_input);
    } else{
        if (elevon.ch1_temp != 0) {
            elevon.trim1 = elevon.ch1_temp;
        }
        if (elevon.ch2_temp != 0) {
            elevon.trim2 = elevon.ch2_temp;
        }
        //Recompute values here using new values for elevon1_trim and elevon2_trim
        //We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
        uint16_t center                         = 1500;
        channel_roll->radio_trim       = center;
        channel_pitch->radio_trim      = center;
    }
    if (channel_rudder->radio_in != 0) {
        channel_rudder->radio_trim = channel_rudder->radio_in;
    }

    // save to eeprom
    channel_roll->save_eeprom();
    channel_pitch->save_eeprom();
    channel_rudder->save_eeprom();
}

static void trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }

    trim_control_surfaces();
}

/*
  return true if throttle level is below throttle failsafe threshold
 */
static bool throttle_failsafe_level(void)
{
    if (!g.throttle_fs_enabled) {
        return false;
    }
    if (hal.scheduler->millis() - failsafe.last_valid_rc_ms > 2000) {
        // we haven't had a valid RC frame for 2 seconds
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->radio_in >= g.throttle_fs_value;
    }
    return channel_throttle->radio_in <= g.throttle_fs_value;
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static uint8_t failsafeCounter = 0;                // we wait a second to take over the throttle and send the plane circling


extern RC_Channel* rc_ch[8];

static void init_rc_in()
{
    // set rc channel ranges
    g.channel_roll.set_angle(SERVO_MAX);
    g.channel_pitch.set_angle(SERVO_MAX);
    g.channel_rudder.set_angle(SERVO_MAX);
    g.channel_throttle.set_range(0, 100);

    // set rc dead zones
    g.channel_roll.set_dead_zone(60);
    g.channel_pitch.set_dead_zone(60);
    g.channel_rudder.set_dead_zone(60);
    g.channel_throttle.set_dead_zone(6);

    //g.channel_roll.dead_zone  = 60;
    //g.channel_pitch.dead_zone     = 60;
    //g.channel_rudder.dead_zone    = 60;
    //g.channel_throttle.dead_zone = 6;

    rc_ch[CH_1] = &g.channel_roll;
    rc_ch[CH_2] = &g.channel_pitch;
    rc_ch[CH_3] = &g.channel_throttle;
    rc_ch[CH_4] = &g.channel_rudder;
    rc_ch[CH_5] = &g.rc_5;
    rc_ch[CH_6] = &g.rc_6;
    rc_ch[CH_7] = &g.rc_7;
    rc_ch[CH_8] = &g.rc_8;

    //set auxiliary ranges
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11);
#else
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
#endif
}

static void init_rc_out()
{
    hal.rcout->enable_ch(CH_1);
    hal.rcout->enable_ch(CH_2);
    hal.rcout->enable_ch(CH_3);
    hal.rcout->enable_ch(CH_4);
    enable_aux_servos();

    // Initialization of servo outputs
    hal.rcout->write(CH_1,   g.channel_roll.radio_trim);
    hal.rcout->write(CH_2,   g.channel_pitch.radio_trim);
    hal.rcout->write(CH_3,   g.channel_throttle.radio_min);
    hal.rcout->write(CH_4,   g.channel_rudder.radio_trim);

    hal.rcout->write(CH_5,   g.rc_5.radio_trim);
    hal.rcout->write(CH_6,   g.rc_6.radio_trim);
    hal.rcout->write(CH_7,   g.rc_7.radio_trim);
    hal.rcout->write(CH_8,   g.rc_8.radio_trim);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    hal.rcout->write(CH_9,   g.rc_9.radio_trim);
    hal.rcout->write(CH_10,  g.rc_10.radio_trim);
    hal.rcout->write(CH_11,  g.rc_11.radio_trim);
#endif
}

static void read_radio()
{
    ch1_temp = hal.rcin->read(CH_ROLL);
    ch2_temp = hal.rcin->read(CH_PITCH);
    uint16_t pwm_roll, pwm_pitch;

    if (g.mix_mode == 0) {
        pwm_roll = ch1_temp;
        pwm_pitch = ch2_temp;
    }else{
        pwm_roll = BOOL_TO_SIGN(g.reverse_elevons) * (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) - BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500;
        pwm_pitch = (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) + BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500;
    }
    
    if (control_mode == TRAINING) {
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        g.channel_roll.set_pwm_no_deadzone(pwm_roll);
        g.channel_pitch.set_pwm_no_deadzone(pwm_pitch);
        g.channel_throttle.set_pwm_no_deadzone(hal.rcin->read(CH_3));
        g.channel_rudder.set_pwm_no_deadzone(hal.rcin->read(CH_4));
    } else {
        g.channel_roll.set_pwm(pwm_roll);
        g.channel_pitch.set_pwm(pwm_pitch);
        g.channel_throttle.set_pwm(hal.rcin->read(CH_3));
        g.channel_rudder.set_pwm(hal.rcin->read(CH_4));
    }

    g.rc_5.set_pwm(hal.rcin->read(CH_5));
    g.rc_6.set_pwm(hal.rcin->read(CH_6));
    g.rc_7.set_pwm(hal.rcin->read(CH_7));
    g.rc_8.set_pwm(hal.rcin->read(CH_8));

    control_failsafe(g.channel_throttle.radio_in);

    g.channel_throttle.servo_out = g.channel_throttle.control_in;

    if (g.throttle_nudge && g.channel_throttle.servo_out > 50) {
        float nudge = (g.channel_throttle.servo_out - 50) * 0.02;
        if (alt_control_airspeed()) {
            airspeed_nudge_cm = (g.flybywire_airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
        } else {
            throttle_nudge = (g.throttle_max - g.throttle_cruise) * nudge;
        }
    } else {
        airspeed_nudge_cm = 0;
        throttle_nudge = 0;
    }

    /*
     *  cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"),
     *                       (int)g.rc_1.control_in,
     *                       (int)g.rc_2.control_in,
     *                       (int)g.rc_3.control_in,
     *                       (int)g.rc_4.control_in);
     */
}

static void control_failsafe(uint16_t pwm)
{
    if(g.throttle_fs_enabled == 0)
        return;

    // Check for failsafe condition based on loss of GCS control
    if (rc_override_active) {
        if(millis() - rc_override_fs_timer > FAILSAFE_SHORT_TIME) {
            ch3_failsafe = true;
        } else {
            ch3_failsafe = false;
        }

        //Check for failsafe and debounce funky reads
    } else if (g.throttle_fs_enabled) {
        if (pwm < (unsigned)g.throttle_fs_value) {
            // we detect a failsafe from radio
            // throttle has dropped below the mark
            failsafeCounter++;
            if (failsafeCounter == 9) {
                gcs_send_text_fmt(PSTR("MSG FS ON %u"), (unsigned)pwm);
            }else if(failsafeCounter == 10) {
                ch3_failsafe = true;
            }else if (failsafeCounter > 10) {
                failsafeCounter = 11;
            }

        }else if(failsafeCounter > 0) {
            // we are no longer in failsafe condition
            // but we need to recover quickly
            failsafeCounter--;
            if (failsafeCounter > 3) {
                failsafeCounter = 3;
            }
            if (failsafeCounter == 1) {
                gcs_send_text_fmt(PSTR("MSG FS OFF %u"), (unsigned)pwm);
            }else if(failsafeCounter == 0) {
                ch3_failsafe = false;
            }else if (failsafeCounter <0) {
                failsafeCounter = -1;
            }
        }
    }
}

static void trim_control_surfaces()
{
    read_radio();
    // Store control surface trim values
    // ---------------------------------
    if(g.mix_mode == 0) {
        g.channel_roll.radio_trim = g.channel_roll.radio_in;
        g.channel_pitch.radio_trim = g.channel_pitch.radio_in;

        // the secondary aileron is trimmed only if it has a
        // corresponding transmitter input channel, which k_aileron
        // doesn't have
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_aileron_with_input);
    } else{
        elevon1_trim = ch1_temp;
        elevon2_trim = ch2_temp;
        //Recompute values here using new values for elevon1_trim and elevon2_trim
        //We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
        uint16_t center                         = 1500;
        g.channel_roll.radio_trim       = center;
        g.channel_pitch.radio_trim      = center;
    }
    g.channel_rudder.radio_trim = g.channel_rudder.radio_in;

    // save to eeprom
    g.channel_roll.save_eeprom();
    g.channel_pitch.save_eeprom();
    g.channel_rudder.save_eeprom();
}

static void trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }

    trim_control_surfaces();
}

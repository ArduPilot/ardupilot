// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

extern RC_Channel* rc_ch[NUM_CHANNELS];

static void default_dead_zones()
{
    g.rc_1.set_dead_zone(60);
    g.rc_2.set_dead_zone(60);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_dead_zone(20);
    g.rc_4.set_dead_zone(30);
#else
    g.rc_3.set_dead_zone(60);
    g.rc_4.set_dead_zone(80);
#endif
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(MAX_INPUT_ROLL_ANGLE);
    g.rc_2.set_angle(MAX_INPUT_PITCH_ANGLE);
#if FRAME_CONFIG == HELI_FRAME
    // we do not want to limit the movment of the heli's swash plate
    g.rc_3.set_range(0, 1000);
#else
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
#endif
    g.rc_4.set_angle(4500);

    // reverse: CW = left
    // normal:  CW = left???

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_ch[CH_1] = &g.rc_1;
    rc_ch[CH_2] = &g.rc_2;
    rc_ch[CH_3] = &g.rc_3;
    rc_ch[CH_4] = &g.rc_4;
    rc_ch[CH_5] = &g.rc_5;
    rc_ch[CH_6] = &g.rc_6;
    rc_ch[CH_7] = &g.rc_7;
    rc_ch[CH_8] = &g.rc_8;

    //set auxiliary ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

#if MOUNT == ENABLED
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
}

static void init_rc_out()
{
    APM_RC.Init( &isr_registry );               // APM Radio initialization

    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);
    motors.set_max_throttle(g.throttle_max);

    for(byte i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // sanity check - prevent unconfigured radios from outputting
    if(g.rc_3.radio_min >= 1300) {
        g.rc_3.radio_min = g.rc_3.radio_in;
    }

    // we are full throttle
    if(g.rc_3.control_in >= (MAXIMUM_THROTTLE - 50)) {
        if(g.esc_calibrate == 0) {
            // we will enter esc_calibrate mode on next reboot
            g.esc_calibrate.set_and_save(1);
            // send miinimum throttle out to ESC
            motors.output_min();
            // display message on console
            cliSerial->printf_P(PSTR("Entering ESC Calibration: please restart APM.\n"));
            // block until we restart
            while(1) {
                delay(200);
                dancing_light();
            }
        }else{
            cliSerial->printf_P(PSTR("ESC Calibration active: passing throttle through to ESCs.\n"));
            // clear esc flag
            g.esc_calibrate.set_and_save(0);
            // block until we restart
            init_esc();
        }
    }else{
        // did we abort the calibration?
        if(g.esc_calibrate == 1)
            g.esc_calibrate.set_and_save(0);

        // send miinimum throttle out to ESC
        output_min();
    }

#if TOY_EDF == ENABLED
    // add access to CH8 and CH6
    APM_RC.enable_out(CH_8);
    APM_RC.enable_out(CH_6);
#endif
}

void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

#define RADIO_FS_TIMEOUT_MS 2000       // 2 seconds
static void read_radio()
{
    if (APM_RC.GetState() == 1) {
        ap_system.new_radio_frame = true;
        g.rc_1.set_pwm(APM_RC.InputCh(CH_1));
        g.rc_2.set_pwm(APM_RC.InputCh(CH_2));

        set_throttle_and_failsafe(APM_RC.InputCh(CH_3));

        g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
        g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
        g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
        g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
        g.rc_8.set_pwm(APM_RC.InputCh(CH_8));

#if FRAME_CONFIG != HELI_FRAME
        // limit our input to 800 so we can still pitch and roll
        g.rc_3.control_in = min(g.rc_3.control_in, MAXIMUM_THROTTLE);
#endif
    }else{
        // turn on throttle failsafe if no update from ppm encoder for 2 seconds
        if ((millis() - APM_RC.get_last_update() >= RADIO_FS_TIMEOUT_MS) && g.throttle_fs_enabled && motors.armed() && !ap.failsafe) {
            set_failsafe(true);
        }
    }
}

#define FS_COUNTER 3
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    static int8_t failsafe_counter = 0;

    // if failsafe not enabled pass through throttle and exit
    if(g.throttle_fs_enabled == 0) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.throttle_fs_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (ap.failsafe || !motors.armed()) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe_counter++;
        if( failsafe_counter >= FS_COUNTER ) {
            failsafe_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe_counter--;
        if( failsafe_counter <= 0 ) {
            failsafe_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (ap.failsafe) {
                set_failsafe(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

static void trim_radio()
{
    for (byte i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}


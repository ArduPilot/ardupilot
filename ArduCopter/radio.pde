// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static int8_t failsafeCounter = 0;              // we wait a second to take over the throttle and send the plane circling


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
    g.rc_1.set_angle(4500);
    g.rc_2.set_angle(4500);
#if FRAME_CONFIG == HELI_FRAME
    // we do not want to limit the movment of the heli's swash plate
    g.rc_3.set_range(0, 1000);
#else
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
#endif
    g.rc_4.set_angle(4500);

    // reverse: CW = left
    // normal:  CW = left???

    g.rc_1.set_type(RC_CHANNEL_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_ANGLE_RAW);

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
            // block until we restart
            while(1) {
                //Serial.println("esc");
                delay(200);
                dancing_light();
            }
        }else{
            //Serial.println("esc init");
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
static void read_radio()
{
    if (APM_RC.GetState() == 1) {
        new_radio_frame = true;
        g.rc_1.set_pwm(APM_RC.InputCh(CH_1));
        g.rc_2.set_pwm(APM_RC.InputCh(CH_2));
        g.rc_3.set_pwm(APM_RC.InputCh(CH_3));
        g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
        g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
        g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
        g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
        g.rc_8.set_pwm(APM_RC.InputCh(CH_8));

#if FRAME_CONFIG != HELI_FRAME
        // limit our input to 800 so we can still pitch and roll
        g.rc_3.control_in = min(g.rc_3.control_in, MAXIMUM_THROTTLE);
#endif

        throttle_failsafe(g.rc_3.radio_in);
    }
}
#define FS_COUNTER 3
static void throttle_failsafe(uint16_t pwm)
{
    // Don't enter Failsafe if not enabled by user
    if(g.throttle_fs_enabled == 0)
        return;

    //check for failsafe and debounce funky reads
    // ------------------------------------------
    if (pwm < (unsigned)g.throttle_fs_value) {
        // we detect a failsafe from radio
        // throttle has dropped below the mark
        failsafeCounter++;
        if (failsafeCounter == FS_COUNTER-1) {
            // called right before trigger
            // do nothing
        }else if(failsafeCounter == FS_COUNTER) {
            // Don't enter Failsafe if we are not armed
            // home distance is in meters
            // This is to prevent accidental RTL
            if(motors.armed() && takeoff_complete) {
                SendDebug("MSG FS ON ");
                SendDebugln(pwm, DEC);
                set_failsafe(true);
            }
        }else if (failsafeCounter > FS_COUNTER) {
            failsafeCounter = FS_COUNTER+1;
        }

    }else if(failsafeCounter > 0) {
        // we are no longer in failsafe condition
        // but we need to recover quickly
        failsafeCounter--;
        if (failsafeCounter > 3) {
            failsafeCounter = 3;
        }
        if (failsafeCounter == 1) {
            SendDebug("MSG FS OFF ");
            SendDebugln(pwm, DEC);
        }else if(failsafeCounter == 0) {
            set_failsafe(false);
        }else if (failsafeCounter <0) {
            failsafeCounter = -1;
        }
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


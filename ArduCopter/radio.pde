// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

static void default_dead_zones()
{
    g.rc_1.set_default_dead_zone(30);
    g.rc_2.set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_default_dead_zone(10);
    g.rc_4.set_default_dead_zone(15);
    g.rc_8.set_default_dead_zone(10);
#else
    g.rc_3.set_default_dead_zone(30);
    g.rc_4.set_default_dead_zone(40);
#endif
    g.rc_6.set_default_dead_zone(0);
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
    g.rc_4.set_angle(4500);

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    //set auxiliary servo ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
static void init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // check if we should enter esc calibration mode
    esc_calibration_startup_check();

    // enable output to motors
    pre_arm_rc_checks();
    if (ap.pre_arm_rc_check) {
        output_min();
    }

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed. Note: this assumes rc_3 is
    // throttle and should really use rcmap.
    hal.rcout->set_esc_scaling(g.rc_3.radio_min, g.rc_3.radio_max);
}

// output_min - enable and output lowest possible value to motors
void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

static void read_radio()
{
    static uint32_t last_update_ms = 0;
    uint32_t tnow_ms = millis();

    if (hal.rcin->new_input()) {
        last_update_ms = tnow_ms;
        ap.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[rcmap.roll()-1]);
        g.rc_2.set_pwm(periods[rcmap.pitch()-1]);

        set_throttle_and_failsafe(periods[rcmap.throttle()-1]);
        set_throttle_zero_flag(g.rc_3.control_in);

        g.rc_4.set_pwm(periods[rcmap.yaw()-1]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);

        // read channels 9 ~ 14
        for (uint8_t i=8; i<RC_MAX_CHANNELS; i++) {
            if (RC_Channel::rc_channel(i) != NULL) {
                RC_Channel::rc_channel(i)->set_pwm(RC_Channel::rc_channel(i)->read());
            }
        }

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }

        // update output on any aux channels, for manual passthru
        RC_Channel_aux::output_ch_all();
    }else{
        uint32_t elapsed = tnow_ms - last_update_ms;
        // turn on throttle failsafe if no update from the RC Radio for 500ms or 2000ms if we are using RC_OVERRIDE
        if (((!failsafe.rc_override_active && (elapsed >= FS_RADIO_TIMEOUT_MS)) || (failsafe.rc_override_active && (elapsed >= FS_RADIO_RC_OVERRIDE_TIMEOUT_MS))) &&
            (g.failsafe_throttle && (ap.rc_receiver_present||motors.armed()) && !failsafe.radio)) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe_radio(true);
        }
    }
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present || motors.armed())) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control_in
static void set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if non-zero throttle immediately set as non-zero
    if (throttle_control > 0) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}

static void trim_radio()
{
    for (uint8_t i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}


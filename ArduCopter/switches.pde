/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CONTROL_SWITCH_COUNTER  20  // 20 iterations at 100hz (i.e. 2/10th of a second) at a new switch position will cause flight mode change
static void read_control_switch()
{
    static uint8_t switch_counter = 0;

    uint8_t switchPosition = readSwitch();

    // has switch moved?
    // ignore flight mode changes if in failsafe
    if (oldSwitchPosition != switchPosition && !failsafe.radio && failsafe.radio_counter == 0) {
        switch_counter++;
        if(switch_counter >= CONTROL_SWITCH_COUNTER) {
            oldSwitchPosition       = switchPosition;
            switch_counter          = 0;

            // set flight mode and simple mode setting
            if (set_mode(flight_modes[switchPosition])) {

                if(g.ch7_option != AUX_SWITCH_SIMPLE_MODE && g.ch8_option != AUX_SWITCH_SIMPLE_MODE && g.ch7_option != AUX_SWITCH_SUPERSIMPLE_MODE && g.ch8_option != AUX_SWITCH_SUPERSIMPLE_MODE) {
                    // set Simple mode using stored paramters from Mission planner
                    // rather than by the control switch
                    if (BIT_IS_SET(g.super_simple, switchPosition)) {
                        set_simple_mode(2);
                    }else{
                        set_simple_mode(BIT_IS_SET(g.simple_modes, switchPosition));
                    }
                }
            }

        }
    }else{
        // reset switch_counter if there's been no change
        // we don't want 10 intermittant blips causing a flight mode change
        switch_counter = 0;
    }
}

static uint8_t readSwitch(void){
    int16_t pulsewidth = g.rc_5.radio_in;   // default for Arducopter

    if (pulsewidth < 1231) return 0;
    if (pulsewidth < 1361) return 1;
    if (pulsewidth < 1491) return 2;
    if (pulsewidth < 1621) return 3;
    if (pulsewidth < 1750) return 4;        // Software Manual
    return 5;                               // Hardware Manual
}

static void reset_control_switch()
{
    oldSwitchPosition = -1;
    read_control_switch();
}

// read_3pos_switch
static uint8_t read_3pos_switch(int16_t radio_in){
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;      // switch is in low position
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;    // switch is in high position
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// read_aux_switches - checks aux switch positions and invokes configured actions
static void read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    // check if ch7 switch has changed position
    switch_position = read_3pos_switch(g.rc_7.radio_in);
    if (ap.CH7_flag != switch_position) {
        // set the CH7 flag
        ap.CH7_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch7_option, ap.CH7_flag);
    }

    // check if Ch8 switch has changed position
    switch_position = read_3pos_switch(g.rc_8.radio_in);
    if (ap.CH8_flag != switch_position) {
        // set the CH8 flag
        ap.CH8_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch8_option, ap.CH8_flag);
    }
}

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
static void init_aux_switches()
{
    // set the CH7 flag
    ap.CH7_flag = read_3pos_switch(g.rc_7.radio_in);
    ap.CH8_flag = read_3pos_switch(g.rc_8.radio_in);

    // init channel 7 options
    switch(g.ch7_option) {
        case AUX_SWITCH_SIMPLE_MODE:
        case AUX_SWITCH_SONAR:
        case AUX_SWITCH_FENCE:
        case AUX_SWITCH_RESETTOARMEDYAW:
        case AUX_SWITCH_SUPERSIMPLE_MODE:
        case AUX_SWITCH_ACRO_TRAINER:
        case AUX_SWITCH_EPM:
        case AUX_SWITCH_SPRAYER:
        case AUX_SWITCH_EKF:
        case AUX_SWITCH_PARACHUTE_ENABLE:
        case AUX_SWITCH_PARACHUTE_3POS:	    // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUX_SWITCH_RETRACT_MOUNT:
        case AUX_SWITCH_MISSIONRESET:
        case AUX_SWITCH_ATTCON_FEEDFWD:
        case AUX_SWITCH_ATTCON_ACCEL_LIM:
        case AUX_SWITCH_RELAY:
            do_aux_switch_function(g.ch7_option, ap.CH7_flag);
            break;
    }

    // init channel 8 option
    switch(g.ch8_option) {
        case AUX_SWITCH_SIMPLE_MODE:
        case AUX_SWITCH_SONAR:
        case AUX_SWITCH_FENCE:
        case AUX_SWITCH_RESETTOARMEDYAW:
        case AUX_SWITCH_SUPERSIMPLE_MODE:
        case AUX_SWITCH_ACRO_TRAINER:
        case AUX_SWITCH_EPM:
        case AUX_SWITCH_SPRAYER:
        case AUX_SWITCH_EKF:
        case AUX_SWITCH_PARACHUTE_ENABLE:
        case AUX_SWITCH_PARACHUTE_3POS:     // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUX_SWITCH_RETRACT_MOUNT:
        case AUX_SWITCH_MISSIONRESET:
        case AUX_SWITCH_ATTCON_FEEDFWD:
        case AUX_SWITCH_ATTCON_ACCEL_LIM:
        case AUX_SWITCH_RELAY:
            do_aux_switch_function(g.ch8_option, ap.CH8_flag);
            break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{
    int8_t tmp_function = ch_function;

    // multi mode check
    if(ch_function == AUX_SWITCH_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
            tmp_function = AUX_SWITCH_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
            tmp_function = AUX_SWITCH_SAVE_WP;
        }else{
            tmp_function = AUX_SWITCH_RTL;
        }
    }

    switch(tmp_function) {
        case AUX_SWITCH_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ch_flag == AUX_SWITCH_HIGH) {
                set_mode(FLIP);
            }
            break;

        case AUX_SWITCH_SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            set_simple_mode(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
            break;

        case AUX_SWITCH_SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            set_simple_mode(ch_flag);
            break;

        case AUX_SWITCH_RTL:
            if (ch_flag == AUX_SWITCH_HIGH) {
                // engage RTL (if not possible we remain in current flight mode)
                set_mode(RTL);
            }else{
                // return to flight mode switch's flight mode if we are currently in RTL
                if (control_mode == RTL) {
                    reset_control_switch();
                }
            }
            break;

        case AUX_SWITCH_SAVE_TRIM:
            if ((ch_flag == AUX_SWITCH_HIGH) && (control_mode <= ACRO) && (g.rc_3.control_in == 0)) {
                save_trim();
            }
            break;

        case AUX_SWITCH_SAVE_WP:
            // save waypoint when switch is brought high
            if (ch_flag == AUX_SWITCH_HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if(control_mode == AUTO || !motors.armed()) {
                    return;
                }

				// do not allow saving the first waypoint with zero throttle
				if((mission.num_commands() == 0) && (g.rc_3.control_in == 0)){
					return;
				}

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if(mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.options = 0;
                    cmd.p1 = 0;
                    cmd.content.location.lat = 0;
                    cmd.content.location.lng = 0;
                    cmd.content.location.alt = max(current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if(mission.add_cmd(cmd)) {
                        // log event
                        Log_Write_Event(DATA_SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = current_loc;

                // if throttle is above zero, create waypoint command
                if(g.rc_3.control_in > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                }else{
					// with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if(mission.add_cmd(cmd)) {
                    // log event
                    Log_Write_Event(DATA_SAVEWP_ADD_WP);
                }
            }
            break;

#if CAMERA == ENABLED
        case AUX_SWITCH_CAMERA_TRIGGER:
            if (ch_flag == AUX_SWITCH_HIGH) {
                do_take_picture();
            }
            break;
#endif

        case AUX_SWITCH_SONAR:
            // enable or disable the sonar
#if CONFIG_SONAR == ENABLED
            if (ch_flag == AUX_SWITCH_HIGH) {
                sonar_enabled = true;
            }else{
                sonar_enabled = false;
            }
#endif
            break;

#if AC_FENCE == ENABLED
        case AUX_SWITCH_FENCE:
            // enable or disable the fence
            if (ch_flag == AUX_SWITCH_HIGH) {
                fence.enable(true);
                Log_Write_Event(DATA_FENCE_ENABLE);
            }else{
                fence.enable(false);
                Log_Write_Event(DATA_FENCE_DISABLE);
            }
            break;
#endif
        // To-Do: add back support for this feature
        //case AUX_SWITCH_RESETTOARMEDYAW:
        //    if (ch_flag == AUX_SWITCH_HIGH) {
        //        set_yaw_mode(YAW_RESETTOARMEDYAW);
        //    }else{
        //        set_yaw_mode(YAW_HOLD);
        //    }
        //    break;

        case AUX_SWITCH_ACRO_TRAINER:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    g.acro_trainer = ACRO_TRAINER_DISABLED;
                    Log_Write_Event(DATA_ACRO_TRAINER_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    g.acro_trainer = ACRO_TRAINER_LEVELING;
                    Log_Write_Event(DATA_ACRO_TRAINER_LEVELING);
                    break;
                case AUX_SWITCH_HIGH:
                    g.acro_trainer = ACRO_TRAINER_LIMITED;
                    Log_Write_Event(DATA_ACRO_TRAINER_LIMITED);
                    break;
            }
            break;
#if EPM_ENABLED == ENABLED
        case AUX_SWITCH_EPM:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    epm.release();
                    Log_Write_Event(DATA_EPM_RELEASE);
                    break;
                case AUX_SWITCH_HIGH:
                    epm.grab();
                    Log_Write_Event(DATA_EPM_GRAB);
                    break;
            }
            break;
#endif
#if SPRAYER == ENABLED
        case AUX_SWITCH_SPRAYER:
            sprayer.enable(ch_flag == AUX_SWITCH_HIGH);
            // if we are disarmed the pilot must want to test the pump
            sprayer.test_pump((ch_flag == AUX_SWITCH_HIGH) && !motors.armed());
            break;
#endif

        case AUX_SWITCH_AUTO:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(AUTO);
            }else{
                // return to flight mode switch's flight mode if we are currently in AUTO
                if (control_mode == AUTO) {
                    reset_control_switch();
                }
            }
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUX_SWITCH_AUTOTUNE:
            // turn on auto tuner
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                case AUX_SWITCH_MIDDLE:
                    // stop the autotune and return to original gains
                    autotune_stop();
                    // restore flight mode based on flight mode switch position
                    if (control_mode == AUTOTUNE) {
                        reset_control_switch();
                    }
                    break;
                case AUX_SWITCH_HIGH:
                    // start an autotuning session
                    autotune_start();
                    break;
            }
            break;
#endif

        case AUX_SWITCH_LAND:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(LAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in LAND
                if (control_mode == LAND) {
                    reset_control_switch();
                }
            }
            break;

#if AP_AHRS_NAVEKF_AVAILABLE
    case AUX_SWITCH_EKF:
        ahrs.set_ekf_use(ch_flag==AUX_SWITCH_HIGH);
        break;
#endif

#if PARACHUTE == ENABLED
    case AUX_SWITCH_PARACHUTE_ENABLE:
        // Parachute enable/disable
        parachute.enabled(ch_flag == AUX_SWITCH_HIGH);
        break;

    case AUX_SWITCH_PARACHUTE_RELEASE:
        if (ch_flag == AUX_SWITCH_HIGH) {
            parachute_manual_release();
        }
        break;

    case AUX_SWITCH_PARACHUTE_3POS:
        // Parachute disable, enable, release with 3 position switch
        switch (ch_flag) {
            case AUX_SWITCH_LOW:
                parachute.enabled(false);
                Log_Write_Event(DATA_PARACHUTE_DISABLED);
                break;
            case AUX_SWITCH_MIDDLE:
                parachute.enabled(true);
                Log_Write_Event(DATA_PARACHUTE_ENABLED);
                break;
            case AUX_SWITCH_HIGH:
                parachute.enabled(true);
                parachute_manual_release();
                break;
        }
        break;
#endif

    case AUX_SWITCH_MISSIONRESET:
        if (ch_flag == AUX_SWITCH_HIGH) {
            mission.reset();
        }
        break;

    case AUX_SWITCH_ATTCON_FEEDFWD:
        // enable or disable feed forward
        attitude_control.bf_feedforward(ch_flag == AUX_SWITCH_HIGH);
        break;

    case AUX_SWITCH_ATTCON_ACCEL_LIM:
        // enable or disable accel limiting by restoring defaults
        attitude_control.accel_limiting(ch_flag == AUX_SWITCH_HIGH);
        break;
        
#if MOUNT == ENABLE
    case AUX_SWITCH_RETRACT_MOUNT:
        switch (ch_flag) {
            case AUX_SWITCH_HIGH:
                camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
                break;
            case AUX_SWITCH_LOW:
                camera_mount.set_mode_to_default();
                break;
        }
        break;
#endif

    case AUX_SWITCH_RELAY:
        ServoRelayEvents.do_set_relay(0, ch_flag == AUX_SWITCH_HIGH);
        break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
static void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)g.rc_1.control_in/100.0f);
    float pitch_trim = ToRad((float)g.rc_2.control_in/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    Log_Write_Event(DATA_SAVE_TRIM);
    gcs_send_text_P(SEVERITY_HIGH, PSTR("Trim saved"));
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        AP_Notify::flags.save_trim = true;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)g.rc_1.control_in / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)g.rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
            AP_Notify::flags.save_trim = false;
        }
    }
}


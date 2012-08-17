/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void read_control_switch()
{
    static bool switch_debouncer = false;

    byte switchPosition = readSwitch();

    if (oldSwitchPosition != switchPosition) {
        if(switch_debouncer) {
            oldSwitchPosition       = switchPosition;
            switch_debouncer        = false;

            set_mode(flight_modes[switchPosition]);

            if(g.ch7_option != CH7_SIMPLE_MODE) {
                // set Simple mode using stored paramters from Mission planner
                // rather than by the control switch
                do_simple = (g.simple_modes & (1 << switchPosition));
            }
        }else{
            switch_debouncer        = true;
        }
    }
}

static byte readSwitch(void){
    int16_t pulsewidth = g.rc_5.radio_in;                       // default for Arducopter

    if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
    if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
    if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
    if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;              // Software Manual
    if (pulsewidth >= 1750) return 5;                                                           // Hardware Manual
    return 0;
}

static void reset_control_switch()
{
    oldSwitchPosition = -1;
    read_control_switch();
}

// read at 10 hz
// set this to your trainer switch
static void read_trim_switch()
{
    int8_t option;

    if(g.ch7_option == CH7_MULTI_MODE) {
        if (g.rc_6.radio_in < CH_6_PWM_TRIGGER_LOW) {
            option = CH7_FLIP;
        }else if (g.rc_6.radio_in > CH_6_PWM_TRIGGER_HIGH) {
            option = CH7_SAVE_WP;
        }else{
            option = CH7_RTL;
        }
    }else{
        option = g.ch7_option;
    }

    if(option == CH7_SIMPLE_MODE) {
        do_simple = (g.rc_7.radio_in > CH_7_PWM_TRIGGER);

    }else if (option == CH7_FLIP) {
        if (CH7_flag == false && g.rc_7.radio_in > CH_7_PWM_TRIGGER) {
            CH7_flag = true;

            // don't flip if we accidentally engaged flip, but didn't notice and tried to takeoff
            if(g.rc_3.control_in != 0 && takeoff_complete) {
                init_flip();
            }
        }
        if (CH7_flag == true && g.rc_7.control_in < 800) {
            CH7_flag = false;
        }

    }else if (option == CH7_RTL) {
        if (CH7_flag == false && g.rc_7.radio_in > CH_7_PWM_TRIGGER) {
            CH7_flag = true;
            set_mode(RTL);
        }

        if (CH7_flag == true && g.rc_7.control_in < 800) {
            CH7_flag = false;
            if (control_mode == RTL || control_mode == LOITER) {
                reset_control_switch();
            }
        }

    }else if (option == CH7_SAVE_WP) {
        if (g.rc_7.radio_in > CH_7_PWM_TRIGGER) {        // switch is engaged
            CH7_flag = true;

        }else{         // switch is disengaged
            if(CH7_flag) {
                CH7_flag = false;

                if(control_mode == AUTO) {
                    // reset the mission
                    CH7_wp_index = 0;
                    g.command_total.set_and_save(1);
                    set_mode(RTL);
                    return;
                }

                if(CH7_wp_index == 0) {
                    // this is our first WP, let's save WP 1 as a takeoff
                    // increment index to WP index of 1 (home is stored at 0)
                    CH7_wp_index = 1;

                    Location temp   = home;
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    temp.id = MAV_CMD_NAV_TAKEOFF;
                    temp.alt = current_loc.alt;

                    // save command:
                    // we use the current altitude to be the target for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    // If we are above the altitude, we will skip the command.
                    set_cmd_with_index(temp, CH7_wp_index);
                }

                // increment index
                CH7_wp_index++;

                // set the next_WP (home is stored at 0)
                // max out at 100 since I think we need to stay under the EEPROM limit
                CH7_wp_index = constrain(CH7_wp_index, 1, 100);

                if(g.rc_3.control_in > 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    current_loc.id = MAV_CMD_NAV_WAYPOINT;
                }else{
                    // set our location ID to 21, MAV_CMD_NAV_LAND
                    current_loc.id = MAV_CMD_NAV_LAND;
                }

                // save command
                set_cmd_with_index(current_loc, CH7_wp_index);

                copter_leds_nav_blink = 10;                     // Cause the CopterLEDs to blink twice to indicate saved waypoint

                // 0 = home
                // 1 = takeoff
                // 2 = WP 2
                // 3 = command total
            }
        }
    }else if (option == CH7_AUTO_TRIM) {
        if (g.rc_7.radio_in > CH_7_PWM_TRIGGER) {
            auto_level_counter = 10;
        }
    }
}

static void auto_trim()
{
    if(auto_level_counter > 0) {
        //g.rc_1.dead_zone = 60;		// 60 = .6 degrees
        //g.rc_2.dead_zone = 60;

        auto_level_counter--;
        trim_accel();
        led_mode = AUTO_TRIM_LEDS;
        do_simple = false;

        if(auto_level_counter == 1) {
            //g.rc_1.dead_zone = 0;		// 60 = .6 degrees
            //g.rc_2.dead_zone = 0;
            led_mode = NORMAL_LEDS;
            clear_leds();
            imu.save();

            reset_control_switch();

            //Serial.println("Done");
            auto_level_counter = 0;
        }
    }
}


/*
 *  How this works:
 *  Level Example:
 *  A_off: -14.00, -20.59, -30.80
 *
 *  Right roll Example:
 *  A_off: -6.73, 89.05, -46.02
 *
 *  Left Roll Example:
 *  A_off: -18.11, -160.31, -56.42
 *
 *  Pitch Forward:
 *  A_off: -127.00, -22.16, -50.09
 *
 *  Pitch Back:
 *  A_off: 201.95, -24.00, -88.56
 */

static void trim_accel()
{
    reset_stability_I();

    float trim_roll  = (float)g.rc_1.control_in / 30000.0;
    float trim_pitch = (float)g.rc_2.control_in / 30000.0;

    trim_roll       = constrain(trim_roll, -1.5, 1.5);
    trim_pitch      = constrain(trim_pitch, -1.5, 1.5);

    if(g.rc_1.control_in > 200) {    // Roll Right
        imu.ay(imu.ay() - trim_roll);
    }else if (g.rc_1.control_in < -200) {
        imu.ay(imu.ay() - trim_roll);
    }

    if(g.rc_2.control_in > 200) {    // Pitch Back
        imu.ax(imu.ax() + trim_pitch);
    }else if (g.rc_2.control_in < -200) {
        imu.ax(imu.ax() + trim_pitch);
    }

    /*
     *  Serial.printf_P(PSTR("r:%1.2f  %1.2f \t| p:%1.2f  %1.2f\n"),
     *                                               trim_roll,
     *                                               (float)imu.ay(),
     *                                               trim_pitch,
     *                                               (float)imu.ax());
     *  //*/
}


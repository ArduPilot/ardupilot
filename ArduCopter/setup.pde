// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor      (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compassmot        (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_frame             (uint8_t argc, const Menu::arg *argv);
 #if FRAME_CONFIG == HELI_FRAME
static int8_t   setup_gyro              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_heli              (uint8_t argc, const Menu::arg *argv);
 #endif
static int8_t   setup_accel             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_optflow           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_tune              (uint8_t argc, const Menu::arg *argv);


// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"accel",                       setup_accel_scale},
    {"battery",                     setup_batt_monitor},
    {"compass",                     setup_compass},
    {"compassmot",                  setup_compassmot},
    {"declination",                 setup_declination},
    {"erase",                       setup_erase},
    {"frame",                       setup_frame},
 #if FRAME_CONFIG == HELI_FRAME
    {"gyro",                        setup_gyro},
    {"heli",                        setup_heli},
 #endif
    {"level",                       setup_accel},
    {"modes",                       setup_flightmodes},
    {"optflow",                     setup_optflow},
    {"radio",                       setup_radio},
    {"range",                       setup_range},
    {"reset",                       setup_factory},
    {"set",                         setup_set},
    {"show",                        setup_show},
    {"sonar",                       setup_sonar},
    {"tune",                        setup_tune}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));

    if(g.rc_1.radio_min >= 1300) {
        delay(1000);
        cliSerial->printf_P(PSTR("\n!Warning, radio not configured!"));
        delay(1000);
        cliSerial->printf_P(PSTR("\n Type 'radio' now.\n\n"));
    }

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    float trim_roll, trim_pitch;

    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
    report_ins();
    return(0);
}

static int8_t
setup_batt_monitor(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.battery_monitoring.set_and_save(0);

    } else if(argv[1].i > 0 && argv[1].i <= 4) {
        g.battery_monitoring.set_and_save(argv[1].i);

    } else {
        cliSerial->printf_P(PSTR("\nOp: off, 3-4"));
    }

    report_batt_monitor();
    return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.compass_enabled.set_and_save(true);
        init_compass();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        Vector3f mag_offsets(0,0,0);
        compass.set_offsets(mag_offsets);
        compass.save_offsets();
        g.compass_enabled.set_and_save(false);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on,off]\n"));
        report_compass();
        return 0;
    }

    g.compass_enabled.save();
    report_compass();
    return 0;
}

// setup_compassmot - sets compass's motor interference parameters
static int8_t
setup_compassmot(uint8_t argc, const Menu::arg *argv)
{
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base;              // compass vector when throttle is zero
    Vector3f motor_impact;              // impact of motors on compass vector
    Vector3f motor_impact_scaled;       // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation;        // final compensation to be stored to eeprom
    float    throttle_pct;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    interference_pct;          // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint8_t  print_counter = 49;
    bool     updated = false;           // have we updated the compensation vector at least once

    // default compensation type to use current if possible
    if( g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT ) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }else{
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if user wants throttle compensation
    if( !strcmp_P(argv[1].str, PSTR("t")) || !strcmp_P(argv[1].str, PSTR("T")) ) {
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if user wants current compensation
    if( !strcmp_P(argv[1].str, PSTR("c")) || !strcmp_P(argv[1].str, PSTR("C")) ) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }

    // check if radio is calibration
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        cliSerial->print_P(PSTR("radio not calibrated, exiting"));
        return 0;
    }

    // check compass is enabled
    if( !g.compass_enabled ) {
        cliSerial->print_P(PSTR("compass disabled, exiting"));
        return 0;
    }

    // check if we have a current monitor
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT && g.battery_monitoring != BATT_MONITOR_VOLTAGE_AND_CURRENT ) {
        cliSerial->print_P(PSTR("current monitor disabled, exiting"));
        return 0;
    }

    // initialise compass
    init_compass();

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    compass.set_motor_compensation(Vector3f(0,0,0));

    // print warning that motors will spin
    // ask user to raise throttle
    // inform how to stop test
    cliSerial->print_P(PSTR("This records the impact on the compass of running the motors.  The motors will spin!\nHold throttle low, then raise to mid for 5 sec, then quickly back to low.\nAt any time you may press any key to exit.\n\nmeasuring compass vs "));

    // inform what type of compensation we are attempting
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT ) {
        cliSerial->print_P(PSTR("CURRENT\n"));
    }else{
        cliSerial->print_P(PSTR("THROTTLE\n"));
    }

    // clear out user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // disable throttle and battery failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_battery_enabled = false;

    // read radio
    read_radio();

    // exit immediately if throttle is not zero
    if( g.rc_3.control_in != 0 ) {
        cliSerial->print_P(PSTR("throttle not zero, exiting\n"));
        return 0;
    }

    // get some initial compass readings
    last_run_time = millis();
    while( millis() - last_run_time < 2000 ) {
        compass.accumulate();
    }
    compass.read();

    // exit immediately if the compass is not healthy
    if( !compass.healthy ) {
        cliSerial->print_P(PSTR("compass not healthy, exiting\n"));
        return 0;
    }

    // store initial x,y,z compass values
    compass_base.x = compass.mag_x;
    compass_base.y = compass.mag_y;
    compass_base.z = compass.mag_z;

    // initialise motor compensation
    motor_compensation = Vector3f(0,0,0);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // enable motors and pass through throttle
    init_rc_out();
    output_min();
    motors.armed(true);

    // initialise run time
    last_run_time = millis();

    // main run while there is no user input and the compass is healthy
    while(!cliSerial->available() && compass.healthy) {

        // 50hz loop
        if( millis() - last_run_time > 20 ) {
            last_run_time = millis();

            // read radio input
            read_radio();

            // pass through throttle to motors
            motors.throttle_pass_through();

            // read some compass values
            compass.read();

            // read current
            read_battery();

            // calculate scaling for throttle
            throttle_pct = (float)g.rc_3.control_in / 1000.0f;
            throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);

            // if throttle is zero, update base x,y,z values
            if( throttle_pct == 0.0f ) {
                compass_base.x = compass_base.x * 0.99f + (float)compass.mag_x * 0.01f;
                compass_base.y = compass_base.y * 0.99f + (float)compass.mag_y * 0.01f;
                compass_base.z = compass_base.z * 0.99f + (float)compass.mag_z * 0.01f;

                // causing printing to happen as soon as throttle is lifted
                print_counter = 49;
            }else{

                // calculate diff from compass base and scale with throttle
                motor_impact.x = compass.mag_x - compass_base.x;
                motor_impact.y = compass.mag_y - compass_base.y;
                motor_impact.z = compass.mag_z - compass_base.z;

                // throttle based compensation
                if( comp_type == AP_COMPASS_MOT_COMP_THROTTLE ) {
                    // scale by throttle
                    motor_impact_scaled = motor_impact / throttle_pct;

                    // adjust the motor compensation to negate the impact
                    motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                    updated = true;
                }else{
                    // current based compensation if more than 3amps being drawn
                    motor_impact_scaled = motor_impact / current_amps1;

                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if( current_amps1 >= 3.0f ) {
                        motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                        updated = true;
                    }
                }

                // record maximum throttle and current
                throttle_pct_max = max(throttle_pct_max, throttle_pct);
                current_amps_max = max(current_amps_max, current_amps1);

                // display output at 1hz if throttle is above zero
                print_counter++;
                if(print_counter >= 50) {
                    print_counter = 0;
                    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)current_amps1, (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);
                }
            }
        }else{
            // grab some compass values
            compass.accumulate();
        }
    }

    // stop motors
    motors.output_min();
    motors.armed(false);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // print one more time so the last thing printed matches what appears in the report_compass
    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)current_amps1, (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);

    // set and save motor compensation
    if( updated ) {
        compass.motor_compensation_type(comp_type);
        compass.set_motor_compensation(motor_compensation);
        compass.save_motor_compensation();

        // calculate and display interference compensation at full throttle as % of total mag field
        if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            // interference is impact@fullthrottle / mag field * 100
            interference_pct = motor_compensation.length() / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
        }else{
            // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
            interference_pct = motor_compensation.length() * (current_amps_max/throttle_pct_max) / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
        }
        cliSerial->printf_P(PSTR("\nInterference at full throttle is %d%% of mag field\n\n"),(int)interference_pct);
    }else{
        // compensation vector never updated, report failure
        cliSerial->printf_P(PSTR("Failed! Compensation disabled.  Did you forget to raise the throttle high enough?"));
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // display new motor offsets and save
    report_compass();

    return 0;
}

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
    compass.set_declination(radians(argv[1].f));
    report_compass();
    return 0;
}

static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    zero_eeprom();
    return 0;
}

static int8_t
setup_frame(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("x"))) {
        g.frame_orientation.set_and_save(X_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("p"))) {
        g.frame_orientation.set_and_save(PLUS_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("+"))) {
        g.frame_orientation.set_and_save(PLUS_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("v"))) {
        g.frame_orientation.set_and_save(V_FRAME);
    }else{
        cliSerial->printf_P(PSTR("\nOp:[x,+,v]\n"));
        report_frame();
        return 0;
    }

    report_frame();
    return 0;
}

#if FRAME_CONFIG == HELI_FRAME
// setup for external tail gyro (for heli only)
static int8_t
setup_gyro(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        motors.ext_gyro_enabled.set_and_save(true);

        // optionally capture the gain
        if( argc >= 2 && argv[2].i >= 1000 && argv[2].i <= 2000 ) {
            motors.ext_gyro_gain = argv[2].i;
            motors.ext_gyro_gain.save();
        }

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        motors.ext_gyro_enabled.set_and_save(false);

        // capture gain if user simply provides a number
    } else if( argv[1].i >= 1000 && argv[1].i <= 2000 ) {
        motors.ext_gyro_enabled.set_and_save(true);
        motors.ext_gyro_gain = argv[1].i;
        motors.ext_gyro_gain.save();

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off] gain\n"));
    }

    report_gyro();
    return 0;
}

// Perform heli setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_heli(uint8_t argc, const Menu::arg *argv)
{

    uint8_t active_servo = 0;
    int16_t value = 0;
    int16_t temp;
    int16_t state = 0;       // 0 = set rev+pos, 1 = capture min/max
    int16_t max_roll=0, max_pitch=0, min_collective=0, max_collective=0, min_tail=0, max_tail=0;

    // initialise swash plate
    motors.init_swash();

    // source swash plate movements directly from radio
    motors.servo_manual = true;

    // display initial settings
    report_heli();

    // display help
    cliSerial->printf_P(PSTR("Instructions:"));
    print_divider();
    cliSerial->printf_P(PSTR("\td\t\tdisplay settings\n"));
    cliSerial->printf_P(PSTR("\t1~4\t\tselect servo\n"));
    cliSerial->printf_P(PSTR("\ta or z\t\tmove mid up/down\n"));
    cliSerial->printf_P(PSTR("\tc\t\tset coll when blade pitch zero\n"));
    cliSerial->printf_P(PSTR("\tm\t\tset roll, pitch, coll min/max\n"));
    cliSerial->printf_P(PSTR("\tp<angle>\tset pos (i.e. p0 = front, p90 = right)\n"));
    cliSerial->printf_P(PSTR("\tr\t\treverse servo\n"));
    cliSerial->printf_P(PSTR("\tu a|d\t\tupdate rate (a=analog servo, d=digital)\n"));
    cliSerial->printf_P(PSTR("\tt<angle>\tset trim (-500 ~ 500)\n"));
    cliSerial->printf_P(PSTR("\tx\t\texit & save\n"));

    // start capturing
    while( value != 'x' ) {

        // read radio although we don't use it yet
        read_radio();

        // allow swash plate to move
        motors.output_armed();

        // record min/max
        if( state == 1 ) {
            if( abs(g.rc_1.control_in) > max_roll )
                max_roll = abs(g.rc_1.control_in);
            if( abs(g.rc_2.control_in) > max_pitch )
                max_pitch = abs(g.rc_2.control_in);
            if( g.rc_3.radio_out < min_collective )
                min_collective = g.rc_3.radio_out;
            if( g.rc_3.radio_out > max_collective )
                max_collective = g.rc_3.radio_out;
            min_tail = min(g.rc_4.radio_out, min_tail);
            max_tail = max(g.rc_4.radio_out, max_tail);
        }

        if( cliSerial->available() ) {
            value = cliSerial->read();

            // process the user's input
            switch( value ) {
            case '1':
                active_servo = CH_1;
                break;
            case '2':
                active_servo = CH_2;
                break;
            case '3':
                active_servo = CH_3;
                break;
            case '4':
                active_servo = CH_4;
                break;
            case 'a':
            case 'A':
                heli_get_servo(active_servo)->radio_trim += 10;
                break;
            case 'c':
            case 'C':
                if( g.rc_3.radio_out >= 900 && g.rc_3.radio_out <= 2100 ) {
                    motors.collective_mid = g.rc_3.radio_out;
                    cliSerial->printf_P(PSTR("Collective when blade pitch at zero: %d\n"),(int)motors.collective_mid);
                }
                break;
            case 'd':
            case 'D':
                // display settings
                report_heli();
                break;
            case 'm':
            case 'M':
                if( state == 0 ) {
                    state = 1;                          // switch to capture min/max mode
                    cliSerial->printf_P(PSTR("Move coll, roll, pitch and tail to extremes, press 'm' when done\n"));

                    // reset servo ranges
                    motors.roll_max = motors.pitch_max = 4500;
                    motors.collective_min = 1000;
                    motors.collective_max = 2000;
                    motors._servo_4->radio_min = 1000;
                    motors._servo_4->radio_max = 2000;

                    // set sensible values in temp variables
                    max_roll = abs(g.rc_1.control_in);
                    max_pitch = abs(g.rc_2.control_in);
                    min_collective = 2000;
                    max_collective = 1000;
                    min_tail = max_tail = abs(g.rc_4.radio_out);
                }else{
                    state = 0;                          // switch back to normal mode
                    // double check values aren't totally terrible
                    if( max_roll <= 1000 || max_pitch <= 1000 || (max_collective - min_collective < 200) || (max_tail - min_tail < 200) || min_tail < 1000 || max_tail > 2000 )
                        cliSerial->printf_P(PSTR("Invalid min/max captured roll:%d,  pitch:%d,  collective min: %d max: %d,  tail min:%d max:%d\n"),max_roll,max_pitch,min_collective,max_collective,min_tail,max_tail);
                    else{
                        motors.roll_max = max_roll;
                        motors.pitch_max = max_pitch;
                        motors.collective_min = min_collective;
                        motors.collective_max = max_collective;
                        motors._servo_4->radio_min = min_tail;
                        motors._servo_4->radio_max = max_tail;

                        // reinitialise swash
                        motors.init_swash();

                        // display settings
                        report_heli();
                    }
                }
                break;
            case 'p':
            case 'P':
                temp = read_num_from_serial();
                if( temp >= -360 && temp <= 360 ) {
                    if( active_servo == CH_1 )
                        motors.servo1_pos = temp;
                    if( active_servo == CH_2 )
                        motors.servo2_pos = temp;
                    if( active_servo == CH_3 )
                        motors.servo3_pos = temp;
                    motors.init_swash();
                    cliSerial->printf_P(PSTR("Servo %d\t\tpos:%d\n"),active_servo+1, temp);
                }
                break;
            case 'r':
            case 'R':
                heli_get_servo(active_servo)->set_reverse(!heli_get_servo(active_servo)->get_reverse());
                break;
            case 't':
            case 'T':
                temp = read_num_from_serial();
                if( temp > 1000 )
                    temp -= 1500;
                if( temp > -500 && temp < 500 ) {
                    heli_get_servo(active_servo)->radio_trim = 1500 + temp;
                    motors.init_swash();
                    cliSerial->printf_P(PSTR("Servo %d\t\ttrim:%d\n"),active_servo+1, 1500 + temp);
                }
                break;
            case 'u':
            case 'U':
                temp = 0;
                // delay up to 2 seconds for servo type from user
                while( !cliSerial->available() && temp < 20 ) {
                    temp++;
                    delay(100);
                }
                if( cliSerial->available() ) {
                    value = cliSerial->read();
                    if( value == 'a' || value == 'A' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        cliSerial->printf_P(PSTR("Analog Servo %dhz\n"),(int)g.rc_speed);
                    }
                    if( value == 'd' || value == 'D' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        cliSerial->printf_P(PSTR("Digital Servo %dhz\n"),(int)g.rc_speed);
                    }
                }
                break;
            case 'z':
            case 'Z':
                heli_get_servo(active_servo)->radio_trim -= 10;
                break;
            }
        }

        delay(20);
    }

    // display final settings
    report_heli();

    // save to eeprom
    motors._servo_1->save_eeprom();
    motors._servo_2->save_eeprom();
    motors._servo_3->save_eeprom();
    motors._servo_4->save_eeprom();
    motors.servo1_pos.save();
    motors.servo2_pos.save();
    motors.servo3_pos.save();
    motors.roll_max.save();
    motors.pitch_max.save();
    motors.collective_min.save();
    motors.collective_max.save();
    motors.collective_mid.save();

    // return swash plate movements to attitude controller
    motors.servo_manual = false;

    return(0);
}
#endif // FRAME_CONFIG == HELI

static int8_t
setup_accel(uint8_t argc, const Menu::arg *argv)
{
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ins.init_accel();
    ahrs.set_trim(Vector3f(0,0,0));     // clear out saved trim
    report_ins();
    return(0);
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    uint8_t _switchPosition = 0;
    uint8_t _oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMode switch to edit, aileron: select modes, rudder: Simple on/off\n"));
    print_hit_enter();

    while(1) {
        delay(20);
        read_radio();
        _switchPosition = readSwitch();


        // look for control switch change
        if (_oldSwitchPosition != _switchPosition) {

            mode = flight_modes[_switchPosition];
            mode = constrain_int16(mode, 0, NUM_MODES-1);

            // update the user
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));

            // Remember switch position
            _oldSwitchPosition = _switchPosition;
        }

        // look for stick input
        if (abs(g.rc_1.control_in) > 3000) {
            mode++;
            if(mode >= NUM_MODES)
                mode = 0;

            // save new mode
            flight_modes[_switchPosition] = mode;

            // print new mode
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in > 3000) {
            g.simple_modes |= (1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in < -3000) {
            g.simple_modes &= ~(1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));
            delay(500);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            for (mode = 0; mode < 6; mode++)
                flight_modes[mode].save();

            g.simple_modes.save();
            print_done();
            report_flight_modes();
            return (0);
        }
    }
}

static int8_t
setup_optflow(uint8_t argc, const Menu::arg *argv)
{
 #if OPTFLOW == ENABLED
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.optflow_enabled = true;
        init_optflow();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.optflow_enabled = false;

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off]\n"));
        report_optflow();
        return 0;
    }

    g.optflow_enabled.save();
    report_optflow();
 #endif     // OPTFLOW == ENABLED
    return 0;
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("\n\nRadio Setup:"));
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }

    if(g.rc_1.radio_in < 500) {
        while(1) {
            //cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.rc_1.radio_min = g.rc_1.radio_in;
    g.rc_2.radio_min = g.rc_2.radio_in;
    g.rc_3.radio_min = g.rc_3.radio_in;
    g.rc_4.radio_min = g.rc_4.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.rc_1.radio_max = g.rc_1.radio_in;
    g.rc_2.radio_max = g.rc_2.radio_in;
    g.rc_3.radio_max = g.rc_3.radio_in;
    g.rc_4.radio_max = g.rc_4.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.rc_1.radio_trim = g.rc_1.radio_in;
    g.rc_2.radio_trim = g.rc_2.radio_in;
    g.rc_4.radio_trim = g.rc_4.radio_in;
    // 3 is not trimed
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;


    cliSerial->printf_P(PSTR("\nMove all controls to extremes. Enter to save: "));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        g.rc_1.update_min_max();
        g.rc_2.update_min_max();
        g.rc_3.update_min_max();
        g.rc_4.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(cliSerial->available() > 0) {
            delay(20);
            while (cliSerial->read() != -1); /* flush */

            g.rc_1.save_eeprom();
            g.rc_2.save_eeprom();
            g.rc_3.save_eeprom();
            g.rc_4.save_eeprom();
            g.rc_5.save_eeprom();
            g.rc_6.save_eeprom();
            g.rc_7.save_eeprom();
            g.rc_8.save_eeprom();

            print_done();
            break;
        }
    }
    report_radio();
    return(0);
}

static int8_t
setup_range(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

    g.radio_tuning_low.set_and_save(argv[1].i);
    g.radio_tuning_high.set_and_save(argv[2].i);
    report_tuning();
    return 0;
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nReboot APM"));

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        //int32 and float don't need bounds checking, just use the value provoded by Menu::arg
        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
            break;
    }

    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, cliSerial);
        return 0;
    }

    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_sonar();
    report_flight_modes();
    report_ins();
    report_compass();
    report_optflow();

 #if FRAME_CONFIG == HELI_FRAME
    report_heli();
    report_gyro();
 #endif

    AP_Param::show_all(cliSerial);

    return(0);
}

static int8_t
setup_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.sonar_enabled.set_and_save(true);

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.sonar_enabled.set_and_save(false);

    } else if (argc > 1 && (argv[1].i >= 0 && argv[1].i <= 3)) {
        g.sonar_enabled.set_and_save(true);      // if you set the sonar type, surely you want it on
        g.sonar_type.set_and_save(argv[1].i);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

static int8_t
setup_tune(uint8_t argc, const Menu::arg *argv)
{
    g.radio_tuning.set_and_save(argv[1].i);
    report_tuning();
    return 0;
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if(g.battery_monitoring == BATT_MONITOR_DISABLED) print_enabled(false);
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_ONLY) cliSerial->printf_P(PSTR("volts"));
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT) cliSerial->printf_P(PSTR("volts and cur"));
    print_blanks(2);
}

static void report_sonar()
{
    cliSerial->printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    cliSerial->printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
    print_blanks(2);
}

static void report_frame()
{
    cliSerial->printf_P(PSTR("Frame\n"));
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    cliSerial->printf_P(PSTR("Quad frame\n"));
 #elif FRAME_CONFIG == TRI_FRAME
    cliSerial->printf_P(PSTR("TRI frame\n"));
 #elif FRAME_CONFIG == HEXA_FRAME
    cliSerial->printf_P(PSTR("Hexa frame\n"));
 #elif FRAME_CONFIG == Y6_FRAME
    cliSerial->printf_P(PSTR("Y6 frame\n"));
 #elif FRAME_CONFIG == OCTA_FRAME
    cliSerial->printf_P(PSTR("Octa frame\n"));
 #elif FRAME_CONFIG == HELI_FRAME
    cliSerial->printf_P(PSTR("Heli frame\n"));
 #endif

 #if FRAME_CONFIG != HELI_FRAME
    if(g.frame_orientation == X_FRAME)
        cliSerial->printf_P(PSTR("X mode\n"));
    else if(g.frame_orientation == PLUS_FRAME)
        cliSerial->printf_P(PSTR("+ mode\n"));
    else if(g.frame_orientation == V_FRAME)
        cliSerial->printf_P(PSTR("V mode\n"));
 #endif

    print_blanks(2);
}

static void report_radio()
{
    cliSerial->printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_ins()
{
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

static void report_compass()
{
    cliSerial->printf_P(PSTR("Compass\n"));
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    cliSerial->printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f\n"),
                    offsets.x,
                    offsets.y,
                    offsets.z);

    // motor compensation
    cliSerial->print_P(PSTR("Motor Comp: "));
    if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->print_P(PSTR("Off\n"));
    }else{
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->print_P(PSTR("Throttle"));
        }
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->print_P(PSTR("Current"));
        }
        Vector3f motor_compensation = compass.get_motor_compensation();
        cliSerial->printf_P(PSTR("\nComp Vec: %4.2f, %4.2f, %4.2f\n"),
                        motor_compensation.x,
                        motor_compensation.y,
                        motor_compensation.z);
    }
    print_blanks(1);
}

static void report_flight_modes()
{
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], BIT_IS_SET(g.simple_modes, i));
    }
    print_blanks(2);
}

void report_optflow()
{
 #if OPTFLOW == ENABLED
    cliSerial->printf_P(PSTR("OptFlow\n"));
    print_divider();

    print_enabled(g.optflow_enabled);

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

 #if FRAME_CONFIG == HELI_FRAME
static void report_heli()
{
    cliSerial->printf_P(PSTR("Heli\n"));
    print_divider();

    // main servo settings
    cliSerial->printf_P(PSTR("Servo \tpos \tmin \tmax \trev\n"));
    cliSerial->printf_P(PSTR("1:\t%d \t%d \t%d \t%d\n"),(int)motors.servo1_pos, (int)motors._servo_1->radio_min, (int)motors._servo_1->radio_max, (int)motors._servo_1->get_reverse());
    cliSerial->printf_P(PSTR("2:\t%d \t%d \t%d \t%d\n"),(int)motors.servo2_pos, (int)motors._servo_2->radio_min, (int)motors._servo_2->radio_max, (int)motors._servo_2->get_reverse());
    cliSerial->printf_P(PSTR("3:\t%d \t%d \t%d \t%d\n"),(int)motors.servo3_pos, (int)motors._servo_3->radio_min, (int)motors._servo_3->radio_max, (int)motors._servo_3->get_reverse());
    cliSerial->printf_P(PSTR("tail:\t\t%d \t%d \t%d\n"), (int)motors._servo_4->radio_min, (int)motors._servo_4->radio_max, (int)motors._servo_4->get_reverse());

    cliSerial->printf_P(PSTR("roll max: \t%d\n"), (int)motors.roll_max);
    cliSerial->printf_P(PSTR("pitch max: \t%d\n"), (int)motors.pitch_max);
    cliSerial->printf_P(PSTR("coll min:\t%d\t mid:%d\t max:%d\n"),(int)motors.collective_min, (int)motors.collective_mid, (int)motors.collective_max);

    // calculate and print servo rate
    cliSerial->printf_P(PSTR("servo rate:\t%d hz\n"),(int)g.rc_speed);

    print_blanks(2);
}

static void report_gyro()
{

    cliSerial->printf_P(PSTR("Gyro:\n"));
    print_divider();

    print_enabled( motors.ext_gyro_enabled );
    if( motors.ext_gyro_enabled )
        cliSerial->printf_P(PSTR("gain: %d"),(int)motors.ext_gyro_gain);

    print_blanks(2);
}

 #endif // FRAME_CONFIG == HELI_FRAME

/***************************************************************************/
// CLI utilities
/***************************************************************************/

/*static void
 *  print_PID(PI * pid)
 *  {
 *       cliSerial->printf_P(PSTR("P: %4.2f, I:%4.2f, IMAX:%ld\n"),
 *                                               pid->kP(),
 *                                               pid->kI(),
 *                                               (long)pid->imax());
 *  }
 */

static void
print_radio_values()
{
    cliSerial->printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(cliSerial, m);
    cliSerial->printf_P(PSTR(",\t\tSimple: "));
    if(b)
        cliSerial->printf_P(PSTR("ON\n"));
    else
        cliSerial->printf_P(PSTR("OFF\n"));
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved\n"));
}


static void zero_eeprom(void)
{
    cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));

    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        hal.storage->write_byte(i, 0);
    }

    cliSerial->printf_P(PSTR("done\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW
}

static void
print_gyro_offsets(void)
{
    Vector3f gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

 #if FRAME_CONFIG == HELI_FRAME

static RC_Channel *
heli_get_servo(int16_t servo_num){
    if( servo_num == CH_1 )
        return motors._servo_1;
    if( servo_num == CH_2 )
        return motors._servo_2;
    if( servo_num == CH_3 )
        return motors._servo_3;
    if( servo_num == CH_4 )
        return motors._servo_4;
    return NULL;
}

// Used to read integer values from the serial port
static int16_t read_num_from_serial() {
    uint8_t index = 0;
    uint8_t timeout = 0;
    char data[5] = "";

    do {
        if (cliSerial->available() == 0) {
            delay(10);
            timeout++;
        }else{
            data[index] = cliSerial->read();
            timeout = 0;
            index++;
        }
    } while (timeout < 5 && index < 5);

    return atoi(data);
}
 #endif

#endif // CLI_ENABLED

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print_P(PSTR("-"));
    }
    cliSerial->println();
}

static void print_enabled(bool b)
{
    if(b)
        cliSerial->print_P(PSTR("en"));
    else
        cliSerial->print_P(PSTR("dis"));
    cliSerial->print_P(PSTR("abled\n"));
}


static void
init_esc()
{
    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // we enable the motors directly here instead of calling output_min because output_min would send a low signal to the ESC and disrupt the calibration process
    motors.enable();
    motors.armed(true);
    while(1) {
        read_radio();
        delay(100);
        AP_Notify::flags.esc_calibration = true;
        motors.throttle_pass_through();
    }
}

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}


static void report_tuning()
{
    cliSerial->printf_P(PSTR("\nTUNE:\n"));
    print_divider();
    if (g.radio_tuning == 0) {
        print_enabled(g.radio_tuning.get());
    }else{
        float low  = (float)g.radio_tuning_low.get() / 1000;
        float high = (float)g.radio_tuning_high.get() / 1000;
        cliSerial->printf_P(PSTR(" %d, Low:%1.4f, High:%1.4f\n"),(int)g.radio_tuning.get(), low, high);
    }
    print_blanks(2);
}

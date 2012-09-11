// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_motors                    (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel_scale               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_frame                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory                   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass                   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_tune                              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range                             (uint8_t argc, const Menu::arg *argv);
//static int8_t	setup_mag_offset		(uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_optflow                   (uint8_t argc, const Menu::arg *argv);


 #if FRAME_CONFIG == HELI_FRAME
static int8_t   setup_heli                              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_gyro                              (uint8_t argc, const Menu::arg *argv);
 #endif

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"erase",                       setup_erase},
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"frame",                       setup_frame},
    {"motors",                      setup_motors},
    {"level",                       setup_accel},
    {"accel",                       setup_accel_scale},
    {"modes",                       setup_flightmodes},
    {"battery",                     setup_batt_monitor},
    {"sonar",                       setup_sonar},
    {"compass",                     setup_compass},
    {"tune",                        setup_tune},
    {"range",                       setup_range},
//	{"offsets",			setup_mag_offset},
    {"declination",         setup_declination},
    {"optflow",                     setup_optflow},
 #if FRAME_CONFIG == HELI_FRAME
    {"heli",                        setup_heli},
    {"gyro",                        setup_gyro},
 #endif
    {"show",                        setup_show}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    Serial.printf_P(PSTR("Setup Mode\n\n\n"));
    //"\n"
    //"IMPORTANT: if you have not previously set this system up, use the\n"
    //"'reset' command to initialize the EEPROM to sensible default values\n"
    //"and then the 'radio' command to configure for your radio.\n"
    //"\n"));

    if(g.rc_1.radio_min >= 1300) {
        delay(1000);
        Serial.printf_P(PSTR("\n!Warning, radio not configured!"));
        delay(1000);
        Serial.printf_P(PSTR("\n Type 'radio' now.\n\n"));
    }

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_sonar();
    //report_gains();
    //report_xtrack();
    //report_throttle();
    report_flight_modes();
    report_imu();
    report_compass();
    report_optflow();

 #if FRAME_CONFIG == HELI_FRAME
    report_heli();
    report_gyro();
 #endif

    AP_Param::show_all();

    return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    Serial.printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = Serial.read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    Serial.printf_P(PSTR("\nReboot APM"));

    delay(1000);
    //default_gains();

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
    Serial.println("\n\nRadio Setup:");
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }

    if(g.rc_1.radio_in < 500) {
        while(1) {
            //Serial.printf_P(PSTR("\nNo radio; Check connectors."));
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


    Serial.printf_P(PSTR("\nMove all controls to extremes. Enter to save: "));
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

        if(Serial.available() > 0) {
            delay(20);
            Serial.flush();

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
setup_motors(uint8_t argc, const Menu::arg *argv)
{
    Serial.printf_P(PSTR(
                        "Now connect the main lipo and follow the instruction on the wiki for your frame setup.\n"
                        "For security remember to disconnect the main lipo after the test, then hit any key to exit.\n"
                        "Any key to exit.\n"));
    while(1) {
        delay(20);
        read_radio();
        motors.output_test();
        if(Serial.available() > 0) {
            g.esc_calibrate.set_and_save(0);
            return(0);
        }
    }
}

static int8_t
setup_accel(uint8_t argc, const Menu::arg *argv)
{
    imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
    imu.init_accel(delay, flash_leds);
    print_accel_offsets();
    report_imu();
    return(0);
}


static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
 #if CONFIG_ADC == ENABLED && HIL_MODE == HIL_MODE_DISABLED
    int8_t accel_num;
    float accel_avg = 0;

    if (!strcmp_P(argv[1].str, PSTR("x"))) {
        accel_num = 4;
    }else if (!strcmp_P(argv[1].str, PSTR("y"))) {
        accel_num = 5;
    }else if (!strcmp_P(argv[1].str, PSTR("z"))) {
        accel_num = 6;
    }else{
        Serial.printf_P(PSTR("x, y, or z\n"));
        return 0;
    }
    print_hit_enter();
    Serial.printf_P(PSTR("ADC\n"));

    adc.Init(&timer_scheduler);                           // APM ADC library initialization

    int16_t low, high;

    delay(1000);
    accel_avg = adc.Ch(accel_num);
    low = high = accel_avg;

    while(1) {
        delay(50);
        accel_avg = accel_avg * .95 + adc.Ch(accel_num) * .05;

        if(accel_avg > high)
            high = ceil(accel_avg);

        if(accel_avg < low)
            low = floor(accel_avg);

        Serial.printf_P(PSTR("%1.2f, %d, %d\n"), accel_avg, low, high);

        if(Serial.available() > 0) {
            if(wait_for_yes()) {
                if(accel_num == 4) {
                    ins._x_high     = high;
                    ins._x_low              = low;
                    ins._x_high.save();
                    ins._x_low.save();
                }else if(accel_num == 5) {
                    ins._y_high     = high;
                    ins._y_low              = low;
                    ins._y_high.save();
                    ins._y_low.save();
                }else{
                    ins._z_high     = high;
                    ins._z_low              = low;
                    ins._z_high.save();
                    ins._z_low.save();
                }
                print_done();
            }
            return (0);
        }
    }
 #else
    return 0;
 #endif        // CONFIG_ADC
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
        Serial.printf_P(PSTR("\nOp:[x,+,v]\n"));
        report_frame();
        return 0;
    }

    report_frame();
    return 0;
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    byte _switchPosition = 0;
    byte _oldSwitchPosition = 0;
    int8_t mode = 0;

    Serial.printf_P(PSTR("\nMode switch to edit, aileron: select modes, rudder: Simple on/off\n"));
    print_hit_enter();

    while(1) {
        delay(20);
        read_radio();
        _switchPosition = readSwitch();


        // look for control switch change
        if (_oldSwitchPosition != _switchPosition) {

            mode = flight_modes[_switchPosition];
            mode = constrain(mode, 0, NUM_MODES-1);

            // update the user
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));

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
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in > 3000) {
            g.simple_modes |= (1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in < -3000) {
            g.simple_modes &= ~(1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, (g.simple_modes & (1<<_switchPosition)));
            delay(500);
        }

        // escape hatch
        if(Serial.available() > 0) {
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
setup_declination(uint8_t argc, const Menu::arg *argv)
{
    compass.set_declination(radians(argv[1].f));
    report_compass();
    return 0;
}

static int8_t
setup_tune(uint8_t argc, const Menu::arg *argv)
{
    g.radio_tuning.set_and_save(argv[1].i);
    //g.radio_tuning_high.set_and_save(1000);
    //g.radio_tuning_low.set_and_save(0);
    report_tuning();
    return 0;
}

static int8_t
setup_range(uint8_t argc, const Menu::arg *argv)
{
    Serial.printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

    g.radio_tuning_low.set_and_save(argv[1].i);
    g.radio_tuning_high.set_and_save(argv[2].i);
    report_tuning();
    return 0;
}



static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    zero_eeprom();
    return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.compass_enabled.set_and_save(true);
        init_compass();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        clear_offsets();
        g.compass_enabled.set_and_save(false);

    }else{
        Serial.printf_P(PSTR("\nOp:[on,off]\n"));
        report_compass();
        return 0;
    }

    g.compass_enabled.save();
    report_compass();
    return 0;
}

static int8_t
setup_batt_monitor(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.battery_monitoring.set_and_save(0);

    } else if(argv[1].i > 0 && argv[1].i <= 4) {
        g.battery_monitoring.set_and_save(argv[1].i);

    } else {
        Serial.printf_P(PSTR("\nOp: off, 3-4"));
    }

    report_batt_monitor();
    return 0;
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
        Serial.printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

 #if FRAME_CONFIG == HELI_FRAME

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
    Serial.printf_P(PSTR("Instructions:"));
    print_divider();
    Serial.printf_P(PSTR("\td\t\tdisplay settings\n"));
    Serial.printf_P(PSTR("\t1~4\t\tselect servo\n"));
    Serial.printf_P(PSTR("\ta or z\t\tmove mid up/down\n"));
    Serial.printf_P(PSTR("\tc\t\tset coll when blade pitch zero\n"));
    Serial.printf_P(PSTR("\tm\t\tset roll, pitch, coll min/max\n"));
    Serial.printf_P(PSTR("\tp<angle>\tset pos (i.e. p0 = front, p90 = right)\n"));
    Serial.printf_P(PSTR("\tr\t\treverse servo\n"));
    Serial.printf_P(PSTR("\tu a|d\t\tupdate rate (a=analog servo, d=digital)\n"));
    Serial.printf_P(PSTR("\tt<angle>\tset trim (-500 ~ 500)\n"));
    Serial.printf_P(PSTR("\tx\t\texit & save\n"));

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

        if( Serial.available() ) {
            value = Serial.read();

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
                    Serial.printf_P(PSTR("Collective when blade pitch at zero: %d\n"),(int)motors.collective_mid);
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
                    Serial.printf_P(PSTR("Move coll, roll, pitch and tail to extremes, press 'm' when done\n"));

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
                        Serial.printf_P(PSTR("Invalid min/max captured roll:%d,  pitch:%d,  collective min: %d max: %d,  tail min:%d max:%d\n"),max_roll,max_pitch,min_collective,max_collective,min_tail,max_tail);
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
                    Serial.printf_P(PSTR("Servo %d\t\tpos:%d\n"),active_servo+1, temp);
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
                    Serial.printf_P(PSTR("Servo %d\t\ttrim:%d\n"),active_servo+1, 1500 + temp);
                }
                break;
            case 'u':
            case 'U':
                temp = 0;
                // delay up to 2 seconds for servo type from user
                while( !Serial.available() && temp < 20 ) {
                    temp++;
                    delay(100);
                }
                if( Serial.available() ) {
                    value = Serial.read();
                    if( value == 'a' || value == 'A' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        Serial.printf_P(PSTR("Analog Servo %dhz\n"),(int)g.rc_speed);
                    }
                    if( value == 'd' || value == 'D' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        Serial.printf_P(PSTR("Digital Servo %dhz\n"),(int)g.rc_speed);
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
        Serial.printf_P(PSTR("\nOp:[on, off] gain\n"));
    }

    report_gyro();
    return 0;
}

 #endif // FRAME_CONFIG == HELI

static void clear_offsets()
{
    Vector3f _offsets(0.0,0.0,0.0);
    compass.set_offsets(_offsets);
    compass.save_offsets();
}

/*static int8_t
 *  setup_mag_offset(uint8_t argc, const Menu::arg *argv)
 *  {
 *       Vector3f _offsets;
 *
 *       if (!strcmp_P(argv[1].str, PSTR("c"))) {
 *               clear_offsets();
 *               report_compass();
 *               return (0);
 *       }
 *
 *       print_hit_enter();
 *       init_compass();
 *
 *       int16_t _min[3] = {0,0,0};
 *       int16_t _max[3] = {0,0,0};
 *
 *       compass.read();
 *
 *       while(1){
 *               delay(50);
 *       float heading;
 *
 *               compass.read();
 *               heading = compass.calculate_heading(0,0);	// roll = 0, pitch = 0
 *
 *               if(compass.mag_x < _min[0])	_min[0] = compass.mag_x;
 *               if(compass.mag_y < _min[1])	_min[1] = compass.mag_y;
 *               if(compass.mag_z < _min[2])	_min[2] = compass.mag_z;
 *
 *               // capture max
 *               if(compass.mag_x > _max[0])	_max[0] = compass.mag_x;
 *               if(compass.mag_y > _max[1])	_max[1] = compass.mag_y;
 *               if(compass.mag_z > _max[2])	_max[2] = compass.mag_z;
 *
 *               // calculate offsets
 *               _offsets.x = (float)(_max[0] + _min[0]) / -2;
 *               _offsets.y = (float)(_max[1] + _min[1]) / -2;
 *               _offsets.z = (float)(_max[2] + _min[2]) / -2;
 *
 *               // display all to user
 *               Serial.printf_P(PSTR("Heading: %u, \t (%d, %d, %d), (%4.4f, %4.4f, %4.4f)\n"),
 *
 *                               (uint16_t)(wrap_360(ToDeg(heading) * 100)) /100,
 *
 *                               compass.mag_x,
 *                               compass.mag_y,
 *                               compass.mag_z,
 *
 *                               _offsets.x,
 *                               _offsets.y,
 *                               _offsets.z);
 *
 *               if(Serial.available() > 1){
 *                       compass.set_offsets(_offsets);
 *                       //compass.set_offsets(mag_offset_x, mag_offset_y, mag_offset_z);
 *                       report_compass();
 *                       return 0;
 *               }
 *       }
 *       return 0;
 *  }
 */

static int8_t
setup_optflow(uint8_t argc, const Menu::arg *argv)
{
 #ifdef OPTFLOW_ENABLED
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.optflow_enabled = true;
        init_optflow();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.optflow_enabled = false;

    }else{
        Serial.printf_P(PSTR("\nOp:[on, off]\n"));
        report_optflow();
        return 0;
    }

    g.optflow_enabled.save();
    report_optflow();
 #endif
    return 0;
}



/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    Serial.printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if(g.battery_monitoring == 0) print_enabled(false);
    if(g.battery_monitoring == 3) Serial.printf_P(PSTR("volts"));
    if(g.battery_monitoring == 4) Serial.printf_P(PSTR("volts and cur"));
    print_blanks(2);
}

static void report_wp(byte index = 255)
{
    if(index == 255) {
        for(byte i = 0; i < g.command_total; i++) {
            struct Location temp = get_cmd_with_index(i);
            print_wp(&temp, i);
        }
    }else{
        struct Location temp = get_cmd_with_index(index);
        print_wp(&temp, index);
    }
}

static void report_sonar()
{
    Serial.printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    Serial.printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
    print_blanks(2);
}

static void report_frame()
{
    Serial.printf_P(PSTR("Frame\n"));
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    Serial.printf_P(PSTR("Quad frame\n"));
 #elif FRAME_CONFIG == TRI_FRAME
    Serial.printf_P(PSTR("TRI frame\n"));
 #elif FRAME_CONFIG == HEXA_FRAME
    Serial.printf_P(PSTR("Hexa frame\n"));
 #elif FRAME_CONFIG == Y6_FRAME
    Serial.printf_P(PSTR("Y6 frame\n"));
 #elif FRAME_CONFIG == OCTA_FRAME
    Serial.printf_P(PSTR("Octa frame\n"));
 #elif FRAME_CONFIG == HELI_FRAME
    Serial.printf_P(PSTR("Heli frame\n"));
 #endif

 #if FRAME_CONFIG != HELI_FRAME
    if(g.frame_orientation == X_FRAME)
        Serial.printf_P(PSTR("X mode\n"));
    else if(g.frame_orientation == PLUS_FRAME)
        Serial.printf_P(PSTR("+ mode\n"));
    else if(g.frame_orientation == V_FRAME)
        Serial.printf_P(PSTR("V mode\n"));
 #endif

    print_blanks(2);
}

static void report_radio()
{
    Serial.printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_imu()
{
    Serial.printf_P(PSTR("IMU\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets();
    print_blanks(2);
}

static void report_compass()
{
    Serial.printf_P(PSTR("Compass\n"));
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    Serial.printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    Serial.printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f"),
                    offsets.x,
                    offsets.y,
                    offsets.z);
    print_blanks(2);
}

static void report_flight_modes()
{
    Serial.printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], (g.simple_modes & (1<<i)));
    }
    print_blanks(2);
}

void report_optflow()
{
 #ifdef OPTFLOW_ENABLED
    Serial.printf_P(PSTR("OptFlow\n"));
    print_divider();

    print_enabled(g.optflow_enabled);

    // field of view
    //Serial.printf_P(PSTR("FOV: %4.0f\n"),
    //						degrees(g.optflow_fov));

    print_blanks(2);
 #endif
}

 #if FRAME_CONFIG == HELI_FRAME
static void report_heli()
{
    Serial.printf_P(PSTR("Heli\n"));
    print_divider();

    // main servo settings
    Serial.printf_P(PSTR("Servo \tpos \tmin \tmax \trev\n"));
    Serial.printf_P(PSTR("1:\t%d \t%d \t%d \t%d\n"),(int)motors.servo1_pos, (int)motors._servo_1->radio_min, (int)motors._servo_1->radio_max, (int)motors._servo_1->get_reverse());
    Serial.printf_P(PSTR("2:\t%d \t%d \t%d \t%d\n"),(int)motors.servo2_pos, (int)motors._servo_2->radio_min, (int)motors._servo_2->radio_max, (int)motors._servo_2->get_reverse());
    Serial.printf_P(PSTR("3:\t%d \t%d \t%d \t%d\n"),(int)motors.servo3_pos, (int)motors._servo_3->radio_min, (int)motors._servo_3->radio_max, (int)motors._servo_3->get_reverse());
    Serial.printf_P(PSTR("tail:\t\t%d \t%d \t%d\n"), (int)motors._servo_4->radio_min, (int)motors._servo_4->radio_max, (int)motors._servo_4->get_reverse());

    Serial.printf_P(PSTR("roll max: \t%d\n"), (int)motors.roll_max);
    Serial.printf_P(PSTR("pitch max: \t%d\n"), (int)motors.pitch_max);
    Serial.printf_P(PSTR("coll min:\t%d\t mid:%d\t max:%d\n"),(int)motors.collective_min, (int)motors.collective_mid, (int)motors.collective_max);

    // calculate and print servo rate
    Serial.printf_P(PSTR("servo rate:\t%d hz\n"),(int)g.rc_speed);

    print_blanks(2);
}

static void report_gyro()
{

    Serial.printf_P(PSTR("Gyro:\n"));
    print_divider();

    print_enabled( motors.ext_gyro_enabled );
    if( motors.ext_gyro_enabled )
        Serial.printf_P(PSTR("gain: %d"),(int)motors.ext_gyro_gain);

    print_blanks(2);
}

 #endif // FRAME_CONFIG == HELI_FRAME

/***************************************************************************/
// CLI utilities
/***************************************************************************/

/*static void
 *  print_PID(PI * pid)
 *  {
 *       Serial.printf_P(PSTR("P: %4.2f, I:%4.2f, IMAX:%ld\n"),
 *                                               pid->kP(),
 *                                               pid->kI(),
 *                                               (long)pid->imax());
 *  }
 */

static void
print_radio_values()
{
    Serial.printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    Serial.printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    Serial.printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    Serial.printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    Serial.printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    Serial.printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    Serial.printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    //Serial.printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(byte p, byte m, bool b)
{
    Serial.printf_P(PSTR("Pos %d:\t"),p);
    Serial.print(flight_mode_strings[m]);
    Serial.printf_P(PSTR(",\t\tSimple: "));
    if(b)
        Serial.printf_P(PSTR("ON\n"));
    else
        Serial.printf_P(PSTR("OFF\n"));
}

static void
print_done()
{
    Serial.printf_P(PSTR("\nSaved\n"));
}


static void zero_eeprom(void)
{
    byte b = 0;

    Serial.printf_P(PSTR("\nErasing EEPROM\n"));

    for (int16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        eeprom_write_byte((uint8_t *) i, b);
    }

    Serial.printf_P(PSTR("done\n"));
}

static void
print_accel_offsets(void)
{
    Serial.printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)imu.ax(),                                    // Pitch
                    (float)imu.ay(),                                    // Roll
                    (float)imu.az());                                   // YAW
}

static void
print_gyro_offsets(void)
{
    Serial.printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)imu.gx(),
                    (float)imu.gy(),
                    (float)imu.gz());
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
    byte index = 0;
    byte timeout = 0;
    char data[5] = "";

    do {
        if (Serial.available() == 0) {
            delay(10);
            timeout++;
        }else{
            data[index] = Serial.read();
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
        Serial.println("");
    }
}


static bool
wait_for_yes()
{
    int c;
    Serial.flush();
    Serial.printf_P(PSTR("Y to save\n"));

    do {
        c = Serial.read();
    } while (-1 == c);

    if (('y' == c) || ('Y' == c))
        return true;
    else
        return false;
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        Serial.print("-");
    }
    Serial.println("");
}

static void print_enabled(boolean b)
{
    if(b)
        Serial.printf_P(PSTR("en"));
    else
        Serial.printf_P(PSTR("dis"));
    Serial.printf_P(PSTR("abled\n"));
}


static void
init_esc()
{
    motors.enable();
    motors.armed(true);
    while(1) {
        read_radio();
        delay(100);
        dancing_light();
        motors.throttle_pass_through();
    }
}

static void print_wp(struct Location *cmd, byte index)
{
    float t1 = (float)cmd->lat / t7;
    float t2 = (float)cmd->lng / t7;

    Serial.printf_P(PSTR("cmd#: %d id:%d op:%d p1:%d p2:%ld p3:%4.7f p4:%4.7f \n"),
                    (int)index,
                    (int)cmd->id,
                    (int)cmd->options,
                    (int)cmd->p1,
                    (long)cmd->alt,
                    t1,
                    t2);
}

static void report_gps()
{
    Serial.printf_P(PSTR("\nGPS\n"));
    print_divider();
    print_enabled(GPS_enabled);
    print_blanks(2);
}

static void report_version()
{
    Serial.printf_P(PSTR("FW Ver: %d\n"),(int)g.format_version.get());
    print_divider();
    print_blanks(2);
}


static void report_tuning()
{
    Serial.printf_P(PSTR("\nTUNE:\n"));
    print_divider();
    if (g.radio_tuning == 0) {
        print_enabled(g.radio_tuning.get());
    }else{
        float low  = (float)g.radio_tuning_low.get() / 1000;
        float high = (float)g.radio_tuning_high.get() / 1000;
        Serial.printf_P(PSTR(" %d, Low:%1.4f, High:%1.4f\n"),(int)g.radio_tuning.get(), low, high);
    }
    print_blanks(2);
}

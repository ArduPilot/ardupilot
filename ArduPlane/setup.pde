// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show                              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory                   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_level                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase                             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass                   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor              (uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"modes",                       setup_flightmodes},
    {"level",                       setup_level},
    {"compass",                     setup_compass},
    {"declination",         setup_declination},
    {"battery",                     setup_batt_monitor},
    {"show",                        setup_show},
    {"erase",                       setup_erase},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    Serial.printf_P(PSTR("Setup Mode\n"
                         "\n"
                         "IMPORTANT: if you have not previously set this system up, use the\n"
                         "'reset' command to initialize the EEPROM to sensible default values\n"
                         "and then the 'radio' command to configure for your radio.\n"
                         "\n"));

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

    report_radio();
    report_batt_monitor();
    report_gains();
    report_xtrack();
    report_throttle();
    report_flight_modes();
    report_imu();
    report_compass();

    Serial.printf_P(PSTR("Raw Values\n"));
    print_divider();

    AP_Param::show_all();

    return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int c;

    Serial.printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));

    do {
        c = Serial.read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    AP_Param::erase_all();
    Serial.printf_P(PSTR("\nFACTORY RESET complete - please reset APM to continue"));

    //default_flight_modes();   // This will not work here.  Replacement code located in init_ardupilot()

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
    Serial.printf_P(PSTR("\n\nRadio Setup:\n"));
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }


    if(g.channel_roll.radio_in < 500) {
        while(1) {
            Serial.printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.channel_roll.radio_min                = g.channel_roll.radio_in;
    g.channel_pitch.radio_min               = g.channel_pitch.radio_in;
    g.channel_throttle.radio_min    = g.channel_throttle.radio_in;
    g.channel_rudder.radio_min              = g.channel_rudder.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.channel_roll.radio_max                = g.channel_roll.radio_in;
    g.channel_pitch.radio_max               = g.channel_pitch.radio_in;
    g.channel_throttle.radio_max    = g.channel_throttle.radio_in;
    g.channel_rudder.radio_max              = g.channel_rudder.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.channel_roll.radio_trim               = g.channel_roll.radio_in;
    g.channel_pitch.radio_trim              = g.channel_pitch.radio_in;
    g.channel_rudder.radio_trim     = g.channel_rudder.radio_in;
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;

    Serial.printf_P(PSTR("\nMove all controls to each extreme. Hit Enter to save: \n"));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        g.channel_roll.update_min_max();
        g.channel_pitch.update_min_max();
        g.channel_throttle.update_min_max();
        g.channel_rudder.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(Serial.available() > 0) {
            Serial.flush();
            g.channel_roll.save_eeprom();
            g.channel_pitch.save_eeprom();
            g.channel_throttle.save_eeprom();
            g.channel_rudder.save_eeprom();
            g.rc_5.save_eeprom();
            g.rc_6.save_eeprom();
            g.rc_7.save_eeprom();
            g.rc_8.save_eeprom();
            print_done();
            break;
        }
    }
    trim_radio();
    report_radio();
    return(0);
}


static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    byte switchPosition, mode = 0;

    Serial.printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
    print_hit_enter();
    trim_radio();

    while(1) {
        delay(20);
        read_radio();
        switchPosition = readSwitch();


        // look for control switch change
        if (oldSwitchPosition != switchPosition) {
            // force position 5 to MANUAL
            if (switchPosition > 4) {
                flight_modes[switchPosition] = MANUAL;
            }
            // update our current mode
            mode = flight_modes[switchPosition];

            // update the user
            print_switch(switchPosition, mode);

            // Remember switch position
            oldSwitchPosition = switchPosition;
        }

        // look for stick input
        int16_t radioInputSwitch = radio_input_switch();

        if (radioInputSwitch != 0) {

            mode += radioInputSwitch;

            while (
                mode != MANUAL &&
                mode != CIRCLE &&
                mode != STABILIZE &&
                mode != FLY_BY_WIRE_A &&
                mode != FLY_BY_WIRE_B &&
                mode != AUTO &&
                mode != RTL &&
                mode != LOITER)
            {
                if (mode < MANUAL)
                    mode = LOITER;
                else if (mode >LOITER)
                    mode = MANUAL;
                else
                    mode += radioInputSwitch;
            }

            // Override position 5
            if(switchPosition > 4)
                mode = MANUAL;

            // save new mode
            flight_modes[switchPosition] = mode;

            // print new mode
            print_switch(switchPosition, mode);
        }

        // escape hatch
        if(Serial.available() > 0) {
            // save changes
            for (mode=0; mode<6; mode++)
                flight_modes[mode].save();
            report_flight_modes();
            print_done();
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
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    int c;

    Serial.printf_P(PSTR("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: "));

    do {
        c = Serial.read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    zero_eeprom();
    return 0;
}

static int8_t
setup_level(uint8_t argc, const Menu::arg *argv)
{
    startup_IMU_ground(true);
    return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        compass.set_orientation(MAG_ORIENTATION);       // set compass's orientation on aircraft
        if (!compass.init()) {
            Serial.println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            g.compass_enabled = true;
        }
    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.compass_enabled = false;

    } else if (!strcmp_P(argv[1].str, PSTR("reset"))) {
        compass.set_offsets(0,0,0);

    } else {
        Serial.printf_P(PSTR("\nOptions:[on,off,reset]\n"));
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
    if(argv[1].i >= 0 && argv[1].i <= 4) {
        g.battery_monitoring.set_and_save(argv[1].i);

    } else {
        Serial.printf_P(PSTR("\nOptions: 3-4"));
    }

    report_batt_monitor();
    return 0;
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Batt Mointor\n"));
    print_divider();
    if(g.battery_monitoring == 0) Serial.printf_P(PSTR("Batt monitoring disabled"));
    if(g.battery_monitoring == 3) Serial.printf_P(PSTR("Monitoring batt volts"));
    if(g.battery_monitoring == 4) Serial.printf_P(PSTR("Monitoring volts and current"));
    print_blanks(2);
}
static void report_radio()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_gains()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Gains\n"));
    print_divider();

#if APM_CONTROL == DISABLED
	Serial.printf_P(PSTR("servo roll:\n"));
	print_PID(&g.pidServoRoll);

    Serial.printf_P(PSTR("servo pitch:\n"));
    print_PID(&g.pidServoPitch);

	Serial.printf_P(PSTR("servo rudder:\n"));
	print_PID(&g.pidServoRudder);
#endif

    Serial.printf_P(PSTR("nav roll:\n"));
    print_PID(&g.pidNavRoll);

    Serial.printf_P(PSTR("nav pitch airspeed:\n"));
    print_PID(&g.pidNavPitchAirspeed);

    Serial.printf_P(PSTR("energry throttle:\n"));
    print_PID(&g.pidTeThrottle);

    Serial.printf_P(PSTR("nav pitch alt:\n"));
    print_PID(&g.pidNavPitchAltitude);

    print_blanks(2);
}

static void report_xtrack()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Crosstrack\n"));
    print_divider();
    // radio
    Serial.printf_P(PSTR("XTRACK: %4.2f\n"
                         "XTRACK angle: %d\n"),
                    (float)g.crosstrack_gain,
                    (int)g.crosstrack_entry_angle);
    print_blanks(2);
}

static void report_throttle()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Throttle\n"));
    print_divider();

    Serial.printf_P(PSTR("min: %d\n"
                         "max: %d\n"
                         "cruise: %d\n"
                         "failsafe_enabled: %d\n"
                         "failsafe_value: %d\n"),
                    (int)g.throttle_min,
                    (int)g.throttle_max,
                    (int)g.throttle_cruise,
                    (int)g.throttle_fs_enabled,
                    (int)g.throttle_fs_value);
    print_blanks(2);
}

static void report_imu()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("IMU\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets();
    print_blanks(2);
}

static void report_compass()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Compass: "));

    switch (compass.product_id) {
    case AP_COMPASS_TYPE_HMC5883L:
        Serial.println_P(PSTR("HMC5883L"));
        break;
    case AP_COMPASS_TYPE_HMC5843:
        Serial.println_P(PSTR("HMC5843"));
        break;
    case AP_COMPASS_TYPE_HIL:
        Serial.println_P(PSTR("HIL"));
        break;
    default:
        Serial.println_P(PSTR("??"));
        break;
    }

    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    Serial.printf_P(PSTR("Mag Declination: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    Serial.printf_P(PSTR("Mag offsets: %4.4f, %4.4f, %4.4f\n"),
                    offsets.x,
                    offsets.y,
                    offsets.z);
    print_blanks(2);
}

static void report_flight_modes()
{
    //print_blanks(2);
    Serial.printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i]);
    }
    print_blanks(2);
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

static void
print_PID(PID * pid)
{
    Serial.printf_P(PSTR("P: %4.3f, I:%4.3f, D:%4.3f, IMAX:%ld\n"),
                    pid->kP(),
                    pid->kI(),
                    pid->kD(),
                    (long)pid->imax());
}

static void
print_radio_values()
{
    Serial.printf_P(PSTR("CH1: %d | %d | %d\n"), (int)g.channel_roll.radio_min, (int)g.channel_roll.radio_trim, (int)g.channel_roll.radio_max);
    Serial.printf_P(PSTR("CH2: %d | %d | %d\n"), (int)g.channel_pitch.radio_min, (int)g.channel_pitch.radio_trim, (int)g.channel_pitch.radio_max);
    Serial.printf_P(PSTR("CH3: %d | %d | %d\n"), (int)g.channel_throttle.radio_min, (int)g.channel_throttle.radio_trim, (int)g.channel_throttle.radio_max);
    Serial.printf_P(PSTR("CH4: %d | %d | %d\n"), (int)g.channel_rudder.radio_min, (int)g.channel_rudder.radio_trim, (int)g.channel_rudder.radio_max);
    Serial.printf_P(PSTR("CH5: %d | %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_trim, (int)g.rc_5.radio_max);
    Serial.printf_P(PSTR("CH6: %d | %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_trim, (int)g.rc_6.radio_max);
    Serial.printf_P(PSTR("CH7: %d | %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_trim, (int)g.rc_7.radio_max);
    Serial.printf_P(PSTR("CH8: %d | %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_trim, (int)g.rc_8.radio_max);

}

static void
print_switch(byte p, byte m)
{
    Serial.printf_P(PSTR("Pos %d: "),p);
    print_flight_mode(m);
}

static void
print_done()
{
    Serial.printf_P(PSTR("\nSaved Settings\n\n"));
}

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        Serial.println("");
    }
}

static void
print_divider(void)
{
    for (int16_t i = 0; i < 40; i++) {
        Serial.printf_P(PSTR("-"));
    }
    Serial.println("");
}

static int8_t
radio_input_switch(void)
{
    static int8_t bouncer = 0;


    if (int16_t(g.channel_roll.radio_in - g.channel_roll.radio_trim) > 100) {
        bouncer = 10;
    }
    if (int16_t(g.channel_roll.radio_in - g.channel_roll.radio_trim) < -100) {
        bouncer = -10;
    }
    if (bouncer >0) {
        bouncer--;
    }
    if (bouncer <0) {
        bouncer++;
    }

    if (bouncer == 1 || bouncer == -1) {
        return bouncer;
    } else {
        return 0;
    }
}


static void zero_eeprom(void)
{
    byte b = 0;
    Serial.printf_P(PSTR("\nErasing EEPROM\n"));
    for (intptr_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        eeprom_write_byte((uint8_t *) i, b);
    }
    Serial.printf_P(PSTR("done\n"));
}

static void print_enabled(bool b)
{
    if(b)
        Serial.printf_P(PSTR("en"));
    else
        Serial.printf_P(PSTR("dis"));
    Serial.printf_P(PSTR("abled\n"));
}

static void
print_accel_offsets(void)
{
    Serial.printf_P(PSTR("Accel offsets: %4.2f, %4.2f, %4.2f\n"),
                    (float)imu.ax(),
                    (float)imu.ay(),
                    (float)imu.az());
}

static void
print_gyro_offsets(void)
{
    Serial.printf_P(PSTR("Gyro offsets: %4.2f, %4.2f, %4.2f\n"),
                    (float)imu.gx(),
                    (float)imu.gy(),
                    (float)imu.gz());
}


#endif // CLI_ENABLED

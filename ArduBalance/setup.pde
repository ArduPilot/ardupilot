// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_tune              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
//static int8_t	setup_mag_offset		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show				(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======        	===============
    {"erase", 			setup_erase},
    {"reset", 			setup_factory},
    {"radio",			setup_radio},
    {"level",			setup_accel},
    {"accel",			setup_accel_scale},
    {"modes",			setup_flightmodes},
    {"sonar",			setup_sonar},
    {"compass",			setup_compass},
    {"tune",			setup_tune},
    {"range",			setup_range},
//	{"offsets",			setup_mag_offset},
    {"declination",		setup_declination},
    {"show",			setup_show}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));
    //"\n"
    //"IMPORTANT: if you have not previously set this system up, use the\n"
    //"'reset' command to initialize the EEPROM to sensible default values\n"
    //"and then the 'radio' command to configure for your radio.\n"
    //"\n"));

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

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_sonar();
    report_flight_modes();
    report_ins();
    report_compass();
    AP_Param::show_all();

    return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = Serial.read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nReboot APM"));

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
setup_accel(uint8_t argc, const Menu::arg *argv)
{
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             delay, flash_leds, &timer_scheduler);
    ins.init_accel(delay, flash_leds);
    report_ins();
    return(0);
}

/*
  handle full accelerometer calibration via user dialog
 */

static void setup_printf_P(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    cliSerial->vprintf_P(fmt, arg_list);
    va_end(arg_list);
}

static void setup_wait_key(void)
{
    // wait for user input
    while (!cliSerial->available()) {
        delay(20);
    }
    // clear input buffer
    while( cliSerial->available() ) {
        cliSerial->read();
    }
}


static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Initialising gyros"));
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             delay, flash_leds, &timer_scheduler);
    ins.calibrate_accel(delay, flash_leds, setup_printf_P, setup_wait_key);
    report_ins();
    return(0);
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    byte _switchPosition = 0;
    byte _oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMode switch to edit, aileron: select modes\n"));
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
            print_switch(_switchPosition, mode);

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
            print_switch(_switchPosition, mode);
            delay(500);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            for (mode = 0; mode < 6; mode++)
                flight_modes[mode].save();

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
    cliSerial->printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

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
        cliSerial->printf_P(PSTR("\nOp:[on,off]\n"));
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
        cliSerial->printf_P(PSTR("\nOp: off, 3-4"));
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
        cliSerial->printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

static void clear_offsets()
{
    Vector3f _offsets(0.0,0.0,0.0);
    compass.set_offsets(_offsets);
    compass.save_offsets();
}


/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if(g.battery_monitoring == 0) print_enabled(false);
    if(g.battery_monitoring == 3) cliSerial->printf_P(PSTR("volts"));
    if(g.battery_monitoring == 4) cliSerial->printf_P(PSTR("volts and cur"));
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
    cliSerial->printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    cliSerial->printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
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
    cliSerial->printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f"),
                    offsets.x,
                    offsets.y,
                    offsets.z);
    print_blanks(2);
}

static void report_flight_modes()
{
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i]);
    }
    print_blanks(2);
}


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
    //cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(byte p, byte m)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(m);
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved\n"));
}


static void zero_eeprom(void)
{
    byte b = 0;

    cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));

    for (int16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        eeprom_write_byte((uint8_t *) i, b);
    }

    cliSerial->printf_P(PSTR("done\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\tA_scale: %4.2f, %4.2f, %4.2f\n"),
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


#endif // CLI_ENABLED

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}


static bool
wait_for_yes()
{
    int c;
    Serial.flush();
    cliSerial->printf_P(PSTR("Y to save\n"));

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


static void print_wp(struct Location *cmd, byte index)
{
   	//float t1 = (float)cmd->lat / t7;
    //float t2 = (float)cmd->lng / t7;

    cliSerial->printf_P(PSTR("cmd#: %d | %d, %d, %d, %ld, %ld, %ld\n"),
                    index,
                    cmd->id,
                    cmd->options,
                    cmd->p1,
                    cmd->alt,
                    cmd->lat,
                    cmd->lng);

	/*
    cliSerial->printf_P(PSTR("cmd#: %d id:%d op:%d p1:%d p2:%ld p3:%4.7f p4:%4.7f \n"),
                    (int)index,
                    (int)cmd->id,
                    (int)cmd->options,
                    (int)cmd->p1,
                    (long)cmd->alt,
                    t1,
                    t2);
	*/
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

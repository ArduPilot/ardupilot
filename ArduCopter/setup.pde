// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show              (uint8_t argc, const Menu::arg *argv);


// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"reset",                       setup_factory},
    {"show",                        setup_show},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
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
    cliSerial->printf_P(PSTR("\nReboot board"));

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
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

    AP_Param::show_all(cliSerial);

    return(0);
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if (battery.monitoring() == AP_BATT_MONITOR_DISABLED) print_enabled(false);
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_ONLY) cliSerial->printf_P(PSTR("volts"));
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) cliSerial->printf_P(PSTR("volts and cur"));
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

/***************************************************************************/
// CLI utilities
/***************************************************************************/

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
print_accel_offsets_and_scaling(void)
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
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
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

#endif // CLI_ENABLED

// report_compass - displays compass information.  Also called by compassmot.pde
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

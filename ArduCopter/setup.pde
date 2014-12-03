// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define WITH_ESC_CALIB
#endif

// Functions called from the setup menu
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);
#ifdef WITH_ESC_CALIB
static int8_t   esc_calib               (uint8_t argc, const Menu::arg *argv);
#endif

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"reset",                       setup_factory},
    {"show",                        setup_show},
    {"set",                         setup_set},
#ifdef WITH_ESC_CALIB
    {"esc_calib",                   esc_calib},
#endif
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
    report_flight_modes();
    report_ins();
    report_compass();
    report_optflow();

    AP_Param::show_all(cliSerial);

    return(0);
}

#ifdef WITH_ESC_CALIB
#define PWM_CALIB_MIN 1000
#define PWM_CALIB_MAX 2000
#define PWM_HIGHEST_MAX 2200
#define PWM_LOWEST_MAX 1200
#define PWM_HIGHEST_MIN 1800
#define PWM_LOWEST_MIN 800

static int8_t
esc_calib(uint8_t argc,const Menu::arg *argv)
{


	char c;
	unsigned max_channels = 0;
	uint32_t set_mask = 0;

	uint16_t pwm_high = PWM_CALIB_MAX;
	uint16_t pwm_low = PWM_CALIB_MIN;


	if (argc < 2) {
		cliSerial->printf_P(PSTR("Pls provide Channel Mask\n"
                                    "\tusage: esc_calib 1010 - enables calibration for 2nd and 4th Motor\n"));
        return(0);
	}
    

	
    set_mask = strtol (argv[1].str, NULL, 2);
	if (set_mask == 0)
		cliSerial->printf_P(PSTR("no channels chosen"));
    //cliSerial->printf_P(PSTR("\n%d\n"),set_mask);
    set_mask<<=1;
	/* wait 50 ms */
	hal.scheduler->delay(50);


	cliSerial->printf_P(PSTR("\nATTENTION, please remove or fix propellers before starting calibration!\n"
	       "\n"
	       "Make sure\n"
	       "\t - that the ESCs are not powered\n"
	       "\t - that safety is off\n"
	       "\t - that the controllers are stopped\n"
	       "\n"
	       "Do you want to start calibration now: y or n?\n"));

	/* wait for user input */
	while (1) {
            c= cliSerial->read();
			if (c == 'y' || c == 'Y') {

				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				cliSerial->printf_P(PSTR("ESC calibration exited\n"));
				return(0);

			} else if (c == 'n' || c == 'N') {
				cliSerial->printf_P(PSTR("ESC calibration aborted\n"));
				return(0);

			} 

		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}


	/* get number of channels available on the device */
	max_channels = AP_MOTORS_MAX_NUM_MOTORS;

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	motors.armed(true);


	cliSerial->printf_P(PSTR("Outputs armed\n"));


	/* wait for user confirmation */
	cliSerial->printf_P(PSTR("\nHigh PWM set: %d\n"
	       "\n"
	       "Connect battery now and hit c+ENTER after the ESCs confirm the first calibration step\n"
	       "\n"), pwm_high);

	while (1) {
		/* set max PWM */
		for (unsigned i = 0; i < max_channels; i++) {

			if (set_mask & 1<<i) {
				motors.output_test(i, pwm_high);
			}
		}
        c = cliSerial->read();
            
		if (c == 'c') {
            break;

		} else if (c == 0x03 || c == 0x63 || c == 'q') {
			cliSerial->printf_P(PSTR("ESC calibration exited\n"));
			return(0);
		}
        
		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}

	cliSerial->printf_P(PSTR("Low PWM set: %d\n"
	       "\n"
	       "Hit c+Enter when finished\n"
	       "\n"), pwm_low);

	while (1) {

		/* set disarmed PWM */
		for (unsigned i = 0; i < max_channels; i++) {
			if (set_mask & 1<<i) {
				motors.output_test(i, pwm_low);
			}
		}
		c = cliSerial->read();

		if (c == 'c') {

			break;

		} else if (c == 0x03 || c == 0x63 || c == 'q') {
			cliSerial->printf_P(PSTR("ESC calibration exited\n"));
			return(0);
		}
		
		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}

	/* disarm */
	motors.armed(false);
    
	cliSerial->printf_P(PSTR("Outputs disarmed\n"));

	cliSerial->printf_P(PSTR("ESC calibration finished\n"));

	return(0);
}
#endif // WITH_ESC_CALIB


/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if (battery.num_instances() == 0) {
        print_enabled(false);
    } else if (!battery.has_current()) {
        cliSerial->printf_P(PSTR("volts"));
    } else {
        cliSerial->printf_P(PSTR("volts and cur"));
    }
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

    print_enabled(optflow.enabled());

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

    // mag offsets
    Vector3f offsets;
    for (uint8_t i=0; i<compass.get_count(); i++) {
        offsets = compass.get_offsets(i);
        // mag offsets
        cliSerial->printf_P(PSTR("Mag%d off: %4.4f, %4.4f, %4.4f\n"),
                        (int)i,
                        offsets.x,
                        offsets.y,
                        offsets.z);
    }

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
        Vector3f motor_compensation;
        for (uint8_t i=0; i<compass.get_count(); i++) {
            motor_compensation = compass.get_motor_compensation(i);
            cliSerial->printf_P(PSTR("\nComMot%d: %4.2f, %4.2f, %4.2f\n"),
                        (int)i,
                        motor_compensation.x,
                        motor_compensation.y,
                        motor_compensation.z);
        }
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

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}

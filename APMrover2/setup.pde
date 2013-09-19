// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t	setup_radio			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes	(uint8_t argc, const Menu::arg *argv);
#if !defined( __AVR_ATmega1280__ )
static int8_t   setup_accel_scale   (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set           (uint8_t argc, const Menu::arg *argv);
#endif
static int8_t   setup_level         (uint8_t argc, const Menu::arg *argv);
static int8_t	setup_erase			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_compass		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination	(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_batt_monitor	(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"reset", 			setup_factory},
	{"radio",			setup_radio},
	{"modes",			setup_flightmodes},
	{"level",			setup_level},
#if !defined( __AVR_ATmega1280__ )
    {"accel",           setup_accel_scale},
#endif
	{"compass",			setup_compass},
	{"declination",		setup_declination},
	{"battery",			setup_batt_monitor},
	{"show",			setup_show},
#if !defined( __AVR_ATmega1280__ )
	{"set",				setup_set},
#endif
	{"erase",			setup_erase},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
	// Give the user some guidance
	cliSerial->printf_P(PSTR("Setup Mode\n"
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

#if !defined( __AVR_ATmega1280__ )
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

		//Print differently for different types, and include parameter type in output.
		switch (type) {
			case AP_PARAM_INT8:
				cliSerial->printf_P(PSTR("INT8  %s: %d\n"), argv[1].str, (int)((AP_Int8 *)param)->get());
				break;
			case AP_PARAM_INT16:
				cliSerial->printf_P(PSTR("INT16 %s: %d\n"), argv[1].str, (int)((AP_Int16 *)param)->get());
				break;
			case AP_PARAM_INT32:
				cliSerial->printf_P(PSTR("INT32 %s: %ld\n"), argv[1].str, (long)((AP_Int32 *)param)->get());
				break;
			case AP_PARAM_FLOAT:
				cliSerial->printf_P(PSTR("FLOAT %s: %f\n"), argv[1].str, ((AP_Float *)param)->get());
				break;
			default:
				cliSerial->printf_P(PSTR("Unhandled parameter type for %s: %d.\n"), argv[1].str, type);
				break;
		}

		return 0;
	}
#endif

	// clear the area
	print_blanks(8);

	report_radio();
	report_batt_monitor();
	report_gains();
	report_throttle();
	report_modes();
	report_compass();

	cliSerial->printf_P(PSTR("Raw Values\n"));
	print_divider();

    AP_Param::show_all(cliSerial);

	return(0);
}


#if !defined( __AVR_ATmega1280__ )

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
#endif


// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
	int			c;

	cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));

	do {
		c = cliSerial->read();
	} while (-1 == c);

	if (('y' != c) && ('Y' != c))
		return(-1);
	AP_Param::erase_all();
	cliSerial->printf_P(PSTR("\nFACTORY RESET complete - please reset APM to continue"));

	for (;;) {
	}
	// note, cannot actually return here
	return(0);
}


// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("\n\nRadio Setup:\n"));
	uint8_t i;

	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}


	if(channel_steer->radio_in < 500){
		while(1){
			cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
			delay(1000);
			// stop here
		}
	}

	channel_steer->radio_min 		= channel_steer->radio_in;
	channel_throttle->radio_min 	= channel_throttle->radio_in;
	g.rc_2.radio_min = g.rc_2.radio_in;
	g.rc_4.radio_min = g.rc_4.radio_in;
	g.rc_5.radio_min = g.rc_5.radio_in;
	g.rc_6.radio_min = g.rc_6.radio_in;
	g.rc_7.radio_min = g.rc_7.radio_in;
	g.rc_8.radio_min = g.rc_8.radio_in;

	channel_steer->radio_max 		= channel_steer->radio_in;
	channel_throttle->radio_max 	= channel_throttle->radio_in;
	g.rc_2.radio_max = g.rc_2.radio_in;
	g.rc_4.radio_max = g.rc_4.radio_in;
	g.rc_5.radio_max = g.rc_5.radio_in;
	g.rc_6.radio_max = g.rc_6.radio_in;
	g.rc_7.radio_max = g.rc_7.radio_in;
	g.rc_8.radio_max = g.rc_8.radio_in;

	channel_steer->radio_trim 		= channel_steer->radio_in;
	g.rc_2.radio_trim = 1500;
	g.rc_4.radio_trim = 1500;
	g.rc_5.radio_trim = 1500;
	g.rc_6.radio_trim = 1500;
	g.rc_7.radio_trim = 1500;
	g.rc_8.radio_trim = 1500;

	cliSerial->printf_P(PSTR("\nMove all controls to each extreme. Hit Enter to save: \n"));
	while(1){

		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		channel_steer->update_min_max();
		channel_throttle->update_min_max();
		g.rc_2.update_min_max();
		g.rc_4.update_min_max();
		g.rc_5.update_min_max();
		g.rc_6.update_min_max();
		g.rc_7.update_min_max();
		g.rc_8.update_min_max();

		if(cliSerial->available() > 0){
            while (cliSerial->available() > 0) {
                cliSerial->read();
            }
			channel_steer->save_eeprom();
			channel_throttle->save_eeprom();
			g.rc_2.save_eeprom();
			g.rc_4.save_eeprom();
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
	uint8_t switchPosition, mode = 0;

	cliSerial->printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
	print_hit_enter();
	trim_radio();

	while(1){
		delay(20);
		read_radio();
		switchPosition = readSwitch();


		// look for control switch change
		if (oldSwitchPosition != switchPosition){
			// force position 5 to MANUAL
			if (switchPosition > 4) {
				modes[switchPosition] = MANUAL;
			}
			// update our current mode
			mode = modes[switchPosition];

			// update the user
			print_switch(switchPosition, mode);

			// Remember switch position
			oldSwitchPosition = switchPosition;
		}

		// look for stick input
		int radioInputSwitch = radio_input_switch();

		if (radioInputSwitch != 0){

			mode += radioInputSwitch;

			while (
				mode != MANUAL &&
				mode != HOLD &&
				mode != LEARNING &&
				mode != STEERING &&
				mode != AUTO &&
				mode != RTL) 
			{
				if (mode < MANUAL)
					mode = RTL;
				else if (mode > RTL)
					mode = MANUAL;
				else
					mode += radioInputSwitch;
			}

			// Override position 5
			if(switchPosition > 4)
				mode = MANUAL;

			// save new mode
			modes[switchPosition] = mode;

			// print new mode
			print_switch(switchPosition, mode);
		}

		// escape hatch
		if(cliSerial->available() > 0){
		    // save changes
            for (mode=0; mode<6; mode++)
                modes[mode].save();
			report_modes();
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
	int			c;

	cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: "));

	do {
		c = cliSerial->read();
	} while (-1 == c);

	if (('y' != c) && ('Y' != c))
		return(-1);
	zero_eeprom();
	return 0;
}

/*
  handle full accelerometer calibration via user dialog
 */
#if !defined( __AVR_ATmega1280__ )
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
    return(0);
}
#endif

static int8_t
setup_level(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ins.init_accel();
    ahrs.set_trim(Vector3f(0, 0, 0));
    return(0);
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
	if (!strcmp_P(argv[1].str, PSTR("on"))) {
		if (!compass.init()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            g.compass_enabled = true;
        }
	} else if (!strcmp_P(argv[1].str, PSTR("off"))) {
		g.compass_enabled = false;

	} else if (!strcmp_P(argv[1].str, PSTR("reset"))) {
		compass.set_offsets(0,0,0);

	} else {
		cliSerial->printf_P(PSTR("\nOptions:[on,off,reset]\n"));
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
	if(argv[1].i >= 0 && argv[1].i <= 4){
		g.battery_monitoring.set_and_save(argv[1].i);

	} else {
		cliSerial->printf_P(PSTR("\nOptions: 3-4"));
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
	cliSerial->printf_P(PSTR("Batt Mointor\n"));
	print_divider();
	if(g.battery_monitoring == 0)	cliSerial->printf_P(PSTR("Batt monitoring disabled"));
	if(g.battery_monitoring == 3)	cliSerial->printf_P(PSTR("Monitoring batt volts"));
	if(g.battery_monitoring == 4)	cliSerial->printf_P(PSTR("Monitoring volts and current"));
	print_blanks(2);
}
static void report_radio()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Radio\n"));
	print_divider();
	// radio
	print_radio_values();
	print_blanks(2);
}

static void report_gains()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Gains\n"));
	print_divider();

	cliSerial->printf_P(PSTR("speed throttle:\n"));
	print_PID(&g.pidSpeedThrottle);

	print_blanks(2);
}

static void report_throttle()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Throttle\n"));
	print_divider();

	cliSerial->printf_P(PSTR("min: %u\n"
                             "max: %u\n"
                             "cruise: %u\n"
                             "failsafe_enabled: %u\n"
                             "failsafe_value: %u\n"),
						 (unsigned)g.throttle_min,
						 (unsigned)g.throttle_max,
						 (unsigned)g.throttle_cruise,
						 (unsigned)g.fs_throttle_enabled,
						 (unsigned)g.fs_throttle_value);
	print_blanks(2);
}

static void report_compass()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Compass: "));

    switch (compass.product_id) {
    case AP_COMPASS_TYPE_HMC5883L:
        cliSerial->println_P(PSTR("HMC5883L"));
        break;
    case AP_COMPASS_TYPE_HMC5843:
        cliSerial->println_P(PSTR("HMC5843"));
        break;
    case AP_COMPASS_TYPE_HIL:
        cliSerial->println_P(PSTR("HIL"));
        break;
    default:
        cliSerial->println_P(PSTR("??"));
        break;
    }

	print_divider();

	print_enabled(g.compass_enabled);

	// mag declination
	cliSerial->printf_P(PSTR("Mag Declination: %4.4f\n"),
							degrees(compass.get_declination()));

	Vector3f offsets = compass.get_offsets();

	// mag offsets
	cliSerial->printf_P(PSTR("Mag offsets: %4.4f, %4.4f, %4.4f\n"),
							offsets.x,
							offsets.y,
							offsets.z);
	print_blanks(2);
}

static void report_modes()
{
	//print_blanks(2);
	cliSerial->printf_P(PSTR("Flight modes\n"));
	print_divider();

	for(int i = 0; i < 6; i++ ){
		print_switch(i, modes[i]);
	}
	print_blanks(2);
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

static void
print_PID(PID * pid)
{
	cliSerial->printf_P(PSTR("P: %4.3f, I:%4.3f, D:%4.3f, IMAX:%ld\n"),
					pid->kP(),
					pid->kI(),
					pid->kD(),
					(long)pid->imax());
}

static void
print_radio_values()
{
	cliSerial->printf_P(PSTR("CH1: %d | %d | %d\n"), (int)channel_steer->radio_min, (int)channel_steer->radio_trim, (int)channel_steer->radio_max);
	cliSerial->printf_P(PSTR("CH2: %d | %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_trim, (int)g.rc_2.radio_max);
	cliSerial->printf_P(PSTR("CH3: %d | %d | %d\n"), (int)channel_throttle->radio_min, (int)channel_throttle->radio_trim, (int)channel_throttle->radio_max);
	cliSerial->printf_P(PSTR("CH4: %d | %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_trim, (int)g.rc_4.radio_max);
	cliSerial->printf_P(PSTR("CH5: %d | %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_trim, (int)g.rc_5.radio_max);
	cliSerial->printf_P(PSTR("CH6: %d | %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_trim, (int)g.rc_6.radio_max);
	cliSerial->printf_P(PSTR("CH7: %d | %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_trim, (int)g.rc_7.radio_max);
	cliSerial->printf_P(PSTR("CH8: %d | %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_trim, (int)g.rc_8.radio_max);

}

static void
print_switch(uint8_t p, uint8_t m)
{
	cliSerial->printf_P(PSTR("Pos %d: "),p);
    print_mode(cliSerial, m);
    cliSerial->println();
}

static void
print_done()
{
	cliSerial->printf_P(PSTR("\nSaved Settings\n\n"));
}

static void
print_blanks(int num)
{
	while(num > 0){
		num--;
		cliSerial->println("");
	}
}

static void
print_divider(void)
{
	for (int i = 0; i < 40; i++) {
		cliSerial->printf_P(PSTR("-"));
	}
	cliSerial->println("");
}

static int8_t
radio_input_switch(void)
{
	static int8_t bouncer = 0;


	if (int16_t(channel_steer->radio_in - channel_steer->radio_trim) > 100) {
	    bouncer = 10;
	}
	if (int16_t(channel_steer->radio_in - channel_steer->radio_trim) < -100) {
	    bouncer = -10;
	}
	if (bouncer >0) {
	    bouncer --;
	}
	if (bouncer <0) {
	    bouncer ++;
	}

	if (bouncer == 1 || bouncer == -1) {
	    return bouncer;
	} else {
	    return 0;
	}
}


static void zero_eeprom(void)
{
	uint8_t b = 0;
	cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));
	for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
		hal.storage->write_byte(i, b);
	}
	cliSerial->printf_P(PSTR("done\n"));
}

static void print_enabled(bool b)
{
	if(b)
		cliSerial->printf_P(PSTR("en"));
	else
		cliSerial->printf_P(PSTR("dis"));
	cliSerial->printf_P(PSTR("abled\n"));
}

#endif // CLI_ENABLED

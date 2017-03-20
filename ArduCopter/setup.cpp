#include "Copter.h"

#if CLI_ENABLED == ENABLED

#define PWM_CALIB_MIN 1000
#define PWM_CALIB_MAX 2000
#define PWM_HIGHEST_MAX 2200
#define PWM_LOWEST_MAX 1200
#define PWM_HIGHEST_MIN 1800
#define PWM_LOWEST_MIN 800

// Command/function table for the setup menu
static const struct Menu::command setup_menu_commands[] = {
    {"reset",                       MENU_FUNC(setup_factory)},
    {"show",                        MENU_FUNC(setup_show)},
    {"set",                         MENU_FUNC(setup_set)},
    {"esc_calib",                   MENU_FUNC(esc_calib)},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
int8_t Copter::setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf("Setup Mode\n\n\n");

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
int8_t Copter::setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf("\n'Y' = factory reset, any other key to abort:\n");

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf("\nReboot board");

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
int8_t Copter::setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf("Invalid command. Usage: set <name> <value>\n");
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf("Param not found: %s\n", argv[1].str);
        return 0;
    }

    const char *strType = "Value out of range for type";
    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf("%s INT8\n", strType);
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf("%s INT16\n", strType);
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
            cliSerial->printf("Cannot set parameter of type %d.\n", p_type);
            break;
    }

    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
int8_t Copter::setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf("Parameter not found: '%s'\n", argv[1].str);
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

int8_t Copter::esc_calib(uint8_t argc,const Menu::arg *argv)
{


	char c;
	unsigned max_channels = 0;
	uint32_t set_mask = 0;

	uint16_t pwm_high = PWM_CALIB_MAX;
	uint16_t pwm_low = PWM_CALIB_MIN;


	if (argc < 2) {
		cliSerial->printf("Pls provide Channel Mask\n"
                                    "\tusage: esc_calib 1010 - enables calibration for 2nd and 4th Motor\n");
        return(0);
	}
    

	
    set_mask = strtol (argv[1].str, nullptr, 2);
	if (set_mask == 0)
		cliSerial->printf("no channels chosen");
    //cliSerial->printf("\n%d\n",set_mask);
    set_mask<<=1;
	/* wait 50 ms */
	hal.scheduler->delay(50);


	cliSerial->printf("\nATTENTION, please remove or fix propellers before starting calibration!\n"
	       "\n"
	       "Make sure\n"
	       "\t - that the ESCs are not powered\n"
	       "\t - that safety is off\n"
	       "\t - that the controllers are stopped\n"
	       "\n"
	       "Do you want to start calibration now: y or n?\n");

	/* wait for user input */
    const char *strEscCalib = "ESC calibration";
	while (1) {
            c= cliSerial->read();
			if (c == 'y' || c == 'Y') {

				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				cliSerial->printf("%s exited\n", strEscCalib);
				return(0);

			} else if (c == 'n' || c == 'N') {
				cliSerial->printf("%s aborted\n", strEscCalib);
				return(0);

			} 

		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}


	/* get number of channels available on the device */
	max_channels = AP_MOTORS_MAX_NUM_MOTORS;

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	motors->armed(true);


	cliSerial->printf("Outputs armed\n");


	/* wait for user confirmation */
	cliSerial->printf("\nHigh PWM set: %d\n"
	       "\n"
	       "Connect battery now and hit c+ENTER after the ESCs confirm the first calibration step\n"
	       "\n", pwm_high);

	while (1) {
		/* set max PWM */
		for (unsigned i = 0; i < max_channels; i++) {

			if (set_mask & 1<<i) {
				motors->output_test(i, pwm_high);
			}
		}
        c = cliSerial->read();
            
		if (c == 'c') {
            break;

		} else if (c == 0x03 || c == 'q') {
			cliSerial->printf("%s exited\n", strEscCalib);
			return(0);
		}
        
		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}

	cliSerial->printf("Low PWM set: %d\n"
	       "\n"
	       "Hit c+Enter when finished\n"
	       "\n", pwm_low);

	while (1) {

		/* set disarmed PWM */
		for (unsigned i = 0; i < max_channels; i++) {
			if (set_mask & 1<<i) {
				motors->output_test(i, pwm_low);
			}
		}
		c = cliSerial->read();

		if (c == 'c') {

			break;

		} else if (c == 0x03 || c == 'q') {
			cliSerial->printf("%s exited\n", strEscCalib);
			return(0);
		}
		
		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}

	/* disarm */
	motors->armed(false);
    
	cliSerial->printf("Outputs disarmed\n");

	cliSerial->printf("%s finished\n", strEscCalib);

	return(0);
}


/***************************************************************************/
// CLI reports
/***************************************************************************/

void Copter::report_batt_monitor()
{
    cliSerial->printf("\nBatt Mon:\n");
    print_divider();
    if (battery.num_instances() == 0) {
        print_enabled(false);
    } else if (!battery.has_current()) {
        cliSerial->printf("volts");
    } else {
        cliSerial->printf("volts and cur");
    }
    print_blanks(2);
}

void Copter::report_frame()
{
    cliSerial->printf("Frame\n");
    print_divider();
    cliSerial->printf("%s\n", get_frame_string());

    print_blanks(2);
}

void Copter::report_radio()
{
    cliSerial->printf("Radio\n");
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

void Copter::report_ins()
{
    cliSerial->printf("INS\n");
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

void Copter::report_flight_modes()
{
    cliSerial->printf("Flight modes\n");
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, (control_mode_t)flight_modes[i].get(), BIT_IS_SET(g.simple_modes, i));
    }
    print_blanks(2);
}

void Copter::report_optflow()
{
 #if OPTFLOW == ENABLED
    cliSerial->printf("OptFlow\n");
    print_divider();

    print_enabled(optflow.enabled());

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

void Copter::print_radio_values()
{
    for (uint8_t i=0; i<8; i++) {
        RC_Channel *rc = RC_Channels::rc_channel(i);
        cliSerial->printf("CH%u: %d | %d\n", (unsigned)i, rc->get_radio_min(), rc->get_radio_max());
    }
}

void Copter::print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf("Pos %d:\t",p);
    print_flight_mode(cliSerial, m);
    cliSerial->printf(",\t\tSimple: ");
    if(b)
        cliSerial->printf("ON\n");
    else
        cliSerial->printf("OFF\n");
}

void Copter::print_accel_offsets_and_scaling(void)
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    cliSerial->printf("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n",
                    (double)accel_offsets.x,                           // Pitch
                    (double)accel_offsets.y,                           // Roll
                    (double)accel_offsets.z,                           // YAW
                    (double)accel_scale.x,                             // Pitch
                    (double)accel_scale.y,                             // Roll
                    (double)accel_scale.z);                            // YAW
}

void Copter::print_gyro_offsets(void)
{
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf("G_off: %4.2f, %4.2f, %4.2f\n",
                    (double)gyro_offsets.x,
                    (double)gyro_offsets.y,
                    (double)gyro_offsets.z);
}

#endif // CLI_ENABLED

// report_compass - displays compass information.  Also called by compassmot.pde
void Copter::report_compass()
{
    cliSerial->printf("Compass\n");
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf("Mag Dec: %4.4f\n",
            (double)degrees(compass.get_declination()));

    // mag offsets
    Vector3f offsets;
    for (uint8_t i=0; i<compass.get_count(); i++) {
        offsets = compass.get_offsets(i);
        // mag offsets
        cliSerial->printf("Mag%d off: %4.4f, %4.4f, %4.4f\n",
                        (int)i,
                        (double)offsets.x,
                        (double)offsets.y,
                        (double)offsets.z);
    }

    // motor compensation
    cliSerial->printf("Motor Comp: ");
    if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->printf("Off\n");
    }else{
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->printf("Throttle");
        }
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->printf("Current");
        }
        Vector3f motor_compensation;
        for (uint8_t i=0; i<compass.get_count(); i++) {
            motor_compensation = compass.get_motor_compensation(i);
            cliSerial->printf("\nComMot%d: %4.2f, %4.2f, %4.2f\n",
                        (int)i,
                        (double)motor_compensation.x,
                        (double)motor_compensation.y,
                        (double)motor_compensation.z);
        }
    }
    print_blanks(1);
}

void Copter::print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->printf("\n");
    }
}

void Copter::print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->printf("-");
    }
    cliSerial->printf("\n");
}

void Copter::print_enabled(bool b)
{
    if(b)
        cliSerial->printf("en");
    else
        cliSerial->printf("dis");
    cliSerial->printf("abled\n");
}

void Copter::report_version()
{
    cliSerial->printf("FW Ver: %d\n",(int)(g.k_format_version));
    print_divider();
    print_blanks(2);
}

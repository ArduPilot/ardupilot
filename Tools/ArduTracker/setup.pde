// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Functions called from the setup menu
static int8_t	setup_radio(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu 
const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"reset", 			setup_factory},
	{"radio",			setup_radio},
	{"modes",			setup_flightmodes},
	{"show",			setup_show},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
int8_t
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
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
	printAllParams(Serial);
	return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{

	uint8_t		i;
	int			c;

	Serial.printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));

	do {
		c = Serial.read();
	} while (-1 == c);
	
	if (('y' != c) && ('Y' != c))
		return(-1);

	Serial.printf_P(PSTR("\nFACTORY RESET\n\n"));
	param_reset_defaults();
	return(0);
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
	Serial.println("\n\nRadio Setup:");
	uint8_t i;
	
	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}
	
	if(radio_in[CH_ROLL] < 500){
		while(1){
			Serial.print("Radio error");
			delay(1000);
			// stop here
		}
	}

	uint16_t tempMin[4], tempMax[4];
	for(i = 0; i < 4; i++){
		tempMin[i] = radio_in[i];
		tempMax[i] = radio_in[i];
	}
		
	Serial.printf_P(PSTR("\nMove both sticks to each corner. Hit Enter to save: "));
	while(1){
		
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		for(i = 0; i < 4; i++){
			tempMin[i] = min(tempMin[i], radio_in[i]);
			tempMax[i] = max(tempMax[i], radio_in[i]);
		}
				
		if(Serial.available() > 0){
			Serial.flush();
			Serial.println("Saving:");

			for(i = 0; i < 4; i++) 
			{
				set_radio_max(i, tempMax[i]);
				set_radio_min(i, tempMin[i]);
			}
			
			for(i = 0; i < 8; i++)
				Serial.printf_P(PSTR("CH%d: %d | %d\n"), i + 1, radio_min(i), radio_max(i));
			break;
		}
	}
	
	return(0);
}


static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
	byte switchPosition, oldSwitchPosition, mode;
	
	Serial.printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
	print_hit_enter();
	trim_radio();
	
	while(1){
		delay(20);
		read_radio();		
		switchPosition = readSwitch();

		
		// look for control switch change
		if (oldSwitchPosition != switchPosition){
			// Override position 5
			if(switchPosition > 4){
				set_flight_mode(switchPosition,MANUAL);
				mode = MANUAL;
			}else{
				// update our current mode
				mode = flight_mode(switchPosition);
			}

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
				mode != CIRCLE &&
				mode != STABILIZE &&
				mode != FLY_BY_WIRE_A &&
				mode != FLY_BY_WIRE_B &&
				mode != AUTO &&
				mode != RTL &&
				mode != LOITER &&
				mode != TAKEOFF &&
				mode != LAND)
			{
				if (mode < MANUAL)
					mode = LAND;
				else if (mode >LAND)
					mode = MANUAL;
				else
					mode += radioInputSwitch;
			}
			
			// Override position 5
			if(switchPosition > 4)
				mode = MANUAL;
				
			// save new mode
			set_flight_mode(switchPosition,mode);

			// print new mode
			print_switch(switchPosition, mode);
		}

		// escape hatch
		if(Serial.available() > 0){
			//save_EEPROM_flight_modes();
			return (0);
		}
	}
}

void
print_switch(byte p, byte m)
{
	Serial.printf_P(PSTR("Pos %d: "),p);
	Serial.println(flight_mode_strings[m]);
}

int8_t
radio_input_switch(void)
{
	static int8_t bouncer = 0;

	if (int16_t(radio_in[0] - radio_trim(0)) > 200) bouncer = 10;
	if (int16_t(radio_in[0] - radio_trim(0)) < -200) bouncer = -10;
	if (bouncer >0) bouncer --;
	if (bouncer <0) bouncer ++;

	if (bouncer == 1 || bouncer == -1) return bouncer;
	else return 0;
}


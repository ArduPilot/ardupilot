/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void read_control_switch()
{
	
	uint8_t switchPosition = readSwitch();
	
	// If switchPosition = 255 this indicates that the mode control channel input was out of range
	// If we get this value we do not want to change modes.
	if(switchPosition == 255) return;

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
	if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 && 
         hal.rcin->read(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

		set_mode((enum mode)modes[switchPosition].get());

		oldSwitchPosition = switchPosition;
		prev_WP = current_loc;

		// reset speed integrator
        g.pidSpeedThrottle.reset_I();
	}

}

static uint8_t readSwitch(void){
    uint16_t pulsewidth = hal.rcin->read(g.mode_channel - 1);
	if (pulsewidth <= 910 || pulsewidth >= 2090) 	return 255;	// This is an error condition
	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

static void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
}

#define CH_7_PWM_TRIGGER 1800

// read at 10 hz
// set this to your trainer switch
static void read_trim_switch()
{
    switch ((enum ch7_option)g.ch7_option.get()) {
    case CH7_DO_NOTHING:
        break;
    case CH7_SAVE_WP:
		if (channel_learn->radio_in > CH_7_PWM_TRIGGER) {
            // switch is engaged
			ch7_flag = true;
		} else { // switch is disengaged
			if (ch7_flag) {
				ch7_flag = false;

				if (control_mode == MANUAL) {
                    hal.console->println_P(PSTR("Erasing waypoints"));
                    // if SW7 is ON in MANUAL = Erase the Flight Plan
					g.command_total.set_and_save(CH7_wp_index);
                    g.command_total = 0;
                    g.command_index =0;
                    nav_command_index = 0;
                    if (channel_steer->control_in > 3000) {
						// if roll is full right store the current location as home
                        init_home();
                    }
                    CH7_wp_index = 1;     
					return;
				} else if (control_mode == LEARNING || control_mode == STEERING) {    
                    // if SW7 is ON in LEARNING = record the Wp
                    // set the next_WP (home is stored at 0)

                    hal.console->printf_P(PSTR("Learning waypoint %u"), (unsigned)CH7_wp_index);        
                    current_loc.id = MAV_CMD_NAV_WAYPOINT;  
    
                    // store the index
                    g.command_total.set_and_save(CH7_wp_index);
                    g.command_total = CH7_wp_index;
                    g.command_index = CH7_wp_index;
                    nav_command_index = 0;
                                   
                    // save command
                    set_cmd_with_index(current_loc, CH7_wp_index);
                                  
                    // increment index
                    CH7_wp_index++; 
                    CH7_wp_index = constrain_int16(CH7_wp_index, 1, MAX_WAYPOINTS);

                } else if (control_mode == AUTO) {    
                    // if SW7 is ON in AUTO = set to RTL  
                    set_mode(RTL);
                }
            }
        }
        break;
    }
}


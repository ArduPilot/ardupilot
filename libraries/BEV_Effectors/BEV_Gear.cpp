/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdlib.h>
#include <AP_HAL.h>
#include "BEV_Gear.h"

extern const AP_HAL::HAL& hal;

//
// public methods
//

BEV_Gear::BEV_Gear(RC_Channel &rc_gear_switch, RC_Channel &rc_gear_out_1)
                 : _rc_gear_switch(rc_gear_switch) ,
                   _rc_gear_out_1(rc_gear_out_1)
{
	_gear_position = 0; //always start gear down
	_switch_position = _old_switch_position = 0; //low
}

void BEV_Gear::Init()
{
    //prevent multiple calls
    static bool first_call = true;
    if(first_call) {
        first_call = false;
    } else {
        return;
    }

#if BEV_GEAR_DEBUGGING == ENABLED
    hal.console->println("BEV_GEAR: Debugging Enabled");
#endif //BEV_GEAR_DEBUGGING

    //setting up output channel
    _rc_gear_out_1.set_range(0,1000);
    _rc_gear_out_1.enable_out();
}

// update. should be called no slower than 50hz
void BEV_Gear::update(bool radio_failsafe)
{
	//keeps track of how many times switch has been in its current position
	static uint8_t switch_counter = 0;
    //was the last frame a radio failsafe? Used to prevent gear toggle when a radio failsafe clears
    static uint8_t failsafe_last_frame = true; //start as true to prevent raising the gear on startup if switchology is incorrect

    //don't update the switch position if in radio failsafe.
    if(!radio_failsafe) {
        _switch_position = _rc_gear_switch.radio_in > 1500; //magic number 1500 = center of radio input range
        if(failsafe_last_frame) {
            //update the old switch position to the current. This prevents gear toggles
            //when a radio failsafe clears
            _old_switch_position = _switch_position;
        }
        failsafe_last_frame = false;
    } else {
        failsafe_last_frame = true;
    }

	if( _switch_position != _old_switch_position) {
		switch_counter++;
		if (switch_counter >= 5) { //5 good reads, time to transition
			_old_switch_position = _switch_position;
#if BEV_GEAR_DEBUGGING == ENABLED
			hal.console->println("BEV_Gear::update() - Switch position toggled");
#endif
			if(_switch_position) { //switch position is high, raise gear
				raise();
			} else { //switch position is low, lower
				lower();
			}
		}
	} else {
		switch_counter = 0;
	}

    //the rc_channel class is infuriatingly complex. The following gets around most of the complexity
    _rc_gear_out_1.servo_out = ((_gear_position-500)*2);
    _rc_gear_out_1.radio_out = _rc_gear_out_1.angle_to_pwm()+ _rc_gear_out_1.radio_trim;
    _rc_gear_out_1.output();
}

void BEV_Gear::raise()
{
#if BEV_GEAR_DEBUGGING == ENABLED
	hal.console->println("BEV_Gear::raise()");
#endif
	_gear_position = 1000; //up
}

void BEV_Gear::lower()
{
#if BEV_GEAR_DEBUGGING == ENABLED
	hal.console->println("BEV_Gear::lower()");
#endif
	_gear_position = 0; //down
}

void BEV_Gear::toggle()
{
#if BEV_GEAR_DEBUGGIN == ENABLED
    hal.console->println("BEV_Gear::toggle()");
#endif

    static uint32_t last_toggle_time = 0;

    if(hal.scheduler->millis() - last_toggle_time < 1000) {
        //function has been called and executed recently. debounce by rejecting second call
        return;
    }
    //executing. mark time
    last_toggle_time = hal.scheduler->millis();

    //actually toggle
    if(is_raised()) {
        lower();
    } else {
        raise();
    }
}

bool BEV_Gear::is_raised()
{
	return _gear_position > 500; //500 is the midpoint
}

bool BEV_Gear::is_lowered()
{
	return _gear_position <= 500; //500 is the midpoint
}

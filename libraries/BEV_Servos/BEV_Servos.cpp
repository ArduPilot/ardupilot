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

#include "BEV_Servos.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;


BEV_Servos::BEV_Servos()
{
	
}

void BEV_Servos::init()
{
	//initialize the output
	hal.rcout->enable_ch(10);
	hal.rcout->enable_ch(11);
	
	//set the output to be minimum
	hal.rcout->write(10,RC_Channel::rc_channel(10)->radio_min);
	hal.rcout->write(11,RC_Channel::rc_channel(11)->radio_min);
}

void BEV_Servos::output(uint8_t servo, int16_t pwmout)
{
	if(servo==3) { //aux3 is output 10
		pwmout = constrain_int16(pwmout, RC_Channel::rc_channel(10)->radio_min, RC_Channel::rc_channel(10)->radio_max);
		hal.rcout->write(10, pwmout);
	}else if (servo==4) { //aux4 is output 11
		pwmout = constrain_int16(pwmout, RC_Channel::rc_channel(11)->radio_min, RC_Channel::rc_channel(11)->radio_max);
		hal.rcout->write(11, pwmout);
	}
}

void BEV_Servos::toggle()
{
    static uint32_t last_toggle_time = 0;

    if(hal.scheduler->millis() - last_toggle_time < 1000) {
        //function has been called and executed recently. debounce by rejecting second call
        return;
    }
    //executing. mark time
    last_toggle_time = hal.scheduler->millis();

    //if greater than RC_MID, go low. Otherwise go high.
    //channel 10
    if(hal.rcout->read(10) > RC_Channel::rc_channel(10)->radio_trim) {
        output(3, RC_Channel::rc_channel(10)->radio_min);
    } else {
        output(3, RC_Channel::rc_channel(10)->radio_max);
    }

    //channel 11
    if(hal.rcout->read(11) > RC_Channel::rc_channel(11)->radio_trim) {
        output(4, RC_Channel::rc_channel(11)->radio_min);
    } else {
        output(4, RC_Channel::rc_channel(11)->radio_max);
    }
}

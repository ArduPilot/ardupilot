// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
 *
 *  All APM Project credits from the original work are kept intact below as a
 *  courtesy.
 */

/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <AP_HAL.h>
#include "AP_Relay.h"

//51:Pixhawk FMU AUX2,52:Pixhawk FMU AUX3,53:Pixhawk FMU AUX4,54:Pixhawk FMU AUX5,55:Pixhawk FMU AUX6
#define RELAY_PIN (55) //Aux 6

extern const AP_HAL::HAL& hal;

AP_Relay::AP_Relay(void)
{
    ;
}


void AP_Relay::init() 
{
    off();
}

void AP_Relay::on()
{    
    hal.gpio->pinMode(RELAY_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(RELAY_PIN, 1);
}


void AP_Relay::off()
{
    hal.gpio->pinMode(RELAY_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(RELAY_PIN, 0);
}


void AP_Relay::toggle()
{
    bool ison = hal.gpio->read(RELAY_PIN);
    if (ison)
        off();
    else
        on();
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.cpp
 *
 *  Created on: DEC 6, 2013
 *      Author: Andreas Jochum
 */

#include <AP_HAL.h>
#include "AP_EPM.h"

extern const AP_HAL::HAL& hal;

AP_EPM::AP_EPM(void)
{	
	//do nothing?
}


void AP_EPM::init() 
{
	 if (EPM_PIN_1 != -1 && EPM_PIN_1 != -1) {
        hal.gpio->pinMode(EPM_PIN_1, GPIO_OUTPUT);
		hal.gpio->pinMode(EPM_PIN_2, GPIO_OUTPUT);
				
        neutral();
    }
	
}

void AP_EPM::on() 
{    
   if (EPM_PIN_1 != -1 && EPM_PIN_1 != -1) {
        hal.gpio->write(EPM_PIN_1, 1);
		hal.gpio->write(EPM_PIN_2, 0);
	}
}


void AP_EPM::off() 
{
   if (EPM_PIN_1 != -1 && EPM_PIN_1 != -1) {
        hal.gpio->write(EPM_PIN_1, 0);
		hal.gpio->write(EPM_PIN_2, 1);
	}
}

void AP_EPM::neutral() 
{
  if (EPM_PIN_1 != -1 && EPM_PIN_1 != -1) {
        hal.gpio->write(EPM_PIN_1, 0);
		hal.gpio->write(EPM_PIN_2, 0);
	}
}



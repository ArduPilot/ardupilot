/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED
void Copter::init_precland()
{
	copter.precland.init();
}
void Copter::update_precland()
{

	int16_t rng_alt = read_sonar();
	int16_t final_alt;

	if(rng_alt != 0){ //Use rangefinder altitude if it is valid

 #if SONAR_TILT_CORRECTION != 1
    // correct alt for angle of the rangefinder if it hasn't arleady happened
    float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    rng_alt = rng_alt * temp;
 #endif

		final_alt = rng_alt;
	}
	else{ 			//use gps/baro alt otherwise
		final_alt = current_loc.alt;
	}


	copter.precland.update(final_alt);

	// log output
	Log_Write_Precland();
}
#endif

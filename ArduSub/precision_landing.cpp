/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// functions to support precision landing
//

#include "Sub.h"

#if PRECISION_LANDING == ENABLED

void Sub::init_precland()
{
    sub.precland.init();
}

void Sub::update_precland()
{
	int32_t height_above_ground_cm = current_loc.alt;

	// use range finder altitude if it is valid, else try to get terrain alt
	if (rangefinder_alt_ok()) {
		height_above_ground_cm = rangefinder_state.alt_cm;
	} else if (terrain_use()) {
		current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, height_above_ground_cm);
    }

    sub.precland.update(height_above_ground_cm);

    // log output
    Log_Write_Precland();
}
#endif

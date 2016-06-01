/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// Code by Rustom Jehangir: rusty@bluerobotics.com

#include "Sub.h"

// Count total vehicle turns to avoid tangling tether
void Sub::update_turn_counter()
{
	// If yaw went from negative to positive (right turn)
	if ( ahrs.yaw > 0 && last_turn_count_yaw < 0 ) {
		turn_count++;
		gcs_send_text_fmt(MAV_SEVERITY_INFO,"Turn count: %i turns to the right",turn_count);
	}
	// If yaw went from positive to negative (left turn)
	if ( ahrs.yaw < 0 && last_turn_count_yaw > 0 ) {
		turn_count--;
		gcs_send_text_fmt(MAV_SEVERITY_INFO,"Turn count: %i turns to the right",turn_count);
	}
	last_turn_count_yaw = ahrs.yaw;
}

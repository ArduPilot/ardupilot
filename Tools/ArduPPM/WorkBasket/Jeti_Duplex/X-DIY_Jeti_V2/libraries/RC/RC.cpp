/*
	AP_RC.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/



#include "RC.h"

/*
RC::RC()// :
		_direction_mask(255)
{
}
*/

// direction mask: 0 = normal, 1 = reversed servos
void
RC::set_channel_direction(uint8_t ch, int8_t dir)
{
	uint8_t bitflip = 1 << ch;
	
	if(dir == 1){
		_direction_mask |= bitflip;	
	}else{
		_direction_mask &= ~bitflip;	
	}
}

void
RC::set_failsafe(uint16_t v)
{
	_fs_value = v;
}

void
RC::set_mix_mode(uint8_t m)
{
	_mix_mode = m;
}


void
RC::check_throttle_failsafe(uint16_t throttle)
{
	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (throttle < _fs_value){
		// we detect a failsafe from radio 
		// throttle has dropped below the mark
		_fs_counter++;
		if (_fs_counter == 9){
		
		}else if(_fs_counter == 10) {
			failsafe = 1;
		}else if (_fs_counter > 10){
			_fs_counter = 11;
		}
		
	}else if(_fs_counter > 0){
		// we are no longer in failsafe condition
		// but we need to recover quickly		
		_fs_counter--;
		if (_fs_counter > 3){
			_fs_counter = 3;
		}		
		if (_fs_counter == 1){
		
		}else if(_fs_counter == 0) {
			failsafe = 0;
		}else if (_fs_counter <0){
			_fs_counter = -1;
		}
	}
}




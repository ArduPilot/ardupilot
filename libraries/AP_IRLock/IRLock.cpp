/*
 * IRLock.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: MLandes
 */

#include "IRLock.h"

// not sure what is going on here...
//const AP_Param::GroupInfo IRLock::var_info[] PROGMEM = {
//		// @Param: ENABLE
//		// @DisplayName: Optical flow enable/disable
//		// @Description: Setting thisto Enabled(1) will enable irlock. Setting this to Disabled(0) will disable irlock.
//		// @Values: 0:Disabled, 1:Enabled
//		// @user: Standard
//		AP_GROUPINFO("_ENABLE", 0, IRLock, _enabled, 0),
//
//		AP_GROUPEND
//};

// default constructor
IRLock::IRLock(const AP_AHRS &ahrs) :
		_ahrs(ahrs),
		_last_update(0),
		_num_blocks(0)
{
	// can't figure out how to get this to set to enabled, so doing it manually
//	AP_Param::setup_object_defaults(this, var_info);
	_enabled = true;

	// clear the frame buffer
	for(int i = 0; i < IRLOCK_MAX_BLOCKS_PER_FRAME; ++i) {
		memset(&(_current_frame[i]), 0, sizeof(irlock_block));
	}

	// will be adjusted when init is called
	_flags.healthy = false;
}

IRLock::~IRLock() {}

void IRLock::get_current_frame(irlock_block data[IRLOCK_MAX_BLOCKS_PER_FRAME]) const
{
	int index = 0;
	for (; index < _num_blocks; ++index) {
		data[index].signature = _current_frame[index].signature;
		data[index].center_x = _current_frame[index].center_x;
		data[index].center_y = _current_frame[index].center_y;
		data[index].width = _current_frame[index].width;
		data[index].height = _current_frame[index].height;
	}

	for (; index < IRLOCK_MAX_BLOCKS_PER_FRAME; ++index) {
		data[index].signature = 0;
		data[index].center_x = 0;
		data[index].center_y = 0;
		data[index].width = 0;
		data[index].height = 0;
	}
}

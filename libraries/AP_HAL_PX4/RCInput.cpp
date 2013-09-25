#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

void PX4RCInput::init(void* unused)
{
	init_overrides(_override, RC_INPUT_MAX_CHANNELS);
	_perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	if (_rc_sub == -1) {
		hal.scheduler->panic("Unable to subscribe to input_rc");		
	}
	clear_overrides();
}

uint8_t PX4RCInput::valid_channels() 
{
	return _rcin.timestamp != _last_read || _override_valid;
}

uint16_t PX4RCInput::read(uint8_t ch) 
{
	if (ch >= RC_INPUT_MAX_CHANNELS) {
		return 0;
	}
	_last_read = _rcin.timestamp;
	_override_valid = false;
	if (_override[ch]) {
		return _override[ch];
	}
	if (ch >= _rcin.channel_count) {
		return 0;
	}
	return _rcin.values[ch];
}

uint8_t PX4RCInput::read(uint16_t* periods, uint8_t len) 
{
	if (len > RC_INPUT_MAX_CHANNELS) {
		len = RC_INPUT_MAX_CHANNELS;
	}
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
	}
	return len;
}

void PX4RCInput::_timer_tick(void)
{
	perf_begin(_perf_rcin);
	bool rc_updated = false;
	if (orb_check(_rc_sub, &rc_updated) == 0 && rc_updated) {
		orb_copy(ORB_ID(input_rc), _rc_sub, &_rcin);
		_last_input = _rcin.timestamp;
	} else if (hrt_absolute_time() - _last_input > 300000) {
		// we've lost RC input, force channel 3 low
		_rcin.values[2] = 900;
	}
	perf_end(_perf_rcin);
}

#endif

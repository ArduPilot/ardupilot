#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

#include <stdio.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

void PX4RCInput::init(void* unused)
{
	_perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	if (_rc_sub == -1) {
		hal.scheduler->panic("Unable to subscribe to input_rc");		
	}

        _status_sub = orb_subscribe(ORB_ID(rc_status));
        if (_status_sub == -1) {
                hal.scheduler->panic("Unable to subscribe to rc_status");
        }

	clear_overrides();
        pthread_mutex_init(&rcin_mutex, NULL);
}

uint8_t PX4RCInput::valid_channels() 
{
    pthread_mutex_lock(&rcin_mutex);
    bool valid = _rcin.timestamp != _last_read || _status.timestamp != _last_read || _override_valid;
    pthread_mutex_unlock(&rcin_mutex);
    return valid;
}

uint16_t PX4RCInput::read(uint8_t ch) 
{
	if (ch >= RC_INPUT_MAX_CHANNELS) {
		return 0;
	}
        pthread_mutex_lock(&rcin_mutex);
	_last_read = _rcin.timestamp;
	_override_valid = false;
	if (_override[ch]) {
            uint16_t v = _override[ch];
            pthread_mutex_unlock(&rcin_mutex);
            return v;
	}
	if (ch >= _rcin.channel_count) {
            pthread_mutex_unlock(&rcin_mutex);
            return 0;
	}
	uint16_t v = _rcin.values[ch];
        pthread_mutex_unlock(&rcin_mutex);
        return v;
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

bool PX4RCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
	bool res = false;
	for (uint8_t i = 0; i < len; i++) {
		res |= set_override(i, overrides[i]);
	}
	return res;
}

bool PX4RCInput::set_override(uint8_t channel, int16_t override) {
	if (override < 0) {
		return false; /* -1: no change. */
	}
	if (channel >= RC_INPUT_MAX_CHANNELS) {
		return false;
	}
	_override[channel] = override;
	if (override != 0) {
		_override_valid = true;
		return true;
	}
	return false;
}

void PX4RCInput::clear_overrides()
{
	for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
		set_override(i, 0);
	}
}

void PX4RCInput::_timer_tick(void)
{
	perf_begin(_perf_rcin);

	// check for status and channel values and update them consistently
	bool rc_updated = orb_check(_rc_sub, &rc_updated) == 0 && rc_updated;
	bool status_updated = orb_check(_status_sub, &status_updated) == 0 && status_updated;
	if (rc_updated || status_updated) {
		pthread_mutex_lock(&rcin_mutex);
		orb_copy(ORB_ID(input_rc), _rc_sub, &_rcin);
		orb_copy(ORB_ID(rc_status), _status_sub, &_status);

		// pull throttle channel low for easy and intuitive failsafe testing, preserve 
		// all other channel values, e.g. to lower retracts on failsafe
		if (_status.rc_lost) {
			// we've lost RC input, RC receiver failed or cable break
			// force channel 3 low
			_rcin.values[2] = 900;
		}
		else if (_status.rc_failsafe) {
			// we got a valid RC signal, but it contains a failsafe flag (e.g. TX switched off or out of range)
			// force channel 3 low
			// slightly different value allows to map failsafe action only to rc_lost state
			_rcin.values[2] = 910;
		}
		pthread_mutex_unlock(&rcin_mutex);
	}
	perf_end(_perf_rcin);
}

#endif

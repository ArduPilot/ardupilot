#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <drivers/drv_hrt.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

extern uint16_t ppm_buffer[];
extern unsigned ppm_decoded_channels;
extern uint64_t ppm_last_valid_decode;

void PX4RCInput::init(void* unused)
{
	clear_overrides();
}

uint8_t PX4RCInput::valid() {
	return ppm_last_valid_decode != _last_read;
}

uint16_t PX4RCInput::read(uint8_t ch) {
	_last_read = ppm_last_valid_decode;
	if (ch >= PX4_NUM_RCINPUT_CHANNELS) {
		return 0;
	}
	if (_override[ch]) {
		return _override[ch];
	}
	return ppm_buffer[ch];
}

uint8_t PX4RCInput::read(uint16_t* periods, uint8_t len) {
	if (len > PX4_NUM_RCINPUT_CHANNELS) {
		len = PX4_NUM_RCINPUT_CHANNELS;
	}
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
	}
	return len;
}

bool PX4RCInput::set_overrides(int16_t *overrides, uint8_t len) {
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
	if (channel < PX4_NUM_RCINPUT_CHANNELS) {
		_override[channel] = override;
		if (override != 0) {
			return true;
		}
	}
	return false;
}

void PX4RCInput::clear_overrides()
{
	for (uint8_t i = 0; i < PX4_NUM_RCINPUT_CHANNELS; i++) {
		_override[i] = 0;
	}
}

#endif

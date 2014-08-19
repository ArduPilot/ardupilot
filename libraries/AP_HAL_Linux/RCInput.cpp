#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>

#include "RCInput.h"

using namespace Linux;

LinuxRCInput::LinuxRCInput() :
    new_rc_input(false),
    _channel_counter(-1)
{}

void LinuxRCInput::init(void* machtnichts)
{
}

bool LinuxRCInput::new_input() 
{
    return new_rc_input;
}

uint8_t LinuxRCInput::num_channels() 
{
    return _num_channels;
}

uint16_t LinuxRCInput::read(uint8_t ch) 
{
    new_rc_input = false;
    if (_override[ch]) {
        return _override[ch];
    }
    
    return _pulse_capt[ch];
}

uint8_t LinuxRCInput::read(uint16_t* periods, uint8_t len) 
{
    uint8_t i;
    for (i=0; i<len; i++) {
        if((periods[i] = read(i))){
            continue;
        }
        else{
            break;
        }
    }
    return (i+1);
}

bool LinuxRCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
    bool res = false;
    if(len > LINUX_RC_INPUT_NUM_CHANNELS){
        len = LINUX_RC_INPUT_NUM_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool LinuxRCInput::set_override(uint8_t channel, int16_t override) 
{
    if (override < 0) return false; /* -1: no change. */
    if (channel < LINUX_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            new_rc_input = true;
            return true;
        }
    }
    return false;
}

void LinuxRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
       _override[i] = 0;
    }
}


/*
  process a pulse of the given width
 */
void LinuxRCInput::_process_ppmsum_pulse(uint16_t width_usec)
{
    if (width_usec >= 4000) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (_channel_counter != -1) {
            new_rc_input = true;
            _num_channels = _channel_counter;
        }
        _channel_counter = 0;
        return;
    }
    if (_channel_counter == -1) {
        // we are not synchronised
        return;
    }

    // take a reading for the current channel
    _pulse_capt[_channel_counter] = width_usec;

    // move to next channel
    _channel_counter++;

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (_channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
        new_rc_input = true;
        _channel_counter = -1;
        _num_channels = _channel_counter;
    }
}

#endif // CONFIG_HAL_BOARD

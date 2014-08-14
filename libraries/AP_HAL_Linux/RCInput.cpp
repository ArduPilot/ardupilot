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
new_rc_input(false)
{}

void LinuxRCInput::init(void* machtnichts)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    ring_buffer = (volatile struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
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

void LinuxRCInput::_timer_tick()
{
    uint8_t channel_ctr;
    uint16_t s1_time, s2_time;
    //scan for the latest start pulse
    while(ring_buffer->buffer[ring_buffer->ring_head].delta_t < 8000){
        ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;
    }
    for(channel_ctr = 0; channel_ctr < LINUX_RC_INPUT_NUM_CHANNELS; channel_ctr++){
        ring_buffer->ring_head = (ring_buffer->ring_head + 2) % NUM_RING_ENTRIES;
        //wait until we receive two pulse_width(State1 & State2) values inside buffer
        while(ring_buffer->ring_head <= (ring_buffer->ring_tail));
        s1_time = (uint32_t)ring_buffer->buffer[ring_buffer->ring_head - 1].delta_t;
        s2_time = (uint32_t)ring_buffer->buffer[ring_buffer->ring_head].delta_t;
        _pulse_capt[channel_ctr] = s1_time + s2_time;
        if(_pulse_capt[channel_ctr] > RC_INPUT_MAX_PULSEWIDTH){
            _pulse_capt[channel_ctr] = RC_INPUT_MAX_PULSEWIDTH;
        }
        else if(_pulse_capt[channel_ctr] < RC_INPUT_MIN_PULSEWIDTH){
            _pulse_capt[channel_ctr] = RC_INPUT_MIN_PULSEWIDTH;
        }
    }    
}

#endif // CONFIG_HAL_BOARD

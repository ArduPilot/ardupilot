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
#include "sbus.h"

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
  process a PPM-sum pulse of the given width
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

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        _pulse_capt[_channel_counter] = width_usec;

        // move to next channel
        _channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (_channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
        new_rc_input = true;
        _channel_counter = -1;
        _num_channels = _channel_counter;
    }
}

/*
  process a SBUS input pulse of the given width
 */
void LinuxRCInput::_process_sbus_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    uint16_t bits_s0 = (width_s0+1) / 10;
    uint16_t bits_s1 = (width_s1+1) / 10;
    uint16_t nlow;

    uint8_t byte_ofs = sbus_state.bit_ofs/12;
    uint8_t bit_ofs = sbus_state.bit_ofs%12;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }
	
    if (bits_s0+bit_ofs > 10) {
        // invalid data as last two bits must be stop bits
        goto reset;
    }

    // pull in the high bits
    sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
    sbus_state.bit_ofs += bits_s0;
    bit_ofs += bits_s0;

    // pull in the low bits
    nlow = bits_s1;
    if (nlow + bit_ofs > 12) {
        nlow = 12 - bit_ofs;
    }
    bits_s1 -= nlow;
    sbus_state.bit_ofs += nlow;

    if (sbus_state.bit_ofs == 25*12 && bits_s1 > 12) {
        // we have a full frame
        uint8_t bytes[25];
        uint8_t i;
        for (i=0; i<25; i++) {
            // get inverted data
            uint16_t v = ~sbus_state.bytes[i];
            // check start bit
            if ((v & 1) != 0) {
                goto reset;
            }
            // check stop bits
            if ((v & 0xC00) != 0xC00) {
                goto reset;
            }
            // check parity
            uint8_t parity = 0, j;
            for (j=1; j<=8; j++) {
                parity ^= (v & (1U<<j))?1:0;
            }
            if (parity != (v&0x200)>>9) {
                goto reset;
            }
            bytes[i] = ((v>>1) & 0xFF);
        }
        uint16_t values[LINUX_RC_INPUT_NUM_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe=false, sbus_frame_drop=false;
        if (sbus_decode(bytes, values, &num_values, 
                        &sbus_failsafe, &sbus_frame_drop, 
                        LINUX_RC_INPUT_NUM_CHANNELS)) {
            for (i=0; i<num_values; i++) {
                _pulse_capt[i] = values[i];
            }
            _num_channels = num_values;
        }
        goto reset;
    } else if (bits_s1 > 12) {
        // break
        goto reset;
    }
    return;
reset:
    memset(&sbus_state, 0, sizeof(sbus_state));        
}

/*
  process a RC input pulse of the given width
 */
void LinuxRCInput::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // treat as PPM-sum
    _process_ppmsum_pulse(width_s0 + width_s1);

    // treat as SBUS
    _process_sbus_pulse(width_s0, width_s1);
}

#endif // CONFIG_HAL_BOARD

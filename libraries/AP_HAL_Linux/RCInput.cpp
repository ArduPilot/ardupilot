#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/dsm.h>
#include <AP_HAL/utility/sumd.h>
#include <AP_HAL/utility/st24.h>
#include <AP_HAL/utility/srxl.h>
#include <AP_RCProtocol/AP_RCProtocol_SBUS.h>

#include "RCInput.h"

#define MIN_NUM_CHANNELS 5

extern const AP_HAL::HAL& hal;

using namespace Linux;

RCInput::RCInput()
{
    ppm_state._channel_counter = -1;
}

void RCInput::init()
{
}

bool RCInput::new_input()
{
    bool ret = rc_input_count != last_rc_input_count;
    if (ret) {
        last_rc_input_count.store(rc_input_count);
    }
    return ret;
}

uint8_t RCInput::num_channels()
{
    return _num_channels;
}

void RCInput::set_num_channels(uint8_t num)
{
    _num_channels = num;
}

uint16_t RCInput::read(uint8_t ch)
{
    if (ch >= _num_channels) {
        return 0;
    }
    return _pwm_values[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    uint8_t i;
    for (i=0; i<len; i++) {
        periods[i] = read(i);
    }
    return len;
}

/*
  process a PPM-sum pulse of the given width
 */
void RCInput::_process_ppmsum_pulse(uint16_t width_usec)
{
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        if (ppm_state._channel_counter >= MIN_NUM_CHANNELS) {
            for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
                _pwm_values[i] = ppm_state._pulse_capt[i];
            }
            _num_channels = ppm_state._channel_counter;
            rc_input_count++;
        }
        ppm_state._channel_counter = 0;
        return;
    }
    if (ppm_state._channel_counter == -1) {
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
        // buffer these
        ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

        // move to next channel
        ppm_state._channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (ppm_state._channel_counter >= LINUX_RC_INPUT_NUM_CHANNELS) {
        for (uint8_t i=0; i<ppm_state._channel_counter; i++) {
            _pwm_values[i] = ppm_state._pulse_capt[i];
        }
        _num_channels = ppm_state._channel_counter;
        rc_input_count++;
        ppm_state._channel_counter = -1;
    }
}

/*
  process a SBUS input pulse of the given width
 */
void RCInput::_process_sbus_pulse(uint16_t width_s0, uint16_t width_s1)
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
        bool sbus_failsafe=false;
        if (AP_RCProtocol_SBUS::sbus_decode(bytes, values, &num_values,
                                            sbus_failsafe,
                                            LINUX_RC_INPUT_NUM_CHANNELS) &&
            num_values >= MIN_NUM_CHANNELS) {
            for (i=0; i<num_values; i++) {
                _pwm_values[i] = values[i];
            }
            _num_channels = num_values;
            if (!sbus_failsafe) {
                rc_input_count++;
            }
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

void RCInput::_process_dsm_pulse(uint16_t width_s0, uint16_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 115200 bps
    uint16_t bits_s0 = ((width_s0+4)*(uint32_t)115200) / 1000000;
    uint16_t bits_s1 = ((width_s1+4)*(uint32_t)115200) / 1000000;
    uint8_t bit_ofs, byte_ofs;
    uint16_t nbits;

    if (bits_s0 == 0 || bits_s1 == 0) {
        // invalid data
        goto reset;
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;

    if(byte_ofs > 15) {
        // invalid data
        goto reset;
    }

    // pull in the high bits
    nbits = bits_s0;
    if (nbits+bit_ofs > 10) {
        nbits = 10 - bit_ofs;
    }
    dsm_state.bytes[byte_ofs] |= ((1U<<nbits)-1) << bit_ofs;
    dsm_state.bit_ofs += nbits;
    bit_ofs += nbits;

    if (bits_s0 - nbits > 10) {
        if (dsm_state.bit_ofs == 16*10) {
            // we have a full frame
            uint8_t bytes[16];
            uint8_t i;
            for (i=0; i<16; i++) {
                // get raw data
                uint16_t v = dsm_state.bytes[i];

                // check start bit
                if ((v & 1) != 0) {
                    goto reset;
                }
                // check stop bits
                if ((v & 0x200) != 0x200) {
                    goto reset;
                }
                bytes[i] = ((v>>1) & 0xFF);
            }
            uint16_t values[8];
            uint16_t num_values=0;
            if (dsm_decode(AP_HAL::micros64(), bytes, values, &num_values, 8) &&
                num_values >= MIN_NUM_CHANNELS) {
                for (i=0; i<num_values; i++) {
                    _pwm_values[i] = values[i];
                }
                _num_channels = num_values;
                rc_input_count++;
            }
        }
        memset(&dsm_state, 0, sizeof(dsm_state));
    }

    byte_ofs = dsm_state.bit_ofs/10;
    bit_ofs = dsm_state.bit_ofs%10;

    if (bits_s1+bit_ofs > 10) {
        // invalid data
        goto reset;
    }

    // pull in the low bits
    dsm_state.bit_ofs += bits_s1;
    return;
reset:
    memset(&dsm_state, 0, sizeof(dsm_state));
}

void RCInput::_process_pwm_pulse(uint16_t channel, uint16_t width_s0, uint16_t width_s1)
{
    if (channel < _num_channels) {
        _pwm_values[channel] = width_s1; // range: 700usec ~ 2300usec
        rc_input_count++;
    }
}

/*
  process a RC input pulse of the given width
 */
void RCInput::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
#if 0
    // useful for debugging
    static FILE *rclog;
    if (rclog == nullptr) {
        rclog = fopen("/tmp/rcin.log", "w");
    }
    if (rclog) {
        fprintf(rclog, "%u %u\n", (unsigned)width_s0, (unsigned)width_s1);
    }
#endif
    // treat as PPM-sum
    _process_ppmsum_pulse(width_s0 + width_s1);

    // treat as SBUS
    _process_sbus_pulse(width_s0, width_s1);

    // treat as DSM
    _process_dsm_pulse(width_s0, width_s1);
}

/*
 * Update channel values directly
 */
void RCInput::_update_periods(uint16_t *periods, uint8_t len)
{
    if (len > LINUX_RC_INPUT_NUM_CHANNELS) {
        len = LINUX_RC_INPUT_NUM_CHANNELS;
    }
    for (unsigned int i=0; i < len; i++) {
        _pwm_values[i] = periods[i];
    }
    _num_channels = len;
    rc_input_count++;
}

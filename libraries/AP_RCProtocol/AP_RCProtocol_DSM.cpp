/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
/*
 * See https://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf for official
 * Spektrum documentation on the format.
 */

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_RCProtocol_DSM.h"
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
#include "AP_RCProtocol_SRXL2.h"
#endif
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// #define DSM_DEBUG
#ifdef DSM_DEBUG
# define debug(fmt, args...)	printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif


#define DSM_FRAME_SIZE		16		/**<DSM frame size in bytes*/
#define DSM_FRAME_CHANNELS	7		/**<Max supported DSM channels*/
#define DSM2_1024_22MS      0x01
#define DSM2_2048_11MS      0x12
#define DSMX_2048_22MS      0xa2
#define DSMX_2048_11MS      0xb2
#define SPEKTRUM_VTX_CONTROL_FRAME_MASK 0xf000f000
#define SPEKTRUM_VTX_CONTROL_FRAME      0xe000e000

#define SPEKTRUM_VTX_BAND_MASK          0x00e00000
#define SPEKTRUM_VTX_CHANNEL_MASK       0x000f0000
#define SPEKTRUM_VTX_PIT_MODE_MASK      0x00000010
#define SPEKTRUM_VTX_POWER_MASK         0x00000007

#define SPEKTRUM_VTX_BAND_SHIFT         21
#define SPEKTRUM_VTX_CHANNEL_SHIFT      16
#define SPEKTRUM_VTX_PIT_MODE_SHIFT     4
#define SPEKTRUM_VTX_POWER_SHIFT        0

void AP_RCProtocol_DSM::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
bool AP_RCProtocol_DSM::dsm_decode(uint32_t frame_time_us, const uint8_t dsm_frame[16],
                                   uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
    /* we have received something we think is a dsm_frame */
    last_frame_time_us = frame_time_us;
    // Get the VTX control bytes in a frame
    uint32_t vtxControl = ((dsm_frame[AP_DSM_FRAME_SIZE-4] << 24)
        | (dsm_frame[AP_DSM_FRAME_SIZE-3] << 16)
        | (dsm_frame[AP_DSM_FRAME_SIZE-2] <<  8)
        | (dsm_frame[AP_DSM_FRAME_SIZE-1] <<  0));

    uint8_t dsm_frame_data_size;
    // Handle VTX control frame.
    if ((vtxControl & SPEKTRUM_VTX_CONTROL_FRAME_MASK) == SPEKTRUM_VTX_CONTROL_FRAME 
        && (dsm_frame[2] & 0x80) == 0)  {
        dsm_frame_data_size = AP_DSM_FRAME_SIZE - 4;
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
        AP_RCProtocol_SRXL2::configure_vtx(
            (vtxControl & SPEKTRUM_VTX_BAND_MASK)     >> SPEKTRUM_VTX_BAND_SHIFT,
            (vtxControl & SPEKTRUM_VTX_CHANNEL_MASK)  >> SPEKTRUM_VTX_CHANNEL_SHIFT,
            (vtxControl & SPEKTRUM_VTX_POWER_MASK)    >> SPEKTRUM_VTX_POWER_SHIFT,
            (vtxControl & SPEKTRUM_VTX_PIT_MODE_MASK) >> SPEKTRUM_VTX_PIT_MODE_SHIFT);
#endif
    } else {
        dsm_frame_data_size = AP_DSM_FRAME_SIZE;
    }

    // Get the RC control channel inputs
    for (uint8_t b = 3; b < dsm_frame_data_size; b += 2) {
        uint8_t channel = 0x0F & (dsm_frame[b - 1] >> channel_shift);

        uint32_t value = ((uint32_t)(dsm_frame[b - 1] & channel_mask) << 8) + dsm_frame[b];

        /* ignore channels out of range */
        if (channel >= max_values) {
            continue;
        }

        /* update the decoded channel count */
        if (channel >= *num_values) {
            *num_values = channel + 1;
        }

        /* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
        if (channel_shift == 2) {
            value *= 2;
        }

        /* Spektrum scaling is defined as (see reference):
            2048: PWM_OUT = (ServoPosition x 58.3μs) + 903
            1024: PWM_OUT = (ServoPosition x 116.6μs) + 903 */
        /* scaled integer for decent accuracy while staying efficient */
        value = ((int32_t)value * 1194) / 2048 + 903;

        /*
        * Store the decoded channel into the R/C input buffer, taking into
        * account the different ideas about channel assignement that we have.
        *
        * Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
        * but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
        */
        switch (channel) {
        case 0:
            channel = 2;
            break;

        case 1:
            channel = 0;
            break;

        case 2:
            channel = 1;
            break;

        default:
            break;
        }

        values[channel] = value;
    }

    /*
     * XXX Note that we may be in failsafe here; we need to work out how to detect that.
     */
    return true;
}


/*
  start bind on DSM satellites
 */
void AP_RCProtocol_DSM::start_bind(void)
{
    bind_state = BIND_STATE1;
}


/*
  update function used for bind state machine
 */
void AP_RCProtocol_DSM::update(void)
{
#if defined(HAL_GPIO_SPEKTRUM_PWR) && defined(HAL_GPIO_SPEKTRUM_RC)
    switch (bind_state) {
    case BIND_STATE_NONE:
        break;

    case BIND_STATE1:
        hal.gpio->write(HAL_GPIO_SPEKTRUM_PWR, !HAL_SPEKTRUM_PWR_ENABLED);
        hal.gpio->pinMode(HAL_GPIO_SPEKTRUM_RC, 1);
        hal.gpio->write(HAL_GPIO_SPEKTRUM_RC, 1);
        bind_last_ms = AP_HAL::millis();
        bind_state = BIND_STATE2;
        break;

    case BIND_STATE2: {
        uint32_t now = AP_HAL::millis();
        if (now - bind_last_ms > 500) {
            hal.gpio->write(HAL_GPIO_SPEKTRUM_PWR, HAL_SPEKTRUM_PWR_ENABLED);
            bind_last_ms = now;
            bind_state = BIND_STATE3;
        }
        break;
    }

    case BIND_STATE3: {
        uint32_t now = AP_HAL::millis();
        if (now - bind_last_ms > 72) {
            // 9 pulses works with all satellite receivers, and supports the highest
            // available protocol
            const uint8_t num_pulses = 9;
            for (uint8_t i=0; i<num_pulses; i++) {
                hal.scheduler->delay_microseconds(120);
                hal.gpio->write(HAL_GPIO_SPEKTRUM_RC, 0);
                hal.scheduler->delay_microseconds(120);
                hal.gpio->write(HAL_GPIO_SPEKTRUM_RC, 1);
            }
            bind_last_ms = now;
            bind_state = BIND_STATE4;
        }
        break;
    }

    case BIND_STATE4: {
        uint32_t now = AP_HAL::millis();
        if (now - bind_last_ms > 50) {
            hal.gpio->pinMode(HAL_GPIO_SPEKTRUM_RC, 0);
            bind_state = BIND_STATE_NONE;
        }
        break;
    }
    }
#endif
}

/*
  parse one DSM byte, maintaining decoder state
 */
bool AP_RCProtocol_DSM::dsm_parse_byte(uint32_t frame_time_us, uint8_t b, uint16_t *values,
                                       uint16_t *num_values, uint16_t max_channels)
{
    /* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

    // we took too long decoding, start again
    if (byte_input.ofs > 0 && (frame_time_us - start_frame_time_us) > 6000U) {
        start_frame_time_us = frame_time_us;
        byte_input.ofs = 0;
    }

    // there will be at least a 5ms gap between successive DSM frames. if we see it
    // assume we are starting a new frame
    if ((frame_time_us - last_rx_time_us) > 5000U) {
        start_frame_time_us = frame_time_us;
        byte_input.ofs = 0;
    }

    /* overflow check */
    if (byte_input.ofs >= AP_DSM_FRAME_SIZE) {
        start_frame_time_us = frame_time_us;
        byte_input.ofs = 0;
    }

    if (byte_input.ofs == 1) {
        // saw a beginning of frame marker
        if (b == DSM2_1024_22MS || b == DSM2_2048_11MS || b == DSMX_2048_22MS || b == DSMX_2048_11MS) {
            if (b == DSM2_1024_22MS) {
                // 10 bit frames
                channel_shift = 2;
                channel_mask = 0x03;
            } else {
                // 11 bit frames
                channel_shift = 3;
                channel_mask = 0x07;
            }
        // bad frame marker so reset
        } else {
            start_frame_time_us = frame_time_us;
            byte_input.ofs = 0;
        }
    }

    byte_input.buf[byte_input.ofs++] = b;

    /* decode whatever we got and expect */
    if (byte_input.ofs == AP_DSM_FRAME_SIZE) {
        log_data(AP_RCProtocol::DSM, frame_time_us, byte_input.buf, byte_input.ofs);
#ifdef DSM_DEBUG
        for (uint16_t i = 0; i < 16; i++) {
            printf("%02x", byte_input.buf[i]);
        }
        printf("\n%02x%02x", byte_input.buf[0], byte_input.buf[1]);
        for (uint16_t i = 2; i < 16; i+=2) {
            printf(" %01x/%03x", (byte_input.buf[i] & 0x78) >> 4, (byte_input.buf[i] & 0x7) << 8 | byte_input.buf[i+1]);
        }
        printf("\n");
#endif
        decode_ret = dsm_decode(frame_time_us, byte_input.buf, values, &chan_count, max_channels);

        /* we consumed the partial frame, reset */
        byte_input.ofs = 0;
    }

    if (decode_ret) {
		*num_values = chan_count;
	}

    last_rx_time_us = frame_time_us;

	/* return false as default */
	return decode_ret;
}

// support byte input
void AP_RCProtocol_DSM::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    uint16_t v[AP_DSM_MAX_CHANNELS];
    uint16_t nchan;
    memcpy(v, last_values, sizeof(v));
    if (dsm_parse_byte(timestamp_us, b, v, &nchan, AP_DSM_MAX_CHANNELS)) {
        memcpy(last_values, v, sizeof(v));
        if (nchan >= MIN_RCIN_CHANNELS) {
            add_input(nchan, last_values, false);
        }
    }
}

// support byte input
void AP_RCProtocol_DSM::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}

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
  with thanks to PX4 dsm.c for DSM decoding approach
 */
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_RCProtocol_DSM.h"
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
#include "AP_RCProtocol_SRXL2.h"
#endif

#include <AP_VideoTX/AP_VideoTX_config.h>

extern const AP_HAL::HAL& hal;

// #define DSM_DEBUG
#ifdef DSM_DEBUG
#include <stdio.h>
# define debug(fmt, args...)	printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif


#define DSM_FRAME_SIZE		16		/**<DSM frame size in bytes*/
#define DSM_FRAME_CHANNELS	7		/**<Max supported DSM channels*/
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
        _process_byte(ss.get_byte_timestamp_us()/1000U, b);
    }
}

/**
 * Attempt to decode a single channel raw channel datum
 *
 * The DSM* protocol doesn't provide any explicit framing,
 * so we detect dsm frame boundaries by the inter-dsm frame delay.
 *
 * The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
 * dsm frame transmission time is ~1.4ms.
 *
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a dsm frame.
 *
 * In the case where byte(s) are dropped from a dsm frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 *
 * Upon receiving a full dsm frame we attempt to decode it
 *
 * @param[in] raw 16 bit raw channel value from dsm frame
 * @param[in] shift position of channel number in raw data
 * @param[out] channel pointer to returned channel number
 * @param[out] value pointer to returned channel value
 * @return true=raw value successfully decoded
 */
bool AP_RCProtocol_DSM::dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

    if (raw == 0xffff) {
        return false;
    }

    *channel = (raw >> shift) & 0xf;

    uint16_t data_mask = (1 << shift) - 1;
    *value = raw & data_mask;

    //debug("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

    return true;
}

/**
 * Attempt to guess if receiving 10 or 11 bit channel values
 *
 * @param[in] reset true=reset the 10/11 bit state to unknown
 */
void AP_RCProtocol_DSM::dsm_guess_format(bool reset, const uint8_t dsm_frame[16], unsigned frame_channels)
{
    /* reset the 10/11 bit sniffed channel masks */
    if (reset) {
        cs10 = 0;
        cs11 = 0;
        samples = 0;
        channel_shift = 0;
        return;
    }

    /* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
    for (unsigned i = 0; i < frame_channels; i++) {

        const uint8_t *dp = &dsm_frame[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;

        /* if the channel decodes, remember the assigned number */
        if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 16)) {
            cs10 |= (1 << channel);
        }

        if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 16)) {
            cs11 |= (1 << channel);
        }

        /* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-dsm_frame format */
    }

    /* wait until we have seen plenty of frames - 5 should normally be enough */
    if (samples++ < 5) {
        return;
    }

    /*
     * Iterate the set of sensible sniffed channel sets and see whether
     * decoding in 10 or 11-bit mode has yielded anything we recognize.
     *
     * XXX Note that due to what seem to be bugs in the DSM2 high-resolution
     *     stream, we may want to sniff for longer in some cases when we think we
     *     are talking to a DSM2 receiver in high-resolution mode (so that we can
     *     reject it, ideally).
     *     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
     *     of this issue.
     */
    static const uint32_t masks[] = {
        0x1f,	/* 5 channels (DX6 VTX frame) */
        0x3f,	/* 6 channels (DX6) */
        0x7f,	/* 7 channels (DX7) */
        0xff,	/* 8 channels (DX8) */
        0x1ff,	/* 9 channels (DX9, etc.) */
        0x3ff,	/* 10 channels (DX10) */
        0x7ff,	/* 11 channels */
        0xfff,	/* 12 channels */
        0x1fff,	/* 13 channels */
        0x3fff,	/* 14 channels */
        0x7fff,	/* 15 channels */
        0xffff	/* 16 channels */   // the remote receiver protocol supports max 16 channels
    };
    unsigned votes10 = 0;
    unsigned votes11 = 0;

    for (unsigned i = 0; i < sizeof(masks)/sizeof(masks[0]); i++) {

        if (cs10 == masks[i]) {
            votes10++;
        }

        if (cs11 == masks[i]) {
            votes11++;
        }
    }

    if ((votes11 == 1) && (votes10 == 0)) {
        channel_shift = 11;
        debug("DSM: 11-bit format");
        return;
    }

    if ((votes10 == 1) && (votes11 == 0)) {
        channel_shift = 10;
        debug("DSM: 10-bit format");
        return;
    }

    /* call ourselves to reset our state ... we have to try again */
    debug("DSM: format detect fail, 10: 0x%08x %u 11: 0x%08x %u", cs10, votes10, cs11, votes11);
    dsm_guess_format(true, dsm_frame, frame_channels);
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
bool AP_RCProtocol_DSM::dsm_decode(uint32_t frame_time_ms, const uint8_t dsm_frame[16],
                                   uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
    /*
     * If we have lost signal for at least 200ms, reset the
     * format guessing heuristic.
     */
    if (((frame_time_ms - last_frame_time_ms) > 200U) && (channel_shift != 0)) {
        dsm_guess_format(true, dsm_frame, DSM_FRAME_CHANNELS);
    }

    /* we have received something we think is a dsm_frame */
    last_frame_time_ms = frame_time_ms;

    // Get the VTX control bytes in a frame
    uint32_t vtxControl = ((dsm_frame[DSM_FRAME_SIZE-4] << 24)
        | (dsm_frame[DSM_FRAME_SIZE-3] << 16)
        | (dsm_frame[DSM_FRAME_SIZE-2] <<  8)
        | (dsm_frame[DSM_FRAME_SIZE-1] <<  0));
    const bool haveVtxControl =
        ((vtxControl & SPEKTRUM_VTX_CONTROL_FRAME_MASK) == SPEKTRUM_VTX_CONTROL_FRAME &&
        (dsm_frame[2] & 0x80) == 0);

    unsigned frame_channels = DSM_FRAME_CHANNELS;
    // Handle VTX control frame.
    if (haveVtxControl)  {
        frame_channels = DSM_FRAME_CHANNELS - 2;
    }

    /* if we don't know the dsm_frame format, update the guessing state machine */
    if (channel_shift == 0) {
        dsm_guess_format(false, dsm_frame, frame_channels);
        return false;
    }

    // Handle VTX control frame.
#if AP_VIDEOTX_ENABLED
    if (haveVtxControl) {
        AP_RCProtocol_SRXL2::configure_vtx(
            (vtxControl & SPEKTRUM_VTX_BAND_MASK)     >> SPEKTRUM_VTX_BAND_SHIFT,
            (vtxControl & SPEKTRUM_VTX_CHANNEL_MASK)  >> SPEKTRUM_VTX_CHANNEL_SHIFT,
            (vtxControl & SPEKTRUM_VTX_POWER_MASK)    >> SPEKTRUM_VTX_POWER_SHIFT,
            (vtxControl & SPEKTRUM_VTX_PIT_MODE_MASK) >> SPEKTRUM_VTX_PIT_MODE_SHIFT);
    }
#endif

    /*
     * The encoding of the first two bytes is uncertain, so we're
     * going to ignore them for now.
     *
     * Each channel is a 16-bit unsigned value containing either a 10-
     * or 11-bit channel value and a 4-bit channel number, shifted
     * either 10 or 11 bits. The MSB may also be set to indicate the
     * second dsm_frame in variants of the protocol where more than
     * seven channels are being transmitted.
     */

    for (unsigned i = 0; i < frame_channels; i++) {

        const uint8_t *dp = &dsm_frame[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;

        if (!dsm_decode_channel(raw, channel_shift, &channel, &value)) {
            continue;
        }

        /* ignore channels out of range */
        if (channel >= max_values) {
            continue;
        }

        /* update the decoded channel count */
        if (channel >= *num_values) {
            *num_values = channel + 1;
        }

        /* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
        if (channel_shift == 10) {
            value *= 2;
        }

        /*
         * Spektrum scaling is special. There are these basic considerations
         *
         *   * Midpoint is 1520 us
         *   * 100% travel channels are +- 400 us
         *
         * We obey the original Spektrum scaling (so a default setup will scale from
         * 1100 - 1900 us), but we do not obey the weird 1520 us center point
         * and instead (correctly) center the center around 1500 us. This is in order
         * to get something useful without requiring the user to calibrate on a digital
         * link for no reason.
         */

        /* scaled integer for decent accuracy while staying efficient */
        value = ((((int)value - 1024) * 1000) / 1700) + 1500;

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
     * Spektrum likes to send junk in higher channel numbers to fill
     * their packets. We don't know about a 13 channel model in their TX
     * lines, so if we get a channel count of 13, we'll return 12 (the last
     * data index that is stable).
     */
    if (*num_values == 13) {
        *num_values = 12;
    }

#if 0
    if (channel_shift == 11) {
        /* Set the 11-bit data indicator */
        *num_values |= 0x8000;
    }
#endif

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
#if defined(HAL_GPIO_SPEKTRUM_RC) && HAL_GPIO_SPEKTRUM_RC
    if (!hal.gpio->get_mode(HAL_GPIO_SPEKTRUM_RC, bind_mode_saved)) {
        return;
    }
#endif
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
            hal.gpio->set_mode(HAL_GPIO_SPEKTRUM_RC, bind_mode_saved);
        }
        break;
    }
    }
#endif
}

/*
  parse one DSM byte, maintaining decoder state
 */
bool AP_RCProtocol_DSM::dsm_parse_byte(uint32_t frame_time_ms, uint8_t b, uint16_t *values,
                                       uint16_t *num_values, uint16_t max_channels)
{
    /* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

    /* overflow check */
    if (byte_input.ofs == sizeof(byte_input.buf) / sizeof(byte_input.buf[0])) {
        byte_input.ofs = 0;
        dsm_decode_state = DSM_DECODE_STATE_DESYNC;
        debug("DSM: RESET (BUF LIM)\n");
        reset_rc_frame_count();
    }

    if (byte_input.ofs == DSM_FRAME_SIZE) {
        byte_input.ofs = 0;
        dsm_decode_state = DSM_DECODE_STATE_DESYNC;
        debug("DSM: RESET (PACKET LIM)\n");
        reset_rc_frame_count();
    }

#ifdef DSM_DEBUG
    debug("dsm state: %s%s, count: %d, val: %02x\n",
          (dsm_decode_state == DSM_DECODE_STATE_DESYNC) ? "DSM_DECODE_STATE_DESYNC" : "",
          (dsm_decode_state == DSM_DECODE_STATE_SYNC) ? "DSM_DECODE_STATE_SYNC" : "",
          byte_input.ofs,
          (unsigned)b);
#endif

    switch (dsm_decode_state) {
    case DSM_DECODE_STATE_DESYNC:

        /* we are de-synced and only interested in the frame marker */
        if ((frame_time_ms - last_rx_time_ms) >= 5) {
            dsm_decode_state = DSM_DECODE_STATE_SYNC;
            byte_input.ofs = 0;
            byte_input.buf[byte_input.ofs++] = b;
        }
        break;

    case DSM_DECODE_STATE_SYNC: {
        if ((frame_time_ms - last_rx_time_ms) >= 5 && byte_input.ofs > 0) {
            byte_input.ofs = 0;
            dsm_decode_state = DSM_DECODE_STATE_DESYNC;
            break;
        }
        byte_input.buf[byte_input.ofs++] = b;

        /* decode whatever we got and expect */
        if (byte_input.ofs < DSM_FRAME_SIZE) {
            break;
        }

        /*
         * Great, it looks like we might have a frame.  Go ahead and
         * decode it.
         */
        log_data(AP_RCProtocol::DSM, frame_time_ms * 1000, byte_input.buf, byte_input.ofs);

        decode_ret = dsm_decode(frame_time_ms, byte_input.buf, values, &chan_count, max_channels);

        /* we consumed the partial frame, reset */
        byte_input.ofs = 0;

        /* if decoding failed, set proto to desync */
        if (decode_ret == false) {
            dsm_decode_state = DSM_DECODE_STATE_DESYNC;
            reset_rc_frame_count();
        }
        break;
    }

    default:
        debug("UNKNOWN PROTO STATE");
        decode_ret = false;
    }


    if (decode_ret) {
		*num_values = chan_count;
	}

    last_rx_time_ms = frame_time_ms;

	/* return false as default */
	return decode_ret;
}

// support byte input
void AP_RCProtocol_DSM::_process_byte(uint32_t timestamp_ms, uint8_t b)
{
    uint16_t v[AP_DSM_MAX_CHANNELS];
    uint16_t nchan;
    memcpy(v, last_values, sizeof(v));
    if (dsm_parse_byte(timestamp_ms, b, v, &nchan, AP_DSM_MAX_CHANNELS)) {
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
    _process_byte(AP_HAL::millis(), b);
}

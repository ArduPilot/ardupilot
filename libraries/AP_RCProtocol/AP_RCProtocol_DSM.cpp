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
#include "AP_RCProtocol_DSM.h"

extern const AP_HAL::HAL& hal;

// #define DEBUG
#ifdef DEBUG
# define debug(fmt, args...)	hal.console->printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif


#define DSM_FRAME_SIZE		16		/**<DSM frame size in bytes*/
#define DSM_FRAME_CHANNELS	7		/**<Max supported DSM channels*/

void AP_RCProtocol_DSM::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
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
void AP_RCProtocol_DSM::dsm_guess_format(bool reset, const uint8_t dsm_frame[16])
{
    /* reset the 10/11 bit sniffed channel masks */
    if (reset) {
        cs10 = 0;
        cs11 = 0;
        samples = 0;
        dsm_channel_shift = 0;
        return;
    }

    /* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
    for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

        const uint8_t *dp = &dsm_frame[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;

        /* if the channel decodes, remember the assigned number */
        if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31)) {
            cs10 |= (1 << channel);
        }

        if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31)) {
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
        0x3f,	/* 6 channels (DX6) */
        0x7f,	/* 7 channels (DX7) */
        0xff,	/* 8 channels (DX8) */
        0x1ff,	/* 9 channels (DX9, etc.) */
        0x3ff,	/* 10 channels (DX10) */
        0x1fff,	/* 13 channels (DX10t) */
        0x3fff	/* 18 channels (DX10) */
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
        dsm_channel_shift = 11;
        debug("DSM: 11-bit format");
        return;
    }

    if ((votes10 == 1) && (votes11 == 0)) {
        dsm_channel_shift = 10;
        debug("DSM: 10-bit format");
        return;
    }

    /* call ourselves to reset our state ... we have to try again */
    debug("DSM: format detect fail, 10: 0x%08x %d 11: 0x%08x %d", cs10, votes10, cs11, votes11);
    dsm_guess_format(true, dsm_frame);
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
bool AP_RCProtocol_DSM::dsm_decode(uint32_t frame_time, const uint8_t dsm_frame[16],
                                   uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
#if 1
    debug("DSM dsm_frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
          dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
          dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);
#endif
    /*
     * If we have lost signal for at least 200ms, reset the
     * format guessing heuristic.
     */
    if (((frame_time - dsm_last_frame_time) > 200000U) && (dsm_channel_shift != 0)) {
        dsm_guess_format(true, dsm_frame);
    }

    /* we have received something we think is a dsm_frame */
    dsm_last_frame_time = frame_time;

    /* if we don't know the dsm_frame format, update the guessing state machine */
    if (dsm_channel_shift == 0) {
        dsm_guess_format(false, dsm_frame);
        return false;
    }

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

    for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

        const uint8_t *dp = &dsm_frame[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;

        if (!dsm_decode_channel(raw, dsm_channel_shift, &channel, &value)) {
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
        if (dsm_channel_shift == 10) {
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
    if (dsm_channel_shift == 11) {
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

// support byte input
void AP_RCProtocol_DSM::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    if (timestamp_us - byte_input.last_byte_us > 3000U ||
        byte_input.ofs == sizeof(byte_input.buf)) {
        byte_input.ofs = 0;
    }
    byte_input.last_byte_us = timestamp_us;
    byte_input.buf[byte_input.ofs++] = b;
    if (byte_input.ofs == 16) {
        if (dsm_decode(timestamp_us, byte_input.buf, &last_values[0], &num_channels, AP_DSM_MAX_CHANNELS) &&
            num_channels >= MIN_RCIN_CHANNELS) {
            add_input(num_channels, last_values, false);
        }

        byte_input.ofs = 0;
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

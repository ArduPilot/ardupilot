/*
  DSM decoder, based on src/modules/px4iofirmware/dsm.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>

#include "dsm.h"

#define DSM_FRAME_SIZE		16		/**<DSM frame size in bytes*/
#define DSM_FRAME_CHANNELS	7		/**<Max supported DSM channels*/

static uint64_t dsm_last_frame_time;		/**< Timestamp for start of last dsm frame */
static unsigned dsm_channel_shift;			/**< Channel resolution, 0=unknown, 10=10 bit, 11=11 bit */

//#define DEBUG

#ifdef DEBUG
# define debug(fmt, args...)	printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

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
static bool
dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

	if (raw == 0xffff)
		return false;

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
static void
dsm_guess_format(bool reset, const uint8_t dsm_frame[16])
{
	static uint32_t	cs10;
	static uint32_t	cs11;
	static unsigned samples;

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
		if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31))
			cs10 |= (1 << channel);

		if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31))
			cs11 |= (1 << channel);

		/* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-dsm_frame format */
	}

	/* wait until we have seen plenty of frames - 5 should normally be enough */
	if (samples++ < 5)
		return;

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
	static uint32_t masks[] = {
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

		if (cs10 == masks[i])
			votes10++;

		if (cs11 == masks[i])
			votes11++;
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
	debug("DSM: format detect fail, 10: 0x%08x %u 11: 0x%08x %u", cs10, votes10, cs11, votes11);
	dsm_guess_format(true, dsm_frame);
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
bool
dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[16], uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
	/*
	debug("DSM dsm_frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
		dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
		dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);
	*/
	/*
	 * If we have lost signal for at least a second, reset the
	 * format guessing heuristic.
	 */
	if (((frame_time - dsm_last_frame_time) > 1000000) && (dsm_channel_shift != 0))
            dsm_guess_format(true, dsm_frame);

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

		if (!dsm_decode_channel(raw, dsm_channel_shift, &channel, &value))
			continue;

		/* ignore channels out of range */
		if (channel >= max_values)
			continue;

		/* update the decoded channel count */
		if (channel >= *num_values)
			*num_values = channel + 1;

		/* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
		if (dsm_channel_shift == 10)
			value *= 2;

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
	if (*num_values == 13)
		*num_values = 12;

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

#if defined(TEST_MAIN_PROGRAM) || defined(TEST_HEX_STRING)
static uint8_t dsm_partial_frame_count;
static uint8_t dsm_frame[DSM_FRAME_SIZE];
static enum DSM_DECODE_STATE {
	DSM_DECODE_STATE_DESYNC = 0,
	DSM_DECODE_STATE_SYNC
} dsm_decode_state = DSM_DECODE_STATE_DESYNC;
static uint64_t dsm_last_rx_time;            /**< Timestamp when we last received data */
static uint16_t dsm_chan_count;
static uint16_t dsm_frame_drops;

static bool
dsm_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
	  uint16_t *num_values, bool *dsm_11_bit, unsigned *frame_drops, uint16_t max_channels)
{

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

	/* keep decoding until we have consumed the buffer */
	for (unsigned d = 0; d < len; d++) {

		/* overflow check */
		if (dsm_partial_frame_count == sizeof(dsm_frame) / sizeof(dsm_frame[0])) {
			dsm_partial_frame_count = 0;
			dsm_decode_state = DSM_DECODE_STATE_DESYNC;
#ifdef DSM_DEBUG
			printf("DSM: RESET (BUF LIM)\n");
#endif
		}

		if (dsm_partial_frame_count == DSM_FRAME_SIZE) {
			dsm_partial_frame_count = 0;
			dsm_decode_state = DSM_DECODE_STATE_DESYNC;
#ifdef DSM_DEBUG
			printf("DSM: RESET (PACKET LIM)\n");
#endif
		}

#ifdef DSM_DEBUG
#if 1
		printf("dsm state: %s%s, count: %d, val: %02x\n",
		       (dsm_decode_state == DSM_DECODE_STATE_DESYNC) ? "DSM_DECODE_STATE_DESYNC" : "",
		       (dsm_decode_state == DSM_DECODE_STATE_SYNC) ? "DSM_DECODE_STATE_SYNC" : "",
		       dsm_partial_frame_count,
		       (unsigned)frame[d]);
#endif
#endif

		switch (dsm_decode_state) {
		case DSM_DECODE_STATE_DESYNC:

			/* we are de-synced and only interested in the frame marker */
			if ((now - dsm_last_rx_time) > 5000) {
				printf("resync %u\n", dsm_partial_frame_count);
				dsm_decode_state = DSM_DECODE_STATE_SYNC;
				dsm_partial_frame_count = 0;
				dsm_chan_count = 0;
				dsm_frame[dsm_partial_frame_count++] = frame[d];
			}

			break;

		case DSM_DECODE_STATE_SYNC: {
				dsm_frame[dsm_partial_frame_count++] = frame[d];

				/* decode whatever we got and expect */
				if (dsm_partial_frame_count < DSM_FRAME_SIZE) {
					break;
				}

				/*
				 * Great, it looks like we might have a frame.  Go ahead and
				 * decode it.
				 */
				decode_ret = dsm_decode(now, dsm_frame, values, &dsm_chan_count, max_channels);

#if 1
                                printf("%u %u: ", ((unsigned)(now/1000)) % 1000000, len);
                                for (uint8_t i=0; i<DSM_FRAME_SIZE; i++) {
                                    printf("%02x ", (unsigned)dsm_frame[i]);
                                }
                                printf("\n");
#endif
	
                                
				/* we consumed the partial frame, reset */
				dsm_partial_frame_count = 0;

				/* if decoding failed, set proto to desync */
				if (decode_ret == false) {
					dsm_decode_state = DSM_DECODE_STATE_DESYNC;
					dsm_frame_drops++;
					printf("drop ");
					for (uint8_t i=0; i<DSM_FRAME_SIZE; i++) {
						printf("%02x ", (unsigned)dsm_frame[i]);
					}
					printf("\n");
				}
			}
			break;

		default:
#ifdef DSM_DEBUG
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}


	}

	if (frame_drops) {
		*frame_drops = dsm_frame_drops;
	}

	if (decode_ret) {
		*num_values = dsm_chan_count;
	}

	dsm_last_rx_time = now;

	/* return false as default */
	return decode_ret;
}
#endif


#ifdef TEST_MAIN_PROGRAM
/*
  test harness for use under Linux with USB serial adapter
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>

static uint64_t micros64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)));
}

int main(int argc, const char *argv[])
{
    int fd = open(argv[1], O_RDONLY|O_CLOEXEC);
    if (fd == -1) {
        perror(argv[1]);
        exit(1);
    }

    struct termios options;

    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    options.c_iflag &= ~(IXON|IXOFF|IXANY);
    options.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr");
        exit(1);
    }
    tcflush(fd, TCIOFLUSH);

    uint16_t values[18];
    memset(values, 0, sizeof(values));

    while (true) {
        uint8_t b[16];
        uint16_t num_values = 0;
        fd_set fds;
        struct timeval tv;
    
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        tv.tv_sec = 1;
        tv.tv_usec = 0;

        // check if any bytes are available
        if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
            break;
        }

        ssize_t nread;
        if ((nread = read(fd, b, sizeof(b))) < 1) {
            break;
        }

        bool dsm_11_bit;
        unsigned frame_drops;
        
        if (dsm_parse(micros64(), b, nread, values, &num_values, &dsm_11_bit, &frame_drops, 18)) {
#if 1
            printf("%u: ", num_values);
            for (uint8_t i=0; i<num_values; i++) {
                printf("%u:%4u ", i+1, values[i]);
            }
            printf("\n");
#endif
        }
    }
}
#elif defined(TEST_HEX_STRING)
/*
  test harness providing hex string to decode
 */
#include <string.h>

int main(int argc, const char *argv[])
{
    uint8_t b[16];
    uint64_t t = 0;

    for (uint8_t i=1; i<argc; i++) {
        unsigned v;
        if (sscanf(argv[i], "%02x", &v) != 1 || v > 255) {
            printf("Bad hex value at %u : %s\n", (unsigned)i, argv[i]);
            return 1;
        }
        b[i-1] = v;
    }
    uint16_t values[18];
    memset(values, 0, sizeof(values));
    
    while (true) {
        uint16_t num_values = 0;
        bool dsm_11_bit;
        unsigned frame_drops;

        t += 11000;
        
        if (dsm_parse(t, b, sizeof(b), values, &num_values, &dsm_11_bit, &frame_drops, 18)) {
#if 1
            printf("%u: ", num_values);
            for (uint8_t i=0; i<num_values; i++) {
                printf("%u:%4u ", i+1, values[i]);
            }
            printf("\n");
#endif
        }
    }
}
#endif

// -*- tab-width: 8; Mode: C++; c-basic-offset: 8; indent-tabs-mode: -*- t -*-
/*
  SBUS decoder, based on src/modules/px4iofirmware/sbus.c from PX4Firmware
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
#include <stdbool.h>
#include "sbus.h"


#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};


bool
sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values, 
            bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{
	/* check frame boundary markers to avoid out-of-sync cases */
	if ((frame[0] != 0x0f)) {
		return false;
	}

	switch (frame[24]) {
		case 0x00:
		/* this is S.BUS 1 */
		break;
		case 0x03:
		/* S.BUS 2 SLOT0: RX battery and external voltage */
		break;
		case 0x83:
		/* S.BUS 2 SLOT1 */
		break;
		case 0x43:
		case 0xC3:
		case 0x23:
		case 0xA3:
		case 0x63:
		case 0xE3:
		break;
		default:
		/* we expect one of the bits above, but there are some we don't know yet */
		break;
	}

	unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
			     SBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	for (unsigned channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}


		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR +.5f) + SBUS_SCALE_OFFSET;
	}

	/* decode switch channels if data fields are wide enough */
	if (max_values > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
		/* channel 18 (index 17) */
		values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	/* decode and handle failsafe and frame-lost flags */
	if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		*sbus_failsafe = true;
		*sbus_frame_drop = true;
	}
	else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 * 
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this 
		 * condition as fail-safe greatly reduces the reliability and range of the radio link, 
		 * e.g. by prematurely issueing return-to-launch!!! */

		*sbus_failsafe = false;
		*sbus_frame_drop = true;
	} else {
		*sbus_failsafe = false;
		*sbus_frame_drop = false;
	}

	return true;
}

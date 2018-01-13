/*
 * Copyright (C) Siddharth Bharat Purohit 2017
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
 */

#include "ppm.h"
/*
 * PPM decoder tuning parameters.
 *
 * The PPM decoder works as follows.
 *
 * Initially, the decoder waits in the UNSYNCH state for two edges
 * separated by PPM_MIN_START.  Once the second edge is detected,
 * the decoder moves to the ARM state.
 *
 * The ARM state expects an edge within PPM_MAX_PULSE_WIDTH, being the
 * timing mark for the first channel.  If this is detected, it moves to
 * the INACTIVE state.
 *
 * The INACTIVE phase waits for and discards the next edge, as it is not
 * significant.  Once the edge is detected, it moves to the ACTIVE stae.
 *
 * The ACTIVE state expects an edge within PPM_MAX_PULSE_WIDTH, and when
 * received calculates the time from the previous mark and records
 * this time as the value for the next channel.
 *
 * If at any time waiting for an edge, the delay from the previous edge
 * exceeds PPM_MIN_START the frame is deemed to have ended and the recorded
 * values are advertised to clients.
 */
#define PPM_MAX_PULSE_WIDTH	500		/* maximum width of a pulse */
#define PPM_MIN_CHANNEL_VALUE	800		/* shortest valid channel signal */
#define PPM_MAX_CHANNEL_VALUE	2200		/* longest valid channel signal */
#define PPM_MIN_START		2500		/* shortest valid start gap */

/* Input timeout - after this interval we assume signal is lost */
#define PPM_INPUT_TIMEOUT	100 * 1000	/* 100ms */

/* Number of same-sized frames required to 'lock' */
#define PPM_CHANNEL_LOCK	3		/* should be less than the input timeout */

/* decoded PPM buffer */
#define PPM_MIN_CHANNELS	4
#define PPM_MAX_CHANNELS	12

/*
 * Public decoder state
 */
uint16_t	ppm_buffer[PPM_MAX_CHANNELS];
unsigned	ppm_decoded_channels;

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];
#if HAL_USE_EICU
/* PPM decoder state machine */
static struct {
	uint16_t	last_edge;	/* last capture time */
	uint16_t	last_mark;	/* last significant edge */
	unsigned	next_channel;
	unsigned	count_max;
	enum {
		UNSYNCH = 0,
		ARM,
		ACTIVE,
		INACTIVE
	} phase;
} ppm;

static EICUChannelConfig eicuchancfg;    //Input Capture Unit Config
static EICUConfig eicucfg;    //Input Capture Unit Config
static uint8_t buf_ptr = 0;
static void ppm_measurement_cb(EICUDriver *eicup, eicuchannel_t channel);

//Initiallise ppm ICU with requested configuration
bool ppm_init(uint32_t freq, bool active_high)
{
    if (active_high) {
        eicuchancfg.alvl = EICU_INPUT_ACTIVE_HIGH;
    } else {
        eicuchancfg.alvl = EICU_INPUT_ACTIVE_LOW;
    }

    eicuchancfg.capture_cb = ppm_measurement_cb;
    memset(&eicucfg, 0, sizeof(eicucfg));
    eicucfg.frequency = freq;
    eicucfg.dier = 0;
    eicucfg.iccfgp[PPM_EICU_CHANNEL] = &eicuchancfg;
    eicuStart(&PPM_EICU_TIMER, &eicucfg);
    eicuEnable(&PPM_EICU_TIMER);
    return true;
}

uint16_t ppm_read(uint8_t channel)
{
    //return 0 if channel requested is out range
    if(channel >= ppm_decoded_channels) {
        return 0;
    }
    return ppm_buffer[channel];
}

uint8_t ppm_read_bulk(uint16_t periods[], uint8_t len)
{
    uint8_t i;
    for(i = 0; (i < ppm_decoded_channels) && (i < len); i++) {
        periods[i] = ppm_buffer[i];
    }
    return i;
}

bool ppm_available()
{
    return (ppm_decoded_channels > 0);
}

uint8_t ppm_num_channels()
{
    return ppm_decoded_channels;
}

void
ppm_input_decode(bool reset, unsigned count)
{
	uint16_t width;
	uint16_t interval;
	unsigned i;

	/* if we missed an edge, we have to give up */
	if (reset) {
		goto error;
	}

	/* how long since the last edge? */
	width = count - ppm.last_edge;

	if (count < ppm.last_edge) {
		width += ppm.count_max;        /* handle wrapped count */
	}

	ppm.last_edge = count;

	/*
	 * If this looks like a start pulse, then push the last set of values
	 * and reset the state machine.
	 *
	 * Note that this is not a "high performance" design; it implies a whole
	 * frame of latency between the pulses being received and their being
	 * considered valid.
	 */
	if (width >= PPM_MIN_START) {

		/*
		 * If the number of channels changes unexpectedly, we don't want
		 * to just immediately jump on the new count as it may be a result
		 * of noise or dropped edges.  Instead, take a few frames to settle.
		 */
		if (ppm.next_channel != ppm_decoded_channels) {
			static unsigned new_channel_count;
			static unsigned new_channel_holdoff;

			if (new_channel_count != ppm.next_channel) {
				/* start the lock counter for the new channel count */
				new_channel_count = ppm.next_channel;
				new_channel_holdoff = PPM_CHANNEL_LOCK;

			} else if (new_channel_holdoff > 0) {
				/* this frame matched the last one, decrement the lock counter */
				new_channel_holdoff--;

			} else {
				/* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
				ppm_decoded_channels = new_channel_count;
				new_channel_count = 0;
			}

		} else {
			/* frame channel count matches expected, let's use it */
			if (ppm.next_channel > PPM_MIN_CHANNELS) {
				for (i = 0; i < ppm.next_channel; i++) {
					ppm_buffer[i] = ppm_temp_buffer[i];
				}
			}
		}

		/* reset for the next frame */
		ppm.next_channel = 0;

		/* next edge is the reference for the first channel */
		ppm.phase = ARM;

		return;
	}

	switch (ppm.phase) {
	case UNSYNCH:
		/* we are waiting for a start pulse - nothing useful to do here */
		return;

	case ARM:

		/* we expect a pulse giving us the first mark */
		if (width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too long */
		}

		/* record the mark timing, expect an inactive edge */
		ppm.last_mark = count;
		ppm.phase = INACTIVE;
		return;

	case INACTIVE:
		/* this edge is not interesting, but now we are ready for the next mark */
		ppm.phase = ACTIVE;

		/* note that we don't bother looking at the timing of this edge */

		return;

	case ACTIVE:

		/* we expect a well-formed pulse */
		if (width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too long */
		}

		/* determine the interval from the last mark */
		interval = count - ppm.last_mark;
		ppm.last_mark = count;

		/* if the mark-mark timing is out of bounds, abandon the frame */
		if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE)) {
			goto error;
		}

		/* if we have room to store the value, do so */
		if (ppm.next_channel < PPM_MAX_CHANNELS) {
			ppm_temp_buffer[ppm.next_channel++] = interval;
		}

		ppm.phase = INACTIVE;
		return;

	}

	/* the state machine is corrupted; reset it */

error:
	/* we don't like the state of the decoder, reset it and try again */
	ppm.phase = UNSYNCH;
	ppm_decoded_channels = 0;
}

static void ppm_measurement_cb(EICUDriver *eicup, eicuchannel_t channel) {
    ppm_input_decode(false, eicup->tim->CCR[channel]);
}
#endif // HAL_USE_EICU

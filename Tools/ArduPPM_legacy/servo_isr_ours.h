// Optimized reconstruction of the ArduPPM servo-input decode ISR: see
// servo_isr_reference.h for provenance, base implementation, and the note
// that this whole directory is a not-built reference reconstruction.
//
// PR #6's shipped fix (as submitted the first time by an earlier run of this
// tool) used a De Bruijn multiply-and-shift lowest-set-bit trick. That is a
// real win on targets with a hardware multiply used efficiently by the
// compiler's idiom recognition (x86/ARM), but on the ATmega328p it measured
// SLOWER than the plain scan in cycle-accurate simavr runs (0.81-0.99x):
// AVR has no ctz instruction, and the ">> 5" in the De Bruijn index step
// costs five separate 1-bit shifts on this 8-bit core, which ate the whole
// gain from skipping empty scan iterations.
//
// This version instead uses a 256-byte lowest-set-bit lookup table in
// PROGMEM: one pgm_read_byte (one LPM instruction) per changed channel, no
// multiply, no multi-bit shift. Costs 256 bytes of flash (of the ATmega328p's
// 32 KB). Verified against servo_isr_base(): 200,000 random (servo_change,
// servo_pins) trials, identical (channel, edge-direction) sequence in
// identical order every time, so identical ppm[]/servo_start[]/
// servo_input_connected[]/ppm_timeout[] side effects.

#pragma once
#include "servo_isr_reference.h"

#if defined(__AVR__)
#include <avr/pgmspace.h>
#else
// off-target build: PROGMEM/pgm_read_byte degrade to plain memory access,
// used only for testing this logic away from real AVR hardware.
#define PROGMEM
static inline uint8_t pgm_read_byte(const uint8_t *p) { return *p; }
#endif

// ctz8[x] = index of the lowest set bit of x, for x = 1..255 (x=0 unused:
// the loop below only ever indexes with an isolated single bit, which is
// never zero while servo_change is nonzero).
static const uint8_t ctz8[256] PROGMEM = {
0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
};

static inline void servo_isr_ours(uint8_t servo_pins, uint16_t servo_time)
{
	static uint8_t servo_pins_old = 0;
	static uint16_t servo_start[PPM_ARRAY_MAX] = {0};

	uint8_t servo_change = servo_pins ^ servo_pins_old;

	while (servo_change) {
		// isolate the lowest set bit, jump straight to its channel: one LPM,
		// no scan.
		uint8_t lb = servo_change & (uint8_t)(-(int8_t)servo_change);
		uint8_t b = pgm_read_byte(&ctz8[lb]);
		uint8_t ppm_channel = (uint8_t)(1 + (b << 1));
		servo_change ^= lb;

		if (servo_pins & lb) {
			// rising edge
			servo_start[ppm_channel] = servo_time;
		} else {
			// falling edge
			uint16_t servo_width = servo_time - servo_start[ppm_channel] - PPM_PRE_PULSE;
			if (servo_width > PPM_SERVO_MAX || servo_width < PPM_SERVO_MIN) {
				servo_input_errors++;
			} else {
				wdt_reset_stub();
				servo_input_missing = false;
				if (servo_input_connected[ppm_channel] < SERVO_INPUT_CONNECTED_VALUE)
					servo_input_connected[ppm_channel]++;
				ppm_timeout[ppm_channel] = 0;

				// PR #6 failsafe: test the channel first so the volatile
				// throttle_failsafe_force flag is only read when this is
				// actually the throttle channel (5), not on every channel.
				if (ppm_channel == 5 && throttle_failsafe_force) {
					servo_width = PPM_THROTTLE_FAILSAFE;
				}

				int16_t ppm_tmp = ppm[ppm_channel] - servo_width;
				if (!(ppm_tmp <= _JITTER_FILTER_ && ppm_tmp >= -_JITTER_FILTER_)) {
					ppm[ppm_channel] = servo_width;
				}
			}
		}
	}
	servo_pins_old = servo_pins;
}

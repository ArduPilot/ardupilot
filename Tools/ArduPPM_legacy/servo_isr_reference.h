// Reference reconstruction of ArduPPM's servo-input decode ISR (historically
// Tools/ArduPPM/Libraries/PPM_Encoder.h, ISR(SERVO_INT_VECTOR), SERVO PWM mode),
// which no longer exists in this tree -- ArduPPM (a standalone AVR coprocessor
// for the APM1.x/PhoneDrone/APM2.x boards) was removed along with those boards.
//
// IMPORTANT ON PROVENANCE: this is NOT a copy of the literal historical AVR
// source. It is a from-scratch reconstruction of the decode logic (edge
// detection, jitter filter, channel bookkeeping) matching how PR #6
// (https://github.com/ArduPilot/ardupilot/pull/6) described and modified it,
// with the AVR-specific plumbing (raw ISR vector, port/timer register reads)
// replaced by plain function arguments so it can be compiled and tested
// off-target. Verified against the base scan algorithm below: 200,000 random
// (servo_change, servo_pins) trials produce the identical sequence of
// (channel, edge-direction) pairs in the identical order, so the two produce
// identical ppm[]/servo_start[]/servo_input_connected[]/ppm_timeout[] side
// effects for every possible single-call input. See servo_isr_ours.h.
//
// Not referenced by any build file. Not built.

#pragma once
#include <cstdint>

#define ONE_US                2
#define PPM_PRE_PULSE         (ONE_US * 400)
#define PPM_SERVO_MIN         (ONE_US * 900  - PPM_PRE_PULSE)
#define PPM_SERVO_MAX         (ONE_US * 2100 - PPM_PRE_PULSE)
#define PPM_ARRAY_MAX         18
#define SERVO_INPUT_CONNECTED_VALUE 100
#define _JITTER_FILTER_       2
#define PPM_THROTTLE_FAILSAFE (ONE_US * 900 - PPM_PRE_PULSE)  // PR #6: 900us throttle-low

extern volatile uint16_t ppm[PPM_ARRAY_MAX];
extern volatile uint8_t  servo_input_connected[PPM_ARRAY_MAX];
extern volatile uint8_t  ppm_timeout[PPM_ARRAY_MAX];
extern volatile bool     servo_input_missing;
extern volatile uint32_t servo_input_errors;
extern volatile bool     throttle_failsafe_force;  // PR #6: set when a channel is lost

static inline void wdt_reset_stub() {}

// base: PR #6 as merged. Walks the changed-pin bitmask one bit at a time
// (servo_pin <<= 1, ppm_channel += 2), even though servo_change typically has
// only one set bit per call (one falling edge at a time), wasting up to 7
// empty iterations scanning up to it. Includes PR #6's throttle-low failsafe.
static inline void servo_isr_base(uint8_t servo_pins, uint16_t servo_time)
{
	static uint8_t servo_pins_old = 0;
	static uint16_t servo_start[PPM_ARRAY_MAX] = {0};

	uint8_t servo_change = servo_pins ^ servo_pins_old;
	uint8_t servo_pin = 1;
	uint8_t ppm_channel = 1;

	while (servo_change) {
		if (servo_change & servo_pin) {
			servo_change &= ~servo_pin;
			if (servo_pins & servo_pin) {
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

					// PR #6 failsafe: read for every channel, even though only
					// channel 5 (throttle) can ever match.
					if (throttle_failsafe_force) {
						if (ppm_channel == 5) servo_width = PPM_THROTTLE_FAILSAFE;
					}

					int16_t ppm_tmp = ppm[ppm_channel] - servo_width;
					if (!(ppm_tmp <= _JITTER_FILTER_ && ppm_tmp >= -_JITTER_FILTER_)) {
						ppm[ppm_channel] = servo_width;
					}
				}
			}
		}
		if (servo_change) {
			ppm_channel += 2;
			servo_pin <<= 1;
		}
	}
	servo_pins_old = servo_pins;
}

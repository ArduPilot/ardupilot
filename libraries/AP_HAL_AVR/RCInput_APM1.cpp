
#include <avr/io.h>
#include <avr/interrupt.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include "RCInput.h"
#include "utility/ISRRegistry.h"
using namespace AP_HAL;
using namespace AP_HAL_AVR;

extern const HAL& hal;

/* private variables to communicate with input capture isr */
volatile uint16_t APM1RCInput::_pulse_capt[AVR_RC_INPUT_NUM_CHANNELS] = {0};  
volatile uint8_t  APM1RCInput::_valid = 0;

/* private callback for input capture ISR */
void APM1RCInput::_timer4_capt_cb(void) {
    static uint16_t icr4_prev;
    static uint8_t  channel_ctr;

    const uint16_t icr4_current = ICR4;
    uint16_t pulse_width;
    if (icr4_current < icr4_prev) {
        /* ICR4 rolls over at TOP=40000 */
        pulse_width = icr4_current + 40000 - icr4_prev;
    } else {
        pulse_width = icr4_current - icr4_prev;
    }

    if (pulse_width > 8000) {
        /* sync pulse detected */
        channel_ctr = 0;
    } else {
        if (channel_ctr < AVR_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = pulse_width;
            channel_ctr++;
            if (channel_ctr == AVR_RC_INPUT_NUM_CHANNELS) {
                _valid = AVR_RC_INPUT_NUM_CHANNELS;
            }
        }
    }
    icr4_prev = icr4_current;
}

void APM1RCInput::init(void* _isrregistry) {
    ISRRegistry* isrregistry = (ISRRegistry*) _isrregistry;
    isrregistry->register_signal(ISR_REGISTRY_TIMER4_CAPT, _timer4_capt_cb);

    /* Arduino pin 49 is ICP4 / PL0,  timer 4 input capture */
    hal.gpio->pinMode(49, GPIO_INPUT);
    /**
     * WGM: 1 1 1 1. Fast WPM, TOP is in OCR4A
     * COM all disabled
     * CS41: prescale by 8 => 0.5us tick
     * ICES4: input capture on rising edge
     * OCR4A: 40000, 0.5us tick => 2ms period / 50hz freq for outbound
     * fast PWM.
     */
    TCCR4A = _BV(WGM40) | _BV(WGM41);
    TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41) | _BV(ICES4);
    OCR4A  = 40000;

    /* OCR4B and OCR4C will be used by RCOutput_APM1. init to nil output */
    OCR4B  = 0xFFFF;
    OCR4C  = 0xFFFF;

    /* Enable input capture interrupt */
    TIMSK4 |= _BV(ICIE4);
}

uint8_t APM1RCInput::valid() { return _valid; }


/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

uint16_t APM1RCInput::read(uint8_t ch) {
    cli();
    uint16_t capt = _pulse_capt[ch];
    sei();
    _valid = 0;
    /* scale _pulse_capt from 0.5us units to 1us units. */
    return constrain_pulse(capt >> 1);
}

uint8_t APM1RCInput::read(uint16_t* periods, uint8_t len) {
    cli();
    for (int i = 0; i < len; i++) {
        periods[i] = _pulse_capt[i];
    }
    sei();
    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (int i = 0; i < len; i++) {
        /* scale _pulse_capt from 0.5us units to 1us units. */
        periods[i] = constrain_pulse(periods[i] >> 1);
    }
    uint8_t v = _valid;
    _valid = 0;
    return v;
}


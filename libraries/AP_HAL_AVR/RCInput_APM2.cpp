#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

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
volatile uint16_t APM2RCInput::_pulse_capt[AVR_RC_INPUT_NUM_CHANNELS] = {0};  
volatile uint8_t  APM2RCInput::_valid_channels = 0;

/* private callback for input capture ISR */
void APM2RCInput::_timer5_capt_cb(void) {
    static uint16_t icr5_prev;
    static uint8_t  channel_ctr;

    const uint16_t icr5_current = ICR5;
    uint16_t pulse_width;
    if (icr5_current < icr5_prev) {
        /* ICR5 rolls over at TOP=40000 */
        pulse_width = icr5_current + 40000 - icr5_prev;
    } else {
        pulse_width = icr5_current - icr5_prev;
    }

    if (pulse_width > 8000) {
        // sync pulse detected.  Pass through values if at least a minimum number of channels received
        if( channel_ctr >= AVR_RC_INPUT_MIN_CHANNELS ) {
            _valid_channels = channel_ctr;
        }
        channel_ctr = 0;
    } else {
        if (channel_ctr < AVR_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = pulse_width;
            channel_ctr++;
            if (channel_ctr == AVR_RC_INPUT_NUM_CHANNELS) {
                _valid_channels = AVR_RC_INPUT_NUM_CHANNELS;
            }
        }
    }
    icr5_prev = icr5_current;
}

void APM2RCInput::init(void* _isrregistry) {
    ISRRegistry* isrregistry = (ISRRegistry*) _isrregistry;
    isrregistry->register_signal(ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb);

    /* initialize overrides */
    clear_overrides();
    /* Arduino pin 48 is ICP5 / PL1,  timer 5 input capture */
    hal.gpio->pinMode(48, GPIO_INPUT);
    /**
     * WGM: 1 1 1 1. Fast WPM, TOP is in OCR5A
     * COM all disabled
     * CS51: prescale by 8 => 0.5us tick
     * ICES5: input capture on rising edge
     * OCR5A: 40000, 0.5us tick => 2ms period / 50hz freq for outbound
     * fast PWM.
     */

    uint8_t oldSREG = SREG;
    cli();
    
    /* Timer cleanup before configuring */
    TCNT5 = 0;
    TIFR5 = 0;
    
    /* Set timer 8x prescaler fast PWM mode toggle compare at OCRA with rising edge input capture */
    TCCR5A = _BV(WGM50) | _BV(WGM51);
    TCCR5B |= _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(ICES5);
    OCR5A  = 40000 - 1; // -1 to correct for wrap

    /* OCR5B and OCR5C will be used by RCOutput_APM2. Init to 0xFFFF to prevent premature PWM output */
    OCR5B  = 0xFFFF;
    OCR5C  = 0xFFFF;

    /* Enable input capture interrupt */
    TIMSK5 |= _BV(ICIE5);
     
    SREG = oldSREG;
}

uint8_t APM2RCInput::valid_channels() { return _valid_channels; }

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}


uint16_t APM2RCInput::read(uint8_t ch) {
    /* constrain ch */
    if (ch >= AVR_RC_INPUT_NUM_CHANNELS) return 0;
    /* grab channel from isr's memory in critical section*/
    uint8_t oldSREG = SREG;
    cli();
    uint16_t capt = _pulse_capt[ch];
    SREG = oldSREG;
    _valid_channels = 0;
    /* scale _pulse_capt from 0.5us units to 1us units. */
    uint16_t pulse = constrain_pulse(capt >> 1);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t APM2RCInput::read(uint16_t* periods, uint8_t len) {
    /* constrain len */
    if (len > AVR_RC_INPUT_NUM_CHANNELS) { len = AVR_RC_INPUT_NUM_CHANNELS; }
    /* grab channels from isr's memory in critical section */
    uint8_t oldSREG = SREG;
    cli();
    for (int i = 0; i < len; i++) {
        periods[i] = _pulse_capt[i];
    }
    SREG = oldSREG;
    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (int i = 0; i < len; i++) {
        /* scale _pulse_capt from 0.5us units to 1us units. */
        periods[i] = constrain_pulse(periods[i] >> 1);
        /* check for override */
        if (_override[i] != 0) {
            periods[i] = _override[i];
        }
    }
    uint8_t v = _valid_channels;
    _valid_channels = 0;
    return v;
}

bool APM2RCInput::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool APM2RCInput::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return false; /* -1: no change. */
    if (channel < AVR_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            _valid_channels = 1;
            return true;
        }
    }
    return false;
}

void APM2RCInput::clear_overrides() {
    for (int i = 0; i < AVR_RC_INPUT_NUM_CHANNELS; i++) {
        _override[i] = 0;
    }
}

#endif

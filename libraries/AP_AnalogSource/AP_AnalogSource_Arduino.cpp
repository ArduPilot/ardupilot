/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <FastSerial.h>
#include "AP_AnalogSource_Arduino.h"

// increase this if we need more than 5 analog sources
#define MAX_PIN_SOURCES 5

// number of times to read a pin
// before considering the value valid.
// This ensures the value has settled on
// the new source
#define PIN_READ_REPEAT 2

static uint8_t num_pins_watched;
static uint8_t next_pin_index;
static uint8_t next_pin_count;
static volatile struct {
    uint8_t pin;
    uint8_t sum_count;
    uint16_t output;
    uint16_t sum;
} pins[MAX_PIN_SOURCES];

// ADC conversion timer. This is called at 1kHz by the timer
// interrupt
// each conversion takes about 125 microseconds
static void adc_timer(uint32_t t)
{
    if (bit_is_set(ADCSRA, ADSC) || num_pins_watched == 0) {
        // conversion is still running. This should be
        // very rare, as we are called at 1kHz
        return;
    }

    next_pin_count++;
    if (next_pin_count != PIN_READ_REPEAT) {
        // we don't want this value, so start the next conversion
        // immediately, discarding this value
        ADCSRA |= _BV(ADSC);
        return;
    }

    // remember the value we got
    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    pins[next_pin_index].output = low | (high<<8);
    pins[next_pin_index].sum += pins[next_pin_index].output;
    if (pins[next_pin_index].sum_count >= 63) {
        // we risk overflowing the 16 bit sum
        pins[next_pin_index].sum >>= 1;
        pins[next_pin_index].sum_count = 32;
    } else {
        pins[next_pin_index].sum_count++;
    }

    next_pin_count = 0;
    if (num_pins_watched != 0) {
        next_pin_index = (next_pin_index+1) % num_pins_watched;
    }
    uint8_t pin = pins[next_pin_index].pin;

    if (pin == ANALOG_PIN_VCC) {
        // we're reading the board voltage
        ADMUX = _BV(REFS0)|_BV(MUX4)|_BV(MUX3)|_BV(MUX2)|_BV(MUX1);
    } else {
        // we're reading an external pin
        ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
        ADMUX = _BV(REFS0) | (pin & 0x07);
    }

    // start the next conversion
    ADCSRA |= _BV(ADSC);
}

// setup the timer process. This must be called before any analog
// values are available
void AP_AnalogSource_Arduino::init_timer(AP_PeriodicProcess * scheduler)
{
    scheduler->register_process(adc_timer);
}

// read raw 16 bit value
uint16_t AP_AnalogSource_Arduino::read_raw(void)
{
    uint16_t ret;
    cli();
    ret = pins[_pin_index].output;
    sei();
    return ret;
}

// scaled read for board Vcc
uint16_t AP_AnalogSource_Arduino::read_vcc(void)
{
    uint16_t v = read_raw();
    if (v == 0) {
        return 0;
    }
    return 1126400UL / v;
}

// read the average 16 bit value since the last
// time read_average() was called. This gives a very cheap
// filtered value, as new values are produced at 500/N Hz
// where N is the total number of analog sources
float AP_AnalogSource_Arduino::read_average(void)
{
    uint16_t sum;
    uint8_t sum_count;

    // we don't expect this loop to trigger, unless
    // you call read_average() very frequently
    while (pins[_pin_index].sum_count == 0) ;

    cli();
    sum = pins[_pin_index].sum;
    sum_count = pins[_pin_index].sum_count;
    pins[_pin_index].sum = 0;
    pins[_pin_index].sum_count = 0;
    sei();
    return sum / (float)sum_count;
}

// read with the prescaler. This uses the averaged value since
// the last read, which matches that the AP_ADC APM1 library does
// for ADC sources
float AP_AnalogSource_Arduino::read(void)
{
    return read_average() * _prescale;
}


// remap pin numbers to physical pin
uint8_t AP_AnalogSource_Arduino::_remap_pin(uint8_t pin)
{
    if (pin != ANALOG_PIN_VCC) {
        // allow pin to be a channel (i.e. "A0") or an actual pin
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        if (pin >= 54) pin -= 54;
#elif defined(__AVR_ATmega32U4__)
        if (pin >= 18) pin -= 18;
#elif defined(__AVR_ATmega1284__)
        if (pin >= 24) pin -= 24;
#else
        if (pin >= 14) pin -= 14;
#endif
    }
    return pin;
}

// assign a slot in the pins_watched
void AP_AnalogSource_Arduino::_assign_pin_index(uint8_t pin)
{
    // ensure we don't try to read from too many analog pins
    if (num_pins_watched == MAX_PIN_SOURCES) {
        while (true) {
            Serial.printf_P(PSTR("MAX_PIN_SOURCES REACHED\n"));
            delay(1000);
        }
    }

    pin = _remap_pin(pin);
    _pin_index = num_pins_watched;
    pins[_pin_index].pin = pin;
    num_pins_watched++;
    if (num_pins_watched == 1) {
        // enable the ADC
        PRR0 &= ~_BV(PRADC);
        ADCSRA |= _BV(ADEN);
    }
}

// change which pin to read
void AP_AnalogSource_Arduino::set_pin(uint8_t pin)
{
    pin = _remap_pin(pin);
    if (pins[_pin_index].pin != pin) {
        cli();
        pins[_pin_index].pin = pin;
        pins[_pin_index].sum = 0;
        pins[_pin_index].sum_count = 0;
        sei();        
    }
}

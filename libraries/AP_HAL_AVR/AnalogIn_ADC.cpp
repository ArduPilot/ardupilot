
#include <avr/io.h>
#include <avr/interrupt.h>

#include "AnalogIn.h"
using namespace AP_HAL_AVR;

ADCSource::ADCSource(uint8_t pin, float prescale) :
    _pin(pin),
    _sum_count(0),
    _sum(0),
    _prescale(prescale)
{}

float ADCSource::read() {
    return _prescale * read_average();
}

void ADCSource::set_pin(uint8_t pin) {
    _pin = pin;
}

/* read_average is called from the normal thread (not an interrupt). */
float ADCSource::read_average() {
    uint16_t sum;
    uint8_t sum_count;

    /* Block until there is a new sample. Will only happen if you
     * call read_average very frequently.
     * I don't like the idea of blocking like this but we don't
     * have a way to bubble control upwards at the moment. -pch */
    while( _sum_count == 0 );

    /* Read and clear in a critical section */
    uint8_t sreg = SREG;
    cli();

    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;

    SREG = sreg;

    float avg = sum / (float) sum_count;
    return avg;

}

void ADCSource::setup_read() {
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        ADMUX = _BV(REFS0)|_BV(MUX4)|_BV(MUX3)|_BV(MUX2)|_BV(MUX1);
    } else if (_pin == ANALOG_INPUT_NONE) {
        /* noop */
    } else {
        ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((_pin >> 3) & 0x01) << MUX5);
        ADMUX = _BV(REFS0) | (_pin & 0x07);
    }
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * cli/sei to read these variables from outside an interrupt. */
void ADCSource::new_sample(uint16_t sample) {
    _sum += sample;
    if (_sum_count >= 63) {
        _sum >>= 1;
        _sum_count = 32;
    } else {
        _sum_count++;
    }
}

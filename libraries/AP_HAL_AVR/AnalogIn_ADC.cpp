/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>
#include <avr/interrupt.h>

#include "AnalogIn.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

ADCSource::ADCSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _settle_time_ms(0)
{
    set_pin(pin);
}

float ADCSource::read_average() {
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        uint16_t v = (uint16_t) _read_average();
        return 1126400UL / v;
    } else {
        return _read_average();
    }
}

float ADCSource::read_latest() {
    uint8_t sreg = SREG;
    cli();
    uint16_t latest = _latest;
    SREG = sreg;
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        return 1126400UL / latest;
    } else {
        return latest;
    }
}

/*
  return voltage from 0.0 to 5.0V, scaled to Vcc
 */
float ADCSource::voltage_average(void)
{
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float v = read_average();
    // constrain Vcc reading so that a bad Vcc doesn't throw off
    // the reading of other sources too badly
    if (vcc_mV < 4000) {
        vcc_mV = 4000;
    } else if (vcc_mV > 6000) {
        vcc_mV = 6000;
    }
    return v * vcc_mV * 9.765625e-7; // 9.765625e-7 = 1.0/(1024*1000)
}

/*
  return voltage from 0.0 to 5.0V, scaled to Vcc
 */
float ADCSource::voltage_latest(void)
{
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        return read_latest() * 0.001f;
    }
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float v = read_latest();
    // constrain Vcc reading so that a bad Vcc doesn't throw off
    // the reading of other sources too badly
    if (vcc_mV < 4000) {
        vcc_mV = 4000;
    } else if (vcc_mV > 6000) {
        vcc_mV = 6000;
    }
    return v * vcc_mV * 9.765625e-7; // 9.765625e-7 = 1.0/(1024*1000)
}

/*
  return voltage from 0.0 to 5.0V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 5.0V.
 */
float ADCSource::voltage_average_ratiometric(void)
{
    float v = read_average();
    return v * (5.0f / 1023.0f);
}

void ADCSource::set_pin(uint8_t pin) {
    if (pin != _pin) {
        // ensure the pin is marked as an INPUT pin
        if (pin != ANALOG_INPUT_NONE && pin != ANALOG_INPUT_BOARD_VCC) {
            int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
            if (dpin != -1) {
                // enable as input without a pull-up. This gives the
                // best results for our analog sensors
                hal.gpio->pinMode(dpin, HAL_GPIO_INPUT);
                hal.gpio->write(dpin, 0);
            }
        }
        uint8_t sreg = SREG;
        cli();
        _sum = 0;
        _sum_count = 0;
        _last_average = 0;
        _latest = 0;
        _pin = pin;
        SREG = sreg;
    }
}

void ADCSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void ADCSource::set_settle_time(uint16_t settle_time_ms) 
{
    _settle_time_ms = settle_time_ms;
}

/* read_average is called from the normal thread (not an interrupt). */
float ADCSource::_read_average() {
    uint16_t sum;
    uint8_t sum_count;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    uint8_t sreg = SREG;
    cli();

    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;

    SREG = sreg;

    float avg = sum / (float) sum_count;

    _last_average = avg;
    return avg;
}

void ADCSource::setup_read() {
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(digital_pin, 1);
    }
    if (_settle_time_ms != 0) {
        _read_start_time_ms = hal.scheduler->millis();
    }
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        ADCSRB = (ADCSRB & ~(1 << MUX5));
        ADMUX = _BV(REFS0)|_BV(MUX4)|_BV(MUX3)|_BV(MUX2)|_BV(MUX1);
    } else if (_pin == ANALOG_INPUT_NONE) {
        /* noop */
    } else {
        ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((_pin >> 3) & 0x01) << MUX5);
        ADMUX = _BV(REFS0) | (_pin & 0x07);
    }
}

void ADCSource::stop_read() {
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(digital_pin, 0);
    }
}

bool ADCSource::reading_settled() 
{
    if (_settle_time_ms != 0 && (hal.scheduler->millis() - _read_start_time_ms) < _settle_time_ms) {
        return false;
    }
    return true;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * cli/sei to read these variables from outside an interrupt. */
void ADCSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    if (_sum_count >= 63) {
        _sum >>= 1;
        _sum_count = 32;
    } else {
        _sum_count++;
    }
}
#endif

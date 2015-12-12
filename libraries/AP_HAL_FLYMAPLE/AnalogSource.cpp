/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "FlymapleWirish.h"
#include "AnalogIn.h"
using namespace AP_HAL_FLYMAPLE_NS;

extern const AP_HAL::HAL& hal;

FLYMAPLEAnalogSource::FLYMAPLEAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _settle_time_ms(0)
{
    set_pin(pin);
}

float FLYMAPLEAnalogSource::read_average() {
    return _read_average();
}

float FLYMAPLEAnalogSource::read_latest() {
    noInterrupts();
    uint16_t latest = _latest;
    interrupts();
    return latest;
}

/*
  return voltage from 0.0 to 3.3V, scaled to Vcc
 */
float FLYMAPLEAnalogSource::voltage_average(void)
{
    return voltage_average_ratiometric();
}

float FLYMAPLEAnalogSource::voltage_latest(void)
{
    float v = read_latest();
    return v * (3.3f / 4095.0f);
}

/*
  return voltage from 0.0 to 3.3V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 3.3V.
 */
float FLYMAPLEAnalogSource::voltage_average_ratiometric(void)
{
    float v = read_average();
    return v * (3.3f / 4095.0f);
}

void FLYMAPLEAnalogSource::set_pin(uint8_t pin) {
    if (pin != _pin) {
        // ensure the pin is marked as an INPUT pin
        if (pin != ANALOG_INPUT_NONE && pin != ANALOG_INPUT_BOARD_VCC) {
            int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
            if (dpin != -1) {
                pinMode(dpin, INPUT_ANALOG);
            }
        }
        noInterrupts();
        _sum = 0;
        _sum_count = 0;
        _last_average = 0;
        _latest = 0;
        _pin = pin;
        interrupts();
    }
}

void FLYMAPLEAnalogSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void FLYMAPLEAnalogSource::set_settle_time(uint16_t settle_time_ms) 
{
    _settle_time_ms = settle_time_ms;
}

/* read_average is called from the normal thread (not an interrupt). */
float FLYMAPLEAnalogSource::_read_average() 
{
    uint16_t sum;
    uint8_t sum_count;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    noInterrupts();
    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;

    interrupts();
    float avg = sum / (float) sum_count;

    _last_average = avg;
    return avg;
}

void FLYMAPLEAnalogSource::setup_read() {
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(digital_pin, 1);
    }
    if (_settle_time_ms != 0) {
        _read_start_time_ms = AP_HAL::millis();
    }
    adc_reg_map *regs = ADC1->regs;
    adc_set_reg_seqlen(ADC1, 1);
    uint8 channel = 0;
    if (_pin == ANALOG_INPUT_BOARD_VCC)
        channel = PIN_MAP[FLYMAPLE_VCC_ANALOG_IN_PIN].adc_channel; 
    else if (_pin == ANALOG_INPUT_NONE) 
        ; // NOOP
    else
        channel = PIN_MAP[_pin].adc_channel;
    regs->SQR3 = channel;
}

void FLYMAPLEAnalogSource::stop_read() {
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(digital_pin, 0);
    }
}

bool FLYMAPLEAnalogSource::reading_settled() 
{
    if (_settle_time_ms != 0 && (AP_HAL::millis() - _read_start_time_ms) < _settle_time_ms) {
        return false;
    }
    return true;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * noInterrupts()/interrupts() to read these variables from outside an interrupt. */
void FLYMAPLEAnalogSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    if (_sum_count >= 15) { // Flymaple has a 12 bit ADC, so can only sum 16 in a uint16_t
        _sum >>= 1;
        _sum_count = 8;
    } else {
        _sum_count++;
    }
}

#endif 

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
  Copied from: Flymaple port by Mike McCauley
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI

#include <AP_HAL/AP_HAL.h>
#include "AnalogIn.h"
#include <adc.h>
#include <boards.h>
#include <gpio_hal.h>
#include "GPIO.h"
#include <stm32f4xx.h>

extern const AP_HAL::HAL& hal;

using namespace REVOMINI;

REVOMINIAnalogSource::REVOMINIAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _latest(0),
    _last_average(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _settle_time_ms(0),
    _read_start_time_ms(0)
{
    if(pin != ANALOG_INPUT_NONE) {
	set_pin(pin);
    }
}

float REVOMINIAnalogSource::read_average() {
    return _read_average();
}

float REVOMINIAnalogSource::read_latest() {
    noInterrupts();
    uint16_t latest = _latest;
    interrupts();
    return latest;
}

/*
  return voltage from 0.0 to 3.3V, scaled to Vcc
 */
float REVOMINIAnalogSource::voltage_average(void)
{
    return voltage_average_ratiometric();
}

float REVOMINIAnalogSource::voltage_latest(void)
{
    float v = read_latest();
    return v * (3.3f / 4096.0f);
}

/*
  return voltage from 0.0 to 3.3V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 3.3V.
 */
float REVOMINIAnalogSource::voltage_average_ratiometric(void)
{
    float v = read_average();
    return v * (3.3f / 4096.0f);
}

void REVOMINIAnalogSource::set_pin(uint8_t pin) {
    if (pin != _pin) {
	// ensure the pin is marked as an INPUT pin
	if (pin != ANALOG_INPUT_NONE && pin != ANALOG_INPUT_BOARD_VCC && pin < BOARD_NR_GPIO_PINS) {
		hal.gpio->pinMode(pin, INPUT_ANALOG);
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

void REVOMINIAnalogSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void REVOMINIAnalogSource::set_settle_time(uint16_t settle_time_ms)
{
    _settle_time_ms = settle_time_ms;
}

/* read_average is called from the normal thread (not an interrupt). */
float REVOMINIAnalogSource::_read_average()
{

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    hal.scheduler->suspend_timer_procs();
    _last_average = _sum / _sum_count;
    _sum = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _last_average;
}

void REVOMINIAnalogSource::setup_read() {
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, OUTPUT);
        hal.gpio->write(digital_pin, 1);
    }
    if (_settle_time_ms != 0) {
        _read_start_time_ms = /* hal.scheduler->*/ AP_HAL::millis();
    }
    const adc_dev *dev = _find_device();//PIN_MAP[_pin].adc_device;

    if (_pin == ANALOG_INPUT_BOARD_VCC){
	  ADC_TempSensorVrefintCmd(ENABLE);
	  /* Wait until ADC + Temp sensor start */
	  uint16_t T_StartupTimeDelay = 1024;
	  while (T_StartupTimeDelay--);

	  /* Enable Vrefint on Channel17 */
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 2, ADC_SampleTime_84Cycles);
    } else if (_pin == ANALOG_INPUT_NONE) {

    } else if(dev != NULL) {
	adc_set_reg_seqlen(dev, 1);
	uint8_t channel = 0;
	channel = PIN_MAP[_pin].adc_channel;
	adc_disable(dev);
	    ADC_RegularChannelConfig(dev->adcx, channel, 1, ADC_SampleTime_84Cycles);
	adc_enable(dev);
    }
}

void REVOMINIAnalogSource::stop_read() {
    if(_pin == ANALOG_INPUT_BOARD_VCC) {
	ADC_TempSensorVrefintCmd(DISABLE);
    }
    if (_stop_pin != ANALOG_INPUT_NONE) {
        uint8_t digital_pin = hal.gpio->analogPinToDigitalPin(_stop_pin);
        hal.gpio->pinMode(digital_pin, OUTPUT);
        hal.gpio->write(digital_pin, 0);
    }
}

bool REVOMINIAnalogSource::reading_settled()
{
    if (_settle_time_ms != 0 && (/* hal.scheduler->*/AP_HAL::millis() - _read_start_time_ms) < _settle_time_ms) {
        return false;
    }
    return true;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * noInterrupts()/interrupts() to read these variables from outside an interrupt. */
void REVOMINIAnalogSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    // Copied from AVR code in ArduPlane-2.74b, but AVR code is wrong!
    if (_sum_count >= 15) { // REVOMINI has a 12 bit ADC, so can only sum 16 in a uint16_t
        _sum >>= 1;
        _sum_count = 8;
    } else {
        _sum_count++;
    }
}

const adc_dev* REVOMINIAnalogSource::_find_device() {

    if(_pin != ANALOG_INPUT_NONE) {
	return _ADC1;
    }
    return NULL;
}
#endif 

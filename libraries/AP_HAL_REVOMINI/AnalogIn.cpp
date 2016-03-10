
#include <AP_HAL/AP_HAL.h>
#include "AnalogIn.h"
#include <adc.h>
#include <boards.h>
#include <gpio_hal.h>
#include <AP_HAL_REVOMINI/GPIO.h>

extern const AP_HAL::HAL& hal;

using namespace REVOMINI;

REVOMINIAnalogSource::REVOMINIAnalogSource(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _last_value(initial_value)
{
    if ((_pin < 0) || (_pin >= BOARD_NR_GPIO_PINS)) {
            _pin = 200;
        }

    hal.gpio->pinMode(_pin, INPUT_ANALOG);
}

float REVOMINIAnalogSource::read_average() {
    float temp;
    temp = (read_latest() * 0.8) + (_last_value * 0.2f);
    _last_value = _value;
    return temp;
}

float REVOMINIAnalogSource::read_latest() {
    if ((_pin < 0) || (_pin >= BOARD_NR_GPIO_PINS)) {
            return 0.0;
        }

    const adc_dev *dev = PIN_MAP[_pin].adc_device;
    if (dev == NULL) {
        return 0.0;
    }
    _value = adc_read(dev, PIN_MAP[_pin].adc_channel);
    return _value;
}
float REVOMINIAnalogSource::voltage_average_ratiometric(){
    return _value;
}
void REVOMINIAnalogSource::set_stop_pin(uint8_t p){}
void REVOMINIAnalogSource::set_settle_time(uint16_t settle_time_ms){}
/*
  return voltage in Volts
 */
float REVOMINIAnalogSource::voltage_average()
{
    return (3.3f/4096.0f) * read_average();
}
float REVOMINIAnalogSource::voltage_latest()
{
    return (3.3f/4096.0f) * read_latest();
}


void REVOMINIAnalogSource::set_pin(uint8_t pin)
{
    if(pin == _pin)
	return;
    if ((_pin < 0) || (_pin >= BOARD_NR_GPIO_PINS)) {
            return;
        }
    _pin = pin;
    hal.gpio->pinMode(_pin, INPUT_ANALOG);
}


REVOMINIAnalogIn::REVOMINIAnalogIn() : 
    _board_voltage(0),
    _servorail_voltage(0),
    _power_flags(0)
{}

void REVOMINIAnalogIn::init()
{}

AP_HAL::AnalogSource* REVOMINIAnalogIn::channel(int16_t pin) {
    /*
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
            return new EmptyAnalogSource(0.0);
        }
    */
    return new REVOMINIAnalogSource(pin, 0.0);
}



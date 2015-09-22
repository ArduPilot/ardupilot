#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AnalogIn_ErleBrain2.h"

#define ERLEBRAIN2_ANALOGIN_DEBUG 0
#if ERLEBRAIN2_ANALOGIN_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

ErleBrain2AnalogSource::ErleBrain2AnalogSource(int16_t pin):
    _pin(pin),
    _value(0.0f)
{
}

void ErleBrain2AnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
}

float ErleBrain2AnalogSource::read_average()
{ 
    return read_latest();
}

float ErleBrain2AnalogSource::read_latest()
{
    return _value;
}

float ErleBrain2AnalogSource::voltage_average()
{
    return _value;
}

float ErleBrain2AnalogSource::voltage_latest()
{
    return _value;
}

float ErleBrain2AnalogSource::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

ErleBrain2AnalogIn::ErleBrain2AnalogIn() 
{
    _adc = new AP_ADC_ADS1115();
    _channels_number = _adc->get_channels_number();
}

AP_HAL::AnalogSource* ErleBrain2AnalogIn::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new ErleBrain2AnalogSource(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return NULL;
}

void ErleBrain2AnalogIn::init(void* implspecific)
{
    _adc->init();
    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ErleBrain2AnalogIn::_update, void));
    hal.scheduler->resume_timer_procs();
}

void ErleBrain2AnalogIn::_update()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 100000) {
        return;
    }

    adc_report_s reports[ERLEBRAIN2_ADC_MAX_CHANNELS];

    size_t rc = _adc->read(reports, 6);

    for (size_t i = 0; i < rc; i++) {
        for (uint8_t j=0; j < rc; j++) {
            ErleBrain2AnalogSource *source = _channels[j];

#if 0
            if (source != NULL) {
                fprintf(stderr, "pin: %d id: %d data: %.3f\n", source->_pin, reports[i].id, reports[i].data);
            }
#endif

            if (source != NULL && reports[i].id == source->_pin) {
                source->_value = reports[i].data / 1000;
            }
        }
    }

    _last_update_timestamp = hal.scheduler->micros();
}

#endif

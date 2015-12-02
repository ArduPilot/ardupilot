#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AnalogIn_ADS1115.h"

#define ADS1115_ANALOGIN_DEBUG 0
#if ADS1115_ANALOGIN_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

ADS1115AnalogSource::ADS1115AnalogSource(int16_t pin):
    _pin(pin),
    _value(0.0f)
{
}

void ADS1115AnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
}

float ADS1115AnalogSource::read_average()
{ 
    return read_latest();
}

float ADS1115AnalogSource::read_latest()
{
    return _value;
}

float ADS1115AnalogSource::voltage_average()
{
    return _value;
}

float ADS1115AnalogSource::voltage_latest()
{
    return _value;
}

float ADS1115AnalogSource::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

ADS1115AnalogIn::ADS1115AnalogIn() 
{
    _adc = new AP_ADC_ADS1115();
    _channels_number = _adc->get_channels_number();
}

AP_HAL::AnalogSource* ADS1115AnalogIn::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new ADS1115AnalogSource(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return NULL;
}

void ADS1115AnalogIn::init()
{
    _adc->init();
    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&ADS1115AnalogIn::_update, void));
    hal.scheduler->resume_timer_procs();
}

void ADS1115AnalogIn::_update()
{
    if (AP_HAL::micros() - _last_update_timestamp < 100000) {
        return;
    }

    adc_report_s reports[ADS1115_ADC_MAX_CHANNELS];

    size_t rc = _adc->read(reports, 6);

    for (size_t i = 0; i < rc; i++) {
        for (uint8_t j=0; j < rc; j++) {
            ADS1115AnalogSource *source = _channels[j];

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

    _last_update_timestamp = AP_HAL::micros();
}

#endif

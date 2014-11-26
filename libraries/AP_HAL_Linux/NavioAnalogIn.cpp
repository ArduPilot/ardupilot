#include "NavioAnalogIn.h"

#define NAVIO_ANALOGIN_DEBUG 1
#if NAVIO_ANALOGIN_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

NavioAnalogSource::NavioAnalogSource(int16_t pin):
    _pin(pin),
    _value(0.0f)
{
}

void NavioAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
}

float NavioAnalogSource::read_average()
{ 
    return read_latest();
}

float NavioAnalogSource::read_latest()
{
    return _value;
}

float NavioAnalogSource::voltage_average()
{
    return _value;
}

float NavioAnalogSource::voltage_latest()
{
    return _value;
}

float NavioAnalogSource::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

NavioAnalogIn::NavioAnalogIn() 
{
    _adc = new AP_ADC_ADS1115();
}

AP_HAL::AnalogSource* NavioAnalogIn::channel(int16_t pin)
{
    for (uint8_t j=0; j<NAVIO_ADC_MAX_CHANNELS; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new NavioAnalogSource(pin);
            return _channels[j];
        }
    }
    hal.console->println("Out of analog channels");
    return NULL;
}

void NavioAnalogIn::init(void* implspecific)
{
    _adc->Init();
    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process( AP_HAL_MEMBERPROC(&NavioAnalogIn::_update));
    hal.scheduler->resume_timer_procs();
}

void NavioAnalogIn::_update()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 100000) {
        return;
    }


    adc_report_s reports[NAVIO_ADC_MAX_CHANNELS];
    uint8_t channel_numbers = NAVIO_ADC_MAX_CHANNELS;
    /* ugly cast in order to conform to the old API */
    ssize_t rc = _adc->Ch6(&channel_numbers, (float *) &reports);

#if 0
    for (int i = 4; i < rc; i++) {
        fprintf(stderr,"CH%d: %f ", reports[i].id, reports[i].data);
    }
    fprintf(stderr, "\n");
#endif

    for (int i = 0; i < rc; i++) {
        for (uint8_t j=0; j<NAVIO_ADC_MAX_CHANNELS; j++) {
            NavioAnalogSource *source = _channels[j];
            if (source != NULL && reports[i].id == source->_pin) {
                source->_value = reports[i].data / 1000;
            }
        }
    }

    _last_update_timestamp = hal.scheduler->micros();
}

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AnalogIn.h"
#include <drivers/drv_adc.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <nuttx/analog/adc.h>

#define ANLOGIN_DEBUGGING 0

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace PX4;

int PX4AnalogIn::_adc_fd;
uint32_t PX4AnalogIn::_last_run;
PX4AnalogSource* PX4AnalogIn::_channels[PX4_ANALOG_MAX_CHANNELS] = {};

PX4AnalogSource::PX4AnalogSource(int16_t pin, float initial_value, float scale) :
	_pin(pin),
    _value(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0),
    _scale(scale)
{}

float PX4AnalogSource::read_average() 
{
    if (_sum_count == 0) {
        return _value;
    }
    hal.scheduler->suspend_timer_procs();
    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _value;
}

float PX4AnalogSource::read_latest() 
{
    return _latest_value;
}

void PX4AnalogSource::set_pin(uint8_t pin)
{
    _pin = pin;
}


PX4AnalogIn::PX4AnalogIn()
{}

void PX4AnalogIn::init(void* machtnichts)
{
	_adc_fd = open(ADC_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (_adc_fd == -1) {
        hal.scheduler->panic("Unable to open " ADC_DEVICE_PATH);
	}
    hal.scheduler->register_timer_process(_analogin_timer);
}

/*
  called at 1kHz
 */
void PX4AnalogIn::_analogin_timer(uint32_t now)
{
    // read adc at 100Hz
    uint32_t delta_t = now - _last_run;
    if (delta_t < 10000) {
        return;
    }
    _last_run = now;

    struct adc_msg_s buf_adc[8];

    /* read all channels available */
    int ret = read(_adc_fd, &buf_adc, sizeof(buf_adc));
    if (ret == -1) return;

    // match the incoming channels to the currently active pins
    for (uint8_t i=0; i<ret/sizeof(buf_adc[0]); i++) {
        Debug("chan %u value=%u\n",
              (unsigned)buf_adc[i].am_channel,
              (unsigned)buf_adc[i].am_data);
        for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
            PX4::PX4AnalogSource *c = _channels[j];
            if (c != NULL && buf_adc[i].am_channel == c->_pin) {
                c->_latest_value = buf_adc[i].am_data;
                c->_sum_value += c->_latest_value;
                c->_sum_count++;
                if (c->_sum_count == 254) {
                    c->_sum_value /= 2;
                    c->_sum_count /= 2;
                }
            }
        }
    }
}

AP_HAL::AnalogSource* PX4AnalogIn::channel(int16_t pin) 
{
    return channel(pin, 1.0);
}

AP_HAL::AnalogSource* PX4AnalogIn::channel(int16_t pin, float scale) 
{
    for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new PX4AnalogSource(pin, 0.0, scale);
            return _channels[j];
        }
    }
    hal.console->println("Out of analog channels");
    return NULL;
}

#endif // CONFIG_HAL_BOARD

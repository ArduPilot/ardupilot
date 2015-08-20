/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn_PXF.h"

extern const AP_HAL::HAL& hal;



const char* PXFAnalogSource::analog_sources[PXF_ANALOG_IN_COUNT] = {
    "in_voltage0_raw",
    "in_voltage1_raw",
    "in_voltage2_raw",
    "in_voltage3_raw",
    "in_voltage4_raw",
    "in_voltage5_raw",
    "in_voltage6_raw",            
    "in_voltage7_raw",                        
};

PXFAnalogSource::PXFAnalogSource(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _sum_value(0),
    _sum_count(0),
    _pin_fd(-1)
{
    reopen_pin();
}

void PXFAnalogSource::reopen_pin(void)
{
    char buf[100];

    if (_pin_fd != -1) {
        close(_pin_fd);
        _pin_fd = -1;
    }

    if (_pin < 0) {
        return;
    }
    
    if (_pin > PXF_ANALOG_IN_COUNT) {
        // invalid pin
        return;
    }

    // Construct the path by appending strings
    strncpy(buf, PXF_ANALOG_IN_DIR, sizeof(buf));
    strncat(buf, PXFAnalogSource::analog_sources[_pin], sizeof(buf));
    
    _pin_fd = open(buf, O_RDONLY | O_NONBLOCK);
    if (_pin_fd == -1) {
        ::printf("Failed to open analog pin %s\n", buf);
    }    
}

float PXFAnalogSource::read_average() 
{
    read_latest();
    if (_sum_count == 0) {
        return _value;
    }
    hal.scheduler->suspend_timer_procs();
    _value = _sum_value / _sum_count;
    // _value_ratiometric = _sum_ratiometric / _sum_count;
    _sum_value = 0;
    // _sum_ratiometric = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _value;
}

float PXFAnalogSource::read_latest() 
{
    char sbuf[10];

    if (_pin_fd == -1) {
        _latest = 0;
        return 0;
    }

    memset(sbuf, 0, sizeof(sbuf));
    pread(_pin_fd, sbuf, sizeof(sbuf)-1, 0);

    _latest = atoi(sbuf) * BBB_VOLTAGE_SCALING;      
    _sum_value += _latest;
    _sum_count++;

    return _latest;
}

// output is in volts
float PXFAnalogSource::voltage_average() 
{
    return read_average();
}

float PXFAnalogSource::voltage_latest() 
{
    read_latest();
    return _latest;
}

void PXFAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }

    hal.scheduler->suspend_timer_procs();
    _pin = pin;
    _sum_value = 0;
    // _sum_ratiometric = 0;
    _sum_count = 0;
    _latest = 0;
    _value = 0;    
    reopen_pin();
    // _value_ratiometric = 0;
    hal.scheduler->resume_timer_procs();
}

void PXFAnalogSource::set_stop_pin(uint8_t p)
{}

void PXFAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

PXFAnalogIn::PXFAnalogIn()
{}

void PXFAnalogIn::init(void* machtnichts)
{}


AP_HAL::AnalogSource* PXFAnalogIn::channel(int16_t pin) {
    return new PXFAnalogSource(pin, 0);
}

#endif // CONFIG_HAL_BOARD

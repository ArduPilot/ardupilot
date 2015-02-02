/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

// Note that echo BB-ADC cape should be loaded
#define ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"
#define ANALOG_IN_COUNT 8

#define BBB_VOLTAGE_SCALING 0.0007232f

const char* LinuxAnalogSource::analog_sources[ANALOG_IN_COUNT] = {
    "in_voltage0_raw",
    "in_voltage1_raw",
    "in_voltage2_raw",
    "in_voltage3_raw",
    "in_voltage4_raw",
    "in_voltage5_raw",
    "in_voltage6_raw",            
    "in_voltage7_raw",                        
};

LinuxAnalogSource::LinuxAnalogSource(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _sum_value(0),
    _sum_count(0),
    _pin_fd(-1)
{
    reopen_pin();
}

void LinuxAnalogSource::reopen_pin(void)
{
    char buf[100];

    if (_pin_fd != -1) {
        close(_pin_fd);
        _pin_fd = -1;
    }

    if (_pin < 0) {
        return;
    }
    
    if (_pin > ANALOG_IN_COUNT) {
        // invalid pin
        return;
    }

    // Construct the path by appending strings
    strncpy(buf, ANALOG_IN_DIR, sizeof(buf));
    strncat(buf, LinuxAnalogSource::analog_sources[_pin], sizeof(buf));
    
    _pin_fd = open(buf, O_RDONLY | O_NONBLOCK);
    if (_pin_fd == -1) {
        ::printf("Failed to open analog pin %s\n", buf);
    }    
}

float LinuxAnalogSource::read_average() 
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

float LinuxAnalogSource::read_latest() 
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
float LinuxAnalogSource::voltage_average() 
{
    return read_average();
}

float LinuxAnalogSource::voltage_latest() 
{
    read_latest();
    return _latest;
}

void LinuxAnalogSource::set_pin(uint8_t pin)
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

void LinuxAnalogSource::set_stop_pin(uint8_t p)
{}

void LinuxAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

LinuxAnalogIn::LinuxAnalogIn()
{}

void LinuxAnalogIn::init(void* machtnichts)
{}


AP_HAL::AnalogSource* LinuxAnalogIn::channel(int16_t pin) {
    return new LinuxAnalogSource(pin, 0);
}

#endif // CONFIG_HAL_BOARD

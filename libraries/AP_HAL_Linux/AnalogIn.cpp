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
    _sum_count(0)
{
    char buf[100];

    if (_pin < 0){
        return;
    }
    
    if (_pin > ANALOG_IN_COUNT){
        hal.scheduler->panic("Analog pin out of bounds\n");
    }
    // Construct the path by appending strings
    sprintf(buf,ANALOG_IN_DIR);
    sprintf(buf + strlen(buf), LinuxAnalogSource::analog_sources[_pin]);
    
    pin_fd = open(buf, O_RDONLY | O_NONBLOCK);
    if (pin_fd == -1) {
        hal.scheduler->panic("Unable to open ADC pin");
    }    
}

float LinuxAnalogSource::read_average() 
{
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
    char buffer;
    char sbuf[10];
    int bytes, count = 0;
    char buf[100];    

    if (pin_fd == 0){
        return 0;
    }

    // Open the file every time (that's the way these files should be handled)
    // Construct the path by appending strings
    sprintf(buf,ANALOG_IN_DIR);
    sprintf(buf + strlen(buf), LinuxAnalogSource::analog_sources[_pin]);

    pin_fd = open(buf, O_RDONLY | O_NONBLOCK);
    if (pin_fd == -1) {
        hal.scheduler->panic("Unable to open ADC pin");
    }    

    do {
        bytes = read(pin_fd, &buffer, sizeof(char));
        // printf("buffer:%c\n", buffer);
        // printf("bytes: %d\n", bytes);
        sbuf[count++] = buffer;
    } while( bytes > 0); 

    // printf("string recorded: %s\n", sbuf);
    // printf("int obtained: %d\n", atoi(sbuf));

    _latest = atoi(sbuf);      
    _sum_value += (float) atoi(sbuf);
    _sum_count++;

    // close the file
    if(pin_fd > 3 && close(pin_fd) < 0) {
        hal.scheduler->panic("Error closing fd");
    }
    //_pin = -1;
    return _latest;
}

// output is a number ranging from 0 to 4096.
float LinuxAnalogSource::voltage_average() 
{
    //return (5.0 * read_average()) / 4096.0;
    // Hack because voltage_latest is never called
    return (5.0 * read_latest()) / 4096.0;
}

float LinuxAnalogSource::voltage_latest() 
{
    return (5.0 * read_latest()) / 4096.0;
}

void LinuxAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }

    // if (pin > ANALOG_IN_COUNT - 1){
    //     return;
    // }

    hal.scheduler->suspend_timer_procs();
    _pin = pin;
    _sum_value = 0;
    // _sum_ratiometric = 0;
    _sum_count = 0;
    _latest = 0;
    _value = 0;    
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

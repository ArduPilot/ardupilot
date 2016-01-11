/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn_IIO.h"

extern const AP_HAL::HAL& hal;

const char* AnalogSource_IIO::analog_sources[IIO_ANALOG_IN_COUNT] = {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
    "in_voltage0_raw",
    "in_voltage1_raw",
    "in_voltage2_raw",
    "in_voltage3_raw",
    "in_voltage4_raw",
    "in_voltage5_raw",
    "in_voltage6_raw",
    "in_voltage7_raw",
#else
    "in_voltage0_raw",
#endif
};

AnalogSource_IIO::AnalogSource_IIO(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _sum_value(0),
    _sum_count(0),
    _pin_fd(-1)
{
    init_pins();
    select_pin();
}

void AnalogSource_IIO::init_pins(void)
{
    char buf[100];
    for (int i=0; i < IIO_ANALOG_IN_COUNT; i++) {
        // Construct the path by appending strings
        strncpy(buf, IIO_ANALOG_IN_DIR, sizeof(buf));
        strncat(buf, AnalogSource_IIO::analog_sources[i], sizeof(buf));

        fd_analog_sources[i] = open(buf, O_RDONLY | O_NONBLOCK);
        if (fd_analog_sources[i] == -1) {
            ::printf("Failed to open analog pin %s\n", buf);
        }
    }
}

/*
  selects a diferent file descriptor among in the fd_analog_sources array
 */
void AnalogSource_IIO::select_pin(void)
{
    _pin_fd = fd_analog_sources[_pin];
}

/*
  reopens an analog source (by closing and opening it again) and selects it
 */
void AnalogSource_IIO::reopen_pin(void)
{
    char buf[100];

    if (fd_analog_sources[_pin] != -1) {
        close(fd_analog_sources[_pin]);
        fd_analog_sources[_pin] = -1;
        _pin_fd = -1;
    }

    if (_pin < 0) {
        return;
    }

    if (_pin > IIO_ANALOG_IN_COUNT) {
        // invalid pin
        return;
    }

    // Construct the path by appending strings
    strncpy(buf, IIO_ANALOG_IN_DIR, sizeof(buf));
    strncat(buf, AnalogSource_IIO::analog_sources[_pin], sizeof(buf));

    fd_analog_sources[_pin] = open(buf, O_RDONLY | O_NONBLOCK);
    if (fd_analog_sources[_pin] == -1) {
        ::printf("Failed to open analog pin %s\n", buf);
    }
    _pin_fd = fd_analog_sources[_pin];
}

float AnalogSource_IIO::read_average()
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

float AnalogSource_IIO::read_latest()
{
    char sbuf[10];

    if (_pin_fd == -1) {
        _latest = 0;
        return 0;
    }

    memset(sbuf, 0, sizeof(sbuf));
    pread(_pin_fd, sbuf, sizeof(sbuf)-1, 0);
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
    _latest = atoi(sbuf) * BBB_VOLTAGE_SCALING;
#else
    _latest = atoi(sbuf);
#endif
    _sum_value += _latest;
    _sum_count++;

    return _latest;
}

// output is in volts
float AnalogSource_IIO::voltage_average()
{
    return read_average();
}

float AnalogSource_IIO::voltage_latest()
{
    read_latest();
    return _latest;
}

void AnalogSource_IIO::set_pin(uint8_t pin)
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
    select_pin();
    // _value_ratiometric = 0;
    hal.scheduler->resume_timer_procs();
}

void AnalogSource_IIO::set_stop_pin(uint8_t p)
{}

void AnalogSource_IIO::set_settle_time(uint16_t settle_time_ms)
{}

AnalogIn_IIO::AnalogIn_IIO()
{}

void AnalogIn_IIO::init()
{}


AP_HAL::AnalogSource* AnalogIn_IIO::channel(int16_t pin) {
    return new AnalogSource_IIO(pin, 0);
}

#endif // CONFIG_HAL_BOARD

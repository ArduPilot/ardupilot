/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC.h>

#include <fcntl.h>
#include <unistd.h>

#define IIO_ANALOG_IN_COUNT 8
#define IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
// Note that echo BB-ADC cape should be loaded
#define IIO_VOLTAGE_SCALING 0.00142602816
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define IIO_VOLTAGE_SCALING 3.0*1.8/4095.0
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
#define IIO_VOLTAGE_SCALING 2.0 / 1000
#else
#define IIO_VOLTAGE_SCALING 1.0
#endif

class AnalogSource_IIO : public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_IIO;
    AnalogSource_IIO(int16_t pin, float initial_value, float voltage_scaling);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }
private:
    float       _value;
    float       _latest;
    float       _sum_value;
    float       _voltage_scaling;
    uint8_t     _sum_count;
    int16_t     _pin;
    int         _pin_fd;
    int         fd_analog_sources[IIO_ANALOG_IN_COUNT];

    void init_pins(void);
    void select_pin(void);

    static const char *analog_sources[];
};

class AnalogIn_IIO : public AP_HAL::AnalogIn {
public:
    AnalogIn_IIO();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
    float board_voltage(void) { return 5.0f; }
};

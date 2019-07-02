#pragma once

#include "AP_HAL_Linux.h"

#include <fcntl.h>
#include <unistd.h>

#define IIO_ANALOG_IN_COUNT 8
#define IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
// Note that echo BB-ADC cape should be loaded
#define IIO_VOLTAGE_SCALING 0.00142602816
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define IIO_VOLTAGE_SCALING 3.0*1.8/4095.0
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
#define IIO_VOLTAGE_SCALING 1.8/4095.0
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
#define IIO_VOLTAGE_SCALING 3.0*1.8/4095.0
#else
#define IIO_VOLTAGE_SCALING 1.0
#endif

class AnalogSource_IIO : public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_IIO;
    AnalogSource_IIO(int16_t pin, float initial_value, float voltage_scaling);
    float read_average() override;
    float read_latest() override;
    void set_pin(uint8_t p) override;
    void set_stop_pin(uint8_t p) override;
    void set_settle_time(uint16_t settle_time_ms) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override { return voltage_average(); }
private:
    float       _value;
    float       _latest;
    float       _sum_value;
    float       _voltage_scaling;
    uint8_t     _sum_count;
    int16_t     _pin;
    int         _pin_fd;
    int         fd_analog_sources[IIO_ANALOG_IN_COUNT];
    HAL_Semaphore _semaphore;

    void init_pins(void);
    void select_pin(void);

    static const char *analog_sources[];
};

class AnalogIn_IIO : public AP_HAL::AnalogIn {
public:
    AnalogIn_IIO();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;

    // we don't yet know how to get the board voltage
    float board_voltage(void) override { return 5.0f; }
};

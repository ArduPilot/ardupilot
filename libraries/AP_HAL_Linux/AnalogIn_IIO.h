
#ifndef __AP_HAL_IIO_ANALOGIN_H__
#define __AP_HAL_IIO_ANALOGIN_H__

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF 
#define IIO_ANALOG_IN_COUNT 8
// Note that echo BB-ADC cape should be loaded
#define IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"
#define BBB_VOLTAGE_SCALING 0.00142602816
#else
#define IIO_ANALOG_IN_COUNT 8
#define IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"
#endif

class IIOAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class IIOAnalogIn;
    IIOAnalogSource(int16_t pin, float v);
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
    // float       _value_ratiometric;
    uint8_t     _sum_count;
    int16_t     _pin;
    int         _pin_fd;    

    void reopen_pin(void);

    static const char *analog_sources[IIO_ANALOG_IN_COUNT];
};

class IIOAnalogIn : public AP_HAL::AnalogIn {
public:
    IIOAnalogIn();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
    float board_voltage(void) { return 0.0f; }

};
#endif // __AP_HAL_IIO_ANALOGIN_H__

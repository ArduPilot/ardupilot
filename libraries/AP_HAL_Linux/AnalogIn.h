
#ifndef __AP_HAL_LINUX_ANALOGIN_H__
#define __AP_HAL_LINUX_ANALOGIN_H__

#include <AP_HAL_Linux.h>

#define ANALOG_IN_COUNT 8

class Linux::LinuxAnalogSource : public AP_HAL::AnalogSource {
public:
    LinuxAnalogSource(int16_t pin, float v);
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
    int32_t     pin_fd;    

    static const char *analog_sources[ANALOG_IN_COUNT];
};

class Linux::LinuxAnalogIn : public AP_HAL::AnalogIn {
public:
    LinuxAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
    float board_voltage(void) { return 0.0f; }

};
#endif // __AP_HAL_LINUX_ANALOGIN_H__

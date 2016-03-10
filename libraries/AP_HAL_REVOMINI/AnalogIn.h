
#ifndef __AP_HAL_REVOMINI_ANALOGIN_H__
#define __AP_HAL_REVOMINI_ANALOGIN_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include <hal.h>
#include <adc.h>


class REVOMINI::REVOMINIAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class REVOMINI::REVOMINIAnalogIn;
    REVOMINIAnalogSource(int16_t pin, float initial_value);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
private:
    uint8_t _pin;
    float _value;
    float _last_value;
};

class REVOMINI::REVOMINIAnalogIn : public AP_HAL::AnalogIn {
public:
    REVOMINIAnalogIn();
    void init();
    float board_voltage(void) { return _board_voltage; }
    float servorail_voltage(void) { return _servorail_voltage; }
    uint16_t power_status_flags(void) { return _power_flags; }
    AP_HAL::AnalogSource* channel(int16_t pin);
private:
    float _board_voltage;
    float _servorail_voltage;
    uint16_t _power_flags;
};
#endif // __AP_HAL_REVOMINI_ANALOGIN_H__

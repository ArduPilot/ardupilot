#pragma once

#include "AP_HAL_PX4.h"
#include <pthread.h>
#include <uORB/uORB.h>

#define PX4_ANALOG_MAX_CHANNELS 16


#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
// these are virtual pins that read from the ORB
#define PX4_ANALOG_ORB_BATTERY_VOLTAGE_PIN     100
#define PX4_ANALOG_ORB_BATTERY_CURRENT_PIN     101
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define PX4_ANALOG_VCC_5V_PIN                4
#define PX4_ANALOG_ORB_SERVO_VOLTAGE_PIN       102
#define PX4_ANALOG_ORB_SERVO_VRSSI_PIN         103
#endif

class PX4::PX4AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class PX4::PX4AnalogIn;
    PX4AnalogSource(int16_t pin, float initial_value);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();

    // implement stop pins
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms) { _settle_time_ms = settle_time_ms; }

private:
    // what pin it is attached to
    int16_t _pin;
    int16_t _stop_pin;
    uint16_t _settle_time_ms;

    // what value it has
    float _value;
    float _value_ratiometric;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    float _sum_ratiometric;
    void _add_value(float v, float vcc5V);
    float _pin_scaler();
};

class PX4::PX4AnalogIn : public AP_HAL::AnalogIn {
public:
    PX4AnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    void _timer_tick(void);
    float board_voltage(void) override { return _board_voltage; }
    float servorail_voltage(void) override { return _servorail_voltage; }
    uint16_t power_status_flags(void) override { return _power_flags; }

private:
    int _adc_fd = -1;
    int _battery_handle;
    int _servorail_handle;
    int _system_power_handle;
    uint64_t _battery_timestamp;
    uint64_t _servorail_timestamp;
    PX4::PX4AnalogSource* _channels[PX4_ANALOG_MAX_CHANNELS];

    // what pin is currently held low to stop a sonar from reading
    uint8_t _current_stop_pin_i;
    uint32_t _stop_pin_change_time;

    uint32_t _last_run;
    float _board_voltage;
    float _servorail_voltage;
    uint16_t _power_flags;

    void next_stop_pin(void);
};

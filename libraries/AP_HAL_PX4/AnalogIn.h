/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_PX4_ANALOGIN_H__
#define __AP_HAL_PX4_ANALOGIN_H__

#include <AP_HAL_PX4.h>
#include <pthread.h>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>

#define PX4_ANALOG_MAX_CHANNELS 16


#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
// these are virtual pins that read from the ORB
#define PX4_ANALOG_ORB_BATTERY_VOLTAGE_PIN     100
#define PX4_ANALOG_ORB_BATTERY_CURRENT_PIN     101
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
#define PX4_ANALOG_VCC_5V_PIN                4
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
    float voltage_average_ratiometric() { return voltage_average(); }

    // stop pins not implemented on PX4 yet
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    // what pin it is attached to
    int16_t _pin;

    // what value it has
    float _value;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    void _add_value(float v);
    float _pin_scaler();
};

class PX4::PX4AnalogIn : public AP_HAL::AnalogIn {
public:
    PX4AnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t pin);
    void _timer_tick(void);

private:
    int _adc_fd;
    int _battery_handle;
    uint64_t _battery_timestamp;
    PX4::PX4AnalogSource* _channels[PX4_ANALOG_MAX_CHANNELS];
    uint32_t _last_run;
};
#endif // __AP_HAL_PX4_ANALOGIN_H__

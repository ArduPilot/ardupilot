#pragma once

#ifdef HAL_PERIPH_ENABLE_FSO_POWER_STACK

#include <AP_DAC/AP_DAC.h>

class FSOPowerStack {
public:
    friend class AP_Periph_FW;
    FSOPowerStack(void);

    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void update(void);

private:

    enum class Option : uint32_t {
        DEBUG = 0
    };

    bool option_is_set(Option option) {
        return (options & (1U << uint32_t(option))) != 0;
    }

    AP_Int32 options;
    AP_Float payload_1_voltage;
    AP_Float payload_2_voltage;
    AP_Float payload_HV_current_max;
    AP_Float payload_1_current_max;
    AP_Float payload_2_current_max;
    AP_Float bec_temperature_max;
    AP_Float fan_1_min_Hz;
    AP_Float fan_2_min_Hz;
    AP_Float fan_3_min_Hz;
    AP_Float fan_4_min_Hz;

    uint32_t last_update_ms;
    
    struct FAN {
        uint8_t pin;
        uint32_t last_pulse_us;
        uint32_t dt_sum;
        uint32_t dt_count;
        float freq_hz;
    };

    // States used during turn on
    enum TurnOnState {
        Off,
        PreChargeStart,
        PreCharge,
        On,
        ShutDown
    };


    uint32_t last_fan_ms;
    FAN fans[4];

    void fan_handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);
    void init_fan(uint8_t pin, FAN &fan);
    void update_fans();
    void update_switches();
    void update_main_power();
    void update_payload_HV_power();
    void update_payload_BEC();
    void update_internal_BEC();
    void update_DAC();

    uint32_t last_report_ms;
    void report();


    bool switch_1_on;
    bool switch_1_switch_released;
    uint32_t switch_1_press_time_ms;

    bool switch_2_on;
    bool switch_2_switch_released;
    uint32_t switch_2_press_time_ms;

    uint32_t start_main_precharge_ms;
    uint32_t start_payload_HV_precharge_ms;
    LowPassFilterFloat  payload_HV_current_filter;  // Payload HV current input filter
    LowPassFilterFloat  payload_1_current_filter;   // Payload BEC 1 current input filter
    LowPassFilterFloat  payload_2_current_filter;   // Payload BEC 2 current input filter
    float   payload_1_temp;     // Payload BEC 1 current input filter
    float   payload_2_temp;     // Payload BEC 2 current input filter
    LowPassFilterFloat  internal_HC_current_filter;  // Internal BEC HC current input filter
    LowPassFilterFloat  internal_1_current_filter;   // Internal BEC 1 current input filter
    LowPassFilterFloat  internal_2_current_filter;   // Internal BEC 2 current input filter
    float   internal_HC_temp;     // Internal BEC HC current input filter
    float   internal_1_temp;     // Internal BEC 1 current input filter
    float   internal_2_temp;     // Internal BEC 2 current input filter

    TurnOnState main_turn_on_state = Off;
    TurnOnState payload_HV_turn_on_state = Off;

    AP_DAC dac;

    // init called after CAN init
    void late_init(void);

    bool done_late_init;
};

#endif // HAL_PERIPH_ENABLE_FSO_POWER_STACK



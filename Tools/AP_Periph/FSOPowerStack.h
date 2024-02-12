#pragma once

#ifdef HAL_PERIPH_ENABLE_FSO_POWER_STACK

#include <AP_DAC/AP_DAC.h>

extern const AP_HAL::HAL &hal;

class FSOPowerStack {
public:
    friend class AP_Periph_FW;
    FSOPowerStack(void);

    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void update(void);

private:

    enum class Option : uint32_t {
        DEBUG = 0,
        CAL = 1
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
    AP_Float cal_main_voltage;
    AP_Float cal_main_current;
    AP_Float cal_HV_current;
    AP_Float cal_HCB_current;
    AP_Float cal_LCB_current;
    AP_Float cal_payload_P1c1;
    AP_Float cal_payload_P1c2;
    AP_Float cal_payload_P2c1;
    AP_Float cal_payload_P2c2;
    

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

    // States used during turn on
    enum CalibrateState {
        Begin,
        Payload_BEC_C1,
        Payload_BEC_C2,
        Payload_BEC_1_Shunt,
        Payload_BEC_2_Shunt,
        Internal_BEC_HC_Shunt,
        Internal_BEC_1_Shunt,
        Internal_BEC_2_Shunt,
        Payload_HV_Shunt,
        Main_V_Divider,
        Bat1_Amp_Offset,
        Bat2_Amp_Offset,
        Finished
    };
    enum CalibrateSubState {
        Setup,
        Start,
        Measure,
        Write
    };

    CalibrateState cal_state = CalibrateState::Begin;
    CalibrateSubState cal_sub_state = CalibrateSubState::Setup;
    uint32_t    cal_ms;
    uint32_t    cal_measurement_count;
    float       cal_measurement_1;
    float       cal_measurement_2;
    float       cal_measurement_3;


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
    void calibrate();

    void set_main_on(){main_on = true;}
    void set_main_off(){main_on = false;}
    
    bool main_is_on(){return main_state == TurnOnState::On;}
    bool main_is_off(){return main_state == TurnOnState::Off;}

    void set_HV_payload_on(){payload_HV_on = true;}
    void set_HV_payload_off(){payload_HV_on = false;}
    
    bool HV_payload_is_on(){return payload_HV_state == TurnOnState::On;}
    bool HV_payload_is_off(){return payload_HV_state == TurnOnState::Off;}

    void set_payload_BEC_1_on(){hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 1);}
    void set_payload_BEC_1_off(){hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 0);}

    void set_payload_BEC_2_on(){hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 1);}
    void set_payload_BEC_2_off(){hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 0);}

    void set_switch_1_on(){switch_1_on = true;}
    void set_switch_1_off(){switch_1_on = false;}
    
    bool switch_1_is_on(){return switch_1_on == true;}
    bool switch_1_is_off(){return switch_1_on == false;}

    void set_switch_2_on(){switch_2_on = true;}
    void set_switch_2_off(){switch_2_on = false;}
    
    bool switch_2_is_on(){return switch_2_on == true;}
    bool switch_2_is_off(){return switch_2_on == false;}

    void set_LED_1_on(){hal.gpio->write(FSO_LED_MAIN_PIN, 1);}
    void set_LED_1_off(){hal.gpio->write(FSO_LED_MAIN_PIN, 0);}

    void set_LED_2_on(){hal.gpio->write(FSO_LED_PAYLOAD_PIN, 1);}
    void set_LED_2_off(){hal.gpio->write(FSO_LED_PAYLOAD_PIN, 0);}

    void set_LED_debug_on(){hal.gpio->write(FSO_LED_DEBUG_PIN, 1);}
    void set_LED_debug_off(){hal.gpio->write(FSO_LED_DEBUG_PIN, 0);}

    void set_main_PC_on(){hal.gpio->write(FSO_MAIN_PC_PIN, 1);}
    void set_main_PC_off(){hal.gpio->write(FSO_MAIN_PC_PIN, 0);}

    void set_bat_1_SW_on(){hal.gpio->write(FSO_BAT_1_EN_PIN, 1);}
    void set_bat_1_SW_off(){hal.gpio->write(FSO_BAT_1_EN_PIN, 0);}

    void set_bat_2_SW_on(){hal.gpio->write(FSO_BAT_2_EN_PIN, 1);}
    void set_bat_2_SW_off(){hal.gpio->write(FSO_BAT_2_EN_PIN, 0);}

    void set_payload_HV_PC_on(){hal.gpio->write(FSO_PAYLOAD_HV_PC_PIN, 1);}
    void set_payload_HV_PC_off(){hal.gpio->write(FSO_PAYLOAD_HV_PC_PIN, 0);}

    void set_payload_HV_SW_on(){hal.gpio->write(FSO_PAYLOAD_HV_EN_PIN, 1);}
    void set_payload_HV_SW_off(){hal.gpio->write(FSO_PAYLOAD_HV_EN_PIN, 0);}

    uint32_t last_report_ms;
    void report();


    bool switch_1_on;
    bool switch_1_switch_released;
    uint32_t switch_1_press_time_ms;

    bool switch_2_on;
    bool switch_2_switch_released;
    uint32_t switch_2_press_time_ms;

    bool main_on;
    bool payload_HV_on;
    bool power_on;
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

    TurnOnState main_state = Off;
    TurnOnState payload_HV_state = Off;

    AP_DAC dac;

    // init called after CAN init
    void late_init(void);

    bool done_late_init;
};

#endif // HAL_PERIPH_ENABLE_FSO_POWER_STACK



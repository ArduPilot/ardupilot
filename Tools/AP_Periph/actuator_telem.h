#pragma once

#if AP_PERIPH_ACTUATOR_TELEM_ENABLED

#ifndef HAL_ACTUATOR_TELEM_CURR_MAX_CHANNELS
#define HAL_ACTUATOR_TELEM_CURR_MAX_CHANNELS 4 
#endif

// Manager for all actuator channels
class ActuatorTelem {
public:
    friend class AP_Periph_FW;
    ActuatorTelem(void);

    void init(void);
    void send_telemetry(uint8_t channel_index, uint8_t actuator_id);
    void update(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int16 rate;
    AP_Int8 num_chans;
    AP_Int8 curr_pin1;
    AP_Float curr_amp_offset;
    AP_Float curr_amp_per_volt;
    AP_Float curr_max;
    uint32_t last_telem_update_ms;
    AP_HAL::AnalogSource *analog_sources[HAL_ACTUATOR_TELEM_CURR_MAX_CHANNELS];
};

#endif // AP_PERIPH_ACTUATOR_TELEM_ENABLED

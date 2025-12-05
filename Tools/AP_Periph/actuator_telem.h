#pragma once

#if AP_PERIPH_ACTUATOR_TELEM_ENABLED

#ifndef HAL_ACTUATOR_TELEM_CHANNELS
#define HAL_ACTUATOR_TELEM_CHANNELS 0 
#endif

// Individual actuator channel with current sensing
class ActuatorChannel {
public:
    friend class ActuatorTelem;
    
    ActuatorChannel(void);

    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void send_telemetry(uint8_t actuator_id);

private:
    AP_Int8 curr_pin;
    AP_Float curr_amp_offset;
    AP_Float curr_amp_per_volt;
    AP_Float curr_max;

    AP_HAL::AnalogSource *analog_source;
};

// Manager for all actuator channels
class ActuatorTelem {
public:
    friend class AP_Periph_FW;
    ActuatorTelem(void);

    void update(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Float rate;
    uint32_t last_telem_update_ms;

    ActuatorChannel channels[HAL_ACTUATOR_TELEM_CHANNELS];
};

typedef ActuatorTelem AP_Actuator_Telem;

#endif // AP_PERIPH_ACTUATOR_TELEM_ENABLED

#pragma once

#define ACTUATOR_CHANNELS 6

class Actuators
{
public:
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    Actuators();
    void initialize_actuators();
    void update_actuators();
    void increase_actuator(uint8_t actuator_num);
    void decrease_actuator(uint8_t actuator_num);
    void min_actuator(uint8_t actuator_num);
    void max_actuator(uint8_t actuator_num);
    void min_toggle_actuator(uint8_t actuator_num);
    void max_toggle_actuator(uint8_t actuator_num);
    void center_actuator(uint8_t actuator_num);

protected:
    AP_Float actuator_increment_step[ACTUATOR_CHANNELS];
    float aux_actuator_change_speed[ACTUATOR_CHANNELS];
    float aux_actuator_value[ACTUATOR_CHANNELS];
public:
   
};

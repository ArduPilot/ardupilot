#pragma once

#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>

class AP_Actuators
{
public:
    AP_Actuators();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Actuators);

    // Singleton access
    static AP_Actuators *get_singleton(void)
    {
        return _singleton;
    }

    void initialize_actuators();
    void update_actuators();
    void increase_actuator(uint8_t actuator_num);
    void decrease_actuator(uint8_t actuator_num);
    void min_actuator(uint8_t actuator_num);
    void max_actuator(uint8_t actuator_num);
    void min_toggle_actuator(uint8_t actuator_num);
    void max_toggle_actuator(uint8_t actuator_num);
    void center_actuator(uint8_t actuator_num);
    void set_actuator(uint8_t actuator_num, float value);


protected:
    AP_Float actuator_increment_step[AP_ACTUATORS_MAX_INSTANCES];
    float aux_actuator_change_speed[AP_ACTUATORS_MAX_INSTANCES];
    float aux_actuator_value[AP_ACTUATORS_MAX_INSTANCES];

private:
    static AP_Actuators *_singleton;
};

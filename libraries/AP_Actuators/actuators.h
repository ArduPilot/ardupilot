#pragma once

#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>

#define ACTUATOR_CHANNELS 6

class Actuators
{
public:
    Actuators();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // Singleton access
    static Actuators *get_singleton(void) {
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

    CLASS_NO_COPY(Actuators);

protected:
    AP_Float actuator_increment_step[ACTUATOR_CHANNELS];
    float aux_actuator_change_speed[ACTUATOR_CHANNELS];
    float aux_actuator_value[ACTUATOR_CHANNELS];

private:
    static Actuators *_singleton;
};

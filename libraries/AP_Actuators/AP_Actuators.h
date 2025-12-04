#pragma once

#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include "AP_Actuators_config.h"

// Always use the real class structure - implementation is conditional
#if AP_ACTUATORS_ENABLED

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

#else

// Stub class when actuators are disabled
// Note: This is only used when the library is NOT compiled
// When the library IS compiled, the real class above is used
class AP_Actuators
{
public:
    AP_Actuators() {}

    static const struct AP_Param::GroupInfo var_info[];

    CLASS_NO_COPY(AP_Actuators);

    // Return nullptr when disabled - code should check for this
    static AP_Actuators *get_singleton(void) { return nullptr; }

    void initialize_actuators() {}
    void update_actuators() {}
    void increase_actuator(uint8_t) {}
    void decrease_actuator(uint8_t) {}
    void min_actuator(uint8_t) {}
    void max_actuator(uint8_t) {}
    void min_toggle_actuator(uint8_t) {}
    void max_toggle_actuator(uint8_t) {}
    void center_actuator(uint8_t) {}
    void set_actuator(uint8_t, float) {}
};

// Define empty var_info for stub (inline to avoid multiple definition)
inline const struct AP_Param::GroupInfo AP_Actuators::var_info[] = {
    AP_GROUPEND
};

#endif // AP_ACTUATORS_ENABLED

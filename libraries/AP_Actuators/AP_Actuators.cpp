#include "AP_Actuators.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>
#include "AP_Actuators_config.h"

#if AP_ACTUATORS_ENABLED
// Singleton instance (initialized to nullptr)
AP_Actuators *AP_Actuators::_singleton = nullptr;

const AP_Param::GroupInfo AP_Actuators::var_info[] = {

    // @Param: 1_INC
    // @DisplayName: Increment step for actuator 1
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("1_INC", 1, AP_Actuators, actuator_increment_step[0], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 2_INC
    // @DisplayName: Increment step for actuator 2
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("2_INC", 2, AP_Actuators, actuator_increment_step[1], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 3_INC
    // @DisplayName: Increment step for actuator 3
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("3_INC", 3, AP_Actuators, actuator_increment_step[2], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 4_INC
    // @DisplayName: Increment step for actuator 4
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("4_INC", 4, AP_Actuators, actuator_increment_step[3], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 5_INC
    // @DisplayName: Increment step for actuator 5
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("5_INC", 5, AP_Actuators, actuator_increment_step[4], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 6_INC
    // @DisplayName: Increment step for actuator 6
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("6_INC", 6, AP_Actuators, actuator_increment_step[5], ACTUATOR_DEFAULT_INCREMENT),

    AP_GROUPEND
};

AP_Actuators::AP_Actuators()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Actuators::initialize_actuators()
{
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    for (int i = 0; i < (int)sizeof(aux_actuator_value); i++) {
        uint8_t channel_number;
        if (!SRV_Channels::find_channel((SRV_Channel::Function)(first_aux + i), channel_number)) {
            continue;
        }
        SRV_Channel* chan = SRV_Channels::srv_channel(channel_number);
        uint16_t servo_min = chan->get_output_min();
        uint16_t servo_max = chan->get_output_max();
        uint16_t servo_range = servo_max - servo_min;

        uint16_t servo_trim = chan->get_trim();
        // set current value to trim
        aux_actuator_value[i] = (servo_trim - servo_min) / static_cast<float>(servo_range);
    }
    update_actuators();
}

void AP_Actuators::update_actuators()
{
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    for (int i = 0; i < (int)sizeof(aux_actuator_value); i++) {
        uint8_t channel_number;
        if (!SRV_Channels::find_channel((SRV_Channel::Function)(first_aux + i), channel_number)) {
            continue;
        }
        SRV_Channel* chan = SRV_Channels::srv_channel(channel_number);
        uint16_t servo_min = chan->get_output_min();
        uint16_t servo_max = chan->get_output_max();
        uint16_t servo_range = servo_max - servo_min;
        chan->set_output_pwm(servo_min + servo_range * aux_actuator_value[i]);
    }
}

void AP_Actuators::increase_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    aux_actuator_value[actuator_num] = constrain_float(aux_actuator_value[actuator_num] + actuator_increment_step[actuator_num], 0.0f, 1.0f);
}

void AP_Actuators::decrease_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    aux_actuator_value[actuator_num] = constrain_float(aux_actuator_value[actuator_num] - actuator_increment_step[actuator_num], 0.0f, 1.0f);
}

void AP_Actuators::min_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    aux_actuator_value[actuator_num] = 0;
}

void AP_Actuators::max_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    aux_actuator_value[actuator_num] = 1;
}

void AP_Actuators::min_toggle_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    if (aux_actuator_value[actuator_num] < 0.4) {
        aux_actuator_value[actuator_num] = 0.5;
    } else {
        aux_actuator_value[actuator_num] = 0;
    }
}

void AP_Actuators::max_toggle_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    if (aux_actuator_value[actuator_num] >= 0.6) {
        aux_actuator_value[actuator_num] = 0.5;
    } else {
        aux_actuator_value[actuator_num] = 1;
    }
}

void AP_Actuators::center_actuator(uint8_t actuator_num)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    aux_actuator_value[actuator_num] = 0.5;
}

void AP_Actuators::set_actuator(uint8_t actuator_num, float value)
{
    if (actuator_num >= (int)sizeof(aux_actuator_value)) {
        return;
    }
    // maps value from[-1,1] to [0,1]
    aux_actuator_value[actuator_num] = constrain_float((value + 1)*0.5f, 0.0f, 1.0f);
    update_actuators();
}

#endif // AP_ACTUATORS_ENABLED
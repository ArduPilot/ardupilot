#include "Sub.h"
#include "actuators.h"


 
const AP_Param::GroupInfo Actuators::var_info[] = {

    // @Param: ACTUATOR1_INC
    // @DisplayName: Increment step for actuator 1
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("1_INC", 22, ParametersG2, actuator_increment_step[0], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: ACTUATOR2_INC
    // @DisplayName: Increment step for actuator 2
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("2_INC", 23, ParametersG2, actuator_increment_step[1], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: ACTUATOR3_INC
    // @DisplayName: Increment step for actuator 3
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("3_INC", 24, ParametersG2, actuator_increment_step[2], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: ACTUATOR4_INC
    // @DisplayName: Increment step for actuator 4
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("4_INC", 25, ParametersG2, actuator_increment_step[3], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: ACTUATOR5_INC
    // @DisplayName: Increment step for actuator 5
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("5_INC", 26, ParametersG2, actuator_increment_step[4], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: ACTUATOR6_INC
    // @DisplayName: Increment step for actuator 6
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("6_INC", 27, ParametersG2, actuator_increment_step[5], ACTUATOR_DEFAULT_INCREMENT),

    AP_GROUPEND
};

Actuators::Actuators() {
    AP_Param::setup_object_defaults(this, var_info);
}


void Actuators::initialize_servos() {
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
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
    update_servos();
}

void Actuators::update_servos() {
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
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

void Actuators::increase_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = constrain_float(aux_actuator_value[servo_num] + aux_actuator_change_speed[servo_num], 0.0f, 1.0f);
}

void Actuators::decrease_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = constrain_float(aux_actuator_value[servo_num] - aux_actuator_change_speed[servo_num], 0.0f, 1.0f);
}

void Actuators::min_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = 0;
}

void Actuators::max_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = 1;
}

void Actuators::min_toggle_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    if (aux_actuator_value[servo_num] < 0.4) {
        aux_actuator_value[servo_num] = 0.5;
    } else {
        aux_actuator_value[servo_num] = 0;
    }
}

void Actuators::max_toggle_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    if (aux_actuator_value[servo_num] >= 0.6) {
        aux_actuator_value[servo_num] = 0.5;
    } else {
        aux_actuator_value[servo_num] = 1;
    }
}

void Actuators::center_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = 0.5;
}

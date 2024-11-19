#include "Sub.h"

void Sub::initialize_servos() {
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

void Sub::update_servos() {
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

void Sub::increase_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = constrain_float(aux_actuator_value[servo_num] + aux_actuator_change_speed[servo_num], 0.0f, 1.0f);
}

void Sub::decrease_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = constrain_float(aux_actuator_value[servo_num] - aux_actuator_change_speed[servo_num], 0.0f, 1.0f);
}

void Sub::min_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = 0;
}

void Sub::max_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = 1;
}

void Sub::min_toggle_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    if (aux_actuator_value[servo_num] < 0.4) {
        aux_actuator_value[servo_num] = 0.5;
    } else {
        aux_actuator_value[servo_num] = 0;
    }
}

void Sub::max_toggle_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    if (aux_actuator_value[servo_num] >= 0.6) {
        aux_actuator_value[servo_num] = 0.5;
    } else {
        aux_actuator_value[servo_num] = 1;
    }
}

void Sub::center_servo(uint8_t servo_num) {
    if (servo_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[servo_num] = 0.5;
}

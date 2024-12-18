#include "Sub.h"

void Sub::initialize_servos() {
    const SRV_Channel::Aux_servo_function_t first_aux = SRV_Channel::Aux_servo_function_t::k_actuator1;
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
        SRV_Channels::set_output_scaled((SRV_Channel::Aux_servo_function_t) ((int16_t) first_aux + i), 0);
    }
}

void Sub::update_servos() {
    const SRV_Channel::Aux_servo_function_t first_aux = SRV_Channel::Aux_servo_function_t::k_actuator1;
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
        SRV_Channels::set_output_scaled((SRV_Channel::Aux_servo_function_t) ((int16_t) first_aux + i), aux_actuator_value[i]);
    }
}

void Sub::increase_servo(uint8_t servo_num, bool should_accelerate) {
    if (!should_accelerate) {
        aux_actuator_change_speed[servo_num] = g2.actuator_increment_step[servo_num];
    } else {
        aux_actuator_change_speed[servo_num] = constrain_float(aux_actuator_change_speed[servo_num] + g2.actuator_increment_step[servo_num], -1, 1);
    }
    aux_actuator_value[servo_num] = constrain_float(aux_actuator_value[servo_num] + aux_actuator_change_speed[servo_num], -1, 1);
}

void Sub::decrease_servo(uint8_t servo_num, bool should_accelerate) {
    if (!should_accelerate) {
        aux_actuator_change_speed[servo_num] = g2.actuator_increment_step[servo_num];
    } else {
        aux_actuator_change_speed[servo_num] = constrain_float(aux_actuator_change_speed[servo_num] + g2.actuator_increment_step[servo_num], -1, 1);
    }
    aux_actuator_value[servo_num] = constrain_float(aux_actuator_value[servo_num] - aux_actuator_change_speed[servo_num], -1, 1);
}

void Sub::min_servo(uint8_t servo_num) {
    aux_actuator_value[servo_num] = -1;
}

void Sub::max_servo(uint8_t servo_num) {
    aux_actuator_value[servo_num] = 1;
}

void Sub::min_toggle_servo(uint8_t servo_num) {
    if (aux_actuator_value[servo_num] > -1) {
        aux_actuator_value[servo_num] = -1;
    } else {
        aux_actuator_value[servo_num] = 0;
    }
}

void Sub::max_toggle_servo(uint8_t servo_num) {
    if (aux_actuator_value[servo_num] < 1) {
        aux_actuator_value[servo_num] = 1;
    } else {
        aux_actuator_value[servo_num] = 0;
    }
}

void Sub::center_servo(uint8_t servo_num) {
    aux_actuator_value[servo_num] = 0;
}

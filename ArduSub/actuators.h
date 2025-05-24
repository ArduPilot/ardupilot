#pragma once

#define ACTUATOR_CHANNELS 6

class Actuators
{
public:
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    Actuators();
    void initialize_servos();
    void update_servos();
    void increase_servo(uint8_t servo_num);
    void decrease_servo(uint8_t servo_num);
    void min_servo(uint8_t servo_num);
    void max_servo(uint8_t servo_num);
    void min_toggle_servo(uint8_t servo_num);
    void max_toggle_servo(uint8_t servo_num);
    void center_servo(uint8_t servo_num);

protected:
    float aux_actuator_change_speed[ACTUATOR_CHANNELS];
    float aux_actuator_value[ACTUATOR_CHANNELS];
public:
   
};

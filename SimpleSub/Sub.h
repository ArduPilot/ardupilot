#pragma once

#include "SimpleGCS.h"

class SimpleSub
{
public:
    uint16_t *current_motor_pwms;
    uint16_t *current_servo_pwms;
    AP_HAL::RCOutput *rcout;
    SimpleGCS *gcs;

    uint32_t last_motor_control_message_time;

    SimpleSub(SimpleGCS *_gcs, AP_HAL::RCOutput *_rcout);
    ~SimpleSub();

    bool pwm_is_valid(uint16_t pwm);
    bool set_motor_speed(uint16_t motor_index, uint16_t pwm);
    bool any_motor_is_on();
    void set_speeds_to_stopped();
    void enable_motor_rc_channels();
    void output_to_motors();
    bool handle_command_long_packet(mavlink_command_long_t &command_long_packet);
    // bool handle_direct_motor_control_packet(mavlink_direct_motor_control_t direct_motor_control_packet);
    bool handle_rc_override_packet(mavlink_rc_channels_override_t rc_override_packet);
    void arm();
    void disarm();
    void stop_if_delay_between_messages_too_long();
    bool get_is_armed();

private:
    bool is_armed;
}; // class SimpleSub

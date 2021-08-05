#include "Sub.h"
#include "SimpleGCS.h"
#include "Config.h"

SimpleSub::SimpleSub(SimpleGCS *_gcs, AP_HAL::RCOutput *_rcout) : gcs(_gcs),
                                                                  rcout(_rcout)
{
    current_motor_pwms = new uint16_t[NUMBER_MOTORS];
    current_servo_pwms = new uint16_t[NUMBER_POSITIONAL_SERVOS];
    set_speeds_to_stopped();

    for (int i = 0; i < NUMBER_POSITIONAL_SERVOS; ++i)
    {
        current_servo_pwms[i] = POSITIONAL_DEFAULT_PWMS[i];
    }
}

SimpleSub::~SimpleSub()
{
    delete current_motor_pwms;
}

bool SimpleSub::pwm_is_valid(uint16_t pwm)
{
    return pwm <= MAX_MOTOR_PWM && pwm >= MIN_MOTOR_PWM;
}

bool SimpleSub::set_motor_speed(uint16_t motor_index, uint16_t pwm)
{
    //gcs->send_text(MAV_SEVERITY_INFO, "Setting pwm %u on chan %u valid %d", pwm, motor_index, pwm_is_valid(pwm));
    if (motor_index >= NUMBER_MOTORS)
    {
        return false;
    }

    if ((is_armed || pwm == NEUTRAL_MOTOR_PWM) && pwm_is_valid(pwm))
    {
        current_motor_pwms[motor_index] = pwm;
        return true;
    }

    return false;
}

bool SimpleSub::any_motor_is_on()
{
    for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; ++motor_index)
    {
        if (current_motor_pwms[motor_index] != NEUTRAL_MOTOR_PWM)
        {
            return true;
        }
    }

    return false;
}

void SimpleSub::set_speeds_to_stopped()
{
    for (int motor_index = 0; motor_index < NUMBER_MOTORS; ++motor_index)
    {
        current_motor_pwms[motor_index] = NEUTRAL_MOTOR_PWM;
    }
}

void SimpleSub::enable_motor_rc_channels()
{
    //hal.rcout->set_freq(0xFF, 490);
    for (int motor_channel = 0; motor_channel < NUMBER_MOTORS; ++motor_channel)
    {
        rcout->enable_ch(motor_channel + MOTOR_RC_CHANNEL_OFFSET);
    }

    for (int servo_channel = 0; servo_channel < NUMBER_POSITIONAL_SERVOS; ++servo_channel)
    {
        rcout->enable_ch(POSITIONAL_SERVO_RC_CHANNEL_OFFSET + servo_channel);
    }

    rcout->set_freq(0xFF, MOTOR_PWM_FREQUENCY);

    rcout->force_safety_off();
}

void SimpleSub::output_to_motors()
{
    for (int motor_channel = 0; motor_channel < NUMBER_MOTORS; ++motor_channel)
    {
        rcout->write(
            motor_channel + MOTOR_RC_CHANNEL_OFFSET,
            current_motor_pwms[motor_channel]);
    }

    for (int servo_number = 0; servo_number < NUMBER_POSITIONAL_SERVOS; ++servo_number)
    {
        rcout->write(
            servo_number + POSITIONAL_SERVO_RC_CHANNEL_OFFSET,
            current_servo_pwms[servo_number]);
    }
}

bool SimpleSub::handle_command_long_packet(mavlink_command_long_t &command_long_packet)
{
    if (command_long_packet.command == MAV_CMD_COMPONENT_ARM_DISARM)
    {
        bool do_arm = command_long_packet.param1 > 0.0;

        if (do_arm)
        {
#ifdef SIMPLE_SUB_DEBUG
            //gcs->send_text(MAV_SEVERITY_INFO, "Arming sub!");
#endif
            arm();
        }
        else
        {
#ifdef SIMPLE_SUB_DEBUG
            //gcs->send_text(MAV_SEVERITY_INFO, "Disarming sub!");
#endif
            disarm();
        }

        return true;
    }

    if (command_long_packet.command == MAV_CMD_DO_SET_SERVO)
    {
        int servo_number = command_long_packet.param1;
        int servo_pwm = command_long_packet.param2;

        if (servo_number < 0 || servo_number >= NUMBER_POSITIONAL_SERVOS)
        {
            gcs->send_text(MAV_SEVERITY_WARNING, "Invalid servo index: %i", servo_number);
            return false;
        }

        if (servo_pwm >= MIN_SERVO_PWM && servo_pwm <= MAX_SERVO_PWM)
        {
            current_servo_pwms[servo_number] = servo_pwm;
        }
        else
        {
            gcs->send_text(MAV_SEVERITY_WARNING, "Invalid servo pwm given: %i", servo_pwm);
            return false;
        }

        return true;
    }

    return false;
}

bool SimpleSub::handle_rc_override_packet(mavlink_rc_channels_override_t rc_override_packet)
{
    bool success = set_motor_speed(0, rc_override_packet.chan1_raw) &&
                   set_motor_speed(1, rc_override_packet.chan2_raw) &&
                   set_motor_speed(2, rc_override_packet.chan3_raw) &&
                   set_motor_speed(3, rc_override_packet.chan4_raw) &&
                   set_motor_speed(4, rc_override_packet.chan5_raw) &&
                   set_motor_speed(5, rc_override_packet.chan6_raw) &&
                   set_motor_speed(6, rc_override_packet.chan7_raw) &&
                   set_motor_speed(7, rc_override_packet.chan8_raw) &&
                   set_motor_speed(8, rc_override_packet.chan9_raw);

    if (success)
    {
        last_motor_control_message_time = AP_HAL::millis();
    }

    return success;
}

// bool SimpleSub::handle_direct_motor_control_packet(mavlink_direct_motor_control_t direct_motor_control_packet) {
//     bool success = set_motor_speed(0, direct_motor_control_packet.motor1) &&
//         set_motor_speed(1, direct_motor_control_packet.motor2) &&
//         set_motor_speed(2, direct_motor_control_packet.motor3) &&
//         set_motor_speed(3, direct_motor_control_packet.motor4) &&
//         set_motor_speed(4, direct_motor_control_packet.motor5) &&
//         set_motor_speed(5, direct_motor_control_packet.motor6) &&
//         set_motor_speed(6, direct_motor_control_packet.motor7) &&
//         set_motor_speed(7, direct_motor_control_packet.motor8);

//     if (success) {
//         last_motor_control_message_time = AP_HAL::millis();
//     }

//     #ifdef SIMPLE_SUB_DEBUG
//         //if (success) {
//         //    gcs->send_text(MAV_SEVERITY_INFO, "Good %u %u %u %u %u %u %u %u", current_motor_pwms[0], current_motor_pwms[1], current_motor_pwms[2], current_motor_pwms[3], current_motor_pwms[4], current_motor_pwms[5], current_motor_pwms[6], current_motor_pwms[7]);
//         //} else {
//         //    gcs->send_text(MAV_SEVERITY_INFO, "Fail %u %u %u %u %u %u %u %u", current_motor_pwms[0], current_motor_pwms[1], current_motor_pwms[2], current_motor_pwms[3], current_motor_pwms[4], current_motor_pwms[5], current_motor_pwms[6], current_motor_pwms[7]);
//         //}
//     #endif

//     return success;
// }

void SimpleSub::arm()
{
    is_armed = true;
}

void SimpleSub::disarm()
{
    set_speeds_to_stopped();
    output_to_motors();
    is_armed = false;
}

void SimpleSub::stop_if_delay_between_messages_too_long()
{
    uint32_t current_time = AP_HAL::millis();

    if (current_time < last_motor_control_message_time)
    {
        last_motor_control_message_time = AP_HAL::millis();
        gcs->send_text(MAV_SEVERITY_WARNING, "Stopping motors! Clock overflow!");
        set_speeds_to_stopped();
        return;
    }

    uint32_t time_gap = current_time - last_motor_control_message_time;

    if (time_gap > MAX_MOTOR_MESSAGE_RECEIVED_GAP_MILLIS)
    {
        if (any_motor_is_on())
        {
            last_motor_control_message_time = AP_HAL::millis();
            gcs->send_text(MAV_SEVERITY_WARNING, "Stopping motors! Too long since motor message!");
        }
        set_speeds_to_stopped();
    }

    return;
}

bool SimpleSub::get_is_armed()
{
    return is_armed;
}

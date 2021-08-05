#include "SimpleGCS.h"

bool SimpleGCS::update_receive()
{
    while (mavlink_uart->available())
    {
        char received_byte = mavlink_uart->read();

        bool message_parsed = mavlink_parse_char(MAVLINK_COMM_0, received_byte, &received_message, &mavlink_status);

        if (message_parsed)
        {
            return true;
        }

        if (mavlink_status.buffer_overrun || mavlink_status.parse_error)
        {
            clear_mavlink_buffer();
            return false;
        }
    }

    return false;
}

mavlink_message_t SimpleGCS::get_last_parsed_message()
{
    return received_message;
}

void SimpleGCS::clear_mavlink_buffer()
{
    // mavlink_message_t new_message;
    // mavlink_status_t new_status;

    received_message = mavlink_message_t();
    mavlink_status = mavlink_status_t();
}

void SimpleGCS::send_mavlink_message(mavlink_message_t *message_ptr)
{
    uint8_t message_buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t message_buffer_length = mavlink_msg_to_send_buffer(message_buffer, message_ptr);
    mavlink_uart->write(message_buffer, message_buffer_length);
}

void SimpleGCS::send_text(MAV_SEVERITY severity, const char *format_string, ...)
{
    // formats and sends a debug text
    va_list format_args;
    va_start(format_args, format_string);

    mavlink_message_t message;

    char trimmed_string[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
    int number_bytes_written = vsnprintf(trimmed_string, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, format_string, format_args);
    va_end(format_args);

    if (number_bytes_written < 0 || number_bytes_written > MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN)
    {
    }

    for (int i = number_bytes_written; i < MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; ++i)
    {
        trimmed_string[i] = ' ';
    }

    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &message, severity, trimmed_string, 0, 0);
    send_mavlink_message(&message);
}

void SimpleGCS::send_imu_data(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t temperature)
{
    mavlink_message_t message;

    mavlink_msg_scaled_imu_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        AP_HAL::millis(),
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
        0,
        0,
        0,
        temperature);

    send_mavlink_message(&message);
}

void SimpleGCS::send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    mavlink_message_t message;
    mavlink_msg_attitude_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        AP_HAL::millis(),
        roll,
        pitch,
        yaw,
        rollspeed,
        pitchspeed,
        yawspeed);

    send_mavlink_message(&message);
}

void SimpleGCS::send_heartbeat(bool sub_is_armed)
{
    mavlink_message_t message;

    MAV_MODE mode = MAV_MODE_MANUAL_DISARMED;
    if (sub_is_armed)
    {
        mode = MAV_MODE_MANUAL_ARMED;
    }

    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        MAV_TYPE_SUBMARINE,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        mode,
        0,
        MAV_STATE_ACTIVE);

    send_mavlink_message(&message);
}

void SimpleGCS::send_command_ack_packet(uint16_t command_id, MAV_RESULT result)
{
    mavlink_message_t message;

    mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &message, command_id, result);
    send_mavlink_message(&message);
}

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "include/mavlink/v2.0/ardupilotmega/version.h"
#include "include/mavlink/v2.0/mavlink_types.h"
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

#include "Config.h"

class SimpleGCS {
private:
    mavlink_message_t received_message;
    mavlink_status_t mavlink_status;
    AP_HAL::UARTDriver* mavlink_uart;

    void send_mavlink_message(mavlink_message_t* message_ptr);

public:
    SimpleGCS(AP_HAL::UARTDriver* _mavlink_uart) : mavlink_uart(_mavlink_uart) {};

    bool update_receive();
    mavlink_message_t get_last_parsed_message();
    void clear_mavlink_buffer();

    void send_text(MAV_SEVERITY severity, const char* format_string, ...);
    void send_imu_data(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t temperature);
    void send_heartbeat(bool sub_is_armed);

    void send_command_ack_packet(uint16_t command_id, MAV_RESULT result);

    void send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
}; // class SimpleGCS

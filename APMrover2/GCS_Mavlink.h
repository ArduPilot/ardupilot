#pragma once

#include <GCS_MAVLink/GCS.h>

// default sensors are present and healthy: gyro, accelerometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

class GCS_MAVLINK_Rover : public GCS_MAVLINK
{
public:

    void data_stream_send(void) override;

protected:

    uint32_t telem_delay() const override;
    bool accept_packet(const mavlink_status_t &status, mavlink_message_t &msg) override;

    Compass *get_compass() const override;
    AP_Mission *get_mission() override;
    AP_Rally *get_rally() const override { return nullptr; };
    AP_Camera *get_camera() const override;
    AP_ServoRelayEvents *get_servorelayevents() const override;
    AP_GPS *get_gps() const override;

    uint8_t sysid_my_gcs() const override;

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;
};

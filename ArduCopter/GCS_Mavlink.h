#pragma once

#include <GCS_MAVLink/GCS.h>

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_BATTERY)

class GCS_MAVLINK_Copter : public GCS_MAVLINK
{

public:

protected:

    uint32_t telem_delay() const override;

    bool accept_packet(const mavlink_status_t &status, mavlink_message_t &msg) override;

    AP_Mission *get_mission() override;
    AP_Rally *get_rally() const override;
    Compass *get_compass() const override;
    AP_Camera *get_camera() const override;
    MAV_RESULT handle_flight_termination(const mavlink_command_long_t &packet) override;
    AP_AdvancedFailsafe *get_advanced_failsafe() const override;
    AP_VisualOdom *get_visual_odom() const override;

    uint8_t sysid_my_gcs() const override;

    bool set_mode(uint8_t mode) override;

    bool params_ready() const override;
    void send_banner() override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;

    void send_position_target_global_int() override;

    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

private:

    void handleMessage(mavlink_message_t * msg) override;
    void handle_command_ack(const mavlink_message_t* msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status,
                        mavlink_message_t &msg) override;

    MAV_TYPE frame_type() const override;
    MAV_MODE base_mode() const override;
    uint32_t custom_mode() const override;
    MAV_STATE system_status() const override;

    int16_t vfr_hud_throttle() const override;

};

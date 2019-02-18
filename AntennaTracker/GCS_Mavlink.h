#pragma once

#include <GCS_MAVLink/GCS.h>

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

class GCS_MAVLINK_Tracker : public GCS_MAVLINK
{

public:

protected:

    // telem_delay is not used by Tracker but is pure virtual, thus
    // this implementaiton.  it probably *should* be used by Tracker,
    // as currently Tracker may brick XBees
    uint32_t telem_delay() const override { return 0; }

    uint8_t sysid_my_gcs() const override;

    bool set_mode(uint8_t mode) override;

    MAV_RESULT _handle_command_preflight_calibration_baro() override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

    int32_t global_position_int_relative_alt() const override {
        return 0; // what if we have been picked up and carried somewhere?
    }

    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED { return false; }
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED { return false; }

    void send_nav_controller_output() const override;

private:

    void packetReceived(const mavlink_status_t &status, mavlink_message_t &msg) override;
    void mavlink_check_target(const mavlink_message_t &msg);
    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;

    MAV_TYPE frame_type() const override;
    MAV_MODE base_mode() const override;
    uint32_t custom_mode() const override;
    MAV_STATE system_status() const override;
    void get_sensor_status_flags(uint32_t &present, uint32_t &enabled, uint32_t &health);

};

#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Tracker : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_int_t &packet) override;
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    int32_t global_position_int_relative_alt() const override {
        return 0; // what if we have been picked up and carried somewhere?
    }

    void send_attitude_target() override;
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

    bool try_send_message(enum ap_message id) override;

private:

    void packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg) override;
    void mavlink_check_target(const mavlink_message_t &msg);
    void handle_message(const mavlink_message_t &msg) override;
    void handle_message_mission_write_partial_list(const mavlink_message_t &msg);
    void handle_message_mission_item(const mavlink_message_t &msg);
    void handle_message_manual_control(const mavlink_message_t &msg);
    void handle_message_global_position_int(const mavlink_message_t &msg);
    void handle_message_scaled_pressure(const mavlink_message_t &msg);
    void handle_set_attitude_target(const mavlink_message_t &msg);

    void send_global_position_int() override;

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    bool waypoint_receiving;
};

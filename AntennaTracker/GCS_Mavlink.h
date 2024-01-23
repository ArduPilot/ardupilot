#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Tracker : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

    uint8_t sysid_my_gcs() const override;

protected:

    // telem_delay is not used by Tracker but is pure virtual, thus
    // this implementation.  it probably *should* be used by Tracker,
    // as currently Tracker may brick XBees
    uint32_t telem_delay() const override { return 0; }


    MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_int_t &packet) override;
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    int32_t global_position_int_relative_alt() const override {
        return 0; // what if we have been picked up and carried somewhere?
    }

    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

private:

    void packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg) override;
    void mavlink_check_target(const mavlink_message_t &msg);
    void handle_message(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_set_attitude_target(const mavlink_message_t &msg);

    void send_global_position_int() override;

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    bool waypoint_receiving;
};

#pragma once

#include <GCS_MAVLink/GCS.h>

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

    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    uint64_t capabilities() const override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

private:

    void packetReceived(const mavlink_status_t &status, mavlink_message_t &msg) override;
    void mavlink_check_target(const mavlink_message_t &msg);
    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void send_global_position_int() override;

    MAV_MODE base_mode() const override;
    MAV_STATE system_status() const override;

};

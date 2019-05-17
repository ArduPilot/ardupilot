#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Rover : public GCS_MAVLINK
{
public:

protected:

    uint32_t telem_delay() const override;

    AP_AdvancedFailsafe *get_advanced_failsafe() const override;

    uint8_t sysid_my_gcs() const override;
    bool sysid_enforce() const override;

    bool set_mode(uint8_t mode) override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

    void send_position_target_global_int() override;

    virtual bool in_hil_mode() const override;

    bool persist_streamrates() const override { return true; }

    bool set_home_to_current_location(bool lock) override;
    bool set_home(const Location& loc, bool lock) override;
    uint64_t capabilities() const override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void handle_rc_channels_override(const mavlink_message_t *msg) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status, mavlink_message_t &msg) override;

    MAV_MODE base_mode() const override;
    MAV_STATE system_status() const override;

    int16_t vfr_hud_throttle() const override;

    void send_rangefinder() const override;

};

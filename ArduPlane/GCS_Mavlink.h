#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

class GCS_MAVLINK_Plane : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    uint32_t telem_delay() const override;

    void handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg) override;

    uint8_t sysid_my_gcs() const override;
    bool sysid_enforce() const override;

    MAV_RESULT handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

    void send_position_target_global_int() override;

    virtual bool in_hil_mode() const override;

    void send_aoa_ssa();
    void send_attitude() const override;
    void send_simstate() const override;
    void send_wind() const;

    bool persist_streamrates() const override { return true; }

    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    uint64_t capabilities() const override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

private:

    void send_pid_info(const AP_Logger::PID_Info *pid_info, const uint8_t axis, const float achieved);

    void handleMessage(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void handle_rc_channels_override(const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet);


    bool try_send_message(enum ap_message id) override;
    void packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg) override;

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    uint8_t radio_in_rssi() const;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_climbrate() const override;

};

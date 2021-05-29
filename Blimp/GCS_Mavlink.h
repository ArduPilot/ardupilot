#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Blimp : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    uint32_t telem_delay() const override;

    MAV_RESULT handle_flight_termination(const mavlink_command_long_t &packet) override;

    uint8_t sysid_my_gcs() const override;
    bool sysid_enforce() const override;

    bool params_ready() const override;
    void send_banner() override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;

    void send_position_target_global_int() override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_mount(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);


    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    void send_nav_controller_output() const override; //TODO Apparently can't remove this or the build fails.
    uint64_t capabilities() const override;

    virtual MAV_VTOL_STATE vtol_state() const override
    {
        return MAV_VTOL_STATE_MC;
    };
    virtual MAV_LANDED_STATE landed_state() const override;

private:

    void handleMessage(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;

    void send_wind() const;
};

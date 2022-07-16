#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Copter : public GCS_MAVLINK
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

    void send_attitude_target() override;
    void send_position_target_global_int() override;
    void send_position_target_local_ned() override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet) override;
#if HAL_MOUNT_ENABLED
    MAV_RESULT handle_command_mount(const mavlink_command_long_t &packet) override;
#endif
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_pause_continue(const mavlink_command_int_t &packet);

#if HAL_MOUNT_ENABLED
    void handle_mount_message(const mavlink_message_t &msg) override;
#endif

    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    void send_nav_controller_output() const override;
    uint64_t capabilities() const override;

    virtual MAV_VTOL_STATE vtol_state() const override { return MAV_VTOL_STATE_MC; };
    virtual MAV_LANDED_STATE landed_state() const override;

private:

    void handleMessage(const mavlink_message_t &msg) override;
    void handle_command_ack(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;

    void send_winch_status() const override;

    void send_wind() const;

#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
    uint8_t high_latency_wind_speed() const override;
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};

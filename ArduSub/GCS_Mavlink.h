#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    uint32_t telem_delay() const override {
        return 0;
    };

    MAV_RESULT handle_flight_termination(const mavlink_command_long_t &packet) override;

    uint8_t sysid_my_gcs() const override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

    // override sending of scaled_pressure3 to send on-board temperature:
    void send_scaled_pressure3() override;

    int32_t global_position_int_alt() const override;
    int32_t global_position_int_relative_alt() const override;

    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;

    void send_banner() override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

    uint64_t capabilities() const override;

private:

    void handleMessage(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    bool send_info(void);

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    int16_t vfr_hud_throttle() const override;

#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};

#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);

    // override sending of scaled_pressure3 to send on-board temperature:
    void send_scaled_pressure3() override;

    int32_t global_position_int_alt() const override;
    int32_t global_position_int_relative_alt() const override;

    void send_banner() override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

    uint64_t capabilities() const override;

    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

private:

    void handle_message(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    bool send_info(void);

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_NAV_LOITER_UNLIM(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_NAV_LAND(const mavlink_command_int_t &packet);

#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};

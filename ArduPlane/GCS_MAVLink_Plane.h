#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Airspeed/AP_Airspeed_config.h>
#include "quadplane.h"
#include "defines.h"

class GCS_MAVLINK_Plane : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

#if HAL_LOGGING_ENABLED
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
    void handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg) override;
#endif

    MAV_RESULT handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_do_set_mission_current(const mavlink_command_int_t &packet) override;

    void send_position_target_global_int() override;

    void send_aoa_ssa();
    void send_attitude() const override;
    void send_attitude_target() override;
    void send_wind() const;

    bool persist_streamrates() const override { return true; }

    uint64_t capabilities() const override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;
    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

private:

    void send_pid_info(const struct AP_PIDInfo *pid_info, const uint8_t axis, const float achieved);

    void handle_message(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(Location &location) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_int_DO_CHANGE_ALTITUDE(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_AUTOTUNE_ENABLE(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_DO_VTOL_TRANSITION(const mavlink_command_int_t &packet);

    void handle_set_position_target_global_int(const mavlink_message_t &msg);
    void handle_set_position_target_local_ned(const mavlink_message_t &msg);
    void handle_set_attitude_target(const mavlink_message_t &msg);

#if HAL_QUADPLANE_ENABLED
#if AP_MAVLINK_COMMAND_LONG_ENABLED
    void convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out);
    void convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT) override;
#endif
    MAV_RESULT handle_command_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet);
#endif

    bool try_send_message(enum ap_message id) override;
    void packetReceived(const mavlink_status_t &status, const mavlink_message_t &msg) override;

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_climbrate() const override;
    
#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
    uint8_t high_latency_wind_speed() const override;
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED

#if AP_AIRSPEED_HYGROMETER_ENABLE
    void send_hygrometer();
    uint8_t last_hygrometer_send_idx;
#endif

    MAV_VTOL_STATE vtol_state() const override;
    MAV_LANDED_STATE landed_state() const override;

};

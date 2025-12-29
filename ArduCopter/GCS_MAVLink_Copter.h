#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Winch/AP_Winch_config.h>
#include "defines.h"

#ifndef AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
#define AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED 1
#endif

class GCS_MAVLINK_Copter : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    bool params_ready() const override;
    void send_banner() override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    void send_attitude_target() override;
    void send_position_target_global_int() override;
    void send_position_target_local_ned() override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
#if HAL_MOUNT_ENABLED
    MAV_RESULT handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
#endif
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_pause_continue(const mavlink_command_int_t &packet);

    void handle_message_set_attitude_target(const mavlink_message_t &msg);
    void handle_message_set_position_target_global_int(const mavlink_message_t &msg);
    void handle_message_set_position_target_local_ned(const mavlink_message_t &msg);

    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

    void send_nav_controller_output() const override;
    uint64_t capabilities() const override;

    virtual MAV_VTOL_STATE vtol_state() const override { return MAV_VTOL_STATE_MC; };
    virtual MAV_LANDED_STATE landed_state() const override;

    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;

#if HAL_LOGGING_ENABLED
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

private:

    // sanity check velocity or acceleration vector components are numbers
    // (e.g. not NaN) and below 1000. vec argument units are in meters/second or
    // metres/second/second
    bool sane_vel_or_acc_vector(const Vector3f &vec) const;

    MISSION_STATE mission_state(const class AP_Mission &mission) const override;

    void handle_message(const mavlink_message_t &msg) override;
    void handle_command_ack(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;

#if AP_WINCH_ENABLED
    void send_winch_status() const override;
#endif

    void send_wind() const;

#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
    uint8_t high_latency_wind_speed() const override;
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED


    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet);

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_FLY_CLICK(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_FLY_HOLD(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(const mavlink_command_int_t &packet);
#endif

#if AP_MAVLINK_COMMAND_LONG_ENABLED
    bool mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const override;
#endif

    MAV_RESULT handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet);

#if AP_WINCH_ENABLED
    MAV_RESULT handle_MAV_CMD_DO_WINCH(const mavlink_command_int_t &packet);
#endif

};

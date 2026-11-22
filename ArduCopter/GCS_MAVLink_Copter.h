#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Winch/AP_Winch_config.h>
#include "defines.h"
#include "config.h"

#ifndef AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
#define AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED 1
#endif

class GCS_MAVLINK_Copter : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    // terminates the flight by disarming unless the advanced failsafe handles it
    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    // returns true once parameters can be sent to the GCS
    bool params_ready() const override;

    // sends the banner text with the frame and motor type added
    void send_banner() override;

    // runs compassmot calibration, passing all other preflight calibrations on
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    // sends ATTITUDE_TARGET with the attitude controller target attitude, body rates and throttle
    void send_attitude_target() override;

    // returns the navigation setpoint commanded by the current flight mode
    bool get_nav_target(NavTarget &target) const override;

    // returns the destination of the current flight mode
    bool get_target_location(Location &loc) const override;

    // sends POSITION_TARGET_LOCAL_NED with the Guided target, only while in a Guided mode
    void send_position_target_local_ned() override;

    // points the vehicle at the region of interest
    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;

    // rejects a reboot while ESC calibration on the next boot is set to auto
    MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

#if HAL_MOUNT_ENABLED
    // yaws the vehicle for DO_MOUNT_CONTROL when the mount has no pan control
    MAV_RESULT handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
#endif

    // handles Copter specific COMMAND_INT commands, passing the rest on
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    // flies to the requested location in Guided, switching to Guided if the change mode flag is set
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);

    // pauses or resumes the current flight mode
    MAV_RESULT handle_command_pause_continue(const mavlink_command_int_t &packet);

    // sets the Guided attitude, body rate and climb rate or thrust target from SET_ATTITUDE_TARGET
    void handle_message_set_attitude_target(const mavlink_message_t &msg);

    // sets the Guided position, velocity or acceleration target from SET_POSITION_TARGET_GLOBAL_INT
    void handle_message_set_position_target_global_int(const mavlink_message_t &msg);

    // sets the Guided position, velocity or acceleration target from SET_POSITION_TARGET_LOCAL_NED
    void handle_message_set_position_target_local_ned(const mavlink_message_t &msg);

    // passes LANDING_TARGET to precision landing
    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

    // sends NAV_CONTROLLER_OUTPUT with the target attitude and the current mode's bearing, distance and errors
    void send_nav_controller_output() const override;

    // returns the MAVLink protocol capabilities Copter supports
    uint64_t capabilities() const override;

    // returns multicopter as the VTOL state
    virtual MAV_VTOL_STATE vtol_state() const override { return MAV_VTOL_STATE_MC; };

    // returns whether the vehicle is on the ground, taking off, in the air or landing
    virtual MAV_LANDED_STATE landed_state() const override;

    // overrides the RC roll, pitch, throttle and yaw inputs from MANUAL_CONTROL
    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;

#if HAL_LOGGING_ENABLED
    // returns the LOG_BITMASK bit that enables radio logging
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

#if MODE_AUTO_ENABLED
    // returns paused while Auto is paused, otherwise the mission's own state
    MISSION_STATE mission_state(const class AP_Mission &mission) const override;
#endif  // MODE_AUTO_ENABLED

    // handles Copter specific messages, passing the rest on
    void handle_message(const mavlink_message_t &msg) override;

    // counts COMMAND_ACKs so compassmot can stop when the GCS acknowledges, then handles them normally
    void handle_command_ack(const mavlink_message_t &msg) override;

    // sets the Guided destination or yaw from a guided request sent as a mission item
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;

    // try to send a message, return false if it wasn't sent
    bool try_send_message(enum ap_message id) override;

    // passes received packets to MAVLink ADSB avoidance when enabled, before normal handling
    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    // returns the HEARTBEAT base mode flags
    uint8_t base_mode() const override;

    // returns critical on any failsafe, standby when landed, boot until initialised and active otherwise
    MAV_STATE vehicle_system_status() const override;

    // returns the VFR_HUD airspeed from an airspeed sensor, the EKF wind estimate or ground speed, in that order
    float vfr_hud_airspeed() const override;

    // returns the VFR_HUD throttle output as a percentage
    int16_t vfr_hud_throttle() const override;

    // returns the VFR_HUD altitude, relative to home if the compatibility option is set
    float vfr_hud_alt() const override;

    // sends PID_TUNING for the axes selected by GCS_PID_MASK
    void send_pid_tuning() override;

#if AP_WINCH_ENABLED
    // send winch status message
    void send_winch_status() const override;
#endif

    // sends WIND when the EKF is estimating wind
    void send_wind() const;

#if HAL_HIGH_LATENCY2_ENABLED
    // returns the HIGH_LATENCY2 target altitude in metres
    int16_t high_latency_target_altitude() const override;

    // returns the HIGH_LATENCY2 bearing to the waypoint in units of 2 degrees
    uint8_t high_latency_tgt_heading() const override;

    // returns the HIGH_LATENCY2 distance to the waypoint in decametres
    uint16_t high_latency_tgt_dist_dam() const override;

    // returns the HIGH_LATENCY2 target speed in units of 0.2 m/s
    uint8_t high_latency_tgt_airspeed() const override;

    // returns the HIGH_LATENCY2 wind speed in units of 0.2 m/s
    uint8_t high_latency_wind_speed() const override;

    // returns the HIGH_LATENCY2 wind direction in units of 2 degrees
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED

    // sets a fixed yaw target
    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);

    // changes the horizontal, climb or descent speed of the current mode
    MAV_RESULT handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);

    // starts a motor test
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);

    // enables, disables or releases the parachute
    MAV_RESULT handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet);

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
    // Solo Fly button click, switches to Loiter or AltHold if Loiter is not available
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_FLY_CLICK(const mavlink_command_int_t &packet);

    // Solo Fly button hold, arms when disarmed, takes off when landed and lands when flying
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_FLY_HOLD(const mavlink_command_int_t &packet);

    // Solo Pause button click, disarms when landed, otherwise brakes unless a Guided shot is running
    MAV_RESULT handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(const mavlink_command_int_t &packet);
#endif

#if AP_MAVLINK_COMMAND_LONG_ENABLED
    // returns true for commands that are only accepted as COMMAND_INT
    bool command_int_only(MAV_CMD command) const override;
#endif

    // switches to Auto and starts or resumes the mission
    MAV_RESULT handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet);

    // takes off to the requested altitude above home
    MAV_RESULT handle_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet);

#if AP_WINCH_ENABLED
    // relaxes the winch or sets its length or rate
    MAV_RESULT handle_MAV_CMD_DO_WINCH(const mavlink_command_int_t &packet);
#endif

};

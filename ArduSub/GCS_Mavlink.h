#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

protected:

    uint32_t telem_delay() const override {
        return 0;
    };

    MAV_RESULT handle_flight_termination(const mavlink_command_long_t &packet) override;

    uint8_t sysid_my_gcs() const override;

    bool set_mode(uint8_t mode) override;
    bool should_zero_rc_outputs_on_reboot() const override { return true; }

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT _handle_command_preflight_calibration_baro() override;
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet) override;
    MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet) override;

    // override sending of scaled_pressure3 to send on-board temperature:
    void send_scaled_pressure3() override;

    int32_t global_position_int_alt() const override;
    int32_t global_position_int_relative_alt() const override;

    bool vehicle_initialised() const override;

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void handle_rc_channels_override(const mavlink_message_t *msg) override;
    bool try_send_message(enum ap_message id) override;

    bool send_info(void);

    MAV_TYPE frame_type() const override;
    MAV_MODE base_mode() const override;
    uint32_t custom_mode() const override;
    MAV_STATE system_status() const override;
    void get_sensor_status_flags(uint32_t &present, uint32_t &enabled, uint32_t &health);

    int16_t vfr_hud_throttle() const override;

};

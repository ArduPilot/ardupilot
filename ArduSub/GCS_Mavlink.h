#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

    void data_stream_send(void) override;

protected:

    uint32_t telem_delay() const override {
        return 0;
    };

    Compass *get_compass() const override;
    AP_Mission *get_mission() override;
    AP_Rally *get_rally() const override;
    AP_Camera *get_camera() const override;
    AP_ServoRelayEvents *get_servorelayevents() const override;
    const AP_FWVersion &get_fwver() const override;
    void set_ekf_origin(const Location& loc) override;

    MAV_RESULT handle_flight_termination(const mavlink_command_long_t &packet) override;

    uint8_t sysid_my_gcs() const override;

    bool set_mode(uint8_t mode) override;

    MAV_RESULT _handle_command_preflight_calibration_baro() override;
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet) override;

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    MAV_TYPE frame_type() const override;
    MAV_MODE base_mode() const override;
    uint32_t custom_mode() const override;
    MAV_STATE system_status() const override;
};

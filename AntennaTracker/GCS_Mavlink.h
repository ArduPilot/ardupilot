#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Tracker : public GCS_MAVLINK
{

public:

    void data_stream_send(void) override;

protected:

    // telem_delay is not used by Tracker but is pure virtual, thus
    // this implementaiton.  it probably *should* be used by Tracker,
    // as currently Tracker may brick XBees
    uint32_t telem_delay() const override { return 0; }

    Compass *get_compass() const override;
    AP_Mission *get_mission() override { return nullptr; };
    AP_Rally *get_rally() const override { return nullptr; };
    AP_Camera *get_camera() const override { return nullptr; };
    AP_ServoRelayEvents *get_servorelayevents() const override { return nullptr; }
    AP_GPS *get_gps() const override;
    const AP_FWVersion &get_fwver() const override;
    void set_ekf_origin(const Location& loc) override;

    uint8_t sysid_my_gcs() const override;

    bool set_mode(uint8_t mode) override;

private:

    void handleMessage(mavlink_message_t * msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

};

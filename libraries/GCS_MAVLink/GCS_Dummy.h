#include "GCS.h"

const AP_FWVersion AP_FWVersion::fwver
{
    major: 3,
    minor: 1,
    patch: 4,
    fw_type: FIRMWARE_VERSION_TYPE_DEV,
    fw_string: "Dummy GCS"
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] {};

/*
 *  GCS backend used for many examples and tools
 */
class GCS_MAVLINK_Dummy : public GCS_MAVLINK
{
    uint32_t telem_delay() const override { return 0; }
    void handleMessage(mavlink_message_t * msg) override {}
    bool try_send_message(enum ap_message id) { return true; }
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override { return true; }
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override {}

protected:

    uint8_t sysid_my_gcs() const override { return 1; }
    bool set_mode(uint8_t mode) override { return false; };

    // dummy information:
    MAV_TYPE frame_type() const override { return MAV_TYPE_FIXED_WING; }
    MAV_MODE base_mode() const override { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    uint32_t custom_mode() const override { return 3; } // magic number
    MAV_STATE system_status() const override { return MAV_STATE_CALIBRATING; }

};

/*
 * a GCS singleton used for many example sketches and tools
 */

extern const AP_HAL::HAL& hal;

class GCS_Dummy : public GCS
{
    GCS_MAVLINK_Dummy dummy_backend;
    uint8_t num_gcs() const override { return 1; }
    GCS_MAVLINK_Dummy &chan(const uint8_t ofs) override { return dummy_backend; }
    const GCS_MAVLINK_Dummy &chan(const uint8_t ofs) const override { return dummy_backend; };

    void send_statustext(MAV_SEVERITY severity, uint8_t dest_bitmask, const char *text) { hal.console->printf("TOGCS: %s\n", text); }
};

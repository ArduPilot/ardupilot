#include "GCS.h"
#include <AP_Common/AP_FWVersion.h>

const AP_FWVersion AP_FWVersion::fwver
{
    major: 3,
    minor: 1,
    patch: 4,
    fw_type: FIRMWARE_VERSION_TYPE_DEV,
    fw_string: "Dummy GCS",
    fw_hash_str: "",
    middleware_name: "",
    middleware_hash_str: "",
    os_name: "",
    os_hash_str: "",
    os_sw_version: 0
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] {};

/*
 *  GCS backend used for many examples and tools
 */
class GCS_MAVLINK_Dummy : public GCS_MAVLINK
{
public:

    using GCS_MAVLINK::GCS_MAVLINK;

private:

    uint32_t telem_delay() const override { return 0; }
    void handleMessage(const mavlink_message_t &msg) override {}
    bool try_send_message(enum ap_message id) override { return true; }
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override { return true; }
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override {}

protected:

    uint8_t sysid_my_gcs() const override { return 1; }

    // dummy information:
    MAV_MODE base_mode() const override { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    MAV_STATE vehicle_system_status() const override { return MAV_STATE_CALIBRATING; }

    bool set_home_to_current_location(bool lock) override { return false; }
    bool set_home(const Location& loc, bool lock) override { return false; }

    void send_nav_controller_output() const override {};
    void send_pid_tuning() override {};
};

/*
 * a GCS singleton used for many example sketches and tools
 */

extern const AP_HAL::HAL& hal;

class GCS_Dummy : public GCS
{
public:

    using GCS::GCS;

protected:

    GCS_MAVLINK_Dummy *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                               AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Dummy(params, uart);
    }

private:
    GCS_MAVLINK_Dummy *chan(const uint8_t ofs) override {
        if (ofs > _num_gcs) {
            AP::internalerror().error(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Dummy *)_chan[ofs];
    };
    const GCS_MAVLINK_Dummy *chan(const uint8_t ofs) const override {
        if (ofs > _num_gcs) {
            AP::internalerror().error(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Dummy *)_chan[ofs];
    };

    void send_statustext(MAV_SEVERITY severity, uint8_t dest_bitmask, const char *text) override { hal.console->printf("TOGCS: %s\n", text); }

    MAV_TYPE frame_type() const override { return MAV_TYPE_FIXED_WING; }
    uint32_t custom_mode() const override { return 3; } // magic number
};

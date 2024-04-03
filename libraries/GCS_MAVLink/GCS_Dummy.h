#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"
#include <AP_Common/AP_FWVersion.h>

#define THISFIRMWARE "GCSDummy V3.1.4-dev"

#define FW_MAJOR 3
#define FW_MINOR 1
#define FW_PATCH 4
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

/*
 *  GCS backend used for many examples and tools
 */
class GCS_MAVLINK_Dummy : public GCS_MAVLINK
{
public:

    using GCS_MAVLINK::GCS_MAVLINK;

private:

    uint32_t telem_delay() const override { return 0; }
    bool try_send_message(enum ap_message id) override { return true; }
    uint8_t sysid_my_gcs() const override { return 1; }

protected:

    // dummy information:
    MAV_MODE base_mode() const override { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    MAV_STATE vehicle_system_status() const override { return MAV_STATE_CALIBRATING; }

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

    uint8_t sysid_this_mav() const override { return 1; }

    GCS_MAVLINK_Dummy *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                               AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Dummy(params, uart);
    }

private:
    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Dummy);

    void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t dest_bitmask) override;

    MAV_TYPE frame_type() const override { return MAV_TYPE_FIXED_WING; }
    uint32_t custom_mode() const override { return 3; } // magic number
};

#endif  // HAL_GCS_ENABLED

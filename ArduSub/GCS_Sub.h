#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Sub : public GCS
{
    friend class Sub; // for access to _chan in parameter declarations

public:

    // return the number of valid GCS objects
    uint8_t num_gcs() const override { return ARRAY_SIZE(_chan); };

    // return GCS link at offset ofs
    GCS_MAVLINK_Sub &chan(uint8_t ofs) override {
        if (ofs >= num_gcs()) {
            AP::internalerror().error(AP_InternalError::error_t::gcs_offset);
            ofs = 0;
        }
        return _chan[ofs];
    };
    const GCS_MAVLINK_Sub &chan(uint8_t ofs) const override {
        if (ofs >= num_gcs()) {
            AP::internalerror().error(AP_InternalError::error_t::gcs_offset);
            ofs = 0;
        }
        return _chan[ofs];
    };

    void update_vehicle_sensor_status_flags() override;

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    bool vehicle_initialised() const override;

protected:

    // minimum amount of time (in microseconds) that must remain in
    // the main scheduler loop before we are allowed to send any
    // mavlink messages.  We want to prioritise the main flight
    // control loop over communications
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;
    }

private:

    GCS_MAVLINK_Sub _chan[MAVLINK_COMM_NUM_BUFFERS];

};

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Tracker : public GCS
{
    friend class Tracker; // for access to _chan in parameter declarations
    friend class GCS_MAVLINK_Tracker;

public:

    // return GCS link at offset ofs
    GCS_MAVLINK_Tracker *chan(const uint8_t ofs) override {
        if (ofs > _num_gcs) {
            INTERNAL_ERROR(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Tracker*)_chan[ofs];
    }
    const GCS_MAVLINK_Tracker *chan(const uint8_t ofs) const override {
        if (ofs > _num_gcs) {
            INTERNAL_ERROR(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Tracker*)_chan[ofs];
    }

    void update_vehicle_sensor_status_flags() override;

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

protected:

    uint8_t sysid_this_mav() const override;

    GCS_MAVLINK_Tracker *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                                 AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Tracker(params, uart);
    }

private:

    void request_datastream_position(uint8_t sysid, uint8_t compid);
    void request_datastream_airpressure(uint8_t sysid, uint8_t compid);

};

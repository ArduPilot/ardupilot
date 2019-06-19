#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Rover : public GCS
{
    friend class Rover; // for access to _chan in parameter declarations

public:

    // return GCS link at offset ofs
    GCS_MAVLINK_Rover *chan(const uint8_t ofs) override {
        if (ofs > _num_gcs) {
            AP::internalerror().error(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Rover*)_chan[ofs];
    }
    // return GCS link at offset ofs
    const GCS_MAVLINK_Rover *chan(const uint8_t ofs) const override {
        if (ofs > _num_gcs) {
            AP::internalerror().error(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Rover*)_chan[ofs];
    }

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    bool vehicle_initialised() const override;

    void update_vehicle_sensor_status_flags(void) override;

    bool simple_input_active() const override;
    bool supersimple_input_active() const override;

protected:

    GCS_MAVLINK_Rover *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                               AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Rover(params, uart);
    }

};

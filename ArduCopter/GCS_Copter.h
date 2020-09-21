#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Copter : public GCS
{
    friend class Copter; // for access to _chan in parameter declarations

public:

    // return GCS link at offset ofs
    GCS_MAVLINK_Copter *chan(const uint8_t ofs) override {
        if (ofs > _num_gcs) {
            INTERNAL_ERROR(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Copter *)_chan[ofs];
    }
    const GCS_MAVLINK_Copter *chan(const uint8_t ofs) const override {
        if (ofs > _num_gcs) {
            INTERNAL_ERROR(AP_InternalError::error_t::gcs_offset);
            return nullptr;
        }
        return (GCS_MAVLINK_Copter *)_chan[ofs];
    }

    void update_vehicle_sensor_status_flags(void) override;

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    const char* frame_string() const override;

    bool vehicle_initialised() const override;

    bool simple_input_active() const override;
    bool supersimple_input_active() const override;

protected:

    uint8_t sysid_this_mav() const override;

    // minimum amount of time (in microseconds) that must remain in
    // the main scheduler loop before we are allowed to send any
    // mavlink messages.  We want to prioritise the main flight
    // control loop over communications
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;
    }

    GCS_MAVLINK_Copter *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                                AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Copter(params, uart);
    }

};

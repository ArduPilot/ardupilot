#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Copter : public GCS
{
    friend class Copter; // for access to _chan in parameter declarations

public:

    // return the number of valid GCS objects
    uint8_t num_gcs() const override { return ARRAY_SIZE(_chan); };

    // return GCS link at offset ofs
    GCS_MAVLINK_Copter &chan(const uint8_t ofs) override {
        return _chan[ofs];
    };
    const GCS_MAVLINK_Copter &chan(const uint8_t ofs) const override {
        return _chan[ofs];
    };

    void update_vehicle_sensor_status_flags(void) override;

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    const char* frame_string() const override;

    bool vehicle_initialised() const override;

    bool simple_input_active() const override;
    bool supersimple_input_active() const override;

private:

    GCS_MAVLINK_Copter _chan[MAVLINK_COMM_NUM_BUFFERS];

};

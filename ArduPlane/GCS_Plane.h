#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Plane : public GCS
{
    friend class Plane;  // for access to _chan in parameter declarations

public:

    // return the number of valid GCS objects
    uint8_t num_gcs() const override { return ARRAY_SIZE(_chan); };

    // return GCS link at offset ofs
    GCS_MAVLINK_Plane &chan(const uint8_t ofs) override {
        return _chan[ofs];
    };
    const GCS_MAVLINK_Plane &chan(const uint8_t ofs) const override {
        return _chan[ofs];
    };

protected:

    void update_vehicle_sensor_status_flags(void) override;
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

private:

    GCS_MAVLINK_Plane _chan[MAVLINK_COMM_NUM_BUFFERS];

};

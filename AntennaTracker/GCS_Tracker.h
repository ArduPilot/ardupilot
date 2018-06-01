#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Tracker : public GCS
{
    friend class Tracker; // for access to _chan in parameter declarations
    friend class GCS_MAVLINK_Tracker;

public:

    // return the number of valid GCS objects
    uint8_t num_gcs() const override { return ARRAY_SIZE(_chan); };

    // return GCS link at offset ofs
    GCS_MAVLINK_Tracker &chan(const uint8_t ofs) override { return _chan[ofs]; };
    const GCS_MAVLINK_Tracker &chan(const uint8_t ofs) const override { return _chan[ofs]; };

private:

    void request_datastream_position(uint8_t sysid, uint8_t compid);
    void request_datastream_airpressure(uint8_t sysid, uint8_t compid);

    GCS_MAVLINK_Tracker _chan[MAVLINK_COMM_NUM_BUFFERS];

};

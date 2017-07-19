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

private:

    GCS_MAVLINK_Copter _chan[MAVLINK_COMM_NUM_BUFFERS];

    bool cli_enabled() const override;
    AP_HAL::BetterStream* cliSerial() override;

};

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"
#include "config.h" // for CLI_ENABLED

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

    void send_airspeed_calibration(const Vector3f &vg);

private:

    GCS_MAVLINK_Plane _chan[MAVLINK_COMM_NUM_BUFFERS];

    bool cli_enabled() const override;
    AP_HAL::BetterStream* cliSerial() override;

};

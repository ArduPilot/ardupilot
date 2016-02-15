#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>

#include "Parameters.h"
#include "GCS_Backend.h"

class GCS_Frontend_Rover : public GCS_Frontend {
    friend class Rover; // for access to _gcs.gcs[i] for Parameters

public:

    GCS_Frontend_Rover(DataFlash_Class &DataFlash, Parameters &g) :
        GCS_Frontend(DataFlash, g)
        { }

protected:

    GCS_Backend_Rover &gcs(const uint8_t i) { return _gcs[i]; }

    uint32_t telem_delay() const;

private:

    GCS_Backend_Rover _gcs[MAVLINK_COMM_NUM_BUFFERS];

};

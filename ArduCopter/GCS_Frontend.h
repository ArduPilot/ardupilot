#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>
#include "GCS_Backend.h"

class GCS_Frontend_Copter : public GCS_Frontend {
    friend class Copter; // for access to _gcs.gcs[i] for Parameters

public:

    GCS_Frontend_Copter(DataFlash_Class &DataFlash, Parameters &g) :
        GCS_Frontend(DataFlash, g)
        { }

    GCS_Backend_Copter &gcs(const uint8_t i) { return _gcs[i]; }

private:

    GCS_Backend_Copter _gcs[MAVLINK_COMM_NUM_BUFFERS];

};

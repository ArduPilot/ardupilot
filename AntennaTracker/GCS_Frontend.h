#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>
#include "GCS_Backend.h"

#include "Parameters.h"

class GCS_Frontend_Tracker : public GCS_Frontend {
    friend class Tracker; // for access to _gcs.gcs[i] for Parameters

public:

    GCS_Frontend_Tracker(DataFlash_Class &DataFlash, Parameters &g) :
        GCS_Frontend(DataFlash),
        _g(g)
        { }
    void setup_uarts(AP_SerialManager & serial_manager, void (&)(const mavlink_message_t*));

    void request_data_streams(uint8_t sysid, uint8_t compid);

    GCS_Backend_Tracker &gcs(const uint8_t i) { return _gcs[i]; };

private:

    GCS_Backend_Tracker _gcs[MAVLINK_COMM_NUM_BUFFERS];

    Parameters &_g;
};

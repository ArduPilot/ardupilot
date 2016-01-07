#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>

#include "Parameters.h"

class GCS_Frontend_Tracker : public GCS_Frontend {

public:

    GCS_Frontend_Tracker(DataFlash_Class &DataFlash, Parameters &g) :
        GCS_Frontend(DataFlash),
        _g(g)
        { }
    void setup_uarts(AP_SerialManager & serial_manager, void (&)(const mavlink_message_t*));

    void request_data_streams(uint8_t sysid, uint8_t compid);

private:

    Parameters &_g;
};

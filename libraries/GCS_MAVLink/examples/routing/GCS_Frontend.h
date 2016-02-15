#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>

#include "GCS_Backend.h"

class GCS_Frontend_RoutingExample : public GCS_Frontend {

public:

    GCS_Frontend_RoutingExample(DataFlash_Class &dataflash, class Parameters &g) :
        GCS_Frontend(dataflash, g)
        { }

    GCS_Backend_RoutingExample &gcs(const uint8_t i) { return my_backend; }

protected:

    uint32_t telem_delay() const override { return 0; };

private:

    GCS_Backend_RoutingExample my_backend;
};

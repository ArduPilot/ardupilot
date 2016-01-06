#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>

#include "Parameters.h"

class GCS_Frontend_Rover : public GCS_Frontend {

public:

    GCS_Frontend_Rover(DataFlash_Class &DataFlash, Parameters &g) :
        GCS_Frontend(DataFlash),
        _g(g)
        { }

private:

    Parameters &_g;
};

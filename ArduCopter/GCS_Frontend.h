#pragma once

#include <GCS_MAVLink/GCS_Frontend.h>

class GCS_Frontend_Copter : public GCS_Frontend {

public:

    GCS_Frontend_Copter(DataFlash_Class &DataFlash, Parameters &g) :
        GCS_Frontend(DataFlash),
        _g(g)
        { }

private:

    Parameters &_g;
};

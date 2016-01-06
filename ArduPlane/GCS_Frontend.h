#pragma once

#include "config.h" // for HIL_MODE

#include <GCS_MAVLink/GCS_Frontend.h>

class GCS_Frontend_Plane : public GCS_Frontend {

public:

    GCS_Frontend_Plane(DataFlash_Class &DataFlash, class Parameters &g, class AP_Airspeed &airspeed);

    void send_airspeed_calibration(const Vector3f &vg);

private:

    Parameters &_g;
    AP_Airspeed &_airspeed;
};

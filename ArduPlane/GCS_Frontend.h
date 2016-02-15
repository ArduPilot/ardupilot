#pragma once

#include "config.h" // for HIL_MODE

#include <GCS_MAVLink/GCS_Frontend.h>

#include "GCS_Backend.h"

class GCS_Frontend_Plane : public GCS_Frontend {
    friend class Plane; // for access to _gcs[i]

public:

    GCS_Frontend_Plane(DataFlash_Class &DataFlash, class Parameters &g, class AP_Airspeed &airspeed);

    GCS_Backend_Plane &gcs(const uint8_t i) { return _gcs[i]; }

    void send_airspeed_calibration(const Vector3f &vg);

protected:

    uint32_t telem_delay() const override;

private:

    GCS_Backend_Plane _gcs[MAVLINK_COMM_NUM_BUFFERS];

    AP_Airspeed &_airspeed;

};

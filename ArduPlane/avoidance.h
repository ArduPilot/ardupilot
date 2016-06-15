#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

#include "avoidance_handler.h"

// Provide Plane-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_AvoidanceHandler, this class allows Plane to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Plane : public AP_Avoidance {

public:

    AP_Avoidance_Plane(AP_AHRS &ahrs, class AP_ADSB &adsb) :
        AP_Avoidance(ahrs, adsb) { }

protected:

private:

    // these objects are individually responsible for providing a
    // specific avoidance behaviour
    AvoidanceHandler_NONE avoidance_handler_none;
    AvoidanceHandler_PERPENDICULAR avoidance_handler_perpendicular{_ahrs};
    AvoidanceHandler_REPORT avoidance_handler_report;

    // returns an object which is responsible for getting the vehicle
    // out of trouble
    AvoidanceHandler &handler_for_action(MAV_COLLISION_ACTION action) override;

};

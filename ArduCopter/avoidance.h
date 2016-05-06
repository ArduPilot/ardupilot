#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

#include "avoidance_handler.h"

// Provide Copter-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_AvoidanceHandler, this class allows Copter to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Copter : public AP_Avoidance {

public:

    AP_Avoidance_Copter(AP_AHRS &ahrs, class AP_ADSB &adsb) :
        AP_Avoidance(ahrs, adsb) { }

protected:

private:

    // these objects are individually responsible for providing a
    // specific avoidance behaviour
    AvoidanceHandler_HOVER avoidance_handler_hover;
    AvoidanceHandler_NONE avoidance_handler_none;
    AvoidanceHandler_PERPENDICULAR avoidance_handler_perpendicular{_ahrs};
    AvoidanceHandler_REPORT avoidance_handler_report;
    AvoidanceHandler_RTL avoidance_handler_rtl;
    AvoidanceHandler_TCAS avoidance_handler_tcas{_ahrs};

    // returns an object which is responsible for getting the vehicle
    // out of trouble
    AvoidanceHandler &handler_for_action(MAV_COLLISION_ACTION action) override;

};

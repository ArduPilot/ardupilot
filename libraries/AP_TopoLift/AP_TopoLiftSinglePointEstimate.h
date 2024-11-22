#pragma once

#include "AP_TopoLift.h"

// Implements a super basic topographic lift model that uses
// the current UAV location, the local terrain gradient,
// the current wind estimate, and current height above
// terrain to calculate the theoretical amount of topographic lift.
// This is not very realistic or accurate, but it is useful as a 
// demonstration of the AP_TopoLift library.
// More comprehensive models may be developed, or run offboard
// and supplied through AP_ExternalTopoLift.

class AP_TopoLiftSinglePointEstimate : public AP_TopoLift {
    public:
    // Get the terrain height of a point.
    virtual bool get_terrain_height(const Location loc, float& height_amsl) override;

    // Calculate the wind rate of topographic influenced lift at the current location.
    virtual bool calc_lift(const Location loc, float& lift) override;

    // Get the wind estimate at the current location.
    virtual bool get_wind_at_current_loc(Vector3f& wind) override;

    // // Update state
    // virtual void update() override;
};
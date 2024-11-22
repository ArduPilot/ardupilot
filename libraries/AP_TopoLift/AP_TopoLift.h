#pragma once

#include <AP_Common/Location.h>
#include <AP_Math/vector3.h>


// The front end for the AP_TopoLift library.
class AP_TopoLift {
    public:

    // Get the terrain height of a point.
    virtual bool get_terrain_height(const Location loc, float& height_amsl) = 0;

    // // Calculate the wind rate of topographic influenced lift at the current location.
    virtual bool calc_lift(const Location loc, float& lift) = 0;

    // Get the wind estimate at the current location.
    virtual bool get_wind_at_current_loc(Vector3f& wind) = 0;

    // // Update state.
    // virtual void update() = 0;
};
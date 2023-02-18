#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{

    // notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight
    float height;
#if AP_TERRAIN_AVAILABLE
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // use the best available alt estimate via baro above home
        if (flight_stage == AP_FixedWing::FlightStage::LAND) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude;
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    rangefinder_height_update();
}

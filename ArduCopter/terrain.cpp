#include "Copter.h"

// update terrain data
void Copter::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // send terrain altitude to AHRS for optical flow when rangefinder is out of range
    Location veh_loc;
    if (ahrs.get_location(veh_loc)) {
        float terrain_alt_amsl_m;
        if (terrain.height_amsl(veh_loc, terrain_alt_amsl_m, true)) {
            ahrs.writeTerrainAMSL(terrain_alt_amsl_m);
        }
    }

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if AP_RANGEFINDER_ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }
#endif
#endif
}

#if HAL_LOGGING_ENABLED
// log terrain data - should be called at 1hz
void Copter::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}
#endif

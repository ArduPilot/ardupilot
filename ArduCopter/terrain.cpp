#include "Copter.h"

// update terrain data
void Copter::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if RANGEFINDER_ENABLED == ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }
#endif
#endif
}

// log terrain data - should be called at 1hz
void Copter::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}

// return terrain following altitude margin.  Vehicle will stop if distance from target altitude is larger than this margin (in meters)
float Copter::get_terrain_margin() const
{
#if AP_TERRAIN_AVAILABLE
    return MAX(g2.terrain_margin.get(), 0.1);
#else
    return 10;
#endif
}

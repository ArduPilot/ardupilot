#include "Copter.h"

// update terrain data
void Copter::terrain_update()
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
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
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}

// should we use terrain data for things including the home altitude
bool Copter::terrain_use()
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    return (g.terrain_follow > 0);
#else
    return false;
#endif
}

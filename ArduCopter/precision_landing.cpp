//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init();
}

void Copter::update_precland()
{
    int32_t height_above_terrain_cm;
    bool rangefinder_height_above_terrain_cm_valid = get_rangefinder_height_above_terrain(height_above_terrain_cm);

    // use range finder altitude if it is valid, else try to get terrain alt, else use height above home
    if (!rangefinder_height_above_terrain_cm_valid) {
        if (!terrain_use() || !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, height_above_terrain_cm)) {
            height_above_terrain_cm = current_loc.alt;
        }
    }

    copter.precland.update(height_above_terrain_cm, rangefinder_height_above_terrain_cm_valid);
}
#endif

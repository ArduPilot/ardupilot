#include "Copter.h"

// check if proximity type Simple Avoidance should be enabled based on alt
void Copter::low_alt_avoidance()
{
#if AC_AVOID_ENABLED == ENABLED
    int32_t alt_cm;
    if (!get_rangefinder_height_interpolated_cm(alt_cm)) {
        // enable avoidance if we don't have a valid rangefinder reading
        avoid.proximity_alt_avoidance_enable(true);
        return;
    }

    bool enable_avoidance = true;
    if (alt_cm < avoid.get_min_alt() * 100.0f) {
        enable_avoidance = false;
    }
    avoid.proximity_alt_avoidance_enable(enable_avoidance);
#endif
}

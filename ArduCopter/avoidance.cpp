#include "Copter.h"

// check if proximity type Simple Avoidance should be enabled based on alt
void Copter::low_alt_avoidance()
{
#if AP_AVOIDANCE_ENABLED
    float alt_m;
    if (!get_rangefinder_height_interpolated_m(alt_m)) {
        // enable avoidance if we don't have a valid rangefinder reading
        avoid.proximity_alt_avoidance_enable(true);
        return;
    }

    bool enable_avoidance = true;
    if (alt_m < avoid.get_min_alt()) {
        enable_avoidance = false;
    }
    avoid.proximity_alt_avoidance_enable(enable_avoidance);
#endif
}

#include "mode.h"

#include "Tracker.h"

void ModeAuto::update()
{
    if (tracker.vehicle.location_valid) {
        update_auto();
    } else if (tracker.target_set || (tracker.g.auto_opts.get() & (1 << 0)) != 0) {
        update_scan();
    }
}

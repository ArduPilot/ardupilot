#include "mode.h"

#include "Tracker.h"

void ModeAuto::update()
{
    if (tracker.vehicle.location_valid) {
        update_auto();
    } else if (tracker.target_set || (tracker.g.auto_opts.get() & (1 << 0)) != 0) {
        update_scan();
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
    }
}

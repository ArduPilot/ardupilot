#include "Copter.h"

#if AP_LANDINGGEAR_ENABLED

// Run landing gear controller at 10Hz
void Copter::landinggear_update()
{
    // exit immediately if no landing gear output has been enabled
    if (!SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        return;
    }

    // support height based triggering using rangefinder or altitude above ground
    int32_t height_cm = flightmode->get_alt_above_ground_cm();

    // use rangefinder if available
#if AP_RANGEFINDER_ENABLED
    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::NotConnected:
    case RangeFinder::Status::NoData:
        // use altitude above home for non-functioning rangefinder
        break;

    case RangeFinder::Status::OutOfRangeLow:
        // altitude is close to zero (gear should deploy)
        height_cm = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
    case RangeFinder::Status::Good:
        // use last good reading
        height_cm = rangefinder_state.alt_cm_filt.get();
        break;
    }
#endif  // AP_RANGEFINDER_ENABLED

    landinggear.update(height_cm * 0.01f); // convert cm->m for update call
}

#endif // AP_LANDINGGEAR_ENABLED

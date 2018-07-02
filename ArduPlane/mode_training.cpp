#include "mode.h"
#include "Plane.h"

bool ModeTraining::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

bool ModeTraining::attitude_stabilized() const
{
    return !plane.training_manual_pitch || !plane.training_manual_roll;
}

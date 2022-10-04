#include "AP_PitchRollController.h"

AP_PitchRollController::AP_PitchRollController(const AP_Vehicle::FixedWing &parms, AP_AutoTune::ATType _autotune_axis, const char *_axis_name)
    : aparm(parms),
      autotune_axis{_autotune_axis},
      axis_name{_axis_name}
{
    rate_pid.set_slew_limit_scale(45);
}

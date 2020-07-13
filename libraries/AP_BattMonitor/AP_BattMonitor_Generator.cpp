#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Generator.h"

extern const AP_HAL::HAL& hal;

// read - read the voltage and current
void AP_BattMonitor_Generator::read()
{
    _state.healthy = false;

#if GENERATOR_ENABLED
    AP_Generator_RichenPower *generator = AP::generator();

    // healthy if we can find a generator
    if (generator == nullptr) {
        return;
    }

    // get voltage
    if (!generator->voltage(_state.voltage)) {
        return;
    }

    // get current
    if (!generator->current(_state.current_amps)) {
        return;
    }

    if (!generator->healthy()) {
        return;
    }

    _state.healthy = true;
#endif
}

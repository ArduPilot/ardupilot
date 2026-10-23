#include "mode.h"
#include "Plane.h"

#include "qautotune.h"

#if QAUTOTUNE_ENABLED

bool ModeQAutotune::_enter()
{
#if QAUTOTUNE_ENABLED
    return quadplane.qautotune.init();
#else
    return false;
#endif
}

void ModeQAutotune::update()
{
    plane.mode_qstabilize.update();
}

void ModeQAutotune::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

#if QAUTOTUNE_ENABLED
    quadplane.qautotune.run();
#endif

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Center rudder
    output_rudder_and_steering(0.0);
}

void ModeQAutotune::_exit()
{
#if QAUTOTUNE_ENABLED
    plane.quadplane.qautotune.stop();
#endif
}

#endif

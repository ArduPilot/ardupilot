#include "mode.h"
#include "Plane.h"

bool ModeQAutotune::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQAutotune::update()
{
    plane.mode_qstabilize.update();
}

void ModeQAutotune::run()
{
#if QAUTOTUNE_ENABLED
    quadplane.qautotune.run();
#endif
}

void ModeQAutotune::_exit()
{
#if QAUTOTUNE_ENABLED
    plane.quadplane.qautotune.stop();
#endif
}


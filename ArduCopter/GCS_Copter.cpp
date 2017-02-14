#include "GCS_Copter.h"
#include "Copter.h"

bool GCS_Copter::cli_enabled() const
{
#if CLI_ENABLED == ENABLED
    return copter.g.cli_enabled;
#else
    return false;
#endif
}

AP_HAL::BetterStream* GCS_Copter::cliSerial() {
    return copter.cliSerial;
}

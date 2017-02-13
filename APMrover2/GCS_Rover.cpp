#include "GCS_Rover.h"
#include "Rover.h"

bool GCS_Rover::cli_enabled() const
{
#if CLI_ENABLED == ENABLED
    return rover.g.cli_enabled;
#else
    return false;
#endif
}

AP_HAL::BetterStream* GCS_Rover::cliSerial() {
    return rover.cliSerial;
}

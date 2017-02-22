#include "GCS_Sub.h"
#include "Sub.h"

bool GCS_Sub::cli_enabled() const
{
    return false;
}

AP_HAL::BetterStream* GCS_Sub::cliSerial() {
    return NULL;
}

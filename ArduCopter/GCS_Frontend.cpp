#include "Copter.h"

#include "GCS_Frontend.h"

uint32_t GCS_Frontend_Copter::telem_delay() const
{
    return (uint32_t)_g.telem_delay;
}

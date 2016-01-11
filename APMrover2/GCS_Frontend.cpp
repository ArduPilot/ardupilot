#include "GCS_Frontend.h"

uint32_t GCS_Frontend_Rover::telem_delay() const
{
    return (uint32_t)_g.telem_delay;
}

#include "GCS_Frontend.h"
#include "Rover.h"

uint32_t GCS_Frontend_Rover::telem_delay() const
{
    return (uint32_t)_g.telem_delay;
}

GCS_Frontend& GCS_Frontend_Static::get_Frontend() {
    return rover.gcs();
}

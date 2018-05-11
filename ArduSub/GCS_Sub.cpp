#include "GCS_Sub.h"
#include "Sub.h"

bool GCS_Sub::vehicle_initialised() const
{
    return sub.ap.initialised;
}

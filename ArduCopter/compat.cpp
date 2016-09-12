#include "Copter.h"

void Copter::delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

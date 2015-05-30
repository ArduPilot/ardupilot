#include "Copter.h"

void Copter::delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

uint32_t Copter::millis()
{
    return hal.scheduler->millis();
}

uint32_t Copter::micros()
{
    return hal.scheduler->micros();
}

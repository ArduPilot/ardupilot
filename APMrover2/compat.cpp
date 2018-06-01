#include "Rover.h"

void Rover::delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

void Rover::mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

uint32_t Rover::millis()
{
    return hal.scheduler->millis();
}

uint32_t Rover::micros()
{
    return hal.scheduler->micros();
}


#include "Rover.h"

void Rover::delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

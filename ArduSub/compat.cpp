#include "Sub.h"

void Sub::delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

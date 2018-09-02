#include "Tracker.h"

// Functions to read the RC radio input

void Tracker::read_radio()
{
    rc().read_input();
}

#include "Blimp.h"

// return barometric altitude in centimeters
void Blimp::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

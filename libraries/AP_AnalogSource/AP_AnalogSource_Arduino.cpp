
#include "wiring.h"
#include "AP_AnalogSource_Arduino.h"

int AP_AnalogSource_Arduino::read(void)
{
    return analogRead(_pin);
}


#include "AP_AnalogSource_ADC.h"

float AP_AnalogSource_ADC::read(void)
{
    float fullscale = _adc->Ch(_ch);
    float scaled = _prescale * fullscale;
    return scaled;
}

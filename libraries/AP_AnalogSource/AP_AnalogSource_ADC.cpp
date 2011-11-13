
#include "AP_AnalogSource_ADC.h"

int AP_AnalogSource_ADC::read(void)
{
    int fullscale = _adc->Ch(_ch);
    int scaled = _prescale * fullscale;
    return scaled;
}

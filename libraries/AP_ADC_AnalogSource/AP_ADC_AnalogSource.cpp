
#include "AP_ADC_AnalogSource.h"

float AP_ADC_AnalogSource::read() {
    float fullscale = _adc->Ch(_ch);
    float scaled = _prescale * fullscale;
    return scaled;
}

void AP_ADC_AnalogSource::set_pin(uint8_t machtnichts) {
    /* it would be an error to call this
     * but for now we'll leave it a no-op. */
}


#include "AP_ADC_AnalogSource.h"

/* Unfortunately we don't have a valid implementaton for read_latest - we
 * only have access to the average from the ADC driver. Not really a big deal
 * in our application currently. */

extern const AP_HAL::HAL& hal;

float AP_ADC_AnalogSource::read_latest() {
    return read_average();
}

float AP_ADC_AnalogSource::read_average() {
    float fullscale = _adc->Ch(_ch);
    float scaled = _prescale * fullscale;
    return scaled;
}

/*
  return voltage in Volts
 */
float AP_ADC_AnalogSource::voltage_average()
{
    float fullscale = _adc->Ch(_ch);
    // note that the Ch6 ADC on APM1 has a 3.3V range, and is against
    // an internal reference, not the 5V power supply
    return fullscale * 3.3 * 2.44140625e-4f;
}


void AP_ADC_AnalogSource::set_pin(uint8_t machtnichts) {
    /* it would be an error to call this
     * but for now we'll leave it a no-op. */
}


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
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float fullscale = _adc->Ch(_ch);
    // note that the Ch6 ADC on APM1 has a 3.3V range
    return fullscale * vcc_mV * (3.3/5.0) * 2.44140625e-7f; // 1.0/(4096*1000)
}


void AP_ADC_AnalogSource::set_pin(uint8_t machtnichts) {
    /* it would be an error to call this
     * but for now we'll leave it a no-op. */
}

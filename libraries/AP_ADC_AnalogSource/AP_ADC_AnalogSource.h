
#ifndef __AP_ADC_ANALOG_SOURCE_H__
#define __AP_ADC_ANALOG_SOURCE_H__

#include <AP_ADC.h>
#include <AP_HAL.h>

class AP_ADC_AnalogSource : public AP_HAL::AnalogSource
{
public:
    AP_ADC_AnalogSource( AP_ADC * adc, uint8_t ch, float prescale = 1.0 ) :
        _adc(adc), _ch(ch), _prescale(prescale)
    {}
    float           read(void);

private:
    AP_ADC *        _adc;
    uint8_t         _ch;
    float           _prescale;
};

#endif // __AP_ADC_ANALOG_SOURCE_H__

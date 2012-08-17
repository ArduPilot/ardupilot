
#ifndef __AP_ANALOG_SOURCE_ADC_H__
#define __AP_ANALOG_SOURCE_ADC_H__

#include "AnalogSource.h"
#include "../AP_ADC/AP_ADC.h"

class AP_AnalogSource_ADC : public AP_AnalogSource
{
public:
    AP_AnalogSource_ADC( AP_ADC * adc, uint8_t ch, float prescale = 1.0 ) :
        _adc(adc), _ch(ch), _prescale(prescale) {
    }
    float           read(void);

private:
    AP_ADC *        _adc;
    uint8_t         _ch;
    float           _prescale;
};

#endif // __AP_ANALOG_SOURCE_ADC_H__

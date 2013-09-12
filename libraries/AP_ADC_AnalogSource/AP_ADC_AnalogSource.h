
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
    float           read_average(void);
    float           read_latest(void);
    void            set_pin(uint8_t);
    float	    voltage_average();
    float	    voltage_latest() { return voltage_average(); }
    float	    voltage_average_ratiometric() { return voltage_average(); }

    // stop pins not implemented on ADC yet
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    AP_ADC *        _adc;
    uint8_t         _ch;
    float           _prescale;
};

#endif // __AP_ADC_ANALOG_SOURCE_H__

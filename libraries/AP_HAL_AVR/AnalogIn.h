
#ifndef __AP_HAL_AVR_ANALOG_IN_H__
#define __AP_HAL_AVR_ANALOG_IN_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVR_ANALOG_PIN_VCC 255
#define AVR_INPUT_MAX_CHANNELS 6

class AP_HAL_AVR::ADCSource : public AP_HAL::AnalogSource {
public:
    /* pin designates the ADC input number, or when == AVR_ANALOG_PIN_VCC,
     * board vcc */
    ADCSource(uint8_t pin, float prescale = 1.0);

    /* implement AnalogSource virtual api: */
    float read();

    /* implementation specific interface: */

    /* new_sample(): called with value of ADC measurments, from interrput */
    void new_sample(uint16_t);

    /* setup_read(): called to setup ADC registers for next measurment,
     * from interrupt */
    void setup_read();

    /* read_average: called to calculate and clear the internal average.
     * implements read(). */
    float read_average();
private:
    /* _sum_count and _sum are used from both an interrupt and normal thread */
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;

    /* _pin designates the ADC input mux for the sample */
    const uint8_t _pin;
    /* prescale scales the raw measurments for read()*/
    const float _prescale;
};


/* AVRAnalogIn : a concrete class providing the implementations of the 
 * timer event */
class AP_HAL_AVR::AVRAnalogIn {
public:
    AVRAnalogIn();
protected: 
    static void _register_channel(ADCSource*);
    static void _timer_event(uint32_t);
    static ADCSource* _channels[AVR_INPUT_MAX_CHANNELS];
    static int _num_channels;
    static int _active_channel;
    static int _channel_repeat_count;
};

/* APM1AnalogIn and APM2AnalogIn match the implementations in the AVRAnalogIn
 * class with the AP_HAL::AnalogIn's channel interface. */
class AP_HAL_AVR::APM1AnalogIn : public AP_HAL::AnalogIn,
                                 public AP_HAL_AVR::AVRAnalogIn {
public:
    APM1AnalogIn();
    void init(void* ap_hal_scheduler);
    AP_HAL::AnalogSource* channel(int n);
private:
    ADCSource _bat_voltage;
    ADCSource _bat_current;
    ADCSource _vdd;
};

class AP_HAL_AVR::APM2AnalogIn : public AP_HAL::AnalogIn,
                                 public AP_HAL_AVR::AVRAnalogIn {
public:
    APM2AnalogIn();
    void init(void* ap_hal_scheduler);
    AP_HAL::AnalogSource* channel(int n);
private:
    ADCSource _pitot;
    ADCSource _bat_voltage;
    ADCSource _bat_current;
    ADCSource _rssi;
    ADCSource _vdd;
};
#endif // __AP_HAL_AVR_ANALOG_IN_H__


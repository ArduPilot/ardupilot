
#ifndef __AP_HAL_AVR_ANALOG_IN_H__
#define __AP_HAL_AVR_ANALOG_IN_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVR_INPUT_MAX_CHANNELS 12

class AP_HAL_AVR::ADCSource : public AP_HAL::AnalogSource {
public:
    friend class AP_HAL_AVR::AVRAnalogIn;
    /* pin designates the ADC input number, or when == AVR_ANALOG_PIN_VCC,
     * board vcc */
    ADCSource(uint8_t pin, float prescale = 1.0);

    /* implement AnalogSource virtual api: */
    float read();
    void set_pin(uint8_t p);

    /* implementation specific interface: */

    /* new_sample(): called with value of ADC measurments, from interrput */
    void new_sample(uint16_t);

    /* setup_read(): called to setup ADC registers for next measurment,
     * from interrupt */
    void setup_read();

    /* read_average: called to calculate and clear the internal average.
     * implements read(). */
    float read_average();

    int16_t get_pin() { return _pin; };
private:
    /* _sum_count and _sum are used from both an interrupt and normal thread */
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;

    /* _pin designates the ADC input mux for the sample */
    uint8_t _pin;
    /* prescale scales the raw measurments for read()*/
    const float _prescale;
};

/* AVRAnalogIn : a concrete class providing the implementations of the 
 * timer event and the AP_HAL::AnalogIn interface */
class AP_HAL_AVR::AVRAnalogIn : public AP_HAL::AnalogIn {
public:
    AVRAnalogIn();
    void init(void* ap_hal_scheduler);
    AP_HAL::AnalogSource* channel(int16_t n);
    AP_HAL::AnalogSource* channel(int16_t n, float prescale);

protected: 
    static ADCSource* _find_or_create_channel(int16_t num, float scale);
    static ADCSource* _create_channel(int16_t num, float scale);
    static void _register_channel(ADCSource*);
    static void _timer_event(uint32_t);
    static ADCSource* _channels[AVR_INPUT_MAX_CHANNELS];
    static int16_t _num_channels;
    static int16_t _active_channel;
    static int16_t _channel_repeat_count;

private:
    ADCSource _vcc;
};

#endif // __AP_HAL_AVR_ANALOG_IN_H__


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
    ADCSource(uint8_t pin);

    /* implement AnalogSource virtual api: */
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);    

    /* implementation specific interface: */

    /* new_sample(): called with value of ADC measurments, from interrput */
    void new_sample(uint16_t);

    /* setup_read(): called to setup ADC registers for next measurment,
     * from interrupt */
    void setup_read();

    /* stop_read(): called to stop device measurement */
    void stop_read();

    /* reading_settled(): called to check if we have read for long enough */
    bool reading_settled();

    /* read_average: called to calculate and clear the internal average.
     * implements read_average(), unscaled. */
    float _read_average();

    int16_t get_pin() { return _pin; };
private:
    /* following three are used from both an interrupt and normal thread */
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;
    volatile uint16_t _latest;
    float _last_average;

    /* _pin designates the ADC input mux for the sample */
    uint8_t _pin;

    /* _stop_pin designates a digital pin to use for
       enabling/disabling the analog device */
    uint8_t _stop_pin;
    uint16_t _settle_time_ms;
    uint32_t _read_start_time_ms;
};

/* AVRAnalogIn : a concrete class providing the implementations of the 
 * timer event and the AP_HAL::AnalogIn interface */
class AP_HAL_AVR::AVRAnalogIn : public AP_HAL::AnalogIn {
public:
    AVRAnalogIn();
    void init(void* ap_hal_scheduler);
    AP_HAL::AnalogSource* channel(int16_t n);

protected: 
    static ADCSource* _create_channel(int16_t num);
    static void _register_channel(ADCSource*);
    static void _timer_event(uint32_t);
    static ADCSource* _channels[AVR_INPUT_MAX_CHANNELS];
    static int16_t _num_channels;
    static int16_t _active_channel;
    static uint16_t _channel_repeat_count;

private:
    ADCSource _vcc;
};

#endif // __AP_HAL_AVR_ANALOG_IN_H__

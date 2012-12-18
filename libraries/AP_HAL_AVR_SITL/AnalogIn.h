
#ifndef __AP_HAL_AVR_SITL_ANALOG_IN_H__
#define __AP_HAL_AVR_SITL_ANALOG_IN_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"

#define SITL_INPUT_MAX_CHANNELS 12

class AVR_SITL::ADCSource : public AP_HAL::AnalogSource {
public:
    friend class AVR_SITL::SITLAnalogIn;
    /* pin designates the ADC input number, or when == AVR_ANALOG_PIN_VCC,
     * board vcc */
    ADCSource(SITL_State *sitlState, uint8_t pin, float prescale = 1.0);

    /* implement AnalogSource virtual api: */
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);

private:
    /* prescale scales the raw measurments for read()*/
    SITL_State *_sitlState;
    uint8_t _pin;
    const float _prescale;
};

/* AVRAnalogIn : a concrete class providing the implementations of the 
 * timer event and the AP_HAL::AnalogIn interface */
class AVR_SITL::SITLAnalogIn : public AP_HAL::AnalogIn {
public:
    SITLAnalogIn(SITL_State *sitlState) {
	_sitlState = sitlState;
    }
    void init(void* ap_hal_scheduler);
    AP_HAL::AnalogSource* channel(int16_t n);
    AP_HAL::AnalogSource* channel(int16_t n, float prescale);

private:
    static ADCSource* _channels[SITL_INPUT_MAX_CHANNELS];
    SITL_State *_sitlState;
};

#endif // __AP_HAL_AVR_SITL_ANALOG_IN_H__

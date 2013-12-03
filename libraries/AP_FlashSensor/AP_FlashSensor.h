/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/* 
KSU AIAA Code:
    Class designed to use a camera flash signal on an analog 
    input in order to send GCS telemetry for each still image.

    Date Update: 11/13/2013
    Author(s): Joel Cranmer [Matlock42],
*/

#ifndef __AP_FLASHSENSOR_H__
#define __AP_FLASHSENSOR_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_ADS_AnalogSource.h>

// Extends the AP_ADC_AnalogSource class
class AP_FlashSensor : public AP_ADC_AnalogSource
{
public:
    // Initializes the class
    AP_FlashSensor( AP_ADC * adc, uint8_t ch, float prescale = 1.0 ) :
        _adc(adc), _ch(ch), _prescale(prescale)
    {}
    void    send_telemetry(void);   // Forces telemetry data message to get sent
    bool    is_hot;                 // Prevents image counter from going up if sensor is still triggered
    void    test_voltage(void);     // Tests the pin voltage
    void    reset_counter(void);    // Resets the frame counter value

private:
    // Count how many images have been taken (triggered). Help with possiblity of lost image.
    uint16_t    frame_counter = 0;
}

#endif // __AP_FLASHSENSOR_H__

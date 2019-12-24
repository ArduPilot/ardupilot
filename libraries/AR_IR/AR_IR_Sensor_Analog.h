#pragma once

#include "AR_IR_Sensor.h"
#include "AR_IR_Sensor_Backend.h"

extern const AP_HAL::HAL& hal ;

class AR_IR_Analog: public AR_IR_Backend
{
public:
    AR_IR_Analog(AR_IR &val, AR_IR::AR_IR_State &val_state);
    void read() override;

private:
    AP_HAL::AnalogSource *_IR_pin_analog_source_1;
    AP_HAL::AnalogSource *_IR_pin_analog_source_2;
};
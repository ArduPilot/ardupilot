#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AR_IR_Sensor_Analog.h"
#include "AR_IR_Sensor.h"

AR_IR_Analog::AR_IR_Analog(AR_IR &val, AR_IR::AR_IR_State &val_state):
    AR_IR_Backend(val,val_state)
{
    _IR_pin_analog_source_1= hal.analogin->channel(ANALOG_INPUT_NONE);
    _IR_pin_analog_source_2= hal.analogin->channel(ANALOG_INPUT_NONE);
}

void AR_IR_Analog::read()
{
    _IR_pin_analog_source_1->set_pin(_val.param._ir_1_pin_num);
    float pin_1_reading = _IR_pin_analog_source_1->voltage_average();

    _IR_pin_analog_source_2->set_pin(_val.param._ir_2_pin_num);
    float pin_2_reading = _IR_pin_analog_source_2->voltage_average();
    
    copy_state_to_frontend(pin_1_reading,pin_2_reading);
}
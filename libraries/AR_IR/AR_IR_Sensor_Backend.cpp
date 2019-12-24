#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"
#include "AR_IR_Sensor_Backend.h"

AR_IR_Backend:: AR_IR_Backend(AR_IR &val, AR_IR::AR_IR_State &val_state) :
    _val(val),
    _state(val_state)
{
}

// copy state to front end helper function
void AR_IR_Backend:: copy_state_to_frontend(float IR_Reading_1, float IR_Reading_2)
{
     _state.voltage_ir_1 = IR_Reading_1;
     _state.voltage_ir_2 = IR_Reading_2;
}
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"
class AR_IR_Backend
{
public:
    AR_IR_Backend(AR_IR &val, AR_IR::AR_IR_State &val_state);
    virtual void read()=0;

protected:
    void copy_state_to_frontend(float IR_Reading_1, float IR_Reading_2); // copy state to front end helper function
    AR_IR &_val;
    AR_IR::AR_IR_State &_state;
};
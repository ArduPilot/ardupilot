#pragma once

#include "AR_IR_Sensor.h"
#include "AR_IR_Sensor_Backend.h"
#include <SITL/SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

extern const AP_HAL::HAL& hal ;

class AR_IR_SITL_Analog: public AR_IR_Backend
{
public:
    AR_IR_SITL_Analog(AR_IR &val, AR_IR::AR_IR_State &val_state);
    void read() override;

private:
    SITL::SITL *_sitl; // pointer to SITL singleton
};

#endif // CONFIG_HAL_BOARD
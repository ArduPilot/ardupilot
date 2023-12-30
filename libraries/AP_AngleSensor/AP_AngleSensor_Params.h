#pragma once

#include "AP_AngleSensor_config.h"

#if AP_ANGLESENSOR_ENABLED

#include <AP_Param/AP_Param.h>
#include "AP_AngleSensor_config.h"


class AP_AngleSensor_Params{
public:

    AP_AngleSensor_Params(void);

    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AngleSensor_Params);

    // parameters for each instance
    AP_Int8  _type;
    AP_Int8  _bus;
    AP_Int8  _addr;
    AP_Float _offset;
    AP_Int8  _direction;
};

#endif // AP_ANGLESENSOR_ENABLED
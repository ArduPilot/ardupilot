#pragma once

#include <AP_Param/AP_Param.h>

class AP_AirSensor_Params {
public:
    // Constructor
    AP_AirSensor_Params();
    CLASS_NO_COPY(AP_AirSensor_Params);
    AP_Int8  type; // TODO forcing scripting for now as default type.
    static const struct AP_Param::GroupInfo var_info[];
};

#pragma once

#include <AP_Param/AP_Param.h>

class CiS_parameter {
public:
    CiS_parameter();
    void init();
    void update();
    static const struct AP_Param::GroupInfo var_info[];
    AP_Int8 greeting;
 
};

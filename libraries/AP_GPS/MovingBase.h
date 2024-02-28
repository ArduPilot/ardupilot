#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class MovingBase {
public:
    static const struct AP_Param::GroupInfo var_info[];

    MovingBase(void);

    /* Do not allow copies */
    CLASS_NO_COPY(MovingBase);

    enum class Type : int8_t {
        RelativeToAlternateInstance = 0,
        RelativeToCustomBase        = 1,
    };

    AP_Int8 type;            // an option from MovingBaseType
    AP_Vector3f base_offset; // base position offset from the selected GPS receiver

};

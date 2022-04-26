#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Compass_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Compass_Params(void);

    /* Do not allow copies */
    AP_Compass_Params(const AP_Compass_Params &other) = delete;
    AP_Compass_Params &operator=(const AP_Compass_Params&) = delete;

    AP_Int8     use_for_yaw;
    AP_Int8     orientation;
    AP_Int8     external;

    // device id detected at init.
    // saved to eeprom when offsets are saved allowing ram &
    // eeprom values to be compared as consistency check
    AP_Int32    dev_id;

    AP_Vector3f offset;
    AP_Vector3f diagonals;
    AP_Vector3f offdiagonals;
    AP_Float    scale_factor;

    // factors multiplied by throttle and added to compass outputs
    AP_Vector3f motor_compensation;

};


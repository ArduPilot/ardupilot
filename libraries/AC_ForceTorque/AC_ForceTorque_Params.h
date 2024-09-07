#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AC_ForceTorque_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AC_ForceTorque_Params(void);

    /* Do not allow copies */
    AC_ForceTorque_Params(const AC_ForceTorque_Params &other) = delete;
    AC_ForceTorque_Params &operator=(const AC_ForceTorque_Params&) = delete;

    AP_Vector3f force_N;
    AP_Vector3f torque_Nm;

    AP_Float force_x_N;
    AP_Float force_y_N;
    AP_Float force_z_N;
    AP_Float min_force_N;
    AP_Float max_force_N;

    AP_Float torque_x_Nm;
    AP_Float torque_y_Nm;
    AP_Float torque_z_Nm;
    AP_Float min_torque_Nm;
    AP_Float max_torque_Nm;

    AP_Int8 type;
    AP_Int8  location;

};
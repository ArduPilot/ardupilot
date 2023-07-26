#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_InertialSensor_tempcal.h"
#include "AP_InertialSensor_config.h"

class AP_InertialSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_InertialSensor_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_InertialSensor_Params);

    AP_Int32 _accel_id;
    AP_Vector3f _accel_scale;
    AP_Vector3f _accel_offset;
    AP_Vector3f _accel_pos;
    AP_Float caltemp_accel;

    AP_Int32 _gyro_id;
    AP_Vector3f _gyro_offset;
    AP_Float caltemp_gyro;

    AP_Int8 _use;

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    AP_InertialSensor_TCal tcal;
#endif
};

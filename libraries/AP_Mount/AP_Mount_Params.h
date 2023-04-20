#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Mount_Params {

public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_Mount_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Params);

    AP_Int8     type;               // mount type (see MountType enum)
    AP_Int8     default_mode;       // default mode on startup and when control is returned from autopilot
    AP_Int16    rc_rate_max;        // Pilot rate control's maximum rate.  Set to zero to use angle control
    AP_Int16    roll_angle_min;     // roll angle min in degrees
    AP_Int16    roll_angle_max;     // roll angle max in degrees
    AP_Int16    pitch_angle_min;    // pitch angle min in degrees
    AP_Int16    pitch_angle_max;    // pitch angle max in degrees
    AP_Int16    yaw_angle_min;      // yaw angle min in degrees
    AP_Int16    yaw_angle_max;      // yaw angle max in degrees

    AP_Vector3f retract_angles;     // retracted position in degrees. vector.x = roll vector.y = pitch, vector.z=yaw
    AP_Vector3f neutral_angles;     // neutral position in degrees.  vector.x = roll vector.y = pitch, vector.z=yaw

    AP_Float    roll_stb_lead;      // roll lead control gain (only used by servo backend)
    AP_Float    pitch_stb_lead;     // pitch lead control gain (only used by servo backend)
    AP_Int8     sysid_default;      // target sysid for mount to follow
};

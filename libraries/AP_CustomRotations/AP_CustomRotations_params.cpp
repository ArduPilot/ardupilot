#include "AP_CustomRotations_config.h"

#if AP_CUSTOMROTATIONS_ENABLED

#include "AP_CustomRotations.h"

const AP_Param::GroupInfo AP_CustomRotation_params::var_info[] = {

    // @Param: ROLL
    // @DisplayName: Custom roll
    // @Description: Custom euler roll, euler 321 (yaw, pitch, roll) ordering
    // @Units: deg
    // @RebootRequired: True
    AP_GROUPINFO("ROLL",  1, AP_CustomRotation_params, roll, 0),

    // @Param: PITCH
    // @DisplayName: Custom pitch
    // @Description: Custom euler pitch, euler 321 (yaw, pitch, roll) ordering
    // @Units: deg
    // @RebootRequired: True
    AP_GROUPINFO("PITCH",  2, AP_CustomRotation_params, pitch, 0),

    // @Param: YAW
    // @DisplayName: Custom yaw
    // @Description: Custom euler yaw, euler 321 (yaw, pitch, roll) ordering
    // @Units: deg
    // @RebootRequired: True
    AP_GROUPINFO("YAW",  3, AP_CustomRotation_params, yaw, 0),

    AP_GROUPEND
};

AP_CustomRotation_params::AP_CustomRotation_params() {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_CUSTOMROTATIONS_ENABLED

#include "AP_AirSensor_Params.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_AirSensor_Params::var_info[] = {

    // 0 should not be used

    // @Param: _TYPE
    // @DisplayName: Air Sensor type
    // @Description: What type of air sensor is connected
    // @SortValues: AlphabeticalZeroAtTop
    // @Values: 0:None,1:Scripting
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE",   1, AP_AirSensor_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_AirSensor_Params::AP_AirSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

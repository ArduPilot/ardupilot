#include "AP_Filter_config.h"

#if AP_FILTER_ENABLED

#include "AP_Filter.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

const AP_Param::GroupInfo AP_Filter_params::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Filter Type
    // @Values: 0:Disable, 1:Notch Filter
    // @Description: Filter Type
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Filter_params, _type, int8_t(AP_Filter::FilterType::FILTER_NONE), AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_Filter_params::AP_Filter_params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_FILTER_ENABLED

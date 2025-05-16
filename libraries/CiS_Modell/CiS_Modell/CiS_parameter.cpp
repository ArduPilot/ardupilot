#include "CiS_parameter.h"

CiS_parameter::CiS_parameter() {
    AP_Param::setup_object_defaults(this, var_info);
}

void CiS_parameter::init() {}

const AP_Param::GroupInfo CiS_parameter::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable CiS Model
    // @Description: Enables or disables CiS functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 0, CiS_parameter, enable, 1),

    // @Param: EXAMPLE
    // @DisplayName: Example Float
    // @Description: Example parameter for test
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("EXAMPLE", 1, CiS_parameter, example_value, 1.23f),

    AP_GROUPEND
};

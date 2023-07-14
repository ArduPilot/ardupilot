#include "AP_Torqeedo_Params.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Torqeedo_Params::var_info[] = {

    // 0 should not be used

    // @Param: _TYPE
    // @DisplayName: Porpellers type
    // @Description: What type of propeelers is connected
    // @Values: 0:None,1:TQBus
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE",   1, AP_Torqeedo_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_Torqeedo_Params::AP_Torqeedo_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

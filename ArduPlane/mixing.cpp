#include "mixing.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Mixing::var_info[] = {
    // @Param: INDV_EN
    // @DisplayName: Individual mix enable
    // @Description: Enable individual mixing parameters for VTAIL and ELEVON
    // @Values: 0:Disable, 1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("INDV_EN", 1, AP_Mixing, individual_mix_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VT_GAIN
    // @DisplayName: V-tail mixing gain
    // @Description: Gain applied to V-tail mixing output
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("VT_GAIN", 2, AP_Mixing, vtail_mgain, 0.5f),

    // @Param: VT_OFFSET
    // @DisplayName: V-tail mixing offset
    // @Description: Offset applied to V-tail mixing output
    // @Range: -1000 1000
    // @User: Advanced
    AP_GROUPINFO("VT_OFFSET", 3, AP_Mixing, vtail_moffset, 0),

    // @Param: EL_GAIN
    // @DisplayName: Elevon mixing gain
    // @Description: Gain applied to elevon mixing output
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("EL_GAIN", 4, AP_Mixing, elevon_mgain, 0.5f),

    // @Param: EL_OFFSET
    // @DisplayName: Elevon mixing offset
    // @Description: Offset applied to elevon mixing output
    // @Range: -1000 1000
    // @User: Advanced
    AP_GROUPINFO("EL_OFFSET", 5, AP_Mixing, elevon_moffset, 0),

    AP_GROUPEND
};

AP_Mixing::AP_Mixing(void)
{
    AP_Param::setup_object_defaults(this, var_info);
} 
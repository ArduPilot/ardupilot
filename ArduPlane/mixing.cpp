#include "mixing.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Mixing::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Individual mix enable
    // @Description: Enable individual mixing parameters for VTAIL and ELEVON
    // @Values: 0:Disable, 1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Mixing, individual_mix_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VTAIL_MGAIN
    // @DisplayName: V-tail mixing gain
    // @Description: Gain applied to V-tail mixing output
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("VTAIL_MGAIN", 2, AP_Mixing, vtail_mgain, 0.5f),

    // @Param: VTAIL_MOFFSET
    // @DisplayName: V-tail mixing offset
    // @Description: Offset applied to V-tail mixing output
    // @Range: -1000 1000
    // @User: Advanced
    AP_GROUPINFO("VTAIL_MOFFSET", 3, AP_Mixing, vtail_moffset, 0),

    // @Param: ELEVON_MGAIN
    // @DisplayName: Elevon mixing gain
    // @Description: Gain applied to elevon mixing output
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("ELEVON_MGAIN", 4, AP_Mixing, elevon_mgain, 0.5f),

    // @Param: ELEVON_MOFFSET
    // @DisplayName: Elevon mixing offset
    // @Description: Offset applied to elevon mixing output
    // @Range: -1000 1000
    // @User: Advanced
    AP_GROUPINFO("ELEVON_MOFFSET", 5, AP_Mixing, elevon_moffset, 0),

    AP_GROUPEND
};

AP_Mixing::AP_Mixing(void)
{
    AP_Param::setup_object_defaults(this, var_info);
} 
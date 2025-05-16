#include "AP_CiS_parameter.h"
#include <GCS_MAVLink/GCS.h>

CiS_parameter::CiS_parameter() {
    AP_Param::setup_object_defaults(this, var_info);
}

void CiS_parameter::init() {}

const AP_Param::GroupInfo CiS_parameter::var_info[] = {
    // @Param: _Greeting
    // @DisplayName: Enable CiS Model
    // @Description: Enables or disables CiS functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_Greeting", 0, CiS_parameter, greeting, 1),

    AP_GROUPEND
};


void CiS_parameter::update() {
    static bool last_state = false;
    bool current = greeting.get();

    if (current && !last_state) {
        gcs().send_text(MAV_SEVERITY_INFO, "CiS_Modell is Active");
    }

    last_state = current;
}

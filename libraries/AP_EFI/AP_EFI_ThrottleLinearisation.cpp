#include "AP_EFI_config.h"

#if AP_EFI_THROTTLE_LINEARISATION_ENABLED

#include "AP_EFI.h"
#include <AP_Param/AP_Param.h>

// settings for throttle linearisation
const AP_Param::GroupInfo AP_EFI_ThrLin::var_info[] = {

    // @Param: _EN
    // @DisplayName: Enable throttle linearisation
    // @Description: Enable EFI throttle linearisation
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_EN", 1, AP_EFI_ThrLin, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _COEF1
    // @DisplayName: Throttle linearisation - First Order
    // @Description: First Order Polynomial Coefficient. (=1, if throttle is first order polynomial trendline)
    // @Range: -1 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_COEF1", 2, AP_EFI_ThrLin, coefficient[0], 1),

    // @Param: _COEF2
    // @DisplayName: Throttle linearisation - Second Order
    // @Description: Second Order Polynomial Coefficient (=0, if throttle is second order polynomial trendline)
    // @Range: -1 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_COEF2", 3, AP_EFI_ThrLin, coefficient[1], 0),

    // @Param: _COEF3
    // @DisplayName: Throttle linearisation - Third Order
    // @Description: Third Order Polynomial Coefficient. (=0, if throttle is third order polynomial trendline)
    // @Range: -1 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_COEF3", 4, AP_EFI_ThrLin, coefficient[2], 0),

    // @Param: _OFS
    // @DisplayName: throttle linearization offset
    // @Description: Offset for throttle linearization 
    // @Range: 0 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_OFS", 5, AP_EFI_ThrLin, offset, 0),

    AP_GROUPEND
};

AP_EFI_ThrLin::AP_EFI_ThrLin(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  apply throttle linearisation
 */
float AP_EFI_ThrLin::linearise_throttle(float throttle_percent)
{
    if (!enable) {
        return throttle_percent;
    }
    float ret = coefficient[0] * throttle_percent;
    ret += coefficient[1] * throttle_percent * throttle_percent;
    ret += coefficient[2] * throttle_percent * throttle_percent * throttle_percent;
    ret += offset;
    return ret;
}

#endif // AP_EFI_THROTTLE_LINEARISATION_ENABLED


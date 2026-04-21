#include "SITL.h"

#if AP_SIM_ENABLED

namespace SITL {
// user settable parameters for barometers
const AP_Param::GroupInfo SIM::BaroParm::var_info[] = {
    // @Param: RND
    // @DisplayName: Barometer noise
    // @Description: Barometer noise in height
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RND",      1, SIM::BaroParm,  noise, 0.2f),
    // @Param: DRIFT
    // @DisplayName: Barometer altitude drift
    // @Description: Barometer altitude drifts at this rate
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("DRIFT",    2, SIM::BaroParm,  drift, 0),
    // @Param: DISABLE
    // @DisplayName: Barometer disable
    // @Description: Disable barometer in SITL
    // @Values: 0:Disable, 1:Enable
    // @User: Advanced
    AP_GROUPINFO("DISABLE",  3, SIM::BaroParm,  disable, 0),
    // @Param: GLITCH
    // @DisplayName: Barometer glitch
    // @Description: Barometer glitch height in SITL
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GLITCH",   4, SIM::BaroParm,  glitch, 0),
    // @Param: FREEZE
    // @DisplayName: Barometer freeze
    // @Description: Freeze barometer to last recorded altitude
    // @User: Advanced
    AP_GROUPINFO("FREEZE",   5, SIM::BaroParm,  freeze, 0),
    // @Param: DELAY
    // @DisplayName: Barometer delay
    // @Description: Barometer data time delay
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("DELAY",    6, SIM::BaroParm,  delay, 0),

    // @Param: WCF_FWD
    // @DisplayName: Wind coefficient forward
    // @Description: Barometer wind coefficient direction forward in SITL
    // @User: Advanced
    AP_GROUPINFO("WCF_FWD", 7,  SIM::BaroParm, wcof_xp, 0.0),
    // @Param: WCF_BAK
    // @DisplayName: Wind coefficient backward
    // @Description: Barometer wind coefficient direction backward in SITL
    // @User: Advanced
    AP_GROUPINFO("WCF_BAK", 8,  SIM::BaroParm, wcof_xn, 0.0),
    // @Param: WCF_RGT
    // @DisplayName: Wind coefficient right
    // @Description: Barometer wind coefficient direction right in SITL
    // @User: Advanced
    AP_GROUPINFO("WCF_RGT", 9,  SIM::BaroParm, wcof_yp, 0.0),
    // @Param: WCF_LFT
    // @DisplayName: Wind coefficient left
    // @Description: Barometer wind coefficient direction left in SITL
    // @User: Advanced
    AP_GROUPINFO("WCF_LFT", 10, SIM::BaroParm, wcof_yn, 0.0),
    // @Param: WCF_UP
    // @DisplayName: Wind coefficient up
    // @Description: Barometer wind coefficient direction up in SITL
    // @User: Advanced
    AP_GROUPINFO("WCF_UP",  11, SIM::BaroParm, wcof_zp, 0.0),
    // @Param: WCF_DN
    // @DisplayName: Wind coefficient down
    // @Description: Barometer wind coefficient direction down in SITL
    // @User: Advanced
    AP_GROUPINFO("WCF_DN",  12, SIM::BaroParm, wcof_zn, 0.0),
    AP_GROUPEND
};
}

#endif  // AP_SIM_ENABLED

#include "AC_ForceTorque_Params.h"
#include "AC_ForceTorque.h"
//用于日志使用的注册函数
// table of user settable parameters
const AP_Param::GroupInfo AC_ForceTorque_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: ForceTorque type
    // @Description: Type of connected ForceTorque
    // @Values: 0:None,1:DR304_Serial
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AC_ForceTorque_Params, type, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: LOCATION
    // @DisplayName: ForceTorque location
    // @Description: Install location of ForceTorque
    // @Values: 0:LOCATION_NONE, 1:Up_Rotor, 2:Down_Rotor
    // @User: Advanced
    AP_GROUPINFO("LOCATION", 2, AC_ForceTorque_Params, location, Up_Rotor),
    // @Param: MIN_FORCE
    // @DisplayName: forceTorque sensor minimum thrust force
    // @Description: Minimum force in N that sensor can reliably read
    // @Units: N
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MIN_FRCE",  3, AC_ForceTorque_Params, min_force_N, -50),

    // @Param: MAX_FORCE
    // @DisplayName: forceTorque sensor maximum thrust
    // @Description: Maximum force in N that sensor can reliably read
    // @Units: N
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MAX_FRCE",  4, AC_ForceTorque_Params, max_force_N, 50),

    // @Param: MIN_TORQ
    // @DisplayName: forceTorque sensor minimum Torque 
    // @Description: Minimum torque in Nm that sensor can reliably read
    // @Units: Nm
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MIN_TORQ",  5, AC_ForceTorque_Params, min_torque_Nm, -20),

    // @Param: MAX_TORQ
    // @DisplayName: forceTorque sensor maximum Torque
    // @Description: Maximum torque in Nm that sensor can reliably read
    // @Units: Nm
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MAX_TORQ",  6, AC_ForceTorque_Params, max_torque_Nm, 20),

    AP_GROUPEND
};


AC_ForceTorque_Params::AC_ForceTorque_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
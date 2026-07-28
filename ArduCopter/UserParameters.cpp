#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    // @Param: USR_STEP_DIST
    // @DisplayName: Step distance
    // @Description: Longueur d'un pas du mode step
    // @Units: m
    AP_GROUPINFO("_STEP_DIST", 3, UserParameters, _step_dist,1),
    
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED

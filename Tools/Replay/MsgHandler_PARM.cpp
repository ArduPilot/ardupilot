#include "MsgHandler_PARM.h"

bool MsgHandler::set_parameter(const char *name, float value)
{
    const char *ignore_parms[] = { "GPS_TYPE", "AHRS_EKF_USE" };
    for (uint8_t i=0; i<sizeof(ignore_parms)/sizeof(ignore_parms[0]); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(name, &var_type);
    if (vp == NULL) {
        return false;
    }
    if (var_type == AP_PARAM_FLOAT) {
        ((AP_Float *)vp)->set(value);
        ::printf("Set %s to %f\n", name, value);
    } else if (var_type == AP_PARAM_INT32) {
        ((AP_Int32 *)vp)->set(value);
        ::printf("Set %s to %d\n", name, (int)value);
    } else if (var_type == AP_PARAM_INT16) {
        ((AP_Int16 *)vp)->set(value);
        ::printf("Set %s to %d\n", name, (int)value);
    } else if (var_type == AP_PARAM_INT8) {
        ((AP_Int8 *)vp)->set(value);
        ::printf("Set %s to %d\n", name, (int)value);
    } else {
        // we don't support mavlink set on this parameter
        return false;
    }
    return true;
}

void MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];

    require_field(msg, "Name", parameter_name, parameter_name_len);

    set_parameter(parameter_name, require_field_float(msg, "Value"));
}

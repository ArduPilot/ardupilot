#include "AP_Scripting_helpers.h"

/// Fast param access via pointer helper class

// init by name
bool Parameter::init(const char *name)
{
    vp = AP_Param::find(name, &vtype);
    if (vp == nullptr) {
        return false;
    }
    return true;
}

// init by info, to get the value of old params
bool Parameter::init_by_info(uint16_t key, uint32_t group_element, enum ap_var_type type)
{
    switch (type) {
    case AP_PARAM_INT8:
        vp = new AP_Int8;
        break;
    case AP_PARAM_INT16:
        vp = new AP_Int16;
        break;
    case AP_PARAM_INT32:
        vp = new AP_Int32;
        break;
    case AP_PARAM_FLOAT:
        vp = new AP_Float;
        break;
    default:
        return false;
    }
    if (vp == nullptr) {
        return false;
    }
    vtype = type;
    AP_Param::ConversionInfo info = {
        key,
        group_element,
        type,
        nullptr
    };
    return AP_Param::find_old_parameter(&info, vp);
}

// set a value
bool Parameter::set(float value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

// get value
bool Parameter::get(float &value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        value = ((AP_Int8 *)vp)->get();
        break;
    case AP_PARAM_INT16:
        value = ((AP_Int16 *)vp)->get();
        break;

    case AP_PARAM_INT32:
        value = ((AP_Int32 *)vp)->get();
        break;

    case AP_PARAM_FLOAT:
        value = ((AP_Float *)vp)->get();
        break;

    default:
        // not a supported type
        return false;
    }
    return true;
}

// set and save value
bool Parameter::set_and_save(float value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_and_save(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_and_save(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

// Check if param had been configured
bool Parameter::configured()
{
    if (vp == nullptr) {
        return false;
    }
    return vp->configured();
}

// set default value
bool Parameter::set_default(float value)
{
    if (vp == nullptr) {
        return false;
    }
    switch (vtype) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)vp)->set_default(value);
        return true;
    case AP_PARAM_INT16:
        ((AP_Int16 *)vp)->set_default(value);
        return true;
    case AP_PARAM_INT32:
        ((AP_Int32 *)vp)->set_default(value);
        return true;
    case AP_PARAM_FLOAT:
        ((AP_Float *)vp)->set_default(value);
        return true;
    default:
        break;
    }
    // not a supported type
    return false;
}

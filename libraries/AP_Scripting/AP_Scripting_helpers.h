#pragma once

#include <AP_Param/AP_Param.h>

/// Fast param access via pointer helper
class Parameter
{
public:

    // init to param by name
    bool init(const char *name);

    // setters and getters
    bool set(float value);
    bool set_and_save(float value);
    bool get(float &value);

private:
    enum ap_var_type vtype;
    AP_Param *vp;
};


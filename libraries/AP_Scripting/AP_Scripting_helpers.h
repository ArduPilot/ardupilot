#pragma once

#include <AP_Param/AP_Param.h>
#include "lua/src/lua.hpp"

int lua_new_Parameter(lua_State *L);

/// Fast param access via pointer helper
class Parameter
{
public:

    // init to param by name
    bool init(const char *name);

    // init by token, to get the value of old params
    bool init_by_info(uint16_t key, uint32_t group_element, enum ap_var_type type);

    // setters and getters
    bool set(float value);
    bool set_and_save(float value);
    bool get(float &value);
    bool configured();
    bool set_default(float value);

private:
    enum ap_var_type vtype;
    AP_Param *vp;
};


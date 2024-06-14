#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "lua_boxed_numerics.h"
#include <AP_Scripting/lua_generated_bindings.h>


extern const AP_HAL::HAL& hal;

uint32_t coerce_to_uint32_t(lua_State *L, int arg) {
    { // userdata
        const uint32_t * ud = static_cast<uint32_t *>(luaL_testudata(L, arg, "uint32_t"));
        if (ud != nullptr) {
            return *ud;
        }
    }
    { // integer

        // if this assert fails, you will need to add an upper bounds
        // check that ensures the value isn't greater then UINT32_MAX
        static_assert(sizeof(lua_Number) == sizeof(uint32_t), "32 bit integers are only supported");

        int success;
        const lua_Integer v = lua_tointegerx(L, arg, &success);
        if (success) {
            return static_cast<uint32_t>(v);
        }
    }
    { // float
        int success;
        const lua_Number v = lua_tonumberx(L, arg, &success);
        if (success && v >= 0 && v <= float(UINT32_MAX)) {
            return static_cast<uint32_t>(v);
        }
    }
    // failure
    return luaL_argerror(L, arg, "Unable to coerce to uint32_t");
}

// the exposed constructor to lua calls to create a uint32_t
int lua_new_uint32_t(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    }

    new_uint32_t(L);
    *check_uint32_t(L, -1) = (args == 1) ? coerce_to_uint32_t(L, 1) : 0;
    return 1;
}

int uint32_t_toint(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t v = *check_uint32_t(L, 1);


    lua_pushinteger(L, static_cast<lua_Integer>(v));

    return 1;
}

int uint32_t_tofloat(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t v = *check_uint32_t(L, 1);


    lua_pushnumber(L, static_cast<lua_Number>(v));

    return 1;
}

int uint32_t___tostring(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t v = *check_uint32_t(L, 1);

    char buf[32];
    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%u", (unsigned)v);

    lua_pushstring(L, buf);

    return 1;
}

#endif  // AP_SCRIPTING_ENABLED

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

uint64_t coerce_to_uint64_t(lua_State *L, int arg) {
    { // uint64_t userdata
        const uint64_t * ud = static_cast<uint64_t *>(luaL_testudata(L, arg, "uint64_t"));
        if (ud != nullptr) {
            return *ud;
        }
    }
    { // integer
        int success;
        const lua_Integer v = lua_tointegerx(L, arg, &success);

        // Lua int maps to int32. However, because of the size difference negatives numbers wont come out correctly as they do for uint32
        if (success && v >= 0) {
            return static_cast<uint64_t>(v);
        }
    }
    { // uint32_t userdata
        const uint32_t * ud = static_cast<uint32_t *>(luaL_testudata(L, arg, "uint32_t"));
        if (ud != nullptr) {
            return static_cast<uint64_t>(*ud);
        }
    }
    { // float
        int success;
        const lua_Number v = lua_tonumberx(L, arg, &success);
        if (success && (v >= 0) && (v <= float(UINT64_MAX))) {
            return static_cast<uint64_t>(v);
        }
    }
    // failure
    return luaL_argerror(L, arg, "Unable to coerce to uint64_t");
}

// the exposed constructor to lua calls to create a uint32_t
int lua_new_uint32_t(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    }

    *new_uint32_t(L) = (args == 1) ? coerce_to_uint32_t(L, 1) : 0;
    return 1;
}

// the exposed constructor to lua calls to create a uint64_t
int lua_new_uint64_t(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    }

    uint64_t value = 0;
    switch (args) {
        case 0:
        default:
            // No arguments, init to 0
            break;
        case 1:
            // Single argument
            value = coerce_to_uint64_t(L, 1);
            break;

        case 2:
            // Two uint32 giving high and low half
            const uint64_t high = coerce_to_uint32_t(L, 1);
            const uint64_t low = coerce_to_uint32_t(L, 2);
            value = (high << 32) | low;
            break;
    }

    *new_uint64_t(L) = value;
    return 1;
}

int uint32_t_toint(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t v = *check_uint32_t(L, 1);

    lua_pushinteger(L, static_cast<lua_Integer>(v));

    return 1;
}

int uint64_t_toint(lua_State *L) {
    binding_argcheck(L, 1);

    const uint64_t v = *check_uint64_t(L, 1);

    if (v > INT32_MAX) {
        // uint64_t too large to convert to int return nill rather than giving error
        return 0;
    }

    lua_pushinteger(L, static_cast<lua_Integer>(v));

    return 1;
}

int uint32_t_tofloat(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t v = *check_uint32_t(L, 1);

    lua_pushnumber(L, static_cast<lua_Number>(v));

    return 1;
}

int uint64_t_tofloat(lua_State *L) {
    binding_argcheck(L, 1);

    const uint64_t v = *check_uint64_t(L, 1);

    lua_pushnumber(L, static_cast<lua_Number>(v));

    return 1;
}

int uint32_t___tostring(lua_State *L) {
    binding_argcheck(L, 1);

    const uint32_t v = *check_uint32_t(L, 1);

    char buf[32];
    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%lu", (unsigned long)v);

    lua_pushstring(L, buf);

    return 1;
}

int uint64_t___tostring(lua_State *L) {
    binding_argcheck(L, 1);

    const uint64_t v = *check_uint64_t(L, 1);

    char buf[32];
    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%llu", (unsigned long long)v);

    lua_pushstring(L, buf);

    return 1;
}

// Split uint64 into a high and low uint32
int uint64_t_split(lua_State *L) {
    binding_argcheck(L, 1);

    const uint64_t v = *check_uint64_t(L, 1);

    // high
    *new_uint32_t(L) = v >> 32;

    // low
    *new_uint32_t(L) = v & 0xFFFFFFFF;

    return 2;
}

#endif  // AP_SCRIPTING_ENABLED

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
        if (success && v >= 0) {
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

    *static_cast<uint32_t *>(lua_newuserdata(L, sizeof(uint32_t))) = (args == 1) ? coerce_to_uint32_t(L, 1) : 0;
    luaL_getmetatable(L, "uint32_t");
    lua_setmetatable(L, -2);
    return 1;
}

#define UINT32_T_BOX_OP(name, sym) \
    int uint32_t___##name(lua_State *L) { \
        binding_argcheck(L, 2); \
          \
        uint32_t v1 = coerce_to_uint32_t(L, 1); \
        uint32_t v2 = coerce_to_uint32_t(L, 2); \
          \
        new_uint32_t(L); \
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = v1 sym v2; \
        return 1; \
    }

UINT32_T_BOX_OP(add, +)
UINT32_T_BOX_OP(sub, -)
UINT32_T_BOX_OP(mul, *)
UINT32_T_BOX_OP(div, /)
UINT32_T_BOX_OP(mod, %)
UINT32_T_BOX_OP(band, &)
UINT32_T_BOX_OP(bor, |)
UINT32_T_BOX_OP(bxor, ^)
UINT32_T_BOX_OP(shl, <<)
UINT32_T_BOX_OP(shr, >>)

#define UINT32_T_BOX_OP_BOOL(name, sym) \
    int uint32_t___##name(lua_State *L) { \
        binding_argcheck(L, 2); \
          \
        uint32_t v1 = coerce_to_uint32_t(L, 1); \
        uint32_t v2 = coerce_to_uint32_t(L, 2); \
          \
        lua_pushboolean(L, v1 sym v2); \
        return 1; \
    }

UINT32_T_BOX_OP_BOOL(eq, ==)
UINT32_T_BOX_OP_BOOL(lt, <)
UINT32_T_BOX_OP_BOOL(le, <=)

#define UINT32_T_BOX_OP_UNARY(name, sym) \
    int uint32_t___##name(lua_State *L) { \
        binding_argcheck(L, 1); \
          \
        uint32_t v1 = coerce_to_uint32_t(L, 1); \
          \
        new_uint32_t(L); \
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = sym v1; \
        return 1; \
    }

// DO NOT SUPPORT UNARY NEGATION
UINT32_T_BOX_OP_UNARY(bnot, ~)

int uint32_t_toint(lua_State *L) {
    binding_argcheck(L, 1);

    uint32_t v = *static_cast<uint32_t *>(luaL_checkudata(L, 1, "uint32_t"));

    lua_pushinteger(L, static_cast<lua_Integer>(v));

    return 1;
}

int uint32_t_tofloat(lua_State *L) {
    binding_argcheck(L, 1);

    uint32_t v = *static_cast<uint32_t *>(luaL_checkudata(L, 1, "uint32_t"));

    lua_pushnumber(L, static_cast<lua_Number>(v));

    return 1;
}

int uint32_t___tostring(lua_State *L) {
    binding_argcheck(L, 1);

    uint32_t v = *static_cast<uint32_t *>(luaL_checkudata(L, 1, "uint32_t"));

    char buf[32];
    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%u", (unsigned)v);

    lua_pushstring(L, buf);

    return 1;
}

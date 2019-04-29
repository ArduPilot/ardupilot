#include <AP_HAL/AP_HAL.h>
#include "lua_boxed_numerics.h"


extern const AP_HAL::HAL& hal;

int new_uint32_t(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    *static_cast<uint32_t *>(lua_newuserdata(L, sizeof(uint32_t))) = 0; // allocated memory is already zerod, no need to manipulate this
    luaL_getmetatable(L, "uint32_t");
    lua_setmetatable(L, -2);
    return 1;
}

uint32_t * check_uint32_t(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "uint32_t");
    return static_cast<uint32_t *>(data);
}

static uint32_t coerce_to_uint32_t(lua_State *L, int arg) {
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
        if (success && v >= 0 && v <= UINT32_MAX) {
            return static_cast<uint32_t>(v);
        }
    }
    // failure
    return luaL_argerror(L, arg, "Unable to coerce to uint32_t");
}

#define UINT32_T_BOX_OP(name, sym) \
    static int uint32_t___##name(lua_State *L) { \
        const int args = lua_gettop(L); \
        if (args > 2) { \
            return luaL_argerror(L, args, "too many arguments"); \
        } else if (args < 2) { \
            return luaL_argerror(L, args, "too few arguments"); \
        } \
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
UINT32_T_BOX_OP(idiv, /)
UINT32_T_BOX_OP(band, &)
UINT32_T_BOX_OP(bor, |)
UINT32_T_BOX_OP(bxor, ^)
UINT32_T_BOX_OP(shl, <<)
UINT32_T_BOX_OP(shr, >>)

#define UINT32_T_BOX_OP_BOOL(name, sym) \
    static int uint32_t___##name(lua_State *L) { \
        const int args = lua_gettop(L); \
        luaL_checkstack(L, 1, "Out of stack"); \
        if (args > 2) { \
            return luaL_argerror(L, args, "too many arguments"); \
        } else if (args < 2) { \
            return luaL_argerror(L, args, "too few arguments"); \
        } \
          \
        uint32_t v1 = coerce_to_uint32_t(L, 1); \
        uint32_t v2 = coerce_to_uint32_t(L, 2); \
          \
        lua_pushboolean(L, v1 sym v2); \
        return 1; \
    }

UINT32_T_BOX_OP_BOOL(eq, =)
UINT32_T_BOX_OP_BOOL(lt, <)
UINT32_T_BOX_OP_BOOL(le, <=)

#define UINT32_T_BOX_OP_UNARY(name, sym) \
    static int uint32_t___##name(lua_State *L) { \
        const int args = lua_gettop(L); \
        luaL_checkstack(L, 1, "Out of stack"); \
        if (args != 1) { \
            return luaL_argerror(L, args, "Expected 1 argument"); \
        } \
          \
        uint32_t v1 = coerce_to_uint32_t(L, 1); \
          \
        new_uint32_t(L); \
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = sym v1; \
        return 1; \
    }

// DO NOT SUPPORT UNARY NEGATION
UINT32_T_BOX_OP_UNARY(bnot, ~)

static int uint32_t_toint(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 1) {
        return luaL_argerror(L, args, "Expected 1 argument");
    }

    uint32_t v = *static_cast<uint32_t *>(luaL_checkudata(L, 1, "uint32_t"));

    lua_pushinteger(L, static_cast<lua_Integer>(v));

    return 1;
}

static int uint32_t_tofloat(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 1) {
        return luaL_argerror(L, args, "Expected 1 argument");
    }

    uint32_t v = *static_cast<uint32_t *>(luaL_checkudata(L, 1, "uint32_t"));

    lua_pushnumber(L, static_cast<lua_Number>(v));

    return 1;
}

static int uint32_t___tostring(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 1) {
        return luaL_argerror(L, args, "Expected 1 argument");
    }

    uint32_t v = *static_cast<uint32_t *>(luaL_checkudata(L, 1, "uint32_t"));

    char buf[32];
    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%u", (unsigned)v);

    lua_pushstring(L, buf);

    return 1;
}

const luaL_Reg uint32_t_meta[] = {
    {"__add", uint32_t___add},
    {"__sub", uint32_t___sub},
    {"__mul", uint32_t___mul},
    {"__div", uint32_t___div},
    {"__mod", uint32_t___mod},
    {"__idiv", uint32_t___idiv},
    {"__band", uint32_t___band},
    {"__bor", uint32_t___bor},
    {"__bxor", uint32_t___bxor},
    {"__shl", uint32_t___shl},
    {"__shr", uint32_t___shr},
    {"__shr", uint32_t___shr},
    {"__eq", uint32_t___eq},
    {"__lt", uint32_t___lt},
    {"__le", uint32_t___le},
    {"__bnot", uint32_t___bnot},
    {"__tostring", uint32_t___tostring},
    {"toint", uint32_t_toint},
    {"tofloat", uint32_t_tofloat},
    {NULL, NULL}
};

void load_boxed_numerics(lua_State *L) {
    luaL_checkstack(L, 5, "Out of stack");
    luaL_newmetatable(L, "uint32_t");
    luaL_setfuncs(L, uint32_t_meta, 0);
    lua_pushstring(L, "__index");
    lua_pushvalue(L, -2);
    lua_settable(L, -3);
    lua_pop(L, 1);
}

void load_boxed_numerics_sandbox(lua_State *L) {
    // if there are ever more drivers then move to a table based solution
    lua_pushstring(L, "uint32_t");
    lua_pushcfunction(L, new_uint32_t);
    lua_settable(L, -3);
}

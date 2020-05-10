#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/HAL.h>

#include "lua_bindings.h"

#include "lua_boxed_numerics.h"
#include <AP_Scripting/lua_generated_bindings.h>

extern const AP_HAL::HAL& hal;

int check_arguments(lua_State *L, int expected_arguments, const char *fn_name);
int check_arguments(lua_State *L, int expected_arguments, const char *fn_name) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
    if (expected_arguments < 0) {
       AP_HAL::panic("Lua: Attempted to check for negative arguments");
    }
#endif

    const int args = lua_gettop(L);
    if (args != expected_arguments) {
        return luaL_error(L, "%s expected %d arguments got %d", fn_name, expected_arguments, args);
    }
    return 0;
}

// millis
static int lua_millis(lua_State *L) {
    check_arguments(L, 0, "millis");

    new_uint32_t(L);
    *check_uint32_t(L, -1) = AP_HAL::millis();

    return 1;
}

static const luaL_Reg global_functions[] =
{
    {"millis", lua_millis},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *L) {
    luaL_setfuncs(L, global_functions, 0);
}


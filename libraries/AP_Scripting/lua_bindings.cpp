#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/HAL.h>

#include "lua_bindings.h"

#include "lua_boxed_numerics.h"
#include "lua_generated_bindings.h"

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

// servo binding

static int lua_servo_set_output_pwm(lua_State *L) {
    check_arguments(L, 2, "set_output_pwm");

    const SRV_Channel::Aux_servo_function_t servo_function = (SRV_Channel::Aux_servo_function_t)luaL_checkinteger(L, -2);
    luaL_argcheck(L, ((servo_function >= SRV_Channel::Aux_servo_function_t::k_scripting1) &&
                      (servo_function <= SRV_Channel::Aux_servo_function_t::k_scripting16)),
                  2, "function out of range");

    const int output = luaL_checknumber(L, -1);
    luaL_argcheck(L, ((output >= 0) && (output <= UINT16_MAX)), 2, "output out of range");

    SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)servo_function, output);

    return 0;
}

// millis
static int lua_millis(lua_State *L) {
    check_arguments(L, 0, "millis");

    new_uint32_t(L);
    *check_uint32_t(L, -1) = AP_HAL::millis();

    return 1;
}

static const luaL_Reg servo_functions[] =
{
    {"set_output_pwm", lua_servo_set_output_pwm},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *L) {
    luaL_newlib(L, servo_functions);
    lua_setglobal(L, "servo");

    load_generated_bindings(L);

    lua_pushcfunction(L, lua_millis);
    lua_setglobal(L, "millis");
}


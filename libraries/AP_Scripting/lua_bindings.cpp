#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "lua_bindings.h"

int check_arguments(lua_State *L, int expected_arguments, const char *fn_name) {
    const int args = lua_gettop(L);
    if (args != expected_arguments) {
        return luaL_error(L, "%s expected %d arguments got %d", fn_name, expected_arguments, args);
    }
    return 0;
}

// GCS binding

int lua_gcs_send_text(lua_State *L) {
    check_arguments(L, 1, "send_text");

    const char* str = lua_tostring(L, 1);

    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}

static const luaL_Reg gcs_functions[] =
{
    {"send_text", lua_gcs_send_text},
    {NULL, NULL}
};

// servo binding

int lua_servo_set_output_pwm(lua_State *L) {
    check_arguments(L, 2, "set_output_pwm");

    const SRV_Channel::Aux_servo_function_t servo_function = (SRV_Channel::Aux_servo_function_t)luaL_checkinteger(L, 1);
    luaL_argcheck(L, ((servo_function >= SRV_Channel::Aux_servo_function_t::k_scripting1) &&
                      (servo_function <= SRV_Channel::Aux_servo_function_t::k_scripting16)),
                  2, "function out of range");

    const int output = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((output >= 0) && (output <= UINT16_MAX)), 2, "output out of range");

    SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)servo_function, output);

    return 0;
}

static const luaL_Reg servo_functions[] =
{
    {"set_output_pwm", lua_servo_set_output_pwm},
    {NULL, NULL}
};

// all bindings

void load_lua_bindings(lua_State *state) {
    luaL_newlib(state, gcs_functions);
    lua_setglobal(state, "gcs");
    luaL_newlib(state, servo_functions);
    lua_setglobal(state, "servo");
}


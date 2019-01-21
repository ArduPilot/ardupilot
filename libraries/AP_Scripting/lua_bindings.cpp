#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "lua_bindings.h"

int lua_gcs_send_text(lua_State *state) {
    const char* str = lua_tostring(state, 1);
    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}

static const luaL_Reg gcs_functions[] =
{
    {"send_text", lua_gcs_send_text},
    {NULL, NULL}
};

int lua_servo_set_output_pwm(lua_State *state) {
    int servo_function = luaL_checkinteger(state, -2);
    int output_value = luaL_checknumber(state, -1);

    // range check the output function
    if ((servo_function < SRV_Channel::Aux_servo_function_t::k_scripting1) ||
        (servo_function > SRV_Channel::Aux_servo_function_t::k_scripting16)) {
        return luaL_error(state, "Servo function (%d) is not a scriptable output", servo_function);
    }

    if (output_value > (int)UINT16_MAX) {
        return luaL_error(state, "Servo range (%d) is out of range", output_value);
    }

    SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)servo_function, output_value);
    return 0;
}

static const luaL_Reg servo_functions[] =
{
    {"set_output_pwm", lua_servo_set_output_pwm},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *state) {
    luaL_newlib(state, gcs_functions);
    lua_setglobal(state, "gcs");
    luaL_newlib(state, servo_functions);
    lua_setglobal(state, "servo");
}


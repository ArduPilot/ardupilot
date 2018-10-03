#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

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

void load_lua_bindings(lua_State *state) {
    luaL_newlib(state, gcs_functions);
    lua_setglobal(state, "gcs");
}

void hook(lua_State *L, lua_Debug *ar) {
    gcs().send_text(MAV_SEVERITY_INFO, "got a debug hook");
    lua_error(L);
}

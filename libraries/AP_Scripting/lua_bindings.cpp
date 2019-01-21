#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "lua_bindings.h"

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

// GCS binding

int lua_gcs_send_text(lua_State *L);
int lua_gcs_send_text(lua_State *L) {
    check_arguments(L, 1, "send_text");

    const char* str = lua_tostring(L, -1);

    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}

static const luaL_Reg gcs_functions[] =
{
    {"send_text", lua_gcs_send_text},
    {NULL, NULL}
};

// servo binding

int lua_servo_set_output_pwm(lua_State *L);
int lua_servo_set_output_pwm(lua_State *L) {
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

static const luaL_Reg servo_functions[] =
{
    {"set_output_pwm", lua_servo_set_output_pwm},
    {NULL, NULL}
};

// location stuff
static int new_location(lua_State *L) {
    Location *loc = (Location *)lua_newuserdata(L, sizeof(Location));
    luaL_getmetatable(L, "location");
    lua_setmetatable(L, -2);
    memset((void *)loc, 0, sizeof(Location));
    return 1;
}

static Location *check_location(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "location");
    luaL_argcheck(L, data != NULL, arg, "`location` expected");
    return (Location *)data;
}

static int location_lat(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushinteger(L, loc->lat);
            return 1;
        case 2: // set
            {
                const int32_t lat = luaL_checkinteger(L, 2);
                luaL_argcheck(L, ((lat >= -900000000) && (lat <= 900000000)), 2, "invalid latitude out of range");
                loc->lat = lat;
                return 0;
            }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_long(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushinteger(L, loc->lng);
            return 1;
        case 2: // set
            {
                const int32_t lng = luaL_checkinteger(L, 2);
                luaL_argcheck(L, ((lng >= -1800000000) && (lng <= 1800000000)), 2, "invalid longitude out of range");
                loc->lng = lng;
                return 0;
            }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_alt(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushinteger(L, loc->alt);
            return 1;
        case 2: // set
            loc->alt = luaL_checknumber(L, 2);
            return 0;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_relative_alt(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushboolean(L, loc->relative_alt);
            return 1;
        case 2: // set
            loc->relative_alt = lua_toboolean(L, 2);
            return 0;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_loiter_ccw(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushboolean(L, loc->loiter_ccw);
            return 1;
        case 2: // set
            loc->loiter_ccw = lua_toboolean(L, 2);
            return 0;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_terrain_alt(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushboolean(L, loc->terrain_alt);
            return 1;
        case 2: // set
            loc->terrain_alt = lua_toboolean(L, 2);
            return 0;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_origin_alt(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushboolean(L, loc->origin_alt);
            return 1;
        case 2: // set
            loc->origin_alt = lua_toboolean(L, 2);
            return 0;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_loiter_xtrack(lua_State *L) {
    Location *loc = check_location(L, 1);
    switch(lua_gettop(L)) {
        case 1: // access
            lua_pushboolean(L, loc->loiter_xtrack);
            return 1;
        case 2: // set
            loc->loiter_xtrack = lua_toboolean(L, 2);
            return 0;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int location_tostring(lua_State *L) {
    Location *loc = check_location(L, -1);
    char buf[64] = {};
    snprintf(buf, sizeof(buf),
                  "Loc(%ld.%07ld %c, %ld.%07ld %c, %.02f)",
                  labs(loc->lat / 10000000),
                  labs(loc->lat % 10000000),
                  loc->lat > 0 ? 'N' : 'S',
                  labs(loc->lng / 10000000),
                  labs(loc->lng % 10000000),
                  loc->lng > 0 ? 'E' : 'W',
                  loc->alt * 1e-2);
    lua_pushstring(L, buf);
    return 1;
}

static int location_distance(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 2) {
        return luaL_argerror(L, args, "too many arguments");
    }

    lua_pushnumber(L, get_distance(*check_location(L, -2),
                                   *check_location(L, -1)));

    return 1;
}

static int location_project(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 3) {
        return luaL_argerror(L, args, "too many arguments");
    }

    location_update(*check_location(L, -3),
                    luaL_checknumber(L, -2),
                    luaL_checknumber(L, -1));

    lua_pop(L, 2);

    return 1;
}

static int location_passed_point(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 3) {
        return luaL_argerror(L, args, "too many arguments");
    }

    lua_pushboolean(L, location_passed_point(*check_location(L, -3),
                                             *check_location(L, -2),
                                             *check_location(L, -1)));

    return 1;
}

static int location_path_proportion(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 3) {
        return luaL_argerror(L, args, "too many arguments");
    }

    lua_pushnumber(L, location_path_proportion(*check_location(L, -3),
                                               *check_location(L, -2),
                                               *check_location(L, -1)));

    return 1;
}

static int location_offset(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 3) {
        return luaL_argerror(L, args, "too many arguments");
    }

    location_offset(*check_location(L, -3),
                    luaL_checknumber(L, -2),
                    luaL_checknumber(L, -1));

    lua_pop(L, 2);

    return 1;
}

static int location_equal(lua_State *L) {
    const int args = lua_gettop(L);
    if (args != 2) {
        return luaL_argerror(L, args, "too many arguments");
    }

    Location *l2 = check_location(L, -2);
    Location *l1 = check_location(L, -1);

    lua_pushboolean(L, locations_are_same(*l1, *l2));

    return 1;
}

static const luaL_Reg locLib[] = {
  {"new", new_location},
  {NULL, NULL}
};

static const luaL_Reg locationMeta[] = {
  {"lat", location_lat},
  {"long", location_long},
  {"alt", location_alt},
  {"relative_alt", location_relative_alt},
  {"loiter_ccw", location_loiter_ccw},
  {"terrain_alt", location_terrain_alt},
  {"origin_alt", location_origin_alt},
  {"loiter_xtrack", location_loiter_xtrack},
  {"distance", location_distance},
  {"project", location_project},
  {"passed_point", location_passed_point},
  {"path_proportion", location_path_proportion},
  {"offset", location_offset},
  {"__tostring", location_tostring},
  {"__eq", location_equal},
  {NULL, NULL}
};

static int ahrs_position(lua_State *L) {
    check_arguments(L, 1, "ahrs:position");

    new_location(L);
    Location *loc = check_location(L, -1);
    AP::ahrs().get_position(*loc);

    return 1;
}

static int ahrs_get_home(lua_State *L) {
    check_arguments(L, 1, "ahrs:home");

    new_location(L);
    Location *loc = check_location(L, -1);
    *loc = AP::ahrs().get_home();

    return 1;
}

static const luaL_Reg ahrsMeta[] = {
  {"position", ahrs_position},
  {"home", ahrs_get_home},
  {NULL, NULL}
};

// all bindings

void load_lua_bindings(lua_State *L) {
    luaL_newlib(L, gcs_functions);
    lua_setglobal(L, "gcs");

    luaL_newlib(L, servo_functions);
    lua_setglobal(L, "servo");

    // location metatable
    luaL_newmetatable(L, "location");
    luaL_setfuncs(L, locationMeta, 0);
    lua_pushstring(L, "__index");
    lua_pushvalue(L, -2);
    lua_settable(L, -3);
    lua_pop(L, 1);
    luaL_newlib(L, locLib);
    lua_setglobal(L, "loc");

    // ahrs metatable
    luaL_newmetatable(L, "ahrs");
    luaL_setfuncs(L, ahrsMeta, 0);
    lua_pushstring(L, "__index");
    lua_pushvalue(L, -2);
    lua_settable(L, -3);
    lua_pop(L, 1);

    // ahrs userdata
    lua_newuserdata(L, 0); // lose the pointer, we don't really care about it
    luaL_getmetatable(L, "ahrs");
    lua_setmetatable(L, -2);
    lua_setglobal(L, "ahrs");
}


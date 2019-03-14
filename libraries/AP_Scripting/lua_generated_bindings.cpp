// auto generated bindings, don't manually edit
#include "lua_generated_bindings.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>


int new_Vector3f(lua_State *L) {
    Vector3f *ud = (Vector3f *)lua_newuserdata(L, sizeof(Vector3f));
    new (ud) Vector3f();
    luaL_getmetatable(L, "Vector3f");
    lua_setmetatable(L, -2);
    return 1;
}

int new_Location(lua_State *L) {
    Location *ud = (Location *)lua_newuserdata(L, sizeof(Location));
    new (ud) Location();
    luaL_getmetatable(L, "Location");
    lua_setmetatable(L, -2);
    return 1;
}

Vector3f * check_Vector3f(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "Vector3f");
    return (Vector3f *)data;
}

Location * check_Location(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "Location");
    return (Location *)data;
}

int Vector3f_z(lua_State *L) {
    Vector3f *ud = check_Vector3f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->z);
            return 1;
        case 2: {
            const float data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((data_2 >= -FLT_MAX) && (data_2 <= FLT_MAX)), 2, "z out of range");
            ud->z = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Vector3f_y(lua_State *L) {
    Vector3f *ud = check_Vector3f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->y);
            return 1;
        case 2: {
            const float data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((data_2 >= -FLT_MAX) && (data_2 <= FLT_MAX)), 2, "y out of range");
            ud->y = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Vector3f_x(lua_State *L) {
    Vector3f *ud = check_Vector3f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->x);
            return 1;
        case 2: {
            const float data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((data_2 >= -FLT_MAX) && (data_2 <= FLT_MAX)), 2, "x out of range");
            ud->x = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_loiter_xtrack(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->loiter_xtrack);
            return 1;
        case 2: {
            const bool data_2 = lua_toboolean(L, 2);
            ud->loiter_xtrack = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_origin_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->origin_alt);
            return 1;
        case 2: {
            const bool data_2 = lua_toboolean(L, 2);
            ud->origin_alt = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_terrain_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->terrain_alt);
            return 1;
        case 2: {
            const bool data_2 = lua_toboolean(L, 2);
            ud->terrain_alt = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_relative_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->relative_alt);
            return 1;
        case 2: {
            const bool data_2 = lua_toboolean(L, 2);
            ud->relative_alt = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_lng(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->lng);
            return 1;
        case 2: {
            const int32_t data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((data_2 >= -1800000000) && (data_2 <= 1800000000)), 2, "lng out of range");
            ud->lng = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_lat(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->lat);
            return 1;
        case 2: {
            const int32_t data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((data_2 >= -900000000) && (data_2 <= 900000000)), 2, "lat out of range");
            ud->lat = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

int Location_get_vector_from_origin_NEU(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 2) {
        return luaL_argerror(L, args, "too few arguments");
    }

    Location * ud = check_Location(L, 1);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    const bool data = ud->get_vector_from_origin_NEU(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

int Location_offset(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 3) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 3) {
        return luaL_argerror(L, args, "too few arguments");
    }

    Location * ud = check_Location(L, 1);
    const float data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((data_2 >= -FLT_MAX) && (data_2 <= FLT_MAX)), 2, "argument out of range");
    const float data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((data_3 >= -FLT_MAX) && (data_3 <= FLT_MAX)), 3, "argument out of range");
    ud->offset(
            data_2,
            data_3);

    return 0;
}

int Location_get_distance(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 2) {
        return luaL_argerror(L, args, "too few arguments");
    }

    Location * ud = check_Location(L, 1);
    Location & data_2 = *check_Location(L, 2);
    const float data = ud->get_distance(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

const luaL_Reg Vector3f_meta[] = {
    {"z", Vector3f_z},
    {"y", Vector3f_y},
    {"x", Vector3f_x},
    {NULL, NULL}
};

const luaL_Reg Location_meta[] = {
    {"loiter_xtrack", Location_loiter_xtrack},
    {"origin_alt", Location_origin_alt},
    {"terrain_alt", Location_terrain_alt},
    {"relative_alt", Location_relative_alt},
    {"lng", Location_lng},
    {"lat", Location_lat},
    {"get_vector_from_origin_NEU", Location_get_vector_from_origin_NEU},
    {"offset", Location_offset},
    {"get_distance", Location_get_distance},
    {NULL, NULL}
};

int notify_play_tune(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 2) {
        return luaL_argerror(L, args, "too few arguments");
    }

    luaL_checkudata(L, 1, "notify");
    const char * data_2 = luaL_checkstring(L, 2);
    AP::notify().play_tune(
            data_2);

    return 0;
}

int ahrs_get_home(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 1) {
        return luaL_argerror(L, args, "too few arguments");
    }

    luaL_checkudata(L, 1, "ahrs");
    const Location &data = AP::ahrs().get_home(
);

    new_Location(L);
    *check_Location(L, -1) = data;
    return 1;
}

int ahrs_get_position(lua_State *L) {
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 2) {
        return luaL_argerror(L, args, "too few arguments");
    }

    luaL_checkudata(L, 1, "ahrs");
    Location & data_2 = *check_Location(L, 2);
    const bool data = AP::ahrs().get_position(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

const luaL_Reg notify_meta[] = {
    {"play_tune", notify_play_tune},
    {NULL, NULL}
};

const luaL_Reg ahrs_meta[] = {
    {"get_home", ahrs_get_home},
    {"get_position", ahrs_get_position},
    {NULL, NULL}
};

const struct userdata_fun {
    const char *name;
    const luaL_Reg *reg;
} userdata_fun[] = {
    {"Vector3f", Vector3f_meta},
    {"Location", Location_meta},
};

const struct singleton_fun {
    const char *name;
    const luaL_Reg *reg;
} singleton_fun[] = {
    {"notify", notify_meta},
    {"ahrs", ahrs_meta},
};

void load_generated_bindings(lua_State *L) {
    // userdata metatables
    for (uint32_t i = 0; i < ARRAY_SIZE(userdata_fun); i++) {
        luaL_newmetatable(L, userdata_fun[i].name);
        luaL_setfuncs(L, userdata_fun[i].reg, 0);
        lua_pushstring(L, "__index");
        lua_pushvalue(L, -2);
        lua_settable(L, -3);
        lua_pop(L, 1);
    }

    // singleton metatables
    for (uint32_t i = 0; i < ARRAY_SIZE(singleton_fun); i++) {
        luaL_newmetatable(L, singleton_fun[i].name);
        luaL_setfuncs(L, singleton_fun[i].reg, 0);
        lua_pushstring(L, "__index");
        lua_pushvalue(L, -2);
        lua_settable(L, -3);
        lua_pop(L, 1);
        lua_newuserdata(L, 0);
        luaL_getmetatable(L, singleton_fun[i].name);
        lua_setmetatable(L, -2);
        lua_setglobal(L, singleton_fun[i].name);
    }
}

const char *singletons[] = {
    "notify",
    "ahrs",
};

const struct userdata {
    const char *name;
    const lua_CFunction fun;
} new_userdata[] = {
    {"Vector3f", new_Vector3f},
    {"Location", new_Location},
};

void load_generated_sandbox(lua_State *L) {
    for (uint32_t i = 0; i < ARRAY_SIZE(singletons); i++) {
        lua_pushstring(L, singletons[i]);
        lua_getglobal(L, singletons[i]);
        lua_settable(L, -3);
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(new_userdata); i++) {
        lua_pushstring(L, new_userdata[i].name);
        lua_pushcfunction(L, new_userdata[i].fun);
        lua_settable(L, -3);
    }
}

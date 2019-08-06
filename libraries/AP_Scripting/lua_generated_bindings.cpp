// auto generated bindings, don't manually edit
#include "lua_generated_bindings.h"
#include "lua_boxed_numerics.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>


static int binding_argcheck(lua_State *L, int expected_arg_count) {
    const int args = lua_gettop(L);
    if (args > expected_arg_count) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < expected_arg_count) {
        return luaL_argerror(L, args, "too few arguments");
    }
    return 0;
}

int new_Vector2f(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(Vector2f));
    memset(ud, 0, sizeof(Vector2f));
    new (ud) Vector2f();
    luaL_getmetatable(L, "Vector2f");
    lua_setmetatable(L, -2);
    return 1;
}

int new_Vector3f(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(Vector3f));
    memset(ud, 0, sizeof(Vector3f));
    new (ud) Vector3f();
    luaL_getmetatable(L, "Vector3f");
    lua_setmetatable(L, -2);
    return 1;
}

int new_Location(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(Location));
    memset(ud, 0, sizeof(Location));
    new (ud) Location();
    luaL_getmetatable(L, "Location");
    lua_setmetatable(L, -2);
    return 1;
}

Vector2f * check_Vector2f(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "Vector2f");
    return (Vector2f *)data;
}

Vector3f * check_Vector3f(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "Vector3f");
    return (Vector3f *)data;
}

Location * check_Location(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "Location");
    return (Location *)data;
}

static int Vector2f_y(lua_State *L) {
    Vector2f *ud = check_Vector2f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->y);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "y out of range");
            const float data_2 = raw_data_2;
            ud->y = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Vector2f_x(lua_State *L) {
    Vector2f *ud = check_Vector2f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->x);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "x out of range");
            const float data_2 = raw_data_2;
            ud->x = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Vector3f_z(lua_State *L) {
    Vector3f *ud = check_Vector3f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->z);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "z out of range");
            const float data_2 = raw_data_2;
            ud->z = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Vector3f_y(lua_State *L) {
    Vector3f *ud = check_Vector3f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->y);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "y out of range");
            const float data_2 = raw_data_2;
            ud->y = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Vector3f_x(lua_State *L) {
    Vector3f *ud = check_Vector3f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->x);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "x out of range");
            const float data_2 = raw_data_2;
            ud->x = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Location_loiter_xtrack(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->loiter_xtrack);
            return 1;
        case 2: {
            const bool data_2 = static_cast<bool>(lua_toboolean(L, 2));
            ud->loiter_xtrack = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Location_origin_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->origin_alt);
            return 1;
        case 2: {
            const bool data_2 = static_cast<bool>(lua_toboolean(L, 2));
            ud->origin_alt = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Location_terrain_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->terrain_alt);
            return 1;
        case 2: {
            const bool data_2 = static_cast<bool>(lua_toboolean(L, 2));
            ud->terrain_alt = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Location_relative_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->relative_alt);
            return 1;
        case 2: {
            const bool data_2 = static_cast<bool>(lua_toboolean(L, 2));
            ud->relative_alt = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Location_lng(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->lng);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-1800000000, INT32_MIN)) && (raw_data_2 <= MIN(1800000000, INT32_MAX))), 2, "lng out of range");
            const int32_t data_2 = raw_data_2;
            ud->lng = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Location_lat(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->lat);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-900000000, INT32_MIN)) && (raw_data_2 <= MIN(900000000, INT32_MAX))), 2, "lat out of range");
            const int32_t data_2 = raw_data_2;
            ud->lat = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Vector2f_is_zero(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    ud->is_zero(
        );

    return 0;
}

static int Vector2f_is_inf(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    ud->is_inf(
        );

    return 0;
}

static int Vector2f_is_nan(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    ud->is_nan(
        );

    return 0;
}

static int Vector2f_normalize(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    ud->normalize(
        );

    return 0;
}

static int Vector2f_length(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const float data = ud->length(
        );

    lua_pushnumber(L, data);
    return 1;
}

static int Vector2f___add(lua_State *L) {
    binding_argcheck(L, 2);
    Vector2f *ud = check_Vector2f(L, 1);
    Vector2f *ud2 = check_Vector2f(L, 2);
    new_Vector2f(L);
    *check_Vector2f(L, -1) = *ud + *ud2;;
    return 1;
}

static int Vector2f___sub(lua_State *L) {
    binding_argcheck(L, 2);
    Vector2f *ud = check_Vector2f(L, 1);
    Vector2f *ud2 = check_Vector2f(L, 2);
    new_Vector2f(L);
    *check_Vector2f(L, -1) = *ud - *ud2;;
    return 1;
}

static int Vector3f_is_zero(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    ud->is_zero(
        );

    return 0;
}

static int Vector3f_is_inf(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    ud->is_inf(
        );

    return 0;
}

static int Vector3f_is_nan(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    ud->is_nan(
        );

    return 0;
}

static int Vector3f_normalize(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    ud->normalize(
        );

    return 0;
}

static int Vector3f_length(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const float data = ud->length(
        );

    lua_pushnumber(L, data);
    return 1;
}

static int Vector3f___add(lua_State *L) {
    binding_argcheck(L, 2);
    Vector3f *ud = check_Vector3f(L, 1);
    Vector3f *ud2 = check_Vector3f(L, 2);
    new_Vector3f(L);
    *check_Vector3f(L, -1) = *ud + *ud2;;
    return 1;
}

static int Vector3f___sub(lua_State *L) {
    binding_argcheck(L, 2);
    Vector3f *ud = check_Vector3f(L, 1);
    Vector3f *ud2 = check_Vector3f(L, 2);
    new_Vector3f(L);
    *check_Vector3f(L, -1) = *ud - *ud2;;
    return 1;
}

static int Location_get_vector_from_origin_NEU(lua_State *L) {
    // 1 Vector3f 14 : 6
    binding_argcheck(L, 2);
    Location * ud = check_Location(L, 1);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    const bool data = ud->get_vector_from_origin_NEU(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int Location_offset(lua_State *L) {
    // 1 float 13 : 8
    // 2 float 13 : 11
    binding_argcheck(L, 3);
    Location * ud = check_Location(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "argument out of range");
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_3 <= MIN(FLT_MAX, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    ud->offset(
            data_2,
            data_3);

    return 0;
}

static int Location_get_distance(lua_State *L) {
    // 1 Location 12 : 6
    binding_argcheck(L, 2);
    Location * ud = check_Location(L, 1);
    Location & data_2 = *check_Location(L, 2);
    const float data = ud->get_distance(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

const luaL_Reg Vector2f_meta[] = {
    {"y", Vector2f_y},
    {"x", Vector2f_x},
    {"is_zero", Vector2f_is_zero},
    {"is_inf", Vector2f_is_inf},
    {"is_nan", Vector2f_is_nan},
    {"normalize", Vector2f_normalize},
    {"length", Vector2f_length},
    {"__add", Vector2f___add},
    {"__sub", Vector2f___sub},
    {NULL, NULL}
};

const luaL_Reg Vector3f_meta[] = {
    {"z", Vector3f_z},
    {"y", Vector3f_y},
    {"x", Vector3f_x},
    {"is_zero", Vector3f_is_zero},
    {"is_inf", Vector3f_is_inf},
    {"is_nan", Vector3f_is_nan},
    {"normalize", Vector3f_normalize},
    {"length", Vector3f_length},
    {"__add", Vector3f___add},
    {"__sub", Vector3f___sub},
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

static int GCS_send_text(lua_State *L) {
    // 1 MAV_SEVERITY 126 : 8
    // 2 enum 126 : 9
    binding_argcheck(L, 3);
    GCS * ud = GCS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 3, "gcs not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAV_SEVERITY_EMERGENCY) && (raw_data_2 <= MAV_SEVERITY_DEBUG)), 2, "argument out of range");
    const MAV_SEVERITY data_2 = static_cast<MAV_SEVERITY>(raw_data_2);
    const char * data_3 = luaL_checkstring(L, 3);
    ud->send_text(
            data_2,
            data_3);

    return 0;
}

static int AP_Relay_toggle(lua_State *L) {
    // 1 uint8_t 122 : 8
    binding_argcheck(L, 2);
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "relay not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->toggle(
            data_2);

    return 0;
}

static int AP_Relay_enabled(lua_State *L) {
    // 1 uint8_t 121 : 8
    binding_argcheck(L, 2);
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "relay not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->enabled(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Relay_off(lua_State *L) {
    // 1 uint8_t 120 : 8
    binding_argcheck(L, 2);
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "relay not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->off(
            data_2);

    return 0;
}

static int AP_Relay_on(lua_State *L) {
    // 1 uint8_t 119 : 8
    binding_argcheck(L, 2);
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "relay not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->on(
            data_2);

    return 0;
}

static int AP_Terrain_height_above_terrain(lua_State *L) {
    // 1 float 114 : 6
    // 2 bool 114 : 7
    binding_argcheck(L, 2);
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "terrain not supported on this firmware");
    }

    float data_5002 = {};
    const bool data_3 = static_cast<bool>(lua_toboolean(L, 3));
    const bool data = ud->height_above_terrain(
            data_5002,
            data_3);

    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_Terrain_height_relative_home_equivalent(lua_State *L) {
    // 1 float 113 : 8
    // 2 float 113 : 9
    // 3 bool 113 : 10
    binding_argcheck(L, 3);
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 3, "terrain not supported on this firmware");
    }

    const float raw_data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_2 <= MIN(FLT_MAX, INFINITY))), 2, "argument out of range");
    const float data_2 = raw_data_2;
    float data_5003 = {};
    const bool data_4 = static_cast<bool>(lua_toboolean(L, 4));
    const bool data = ud->height_relative_home_equivalent(
            data_2,
            data_5003,
            data_4);

    if (data) {
        lua_pushnumber(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_Terrain_height_terrain_difference_home(lua_State *L) {
    // 1 float 112 : 6
    // 2 bool 112 : 7
    binding_argcheck(L, 2);
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "terrain not supported on this firmware");
    }

    float data_5002 = {};
    const bool data_3 = static_cast<bool>(lua_toboolean(L, 3));
    const bool data = ud->height_terrain_difference_home(
            data_5002,
            data_3);

    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_Terrain_height_amsl(lua_State *L) {
    // 1 Location 111 : 6
    // 2 float 111 : 7
    // 3 bool 111 : 8
    binding_argcheck(L, 3);
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 3, "terrain not supported on this firmware");
    }

    Location & data_2 = *check_Location(L, 2);
    float data_5003 = {};
    const bool data_4 = static_cast<bool>(lua_toboolean(L, 4));
    const bool data = ud->height_amsl(
            data_2,
            data_5003,
            data_4);

    if (data) {
        lua_pushnumber(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_Terrain_status(lua_State *L) {
    binding_argcheck(L, 1);
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    const uint8_t data = ud->status(
        );

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Terrain_enabled(lua_State *L) {
    binding_argcheck(L, 1);
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    const bool data = ud->enabled(
        );

    lua_pushboolean(L, data);
    return 1;
}

static int RangeFinder_num_sensors(lua_State *L) {
    binding_argcheck(L, 1);
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "rangefinder not supported on this firmware");
    }

    const uint8_t data = ud->num_sensors(
        );

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Notify_play_tune(lua_State *L) {
    // 1 enum 99 : 6
    binding_argcheck(L, 2);
    AP_Notify * ud = AP_Notify::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "AP_Notify not supported on this firmware");
    }

    const char * data_2 = luaL_checkstring(L, 2);
    ud->play_tune(
            data_2);

    return 0;
}

static int AP_GPS_first_unconfigured_gps(lua_State *L) {
    binding_argcheck(L, 1);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    const uint8_t data = ud->first_unconfigured_gps(
        );

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_all_configured(lua_State *L) {
    binding_argcheck(L, 1);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    const bool data = ud->all_configured(
        );

    lua_pushboolean(L, data);
    return 1;
}

static int AP_GPS_get_antenna_offset(lua_State *L) {
    // 1 uint8_t 70 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const Vector3f &data = ud->get_antenna_offset(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_GPS_have_vertical_velocity(lua_State *L) {
    // 1 uint8_t 69 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->have_vertical_velocity(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_GPS_last_message_time_ms(lua_State *L) {
    // 1 uint8_t 68 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->last_message_time_ms(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_last_fix_time_ms(lua_State *L) {
    // 1 uint8_t 67 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->last_fix_time_ms(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_get_vdop(lua_State *L) {
    // 1 uint8_t 66 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = ud->get_vdop(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_get_hdop(lua_State *L) {
    // 1 uint8_t 65 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = ud->get_hdop(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_time_week_ms(lua_State *L) {
    // 1 uint8_t 64 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->time_week_ms(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_time_week(lua_State *L) {
    // 1 uint8_t 63 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = ud->time_week(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_num_sats(lua_State *L) {
    // 1 uint8_t 62 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = ud->num_sats(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_ground_course(lua_State *L) {
    // 1 uint8_t 61 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->ground_course(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_GPS_ground_speed(lua_State *L) {
    // 1 uint8_t 60 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->ground_speed(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_GPS_velocity(lua_State *L) {
    // 1 uint8_t 59 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const Vector3f &data = ud->velocity(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_GPS_vertical_accuracy(lua_State *L) {
    // 1 uint8_t 58 : 8
    // 2 float 58 : 9
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003 = {};
    const bool data = ud->vertical_accuracy(
            data_2,
            data_5003);

    if (data) {
        lua_pushnumber(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_GPS_horizontal_accuracy(lua_State *L) {
    // 1 uint8_t 57 : 8
    // 2 float 57 : 9
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003 = {};
    const bool data = ud->horizontal_accuracy(
            data_2,
            data_5003);

    if (data) {
        lua_pushnumber(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_GPS_speed_accuracy(lua_State *L) {
    // 1 uint8_t 56 : 8
    // 2 float 56 : 9
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003 = {};
    const bool data = ud->speed_accuracy(
            data_2,
            data_5003);

    if (data) {
        lua_pushnumber(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_GPS_location(lua_State *L) {
    // 1 uint8_t 55 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const Location &data = ud->location(
            data_2);

    new_Location(L);
    *check_Location(L, -1) = data;
    return 1;
}

static int AP_GPS_status(lua_State *L) {
    // 1 uint8_t 54 : 8
    binding_argcheck(L, 2);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "gps not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(GPS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = ud->status(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_primary_sensor(lua_State *L) {
    binding_argcheck(L, 1);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    const uint8_t data = ud->primary_sensor(
        );

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_num_sensors(lua_State *L) {
    binding_argcheck(L, 1);
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    const uint8_t data = ud->num_sensors(
        );

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_get_temperature(lua_State *L) {
    // 1 float 47 : 6
    // 2 uint8_t 47 : 9
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    float data_5002 = {};
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = ud->get_temperature(
            data_5002,
            data_3);

    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_BattMonitor_overpower_detected(lua_State *L) {
    // 1 uint8_t 46 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->overpower_detected(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_has_failsafed(lua_State *L) {
    binding_argcheck(L, 1);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    const bool data = ud->has_failsafed(
        );

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_pack_capacity_mah(lua_State *L) {
    // 1 uint8_t 44 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const int32_t data = ud->pack_capacity_mah(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_capacity_remaining_pct(lua_State *L) {
    // 1 uint8_t 43 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = ud->capacity_remaining_pct(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_consumed_wh(lua_State *L) {
    // 1 uint8_t 42 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->consumed_wh(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_consumed_mah(lua_State *L) {
    // 1 uint8_t 41 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->consumed_mah(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_current_amps(lua_State *L) {
    // 1 uint8_t 40 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->current_amps(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_voltage_resting_estimate(lua_State *L) {
    // 1 uint8_t 39 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->voltage_resting_estimate(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_voltage(lua_State *L) {
    // 1 uint8_t 38 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->voltage(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_has_current(lua_State *L) {
    // 1 uint8_t 37 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->has_current(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_has_consumed_energy(lua_State *L) {
    // 1 uint8_t 36 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->has_consumed_energy(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_healthy(lua_State *L) {
    // 1 uint8_t 35 : 8
    binding_argcheck(L, 2);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 2, "battery not supported on this firmware");
    }

    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_BATT_MONITOR_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->healthy(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_num_instances(lua_State *L) {
    binding_argcheck(L, 1);
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    const uint8_t data = ud->num_instances(
        );

    lua_pushinteger(L, data);
    return 1;
}

static int AP_AHRS_prearm_healthy(lua_State *L) {
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    ud->get_semaphore().take_blocking();
    const bool data = ud->prearm_healthy(
        );

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_home_is_set(lua_State *L) {
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    ud->get_semaphore().take_blocking();
    const bool data = ud->home_is_set(
        );

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_get_relative_position_NED_home(lua_State *L) {
    // 1 Vector3f 27 : 6
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    Vector3f data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = ud->get_relative_position_NED_home(
            data_5002);

    ud->get_semaphore().give();
    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_AHRS_get_velocity_NED(lua_State *L) {
    // 1 Vector3f 26 : 6
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    Vector3f data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = ud->get_velocity_NED(
            data_5002);

    ud->get_semaphore().give();
    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_AHRS_groundspeed_vector(lua_State *L) {
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    ud->get_semaphore().take_blocking();
    const Vector2f &data = ud->groundspeed_vector(
        );

    ud->get_semaphore().give();
    new_Vector2f(L);
    *check_Vector2f(L, -1) = data;
    return 1;
}

static int AP_AHRS_wind_estimate(lua_State *L) {
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->wind_estimate(
        );

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_hagl(lua_State *L) {
    // 1 float 23 : 6
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    float data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = ud->get_hagl(
            data_5002);

    ud->get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_AHRS_get_gyro(lua_State *L) {
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->get_gyro(
        );

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_home(lua_State *L) {
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    ud->get_semaphore().take_blocking();
    const Location &data = ud->get_home(
        );

    ud->get_semaphore().give();
    new_Location(L);
    *check_Location(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_position(lua_State *L) {
    // 1 Location 20 : 6
    binding_argcheck(L, 1);
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    Location data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = ud->get_position(
            data_5002);

    ud->get_semaphore().give();
    if (data) {
        new_Location(L);
        *check_Location(L, -1) = data_5002;
    } else {
        lua_pushnil(L);
    }
    return 1;
}

const luaL_Reg GCS_meta[] = {
    {"send_text", GCS_send_text},
    {NULL, NULL}
};

const luaL_Reg AP_Relay_meta[] = {
    {"toggle", AP_Relay_toggle},
    {"enabled", AP_Relay_enabled},
    {"off", AP_Relay_off},
    {"on", AP_Relay_on},
    {NULL, NULL}
};

const luaL_Reg AP_Terrain_meta[] = {
    {"height_above_terrain", AP_Terrain_height_above_terrain},
    {"height_relative_home_equivalent", AP_Terrain_height_relative_home_equivalent},
    {"height_terrain_difference_home", AP_Terrain_height_terrain_difference_home},
    {"height_amsl", AP_Terrain_height_amsl},
    {"status", AP_Terrain_status},
    {"enabled", AP_Terrain_enabled},
    {NULL, NULL}
};

const luaL_Reg RangeFinder_meta[] = {
    {"num_sensors", RangeFinder_num_sensors},
    {NULL, NULL}
};

const luaL_Reg AP_Notify_meta[] = {
    {"play_tune", AP_Notify_play_tune},
    {NULL, NULL}
};

const luaL_Reg notify_meta[] = {
    {NULL, NULL}
};

const luaL_Reg AP_GPS_meta[] = {
    {"first_unconfigured_gps", AP_GPS_first_unconfigured_gps},
    {"all_configured", AP_GPS_all_configured},
    {"get_antenna_offset", AP_GPS_get_antenna_offset},
    {"have_vertical_velocity", AP_GPS_have_vertical_velocity},
    {"last_message_time_ms", AP_GPS_last_message_time_ms},
    {"last_fix_time_ms", AP_GPS_last_fix_time_ms},
    {"get_vdop", AP_GPS_get_vdop},
    {"get_hdop", AP_GPS_get_hdop},
    {"time_week_ms", AP_GPS_time_week_ms},
    {"time_week", AP_GPS_time_week},
    {"num_sats", AP_GPS_num_sats},
    {"ground_course", AP_GPS_ground_course},
    {"ground_speed", AP_GPS_ground_speed},
    {"velocity", AP_GPS_velocity},
    {"vertical_accuracy", AP_GPS_vertical_accuracy},
    {"horizontal_accuracy", AP_GPS_horizontal_accuracy},
    {"speed_accuracy", AP_GPS_speed_accuracy},
    {"location", AP_GPS_location},
    {"status", AP_GPS_status},
    {"primary_sensor", AP_GPS_primary_sensor},
    {"num_sensors", AP_GPS_num_sensors},
    {NULL, NULL}
};

const luaL_Reg AP_BattMonitor_meta[] = {
    {"get_temperature", AP_BattMonitor_get_temperature},
    {"overpower_detected", AP_BattMonitor_overpower_detected},
    {"has_failsafed", AP_BattMonitor_has_failsafed},
    {"pack_capacity_mah", AP_BattMonitor_pack_capacity_mah},
    {"capacity_remaining_pct", AP_BattMonitor_capacity_remaining_pct},
    {"consumed_wh", AP_BattMonitor_consumed_wh},
    {"consumed_mah", AP_BattMonitor_consumed_mah},
    {"current_amps", AP_BattMonitor_current_amps},
    {"voltage_resting_estimate", AP_BattMonitor_voltage_resting_estimate},
    {"voltage", AP_BattMonitor_voltage},
    {"has_current", AP_BattMonitor_has_current},
    {"has_consumed_energy", AP_BattMonitor_has_consumed_energy},
    {"healthy", AP_BattMonitor_healthy},
    {"num_instances", AP_BattMonitor_num_instances},
    {NULL, NULL}
};

const luaL_Reg AP_AHRS_meta[] = {
    {"prearm_healthy", AP_AHRS_prearm_healthy},
    {"home_is_set", AP_AHRS_home_is_set},
    {"get_relative_position_NED_home", AP_AHRS_get_relative_position_NED_home},
    {"get_velocity_NED", AP_AHRS_get_velocity_NED},
    {"groundspeed_vector", AP_AHRS_groundspeed_vector},
    {"wind_estimate", AP_AHRS_wind_estimate},
    {"get_hagl", AP_AHRS_get_hagl},
    {"get_gyro", AP_AHRS_get_gyro},
    {"get_home", AP_AHRS_get_home},
    {"get_position", AP_AHRS_get_position},
    {NULL, NULL}
};

const struct userdata_fun {
    const char *name;
    const luaL_Reg *reg;
} userdata_fun[] = {
    {"Vector2f", Vector2f_meta},
    {"Vector3f", Vector3f_meta},
    {"Location", Location_meta},
};

const struct singleton_fun {
    const char *name;
    const luaL_Reg *reg;
} singleton_fun[] = {
    {"gcs", GCS_meta},
    {"relay", AP_Relay_meta},
    {"terrain", AP_Terrain_meta},
    {"rangefinder", RangeFinder_meta},
    {"AP_Notify", AP_Notify_meta},
    {"notify", notify_meta},
    {"gps", AP_GPS_meta},
    {"battery", AP_BattMonitor_meta},
    {"ahrs", AP_AHRS_meta},
};

void load_generated_bindings(lua_State *L) {
    luaL_checkstack(L, 5, "Out of stack");
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

    load_boxed_numerics(L);
}

const char *singletons[] = {
    "gcs",
    "relay",
    "terrain",
    "rangefinder",
    "AP_Notify",
    "notify",
    "gps",
    "battery",
    "ahrs",
};

const struct userdata {
    const char *name;
    const lua_CFunction fun;
} new_userdata[] = {
    {"Vector2f", new_Vector2f},
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

    load_boxed_numerics_sandbox(L);
}

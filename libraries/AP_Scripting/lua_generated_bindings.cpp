// auto generated bindings, don't manually edit.  See README.md for details.
#include "lua_generated_bindings.h"
#include "lua_boxed_numerics.h"
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>


#if !defined(AP_TERRAIN_AVAILABLE) || (AP_TERRAIN_AVAILABLE != 1)
  #error Scripting requires terrain to be available

#endif // !defined(AP_TERRAIN_AVAILABLE) || (AP_TERRAIN_AVAILABLE != 1)


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

int new_AP_HAL__UARTDriver(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_HAL::UARTDriver *));
    memset(ud, 0, sizeof(AP_HAL::UARTDriver *));
    luaL_getmetatable(L, "AP_HAL::UARTDriver");
    lua_setmetatable(L, -2);
    return 1;
}

AP_HAL::UARTDriver ** check_AP_HAL__UARTDriver(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "AP_HAL::UARTDriver");
    return (AP_HAL::UARTDriver **)data;
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
    const bool data = ud->is_zero();

    lua_pushboolean(L, data);
    return 1;
}

static int Vector2f_is_inf(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const bool data = ud->is_inf();

    lua_pushboolean(L, data);
    return 1;
}

static int Vector2f_is_nan(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const bool data = ud->is_nan();

    lua_pushboolean(L, data);
    return 1;
}

static int Vector2f_normalize(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    ud->normalize();

    return 0;
}

static int Vector2f_length(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const float data = ud->length();

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
    const bool data = ud->is_zero();

    lua_pushboolean(L, data);
    return 1;
}

static int Vector3f_is_inf(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const bool data = ud->is_inf();

    lua_pushboolean(L, data);
    return 1;
}

static int Vector3f_is_nan(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const bool data = ud->is_nan();

    lua_pushboolean(L, data);
    return 1;
}

static int Vector3f_normalize(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    ud->normalize();

    return 0;
}

static int Vector3f_length(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const float data = ud->length();

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

static int Location_get_distance_NE(lua_State *L) {
    binding_argcheck(L, 2);
    Location * ud = check_Location(L, 1);
    Location & data_2 = *check_Location(L, 2);
    const Vector2f &data = ud->get_distance_NE(
            data_2);

    new_Vector2f(L);
    *check_Vector2f(L, -1) = data;
    return 1;
}

static int Location_get_distance_NED(lua_State *L) {
    binding_argcheck(L, 2);
    Location * ud = check_Location(L, 1);
    Location & data_2 = *check_Location(L, 2);
    const Vector3f &data = ud->get_distance_NED(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int Location_get_bearing(lua_State *L) {
    binding_argcheck(L, 2);
    Location * ud = check_Location(L, 1);
    Location & data_2 = *check_Location(L, 2);
    const float data = ud->get_bearing(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int Location_get_vector_from_origin_NEU(lua_State *L) {
    binding_argcheck(L, 1);
    Location * ud = check_Location(L, 1);
    Vector3f data_5002 = {};
    const bool data = ud->get_vector_from_origin_NEU(
            data_5002);

    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int Location_offset(lua_State *L) {
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
    {"get_distance_NE", Location_get_distance_NE},
    {"get_distance_NED", Location_get_distance_NED},
    {"get_bearing", Location_get_bearing},
    {"get_vector_from_origin_NEU", Location_get_vector_from_origin_NEU},
    {"offset", Location_offset},
    {"get_distance", Location_get_distance},
    {NULL, NULL}
};

static int AP_Param_set_and_save(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "param not supported on this firmware");
    }

    binding_argcheck(L, 3);
    const char * data_2 = luaL_checkstring(L, 2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_3 <= MIN(FLT_MAX, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    const bool data = ud->set_and_save(
            data_2,
            data_3);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Param_set(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "param not supported on this firmware");
    }

    binding_argcheck(L, 3);
    const char * data_2 = luaL_checkstring(L, 2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_3 <= MIN(FLT_MAX, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    const bool data = ud->set(
            data_2,
            data_3);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Param_get(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "param not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const char * data_2 = luaL_checkstring(L, 2);
    float data_5003 = {};
    const bool data = ud->get(
            data_2,
            data_5003);

    if (data) {
        lua_pushnumber(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_ESC_Telem_get_usage_seconds(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "esc_telem not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint32_t data_5003 = {};
    const bool data = ud->get_usage_seconds(
            data_2,
            data_5003);

    if (data) {
        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data_5003;
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_Baro_get_external_temperature(lua_State *L) {
    AP_Baro * ud = AP_Baro::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "baro not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const float data = ud->get_external_temperature();

    lua_pushnumber(L, data);
    return 1;
}

static int AP_Baro_get_temperature(lua_State *L) {
    AP_Baro * ud = AP_Baro::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "baro not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const float data = ud->get_temperature();

    lua_pushnumber(L, data);
    return 1;
}

static int AP_Baro_get_pressure(lua_State *L) {
    AP_Baro * ud = AP_Baro::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "baro not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const float data = ud->get_pressure();

    lua_pushnumber(L, data);
    return 1;
}

static int AP_SerialManager_find_serial(lua_State *L) {
    AP_SerialManager * ud = AP_SerialManager::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "serial not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    AP_HAL::UARTDriver *data = ud->find_serial(
            AP_SerialManager::SerialProtocol_Scripting,
            data_2);

    if (data == NULL) {
        lua_pushnil(L);
    } else {
        new_AP_HAL__UARTDriver(L);
        *check_AP_HAL__UARTDriver(L, -1) = data;
    }
    return 1;
}

static int RC_Channels_get_pwm(lua_State *L) {
    RC_Channels * ud = RC_Channels::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "rc not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(NUM_RC_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint16_t data_5003 = {};
    const bool data = ud->get_pwm(
            data_2,
            data_5003);

    if (data) {
        lua_pushinteger(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int SRV_Channels_find_channel(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "SRV_Channels not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    uint8_t data_5003 = {};
    const bool data = ud->find_channel(
            data_2,
            data_5003);

    if (data) {
        lua_pushinteger(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_SerialLED_send(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "serialLED not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->send();

    return 0;
}

static int AP_SerialLED_set_RGB(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "serialLED not supported on this firmware");
    }

    binding_argcheck(L, 6);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(16, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t raw_data_3 = coerce_to_uint32_t(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0U, 0U)) && (raw_data_3 <= MIN(UINT32_MAX, UINT32_MAX))), 3, "argument out of range");
    const uint32_t data_3 = static_cast<uint32_t>(raw_data_3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(0, 0)) && (raw_data_4 <= MIN(UINT8_MAX, UINT8_MAX))), 4, "argument out of range");
    const uint8_t data_4 = static_cast<uint8_t>(raw_data_4);
    const lua_Integer raw_data_5 = luaL_checkinteger(L, 5);
    luaL_argcheck(L, ((raw_data_5 >= MAX(0, 0)) && (raw_data_5 <= MIN(UINT8_MAX, UINT8_MAX))), 5, "argument out of range");
    const uint8_t data_5 = static_cast<uint8_t>(raw_data_5);
    const lua_Integer raw_data_6 = luaL_checkinteger(L, 6);
    luaL_argcheck(L, ((raw_data_6 >= MAX(0, 0)) && (raw_data_6 <= MIN(UINT8_MAX, UINT8_MAX))), 6, "argument out of range");
    const uint8_t data_6 = static_cast<uint8_t>(raw_data_6);
    ud->set_RGB(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6);

    return 0;
}

static int AP_SerialLED_set_num_LEDs(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "serialLED not supported on this firmware");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(16, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(32, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = ud->set_num_LEDs(
            data_2,
            data_3);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_get_time_flying_ms(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "vehicle not supported on this firmware");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint32_t data = ud->get_time_flying_ms();

    AP::scheduler().get_semaphore().give();
        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_Vehicle_get_likely_flying(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "vehicle not supported on this firmware");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = ud->get_likely_flying();

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_get_mode(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "vehicle not supported on this firmware");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint8_t data = ud->get_mode();

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Vehicle_set_mode(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "vehicle not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = ud->set_mode(
            data_2,
            ModeReason::SCRIPTING);

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int GCS_set_message_interval(lua_State *L) {
    GCS * ud = GCS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gcs not supported on this firmware");
    }

    binding_argcheck(L, 4);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(MAVLINK_COMM_NUM_BUFFERS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t raw_data_3 = coerce_to_uint32_t(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0U, 0U)) && (raw_data_3 <= MIN(UINT32_MAX, UINT32_MAX))), 3, "argument out of range");
    const uint32_t data_3 = static_cast<uint32_t>(raw_data_3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(-1, INT32_MIN)) && (raw_data_4 <= MIN(INT32_MAX, INT32_MAX))), 4, "argument out of range");
    const int32_t data_4 = raw_data_4;
    const MAV_RESULT &data = ud->set_message_interval(
            data_2,
            data_3,
            data_4);

    lua_pushinteger(L, data);
    return 1;
}

static int GCS_send_text(lua_State *L) {
    GCS * ud = GCS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gcs not supported on this firmware");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(MAV_SEVERITY_EMERGENCY)) && (raw_data_2 <= static_cast<int32_t>(MAV_SEVERITY_DEBUG))), 2, "argument out of range");
    const MAV_SEVERITY data_2 = static_cast<MAV_SEVERITY>(raw_data_2);
    const char * data_3 = luaL_checkstring(L, 3);
    ud->send_text(
            data_2,
            "%s",
            data_3);

    return 0;
}

static int AP_Relay_toggle(lua_State *L) {
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "relay not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->toggle(
            data_2);

    return 0;
}

static int AP_Relay_enabled(lua_State *L) {
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "relay not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->enabled(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Relay_off(lua_State *L) {
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "relay not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->off(
            data_2);

    return 0;
}

static int AP_Relay_on(lua_State *L) {
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "relay not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->on(
            data_2);

    return 0;
}

static int AP_Terrain_height_above_terrain(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    binding_argcheck(L, 2);
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

static int AP_Terrain_height_terrain_difference_home(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    binding_argcheck(L, 2);
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
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    binding_argcheck(L, 3);
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
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const uint8_t data = ud->status();

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Terrain_enabled(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "terrain not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const bool data = ud->enabled();

    lua_pushboolean(L, data);
    return 1;
}

static int RangeFinder_num_sensors(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "rangefinder not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const uint8_t data = ud->num_sensors();

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Notify_handle_rgb(lua_State *L) {
    AP_Notify * ud = AP_Notify::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "notify not supported on this firmware");
    }

    binding_argcheck(L, 5);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT8_MAX, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(0, 0)) && (raw_data_4 <= MIN(UINT8_MAX, UINT8_MAX))), 4, "argument out of range");
    const uint8_t data_4 = static_cast<uint8_t>(raw_data_4);
    const lua_Integer raw_data_5 = luaL_checkinteger(L, 5);
    luaL_argcheck(L, ((raw_data_5 >= MAX(0, 0)) && (raw_data_5 <= MIN(UINT8_MAX, UINT8_MAX))), 5, "argument out of range");
    const uint8_t data_5 = static_cast<uint8_t>(raw_data_5);
    ud->handle_rgb(
            data_2,
            data_3,
            data_4,
            data_5);

    return 0;
}

static int AP_Notify_play_tune(lua_State *L) {
    AP_Notify * ud = AP_Notify::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "notify not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const char * data_2 = luaL_checkstring(L, 2);
    ud->play_tune(
            data_2);

    return 0;
}

static int AP_GPS_first_unconfigured_gps(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 1);
    uint8_t data_5002 = {};
    const bool data = ud->first_unconfigured_gps(
            data_5002);

    if (data) {
        lua_pushinteger(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_GPS_get_antenna_offset(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const Vector3f &data = ud->get_antenna_offset(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_GPS_have_vertical_velocity(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->have_vertical_velocity(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_GPS_last_message_time_ms(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->last_message_time_ms(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_last_fix_time_ms(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->last_fix_time_ms(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_get_vdop(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = ud->get_vdop(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_get_hdop(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = ud->get_hdop(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_time_week_ms(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->time_week_ms(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_time_week(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = ud->time_week(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_num_sats(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = ud->num_sats(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_ground_course(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->ground_course(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_GPS_ground_speed(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->ground_speed(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_GPS_velocity(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const Vector3f &data = ud->velocity(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_GPS_vertical_accuracy(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
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
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
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
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
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
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const Location &data = ud->location(
            data_2);

    new_Location(L);
    *check_Location(L, -1) = data;
    return 1;
}

static int AP_GPS_status(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = ud->status(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_primary_sensor(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const uint8_t data = ud->primary_sensor();

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_num_sensors(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "gps not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const uint8_t data = ud->num_sensors();

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_get_cycle_count(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint16_t data_5003 = {};
    const bool data = ud->get_cycle_count(
            data_2,
            data_5003);

    if (data) {
        lua_pushinteger(L, data_5003);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_BattMonitor_get_temperature(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    float data_5002 = {};
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
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
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->overpower_detected(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_has_failsafed(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const bool data = ud->has_failsafed();

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_pack_capacity_mah(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const int32_t data = ud->pack_capacity_mah(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_capacity_remaining_pct(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = ud->capacity_remaining_pct(
            data_2);

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_consumed_wh(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    float data_5002 = {};
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = ud->consumed_wh(
            data_5002,
            data_3);

    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_BattMonitor_consumed_mah(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    float data_5002 = {};
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = ud->consumed_mah(
            data_5002,
            data_3);

    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_BattMonitor_current_amps(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    float data_5002 = {};
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = ud->current_amps(
            data_5002,
            data_3);

    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_BattMonitor_voltage_resting_estimate(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->voltage_resting_estimate(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_voltage(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = ud->voltage(
            data_2);

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_healthy(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = ud->healthy(
            data_2);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_num_instances(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "battery not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const uint8_t data = ud->num_instances();

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Arming_arm(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "arming not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const bool data = ud->arm(            AP_Arming::Method::SCRIPTING);

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Arming_is_armed(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "arming not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const bool data = ud->is_armed();

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Arming_disarm(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "arming not supported on this firmware");
    }

    binding_argcheck(L, 1);
    const bool data = ud->disarm();

    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_airspeed_estimate(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    float data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = ud->airspeed_estimate(
            data_5002);

    ud->get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
    } else {
        lua_pushnil(L);
    }
    return 1;
}

static int AP_AHRS_prearm_healthy(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const bool data = ud->prearm_healthy();

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_home_is_set(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const bool data = ud->home_is_set();

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_get_relative_position_NED_home(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
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
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
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
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const Vector2f &data = ud->groundspeed_vector();

    ud->get_semaphore().give();
    new_Vector2f(L);
    *check_Vector2f(L, -1) = data;
    return 1;
}

static int AP_AHRS_wind_estimate(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->wind_estimate();

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_hagl(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
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
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->get_gyro();

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_home(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const Location &data = ud->get_home();

    ud->get_semaphore().give();
    new_Location(L);
    *check_Location(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_position(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
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

static int AP_AHRS_get_yaw(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = ud->get_yaw();

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

static int AP_AHRS_get_pitch(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = ud->get_pitch();

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

static int AP_AHRS_get_roll(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return luaL_argerror(L, 1, "ahrs not supported on this firmware");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = ud->get_roll();

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

const luaL_Reg AP_Param_meta[] = {
    {"set_and_save", AP_Param_set_and_save},
    {"set", AP_Param_set},
    {"get", AP_Param_get},
    {NULL, NULL}
};

const luaL_Reg AP_ESC_Telem_meta[] = {
    {"get_usage_seconds", AP_ESC_Telem_get_usage_seconds},
    {NULL, NULL}
};

const luaL_Reg AP_Baro_meta[] = {
    {"get_external_temperature", AP_Baro_get_external_temperature},
    {"get_temperature", AP_Baro_get_temperature},
    {"get_pressure", AP_Baro_get_pressure},
    {NULL, NULL}
};

const luaL_Reg AP_SerialManager_meta[] = {
    {"find_serial", AP_SerialManager_find_serial},
    {NULL, NULL}
};

const luaL_Reg RC_Channels_meta[] = {
    {"get_pwm", RC_Channels_get_pwm},
    {NULL, NULL}
};

const luaL_Reg SRV_Channels_meta[] = {
    {"find_channel", SRV_Channels_find_channel},
    {NULL, NULL}
};

const luaL_Reg AP_SerialLED_meta[] = {
    {"send", AP_SerialLED_send},
    {"set_RGB", AP_SerialLED_set_RGB},
    {"set_num_LEDs", AP_SerialLED_set_num_LEDs},
    {NULL, NULL}
};

const luaL_Reg AP_Vehicle_meta[] = {
    {"get_time_flying_ms", AP_Vehicle_get_time_flying_ms},
    {"get_likely_flying", AP_Vehicle_get_likely_flying},
    {"get_mode", AP_Vehicle_get_mode},
    {"set_mode", AP_Vehicle_set_mode},
    {NULL, NULL}
};

const luaL_Reg GCS_meta[] = {
    {"set_message_interval", GCS_set_message_interval},
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
    {"handle_rgb", AP_Notify_handle_rgb},
    {"play_tune", AP_Notify_play_tune},
    {NULL, NULL}
};

const luaL_Reg AP_GPS_meta[] = {
    {"first_unconfigured_gps", AP_GPS_first_unconfigured_gps},
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
    {"get_cycle_count", AP_BattMonitor_get_cycle_count},
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
    {"healthy", AP_BattMonitor_healthy},
    {"num_instances", AP_BattMonitor_num_instances},
    {NULL, NULL}
};

const luaL_Reg AP_Arming_meta[] = {
    {"arm", AP_Arming_arm},
    {"is_armed", AP_Arming_is_armed},
    {"disarm", AP_Arming_disarm},
    {NULL, NULL}
};

const luaL_Reg AP_AHRS_meta[] = {
    {"airspeed_estimate", AP_AHRS_airspeed_estimate},
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
    {"get_yaw", AP_AHRS_get_yaw},
    {"get_pitch", AP_AHRS_get_pitch},
    {"get_roll", AP_AHRS_get_roll},
    {NULL, NULL}
};

static int AP_HAL__UARTDriver_set_flow_control(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE)) && (raw_data_2 <= static_cast<int32_t>(AP_HAL::UARTDriver::FLOW_CONTROL_AUTO))), 2, "argument out of range");
    const AP_HAL::UARTDriver::flow_control data_2 = static_cast<AP_HAL::UARTDriver::flow_control>(raw_data_2);
    ud->set_flow_control(
            data_2);

    return 0;
}

static int AP_HAL__UARTDriver_available(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        luaL_error(L, "Internal error, null pointer");
    }
    const uint32_t data = ud->available();

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_HAL__UARTDriver_write(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = ud->write(
            data_2);

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_HAL__UARTDriver_read(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        luaL_error(L, "Internal error, null pointer");
    }
    const int16_t data = ud->read();

    lua_pushinteger(L, data);
    return 1;
}

static int AP_HAL__UARTDriver_begin(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        luaL_error(L, "Internal error, null pointer");
    }
    const uint32_t raw_data_2 = coerce_to_uint32_t(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1U, 0U)) && (raw_data_2 <= MIN(UINT32_MAX, UINT32_MAX))), 2, "argument out of range");
    const uint32_t data_2 = static_cast<uint32_t>(raw_data_2);
    ud->begin(
            data_2);

    return 0;
}

const luaL_Reg AP_HAL__UARTDriver_meta[] = {
    {"set_flow_control", AP_HAL__UARTDriver_set_flow_control},
    {"available", AP_HAL__UARTDriver_available},
    {"write", AP_HAL__UARTDriver_write},
    {"read", AP_HAL__UARTDriver_read},
    {"begin", AP_HAL__UARTDriver_begin},
    {NULL, NULL}
};

struct userdata_enum {
    const char *name;
    int value;
};

struct userdata_enum AP_Terrain_enums[] = {
    {"TerrainStatusOK", AP_Terrain::TerrainStatusOK},
    {"TerrainStatusUnhealthy", AP_Terrain::TerrainStatusUnhealthy},
    {"TerrainStatusDisabled", AP_Terrain::TerrainStatusDisabled},
    {NULL, 0}};

struct userdata_enum AP_GPS_enums[] = {
    {"GPS_OK_FIX_3D_RTK_FIXED", AP_GPS::GPS_OK_FIX_3D_RTK_FIXED},
    {"GPS_OK_FIX_3D_RTK_FLOAT", AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT},
    {"GPS_OK_FIX_3D_DGPS", AP_GPS::GPS_OK_FIX_3D_DGPS},
    {"GPS_OK_FIX_3D", AP_GPS::GPS_OK_FIX_3D},
    {"GPS_OK_FIX_2D", AP_GPS::GPS_OK_FIX_2D},
    {"NO_FIX", AP_GPS::NO_FIX},
    {"NO_GPS", AP_GPS::NO_GPS},
    {NULL, 0}};

struct userdata_meta {
    const char *name;
    const luaL_Reg *reg;
    const struct userdata_enum *enums;
};

const struct userdata_meta userdata_fun[] = {
    {"Vector2f", Vector2f_meta, NULL},
    {"Vector3f", Vector3f_meta, NULL},
    {"Location", Location_meta, NULL},
};

const struct userdata_meta singleton_fun[] = {
    {"param", AP_Param_meta, NULL},
    {"esc_telem", AP_ESC_Telem_meta, NULL},
    {"baro", AP_Baro_meta, NULL},
    {"serial", AP_SerialManager_meta, NULL},
    {"rc", RC_Channels_meta, NULL},
    {"SRV_Channels", SRV_Channels_meta, NULL},
    {"serialLED", AP_SerialLED_meta, NULL},
    {"vehicle", AP_Vehicle_meta, NULL},
    {"gcs", GCS_meta, NULL},
    {"relay", AP_Relay_meta, NULL},
    {"terrain", AP_Terrain_meta, AP_Terrain_enums},
    {"rangefinder", RangeFinder_meta, NULL},
    {"notify", AP_Notify_meta, NULL},
    {"gps", AP_GPS_meta, AP_GPS_enums},
    {"battery", AP_BattMonitor_meta, NULL},
    {"arming", AP_Arming_meta, NULL},
    {"ahrs", AP_AHRS_meta, NULL},
};

const struct userdata_meta ap_object_fun[] = {
    {"AP_HAL::UARTDriver", AP_HAL__UARTDriver_meta, NULL},
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

    // ap object metatables
    for (uint32_t i = 0; i < ARRAY_SIZE(ap_object_fun); i++) {
        luaL_newmetatable(L, ap_object_fun[i].name);
        luaL_setfuncs(L, ap_object_fun[i].reg, 0);
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
        if (singleton_fun[i].enums != nullptr) {
            int j = 0;
            while (singleton_fun[i].enums[j].name != NULL) {
                lua_pushstring(L, singleton_fun[i].enums[j].name);
                lua_pushinteger(L, singleton_fun[i].enums[j].value);
                lua_settable(L, -3);
                j++;
            }
        }
        lua_pop(L, 1);
        lua_newuserdata(L, 0);
        luaL_getmetatable(L, singleton_fun[i].name);
        lua_setmetatable(L, -2);
        lua_setglobal(L, singleton_fun[i].name);
    }

    load_boxed_numerics(L);
}

const char *singletons[] = {
    "param",
    "esc_telem",
    "baro",
    "serial",
    "rc",
    "SRV_Channels",
    "serialLED",
    "vehicle",
    "gcs",
    "relay",
    "terrain",
    "rangefinder",
    "notify",
    "gps",
    "battery",
    "arming",
    "ahrs",
};

const struct userdata {
    const char *name;
    const lua_CFunction fun;
} new_userdata[] = {
    {"Vector2f", new_Vector2f},
    {"Vector3f", new_Vector3f},
    {"Location", new_Location},
    {"AP_HAL::UARTDriver", new_AP_HAL__UARTDriver},
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

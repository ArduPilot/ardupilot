// auto generated bindings, don't manually edit.  See README.md for details.
#pragma GCC optimize("Os")
#include "lua_generated_bindings.h"
#include <AP_Scripting/lua_boxed_numerics.h>
#include <AP_Common/AP_FWVersion.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_Motors_Class.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
#include <../Tools/AP_Periph/AP_Periph.h>
#endif // defined(HAL_BUILD_AP_PERIPH)
#include <AP_HAL/AP_HAL.h>
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_Scripting/AP_Scripting_CANSensor.h>
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_InertialSensor/AP_InertialSensor.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_HAL/I2CDevice.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AP_Frsky_Telem/AP_Frsky_SPort.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_MotorsMatrix.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <../ArduPlane/quadplane.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <AP_Notify/ScriptingLED.h>
#include <AP_Button/AP_Button.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Scripting/AP_Scripting_helpers.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_Vehicle/AP_Vehicle.h>
#if ENABLE_ONVIF == 1
#include <AP_ONVIF/AP_ONVIF.h>
#endif // ENABLE_ONVIF == 1
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
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

static int not_supported_error(lua_State *L, int arg, const char* name) {
    char error_msg[50];
    snprintf(error_msg, sizeof(error_msg), "%s not supported on this firmware", name);
    return luaL_argerror(L, arg, error_msg);
}

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_AP_HAL__CANFrame(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_HAL::CANFrame));
    memset(ud, 0, sizeof(AP_HAL::CANFrame));
    new (ud) AP_HAL::CANFrame();
    luaL_getmetatable(L, "CANFrame");
    lua_setmetatable(L, -2);
    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
int new_AP_MotorsMatrix_Scripting_Dynamic__factor_table(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_MotorsMatrix_Scripting_Dynamic::factor_table));
    memset(ud, 0, sizeof(AP_MotorsMatrix_Scripting_Dynamic::factor_table));
    new (ud) AP_MotorsMatrix_Scripting_Dynamic::factor_table();
    luaL_getmetatable(L, "motor_factor_table");
    lua_setmetatable(L, -2);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

int new_AP_HAL__PWMSource(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_HAL::PWMSource));
    memset(ud, 0, sizeof(AP_HAL::PWMSource));
    new (ud) AP_HAL::PWMSource();
    luaL_getmetatable(L, "PWMSource");
    lua_setmetatable(L, -2);
    return 1;
}

int new_mavlink_mission_item_int_t(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(mavlink_mission_item_int_t));
    memset(ud, 0, sizeof(mavlink_mission_item_int_t));
    new (ud) mavlink_mission_item_int_t();
    luaL_getmetatable(L, "mavlink_mission_item_int_t");
    lua_setmetatable(L, -2);
    return 1;
}

int new_Parameter(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(Parameter));
    memset(ud, 0, sizeof(Parameter));
    new (ud) Parameter();
    luaL_getmetatable(L, "Parameter");
    lua_setmetatable(L, -2);
    return 1;
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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
AP_HAL::CANFrame * check_AP_HAL__CANFrame(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "CANFrame");
    return (AP_HAL::CANFrame *)data;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
AP_MotorsMatrix_Scripting_Dynamic::factor_table * check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "motor_factor_table");
    return (AP_MotorsMatrix_Scripting_Dynamic::factor_table *)data;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

AP_HAL::PWMSource * check_AP_HAL__PWMSource(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "PWMSource");
    return (AP_HAL::PWMSource *)data;
}

mavlink_mission_item_int_t * check_mavlink_mission_item_int_t(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "mavlink_mission_item_int_t");
    return (mavlink_mission_item_int_t *)data;
}

Parameter * check_Parameter(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "Parameter");
    return (Parameter *)data;
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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_ScriptingCANBuffer(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(ScriptingCANBuffer *));
    memset(ud, 0, sizeof(ScriptingCANBuffer *));
    luaL_getmetatable(L, "ScriptingCANBuffer");
    lua_setmetatable(L, -2);
    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

int new_AP_HAL__AnalogSource(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_HAL::AnalogSource *));
    memset(ud, 0, sizeof(AP_HAL::AnalogSource *));
    luaL_getmetatable(L, "AP_HAL::AnalogSource");
    lua_setmetatable(L, -2);
    return 1;
}

int new_AP_HAL__I2CDevice(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_HAL::I2CDevice *));
    memset(ud, 0, sizeof(AP_HAL::I2CDevice *));
    luaL_getmetatable(L, "AP_HAL::I2CDevice");
    lua_setmetatable(L, -2);
    return 1;
}

int new_AP_HAL__UARTDriver(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(AP_HAL::UARTDriver *));
    memset(ud, 0, sizeof(AP_HAL::UARTDriver *));
    luaL_getmetatable(L, "AP_HAL::UARTDriver");
    lua_setmetatable(L, -2);
    return 1;
}

int new_RC_Channel(lua_State *L) {
    luaL_checkstack(L, 2, "Out of stack");
    void *ud = lua_newuserdata(L, sizeof(RC_Channel *));
    memset(ud, 0, sizeof(RC_Channel *));
    luaL_getmetatable(L, "RC_Channel");
    lua_setmetatable(L, -2);
    return 1;
}

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
ScriptingCANBuffer ** check_ScriptingCANBuffer(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "ScriptingCANBuffer");
    return (ScriptingCANBuffer **)data;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

AP_HAL::AnalogSource ** check_AP_HAL__AnalogSource(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "AP_HAL::AnalogSource");
    return (AP_HAL::AnalogSource **)data;
}

AP_HAL::I2CDevice ** check_AP_HAL__I2CDevice(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "AP_HAL::I2CDevice");
    return (AP_HAL::I2CDevice **)data;
}

AP_HAL::UARTDriver ** check_AP_HAL__UARTDriver(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "AP_HAL::UARTDriver");
    return (AP_HAL::UARTDriver **)data;
}

RC_Channel ** check_RC_Channel(lua_State *L, int arg) {
    void *data = luaL_checkudata(L, arg, "RC_Channel");
    return (RC_Channel **)data;
}

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
static int AP_HAL__CANFrame_dlc(lua_State *L) {
    AP_HAL::CANFrame *ud = check_AP_HAL__CANFrame(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->dlc);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(int(ARRAY_SIZE(ud->data)), UINT8_MAX))), 2, "dlc out of range");
            const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
            ud->dlc = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP_HAL__CANFrame_data(lua_State *L) {
    AP_HAL::CANFrame *ud = check_AP_HAL__CANFrame(L, 1);

    const lua_Integer raw_index = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(int(ARRAY_SIZE(ud->data)), UINT8_MAX))), 2, "index out of range");
    const uint8_t index = static_cast<uint8_t>(raw_index);

    switch(lua_gettop(L)-1) {
        case 1:
            lua_pushinteger(L, ud->data[index]);
            return 1;
        case 2: {
            const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
            luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT8_MAX, UINT8_MAX))), 3, "data out of range");
            const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
            ud->data[index] = data_3;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP_HAL__CANFrame_id(lua_State *L) {
    AP_HAL::CANFrame *ud = check_AP_HAL__CANFrame(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            new_uint32_t(L);
            *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = ud->id;
            return 1;
        case 2: {
            const uint32_t raw_data_2 = coerce_to_uint32_t(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(0U, 0U)) && (raw_data_2 <= MIN(UINT32_MAX, UINT32_MAX))), 2, "id out of range");
            const uint32_t data_2 = static_cast<uint32_t>(raw_data_2);
            ud->id = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_Scripting_Dynamic__factor_table_throttle(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic::factor_table *ud = check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(L, 1);

    const lua_Integer raw_index = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "index out of range");
    const uint8_t index = static_cast<uint8_t>(raw_index);

    switch(lua_gettop(L)-1) {
        case 1:
            lua_pushnumber(L, ud->throttle[index]);
            return 1;
        case 2: {
            const float raw_data_3 = luaL_checknumber(L, 3);
            const float data_3 = raw_data_3;
            ud->throttle[index] = data_3;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP_MotorsMatrix_Scripting_Dynamic__factor_table_yaw(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic::factor_table *ud = check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(L, 1);

    const lua_Integer raw_index = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "index out of range");
    const uint8_t index = static_cast<uint8_t>(raw_index);

    switch(lua_gettop(L)-1) {
        case 1:
            lua_pushnumber(L, ud->yaw[index]);
            return 1;
        case 2: {
            const float raw_data_3 = luaL_checknumber(L, 3);
            const float data_3 = raw_data_3;
            ud->yaw[index] = data_3;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP_MotorsMatrix_Scripting_Dynamic__factor_table_pitch(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic::factor_table *ud = check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(L, 1);

    const lua_Integer raw_index = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "index out of range");
    const uint8_t index = static_cast<uint8_t>(raw_index);

    switch(lua_gettop(L)-1) {
        case 1:
            lua_pushnumber(L, ud->pitch[index]);
            return 1;
        case 2: {
            const float raw_data_3 = luaL_checknumber(L, 3);
            const float data_3 = raw_data_3;
            ud->pitch[index] = data_3;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP_MotorsMatrix_Scripting_Dynamic__factor_table_roll(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic::factor_table *ud = check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(L, 1);

    const lua_Integer raw_index = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "index out of range");
    const uint8_t index = static_cast<uint8_t>(raw_index);

    switch(lua_gettop(L)-1) {
        case 1:
            lua_pushnumber(L, ud->roll[index]);
            return 1;
        case 2: {
            const float raw_data_3 = luaL_checknumber(L, 3);
            const float data_3 = raw_data_3;
            ud->roll[index] = data_3;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int mavlink_mission_item_int_t_current(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->current);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "current out of range");
            const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
            ud->current = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_frame(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->frame);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "frame out of range");
            const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
            ud->frame = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_command(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->command);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT16_MAX, UINT16_MAX))), 2, "command out of range");
            const uint16_t data_2 = static_cast<uint16_t>(raw_data_2);
            ud->command = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_seq(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->seq);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT16_MAX, UINT16_MAX))), 2, "seq out of range");
            const uint16_t data_2 = static_cast<uint16_t>(raw_data_2);
            ud->seq = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_z(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->z);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            const float data_2 = raw_data_2;
            ud->z = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_y(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->y);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-INT32_MAX, INT32_MIN)) && (raw_data_2 <= MIN(INT32_MAX, INT32_MAX))), 2, "y out of range");
            const int32_t data_2 = raw_data_2;
            ud->y = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_x(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->x);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX(-INT32_MAX, INT32_MIN)) && (raw_data_2 <= MIN(INT32_MAX, INT32_MAX))), 2, "x out of range");
            const int32_t data_2 = raw_data_2;
            ud->x = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_param4(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->param4);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            const float data_2 = raw_data_2;
            ud->param4 = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_param3(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->param3);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            const float data_2 = raw_data_2;
            ud->param3 = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_param2(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->param2);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            const float data_2 = raw_data_2;
            ud->param2 = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int mavlink_mission_item_int_t_param1(lua_State *L) {
    mavlink_mission_item_int_t *ud = check_mavlink_mission_item_int_t(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->param1);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
            const float data_2 = raw_data_2;
            ud->param1 = data_2;
            return 0;
         }
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int Vector2f_y(lua_State *L) {
    Vector2f *ud = check_Vector2f(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushnumber(L, ud->y);
            return 1;
        case 2: {
            const float raw_data_2 = luaL_checknumber(L, 2);
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

static int Location_alt(lua_State *L) {
    Location *ud = check_Location(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->alt);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
            luaL_argcheck(L, ((raw_data_2 >= MAX((-LOCATION_ALT_MAX_M*100+1), INT32_MIN)) && (raw_data_2 <= MIN((LOCATION_ALT_MAX_M*100-1), INT32_MAX))), 2, "alt out of range");
            const int32_t data_2 = raw_data_2;
            ud->alt = data_2;
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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
static int AP_HAL__CANFrame_isErrorFrame(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::CANFrame * ud = check_AP_HAL__CANFrame(L, 1);
    const bool data = static_cast<bool>(ud->isErrorFrame());

    lua_pushboolean(L, data);
    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
static int AP_HAL__CANFrame_isRemoteTransmissionRequest(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::CANFrame * ud = check_AP_HAL__CANFrame(L, 1);
    const bool data = static_cast<bool>(ud->isRemoteTransmissionRequest());

    lua_pushboolean(L, data);
    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
static int AP_HAL__CANFrame_isExtended(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::CANFrame * ud = check_AP_HAL__CANFrame(L, 1);
    const bool data = static_cast<bool>(ud->isExtended());

    lua_pushboolean(L, data);
    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

static int AP_HAL__PWMSource_get_pwm_avg_us(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::PWMSource * ud = check_AP_HAL__PWMSource(L, 1);
    const uint16_t data = static_cast<uint16_t>(ud->get_pwm_avg_us());

    lua_pushinteger(L, data);
    return 1;
}

static int AP_HAL__PWMSource_get_pwm_us(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::PWMSource * ud = check_AP_HAL__PWMSource(L, 1);
    const uint16_t data = static_cast<uint16_t>(ud->get_pwm_us());

    lua_pushinteger(L, data);
    return 1;
}

static int AP_HAL__PWMSource_set_pin(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::PWMSource * ud = check_AP_HAL__PWMSource(L, 1);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->set_pin(
            data_2,
            "Scripting"));

    lua_pushboolean(L, data);
    return 1;
}

static int Parameter_set_and_save(lua_State *L) {
    binding_argcheck(L, 2);
    Parameter * ud = check_Parameter(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const bool data = static_cast<bool>(ud->set_and_save(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int Parameter_set(lua_State *L) {
    binding_argcheck(L, 2);
    Parameter * ud = check_Parameter(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const bool data = static_cast<bool>(ud->set(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int Parameter_get(lua_State *L) {
    binding_argcheck(L, 1);
    Parameter * ud = check_Parameter(L, 1);
    float data_5002;
    const bool data = static_cast<bool>(ud->get(
            data_5002));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int Parameter_init(lua_State *L) {
    binding_argcheck(L, 2);
    Parameter * ud = check_Parameter(L, 1);
    const char * data_2 = luaL_checkstring(L, 2);
    const bool data = static_cast<bool>(ud->init(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int Vector2f_copy(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const Vector2f data = (*ud);

    new_Vector2f(L);
    *check_Vector2f(L, -1) = data;
    return 1;
}

static int Vector2f_rotate(lua_State *L) {
    binding_argcheck(L, 2);
    Vector2f * ud = check_Vector2f(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    ud->rotate(
            data_2);

    return 0;
}

static int Vector2f_is_zero(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const bool data = static_cast<bool>(ud->is_zero());

    lua_pushboolean(L, data);
    return 1;
}

static int Vector2f_is_inf(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const bool data = static_cast<bool>(ud->is_inf());

    lua_pushboolean(L, data);
    return 1;
}

static int Vector2f_is_nan(lua_State *L) {
    binding_argcheck(L, 1);
    Vector2f * ud = check_Vector2f(L, 1);
    const bool data = static_cast<bool>(ud->is_nan());

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
    const float data = static_cast<float>(ud->length());

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

static int Vector3f_copy(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const Vector3f data = (*ud);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int Vector3f_scale(lua_State *L) {
    binding_argcheck(L, 2);
    Vector3f * ud = check_Vector3f(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const Vector3f &data = ud->scale(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int Vector3f_cross(lua_State *L) {
    binding_argcheck(L, 2);
    Vector3f * ud = check_Vector3f(L, 1);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    const Vector3f &data = ud->cross(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int Vector3f_dot(lua_State *L) {
    binding_argcheck(L, 2);
    Vector3f * ud = check_Vector3f(L, 1);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    const float data = static_cast<float>(ud->dot(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int Vector3f_is_zero(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const bool data = static_cast<bool>(ud->is_zero());

    lua_pushboolean(L, data);
    return 1;
}

static int Vector3f_is_inf(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const bool data = static_cast<bool>(ud->is_inf());

    lua_pushboolean(L, data);
    return 1;
}

static int Vector3f_is_nan(lua_State *L) {
    binding_argcheck(L, 1);
    Vector3f * ud = check_Vector3f(L, 1);
    const bool data = static_cast<bool>(ud->is_nan());

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
    const float data = static_cast<float>(ud->length());

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

static int Location_copy(lua_State *L) {
    binding_argcheck(L, 1);
    Location * ud = check_Location(L, 1);
    const Location data = (*ud);

    new_Location(L);
    *check_Location(L, -1) = data;
    return 1;
}

static int Location_change_alt_frame(lua_State *L) {
    binding_argcheck(L, 2);
    Location * ud = check_Location(L, 1);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(Location::AltFrame::ABSOLUTE)) && (raw_data_2 <= static_cast<int32_t>(Location::AltFrame::ABOVE_TERRAIN))), 2, "argument out of range");
    const Location::AltFrame data_2 = static_cast<Location::AltFrame>(raw_data_2);
    const bool data = static_cast<bool>(ud->change_alt_frame(
            data_2));

    lua_pushboolean(L, data);
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
    const float data = static_cast<float>(ud->get_bearing(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int Location_get_vector_from_origin_NEU(lua_State *L) {
    binding_argcheck(L, 1);
    Location * ud = check_Location(L, 1);
    Vector3f data_5002 = {};
    const bool data = static_cast<bool>(ud->get_vector_from_origin_NEU(
            data_5002));

    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int Location_offset_bearing(lua_State *L) {
    binding_argcheck(L, 3);
    Location * ud = check_Location(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    ud->offset_bearing(
            data_2,
            data_3);

    return 0;
}

static int Location_offset(lua_State *L) {
    binding_argcheck(L, 3);
    Location * ud = check_Location(L, 1);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
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
    const float data = static_cast<float>(ud->get_distance(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
const luaL_Reg AP_HAL__CANFrame_meta[] = {
    {"dlc", AP_HAL__CANFrame_dlc},
    {"data", AP_HAL__CANFrame_data},
    {"id", AP_HAL__CANFrame_id},
    {"isErrorFrame", AP_HAL__CANFrame_isErrorFrame},
    {"isRemoteTransmissionRequest", AP_HAL__CANFrame_isRemoteTransmissionRequest},
    {"isExtended", AP_HAL__CANFrame_isExtended},
    {NULL, NULL}
};
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
const luaL_Reg AP_MotorsMatrix_Scripting_Dynamic__factor_table_meta[] = {
    {"throttle", AP_MotorsMatrix_Scripting_Dynamic__factor_table_throttle},
    {"yaw", AP_MotorsMatrix_Scripting_Dynamic__factor_table_yaw},
    {"pitch", AP_MotorsMatrix_Scripting_Dynamic__factor_table_pitch},
    {"roll", AP_MotorsMatrix_Scripting_Dynamic__factor_table_roll},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

const luaL_Reg AP_HAL__PWMSource_meta[] = {
    {"get_pwm_avg_us", AP_HAL__PWMSource_get_pwm_avg_us},
    {"get_pwm_us", AP_HAL__PWMSource_get_pwm_us},
    {"set_pin", AP_HAL__PWMSource_set_pin},
    {NULL, NULL}
};

const luaL_Reg mavlink_mission_item_int_t_meta[] = {
    {"current", mavlink_mission_item_int_t_current},
    {"frame", mavlink_mission_item_int_t_frame},
    {"command", mavlink_mission_item_int_t_command},
    {"seq", mavlink_mission_item_int_t_seq},
    {"z", mavlink_mission_item_int_t_z},
    {"y", mavlink_mission_item_int_t_y},
    {"x", mavlink_mission_item_int_t_x},
    {"param4", mavlink_mission_item_int_t_param4},
    {"param3", mavlink_mission_item_int_t_param3},
    {"param2", mavlink_mission_item_int_t_param2},
    {"param1", mavlink_mission_item_int_t_param1},
    {NULL, NULL}
};

const luaL_Reg Parameter_meta[] = {
    {"set_and_save", Parameter_set_and_save},
    {"set", Parameter_set},
    {"get", Parameter_get},
    {"init", Parameter_init},
    {NULL, NULL}
};

const luaL_Reg Vector2f_meta[] = {
    {"y", Vector2f_y},
    {"x", Vector2f_x},
    {"copy", Vector2f_copy},
    {"rotate", Vector2f_rotate},
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
    {"copy", Vector3f_copy},
    {"scale", Vector3f_scale},
    {"cross", Vector3f_cross},
    {"dot", Vector3f_dot},
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
    {"alt", Location_alt},
    {"lng", Location_lng},
    {"lat", Location_lat},
    {"copy", Location_copy},
    {"change_alt_frame", Location_change_alt_frame},
    {"get_distance_NE", Location_get_distance_NE},
    {"get_distance_NED", Location_get_distance_NED},
    {"get_bearing", Location_get_bearing},
    {"get_vector_from_origin_NEU", Location_get_vector_from_origin_NEU},
    {"offset_bearing", Location_offset_bearing},
    {"offset", Location_offset},
    {"get_distance", Location_get_distance},
    {NULL, NULL}
};

static int AP__fwversion___fw_hash_str(lua_State *L) {
    switch(lua_gettop(L)) {
        case 1:
            lua_pushstring(L, AP::fwversion().fw_hash_str);
            return 1;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP__fwversion___patch(lua_State *L) {
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, AP::fwversion().patch);
            return 1;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP__fwversion___minor(lua_State *L) {
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, AP::fwversion().minor);
            return 1;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP__fwversion___major(lua_State *L) {
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, AP::fwversion().major);
            return 1;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP__fwversion___vehicle_type(lua_State *L) {
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, AP::fwversion().vehicle_type);
            return 1;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

static int AP__fwversion___fw_short_string(lua_State *L) {
    switch(lua_gettop(L)) {
        case 1:
            lua_pushstring(L, AP::fwversion().fw_short_string);
            return 1;
        default:
            return luaL_argerror(L, lua_gettop(L), "too many arguments");
    }
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
#endif // defined(HAL_BUILD_AP_PERIPH)
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
#if HAL_BUTTON_ENABLED == 1
#endif // HAL_BUTTON_ENABLED == 1
#if HAL_WITH_ESC_TELEM == 1
#endif // HAL_WITH_ESC_TELEM == 1
#if AP_OPTICALFLOW_ENABLED
#endif // AP_OPTICALFLOW_ENABLED
#if ENABLE_ONVIF == 1
#endif // ENABLE_ONVIF == 1
#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
#if HAL_PROXIMITY_ENABLED == 1
#endif // HAL_PROXIMITY_ENABLED == 1
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP__motors___set_frame_string(lua_State *L) {
    binding_argcheck(L, 2);
    const char * data_2 = luaL_checkstring(L, 2);
    AP::motors()->set_frame_string(
            data_2);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if defined(HAL_BUILD_AP_PERIPH)
static int AP_Periph_FW_get_vehicle_state(lua_State *L) {
    AP_Periph_FW * ud = AP_Periph_FW::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "periph");
    }

    binding_argcheck(L, 1);
    const uint32_t data = static_cast<uint32_t>(ud->get_vehicle_state());

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}
#endif // defined(HAL_BUILD_AP_PERIPH)

#if defined(HAL_BUILD_AP_PERIPH)
static int AP_Periph_FW_get_yaw_earth(lua_State *L) {
    AP_Periph_FW * ud = AP_Periph_FW::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "periph");
    }

    binding_argcheck(L, 1);
    const float data = static_cast<float>(ud->get_yaw_earth());

    lua_pushnumber(L, data);
    return 1;
}
#endif // defined(HAL_BUILD_AP_PERIPH)

static int AP_InertialSensor_get_temperature(lua_State *L) {
    AP_InertialSensor * ud = AP_InertialSensor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ins");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(INS_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = static_cast<float>(ud->get_temperature(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_Scripting_Dynamic_load_factors(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic * ud = AP_MotorsMatrix_Scripting_Dynamic::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "Motors_dynamic");
    }

    binding_argcheck(L, 2);
    AP_MotorsMatrix_Scripting_Dynamic::factor_table & data_2 = *check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(L, 2);
    ud->load_factors(
            data_2);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_Scripting_Dynamic_add_motor(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic * ud = AP_MotorsMatrix_Scripting_Dynamic::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "Motors_dynamic");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN((AP_MOTORS_MAX_NUM_MOTORS-1), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    ud->add_motor(
            data_2,
            data_3);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_Scripting_Dynamic_init(lua_State *L) {
    AP_MotorsMatrix_Scripting_Dynamic * ud = AP_MotorsMatrix_Scripting_Dynamic::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "Motors_dynamic");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->init(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

static int hal_analogin_channel(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::AnalogSource *data = hal.analogin->channel(            ANALOG_INPUT_NONE);

    if (data == NULL) {
        return 0;
    }
    new_AP_HAL__AnalogSource(L);
    *check_AP_HAL__AnalogSource(L, -1) = data;
    return 1;
}

static int hal_gpio_pinMode(lua_State *L) {
    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(1, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    hal.gpio->pinMode(
            data_2,
            data_3);

    return 0;
}

static int hal_gpio_toggle(lua_State *L) {
    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    hal.gpio->toggle(
            data_2);

    return 0;
}

static int hal_gpio_write(lua_State *L) {
    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(1, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    hal.gpio->write(
            data_2,
            data_3);

    return 0;
}

static int hal_gpio_read(lua_State *L) {
    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(hal.gpio->read(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
static int AP_MotorsMatrix_6DoF_Scripting_add_motor(lua_State *L) {
    AP_MotorsMatrix_6DoF_Scripting * ud = AP_MotorsMatrix_6DoF_Scripting::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "Motors_6DoF");
    }

    binding_argcheck(L, 10);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, INT8_MIN)) && (raw_data_2 <= MIN((AP_MOTORS_MAX_NUM_MOTORS-1), INT8_MAX))), 2, "argument out of range");
    const int8_t data_2 = static_cast<int8_t>(raw_data_2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    const float raw_data_4 = luaL_checknumber(L, 4);
    const float data_4 = raw_data_4;
    const float raw_data_5 = luaL_checknumber(L, 5);
    const float data_5 = raw_data_5;
    const float raw_data_6 = luaL_checknumber(L, 6);
    const float data_6 = raw_data_6;
    const float raw_data_7 = luaL_checknumber(L, 7);
    const float data_7 = raw_data_7;
    const float raw_data_8 = luaL_checknumber(L, 8);
    const float data_8 = raw_data_8;
    const bool data_9 = static_cast<bool>(lua_toboolean(L, 9));
    const lua_Integer raw_data_10 = luaL_checkinteger(L, 10);
    luaL_argcheck(L, ((raw_data_10 >= MAX(0, 0)) && (raw_data_10 <= MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 10, "argument out of range");
    const uint8_t data_10 = static_cast<uint8_t>(raw_data_10);
    ud->add_motor(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6,
            data_7,
            data_8,
            data_9,
            data_10);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
static int AP_MotorsMatrix_6DoF_Scripting_init(lua_State *L) {
    AP_MotorsMatrix_6DoF_Scripting * ud = AP_MotorsMatrix_6DoF_Scripting::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "Motors_6DoF");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->init(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
static int AC_AttitudeControl_Multi_6DoF_set_offset_roll_pitch(lua_State *L) {
    AC_AttitudeControl_Multi_6DoF * ud = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "attitude_control");
    }

    binding_argcheck(L, 3);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    ud->set_offset_roll_pitch(
            data_2,
            data_3);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
static int AC_AttitudeControl_Multi_6DoF_set_forward_enable(lua_State *L) {
    AC_AttitudeControl_Multi_6DoF * ud = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "attitude_control");
    }

    binding_argcheck(L, 2);
    const bool data_2 = static_cast<bool>(lua_toboolean(L, 2));
    ud->set_forward_enable(
            data_2);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
static int AC_AttitudeControl_Multi_6DoF_set_lateral_enable(lua_State *L) {
    AC_AttitudeControl_Multi_6DoF * ud = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "attitude_control");
    }

    binding_argcheck(L, 2);
    const bool data_2 = static_cast<bool>(lua_toboolean(L, 2));
    ud->set_lateral_enable(
            data_2);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

static int AP_Frsky_SPort_prep_number(lua_State *L) {
    AP_Frsky_SPort * ud = AP_Frsky_SPort::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "frsky_sport");
    }

    binding_argcheck(L, 4);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(INT32_MIN, INT32_MIN)) && (raw_data_2 <= MIN(INT32_MAX, INT32_MAX))), 2, "argument out of range");
    const int32_t data_2 = raw_data_2;
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT8_MAX, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(0, 0)) && (raw_data_4 <= MIN(UINT8_MAX, UINT8_MAX))), 4, "argument out of range");
    const uint8_t data_4 = static_cast<uint8_t>(raw_data_4);
    const uint16_t data = static_cast<uint16_t>(ud->prep_number(
            data_2,
            data_3,
            data_4));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Frsky_SPort_sport_telemetry_push(lua_State *L) {
    AP_Frsky_SPort * ud = AP_Frsky_SPort::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "frsky_sport");
    }

    binding_argcheck(L, 5);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT8_MAX, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(0, 0)) && (raw_data_4 <= MIN(UINT16_MAX, UINT16_MAX))), 4, "argument out of range");
    const uint16_t data_4 = static_cast<uint16_t>(raw_data_4);
    const lua_Integer raw_data_5 = luaL_checkinteger(L, 5);
    luaL_argcheck(L, ((raw_data_5 >= MAX(-INT32_MAX, INT32_MIN)) && (raw_data_5 <= MIN(INT32_MAX, INT32_MAX))), 5, "argument out of range");
    const int32_t data_5 = raw_data_5;
    const bool data = static_cast<bool>(ud->sport_telemetry_push(
            data_2,
            data_3,
            data_4,
            data_5));

    lua_pushboolean(L, data);
    return 1;
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_set_throttle_factor(lua_State *L) {
    AP_MotorsMatrix * ud = AP_MotorsMatrix::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "MotorsMatrix");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, INT8_MIN)) && (raw_data_2 <= MIN((AP_MOTORS_MAX_NUM_MOTORS-1), INT8_MAX))), 2, "argument out of range");
    const int8_t data_2 = static_cast<int8_t>(raw_data_2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, -INFINITY)) && (raw_data_3 <= MIN(FLT_MAX, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    const bool data = static_cast<bool>(ud->set_throttle_factor(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_add_motor_raw(lua_State *L) {
    AP_MotorsMatrix * ud = AP_MotorsMatrix::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "MotorsMatrix");
    }

    binding_argcheck(L, 6);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, INT8_MIN)) && (raw_data_2 <= MIN((AP_MOTORS_MAX_NUM_MOTORS-1), INT8_MAX))), 2, "argument out of range");
    const int8_t data_2 = static_cast<int8_t>(raw_data_2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    const float raw_data_4 = luaL_checknumber(L, 4);
    const float data_4 = raw_data_4;
    const float raw_data_5 = luaL_checknumber(L, 5);
    const float data_5 = raw_data_5;
    const lua_Integer raw_data_6 = luaL_checkinteger(L, 6);
    luaL_argcheck(L, ((raw_data_6 >= MAX(0, 0)) && (raw_data_6 <= MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 6, "argument out of range");
    const uint8_t data_6 = static_cast<uint8_t>(raw_data_6);
    ud->add_motor_raw(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6);

    return 0;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
static int AP_MotorsMatrix_init(lua_State *L) {
    AP_MotorsMatrix * ud = AP_MotorsMatrix::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "MotorsMatrix");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_MOTORS_MAX_NUM_MOTORS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->init(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
static int QuadPlane_in_assisted_flight(lua_State *L) {
    QuadPlane * ud = QuadPlane::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "quadplane");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->in_assisted_flight());

    lua_pushboolean(L, data);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
static int QuadPlane_in_vtol_mode(lua_State *L) {
    QuadPlane * ud = QuadPlane::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "quadplane");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->in_vtol_mode());

    lua_pushboolean(L, data);
    return 1;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED

static int ScriptingLED_get_rgb(lua_State *L) {
    ScriptingLED * ud = ScriptingLED::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "LED");
    }

    binding_argcheck(L, 1);
    uint8_t data_5002;
    uint8_t data_5003;
    uint8_t data_5004;
    ud->get_rgb(
            data_5002,
            data_5003,
            data_5004);

    lua_pushinteger(L, data_5002);
    lua_pushinteger(L, data_5003);
    lua_pushinteger(L, data_5004);
    return 3;
}

#if HAL_BUTTON_ENABLED == 1
static int AP_Button_get_button_state(lua_State *L) {
    AP_Button * ud = AP_Button::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "button");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(AP_BUTTON_NUM_PINS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->get_button_state(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}
#endif // HAL_BUTTON_ENABLED == 1

static int AP_RPM_get_rpm(lua_State *L) {
    AP_RPM * ud = AP_RPM::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "RPM");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(RPM_MAX_INSTANCES, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->get_rpm(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_Mission_clear(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->clear());

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Mission_set_item(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT16_MAX, UINT16_MAX))), 2, "argument out of range");
    const uint16_t data_2 = static_cast<uint16_t>(raw_data_2);
    mavlink_mission_item_int_t & data_3 = *check_mavlink_mission_item_int_t(L, 3);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_item(
            data_2,
            data_3));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Mission_get_item(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT16_MAX, UINT16_MAX))), 2, "argument out of range");
    const uint16_t data_2 = static_cast<uint16_t>(raw_data_2);
    mavlink_mission_item_int_t data_5003 = {};
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_item(
            data_2,
            data_5003));

    AP::scheduler().get_semaphore().give();
    if (data) {
        new_mavlink_mission_item_int_t(L);
        *check_mavlink_mission_item_int_t(L, -1) = data_5003;
        return 1;
    }
    return 0;
}

static int AP_Mission_num_commands(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint16_t data = static_cast<uint16_t>(ud->num_commands());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Mission_get_current_do_cmd_id(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint16_t data = static_cast<uint16_t>(ud->get_current_do_cmd_id());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Mission_get_current_nav_id(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint16_t data = static_cast<uint16_t>(ud->get_current_nav_id());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Mission_get_prev_nav_cmd_id(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint16_t data = static_cast<uint16_t>(ud->get_prev_nav_cmd_id());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Mission_set_current_cmd(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN((ud->num_commands()-1), UINT16_MAX))), 2, "argument out of range");
    const uint16_t data_2 = static_cast<uint16_t>(raw_data_2);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_current_cmd(
            data_2));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Mission_get_current_nav_index(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint16_t data = static_cast<uint16_t>(ud->get_current_nav_index());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Mission_state(lua_State *L) {
    AP_Mission * ud = AP_Mission::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "mission");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint8_t data = static_cast<uint8_t>(ud->state());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Param_add_param(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "param");
    }

    binding_argcheck(L, 5);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(200, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(1, 0)) && (raw_data_3 <= MIN(63, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const char * data_4 = luaL_checkstring(L, 4);
    const float raw_data_5 = luaL_checknumber(L, 5);
    const float data_5 = raw_data_5;
    const bool data = static_cast<bool>(ud->add_param(
            data_2,
            data_3,
            data_4,
            data_5));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Param_add_table(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "param");
    }

    binding_argcheck(L, 4);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(200, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const char * data_3 = luaL_checkstring(L, 3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(1, 0)) && (raw_data_4 <= MIN(63, UINT8_MAX))), 4, "argument out of range");
    const uint8_t data_4 = static_cast<uint8_t>(raw_data_4);
    const bool data = static_cast<bool>(ud->add_table(
            data_2,
            data_3,
            data_4));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Param_set_and_save_by_name(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "param");
    }

    binding_argcheck(L, 3);
    const char * data_2 = luaL_checkstring(L, 2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    const bool data = static_cast<bool>(ud->set_and_save_by_name(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Param_set_by_name(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "param");
    }

    binding_argcheck(L, 3);
    const char * data_2 = luaL_checkstring(L, 2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    const bool data = static_cast<bool>(ud->set_by_name(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Param_get(lua_State *L) {
    AP_Param * ud = AP_Param::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "param");
    }

    binding_argcheck(L, 2);
    const char * data_2 = luaL_checkstring(L, 2);
    float data_5003;
    const bool data = static_cast<bool>(ud->get(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_usage_seconds(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint32_t data_5003;
    const bool data = static_cast<bool>(ud->get_usage_seconds(
            data_2,
            data_5003));

    if (data) {
        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data_5003;
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_consumption_mah(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->get_consumption_mah(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_voltage(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->get_voltage(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_current(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->get_current(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_motor_temperature(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    int16_t data_5003;
    const bool data = static_cast<bool>(ud->get_motor_temperature(
            data_2,
            data_5003));

    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_temperature(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    int16_t data_5003;
    const bool data = static_cast<bool>(ud->get_temperature(
            data_2,
            data_5003));

    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if HAL_WITH_ESC_TELEM == 1
static int AP_ESC_Telem_get_rpm(lua_State *L) {
    AP_ESC_Telem * ud = AP_ESC_Telem::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "esc_telem");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->get_rpm(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // HAL_WITH_ESC_TELEM == 1

#if AP_OPTICALFLOW_ENABLED
static int OpticalFlow_quality(lua_State *L) {
    OpticalFlow * ud = OpticalFlow::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "optical_flow");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->quality());

    lua_pushinteger(L, data);
    return 1;
}
#endif // AP_OPTICALFLOW_ENABLED

#if AP_OPTICALFLOW_ENABLED
static int OpticalFlow_healthy(lua_State *L) {
    OpticalFlow * ud = OpticalFlow::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "optical_flow");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->healthy());

    lua_pushboolean(L, data);
    return 1;
}
#endif // AP_OPTICALFLOW_ENABLED

#if AP_OPTICALFLOW_ENABLED
static int OpticalFlow_enabled(lua_State *L) {
    OpticalFlow * ud = OpticalFlow::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "optical_flow");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->enabled());

    lua_pushboolean(L, data);
    return 1;
}
#endif // AP_OPTICALFLOW_ENABLED

static int AP_Baro_get_external_temperature(lua_State *L) {
    AP_Baro * ud = AP_Baro::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "baro");
    }

    binding_argcheck(L, 1);
    const float data = static_cast<float>(ud->get_external_temperature());

    lua_pushnumber(L, data);
    return 1;
}

static int AP_Baro_get_temperature(lua_State *L) {
    AP_Baro * ud = AP_Baro::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "baro");
    }

    binding_argcheck(L, 1);
    const float data = static_cast<float>(ud->get_temperature());

    lua_pushnumber(L, data);
    return 1;
}

static int AP_Baro_get_pressure(lua_State *L) {
    AP_Baro * ud = AP_Baro::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "baro");
    }

    binding_argcheck(L, 1);
    const float data = static_cast<float>(ud->get_pressure());

    lua_pushnumber(L, data);
    return 1;
}

static int AP_SerialManager_find_serial(lua_State *L) {
    AP_SerialManager * ud = AP_SerialManager::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "serial");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    AP_HAL::UARTDriver *data = ud->find_serial(
            AP_SerialManager::SerialProtocol_Scripting,
            data_2);

    if (data == NULL) {
        return 0;
    }
    new_AP_HAL__UARTDriver(L);
    *check_AP_HAL__UARTDriver(L, -1) = data;
    return 1;
}

static int RC_Channels_lua_rc_channel(lua_State *L) {
    RC_Channels * ud = RC_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rc");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(NUM_RC_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    AP::scheduler().get_semaphore().take_blocking();
    RC_Channel *data = ud->lua_rc_channel(
            data_2);

    AP::scheduler().get_semaphore().give();
    if (data == NULL) {
        return 0;
    }
    new_RC_Channel(L);
    *check_RC_Channel(L, -1) = data;
    return 1;
}

static int RC_Channels_has_valid_input(lua_State *L) {
    RC_Channels * ud = RC_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rc");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->has_valid_input());

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int RC_Channels_run_aux_function(lua_State *L) {
    RC_Channels * ud = RC_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rc");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(0)) && (raw_data_2 <= static_cast<int32_t>(UINT16_MAX))), 2, "argument out of range");
    const RC_Channel::AUX_FUNC data_2 = static_cast<RC_Channel::AUX_FUNC>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= static_cast<int32_t>(RC_Channel::AuxSwitchPos::LOW)) && (raw_data_3 <= static_cast<int32_t>(RC_Channel::AuxSwitchPos::HIGH))), 3, "argument out of range");
    const RC_Channel::AuxSwitchPos data_3 = static_cast<RC_Channel::AuxSwitchPos>(raw_data_3);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->run_aux_function(
            data_2,
            data_3,
            RC_Channel::AuxFuncTriggerSource::SCRIPTING));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int RC_Channels_find_channel_for_option(lua_State *L) {
    RC_Channels * ud = RC_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rc");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(0)) && (raw_data_2 <= static_cast<int32_t>(UINT16_MAX))), 2, "argument out of range");
    const RC_Channel::AUX_FUNC data_2 = static_cast<RC_Channel::AUX_FUNC>(raw_data_2);
    AP::scheduler().get_semaphore().take_blocking();
    RC_Channel *data = ud->find_channel_for_option(
            data_2);

    AP::scheduler().get_semaphore().give();
    if (data == NULL) {
        return 0;
    }
    new_RC_Channel(L);
    *check_RC_Channel(L, -1) = data;
    return 1;
}

static int RC_Channels_get_pwm(lua_State *L) {
    RC_Channels * ud = RC_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rc");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(NUM_RC_CHANNELS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint16_t data_5003;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_pwm(
            data_2,
            data_5003));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}

static int SRV_Channels_set_range(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT16_MAX, UINT16_MAX))), 3, "argument out of range");
    const uint16_t data_3 = static_cast<uint16_t>(raw_data_3);
    ud->set_range(
            data_2,
            data_3);

    return 0;
}

static int SRV_Channels_set_angle(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT16_MAX, UINT16_MAX))), 3, "argument out of range");
    const uint16_t data_3 = static_cast<uint16_t>(raw_data_3);
    ud->set_angle(
            data_2,
            data_3);

    return 0;
}

static int SRV_Channels_set_output_norm(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-1, -INFINITY)) && (raw_data_3 <= MIN(1, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    ud->set_output_norm(
            data_2,
            data_3);

    return 0;
}

static int SRV_Channels_get_output_scaled(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    const float data = static_cast<float>(ud->get_output_scaled(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int SRV_Channels_get_output_pwm(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    uint16_t data_5003;
    const bool data = static_cast<bool>(ud->get_output_pwm(
            data_2,
            data_5003));

    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}

static int SRV_Channels_set_output_scaled(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-FLT_MAX, -INFINITY)) && (raw_data_3 <= MIN(FLT_MAX, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    ud->set_output_scaled(
            data_2,
            data_3);

    return 0;
}

static int SRV_Channels_set_output_pwm_chan_timeout(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 4);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS-1, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT16_MAX, UINT16_MAX))), 3, "argument out of range");
    const uint16_t data_3 = static_cast<uint16_t>(raw_data_3);
    const lua_Integer raw_data_4 = luaL_checkinteger(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(0, 0)) && (raw_data_4 <= MIN(UINT16_MAX, UINT16_MAX))), 4, "argument out of range");
    const uint16_t data_4 = static_cast<uint16_t>(raw_data_4);
    ud->set_output_pwm_chan_timeout(
            data_2,
            data_3,
            data_4);

    return 0;
}

static int SRV_Channels_set_output_pwm_chan(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(NUM_SERVO_CHANNELS-1, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT16_MAX, UINT16_MAX))), 3, "argument out of range");
    const uint16_t data_3 = static_cast<uint16_t>(raw_data_3);
    ud->set_output_pwm_chan(
            data_2,
            data_3);

    return 0;
}

static int SRV_Channels_set_output_pwm(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT16_MAX, UINT16_MAX))), 3, "argument out of range");
    const uint16_t data_3 = static_cast<uint16_t>(raw_data_3);
    ud->set_output_pwm(
            data_2,
            data_3);

    return 0;
}

static int SRV_Channels_find_channel(lua_State *L) {
    SRV_Channels * ud = SRV_Channels::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "SRV_Channels");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(SRV_Channel::k_none)) && (raw_data_2 <= static_cast<int32_t>(SRV_Channel::k_nr_aux_servo_functions-1))), 2, "argument out of range");
    const SRV_Channel::Aux_servo_function_t data_2 = static_cast<SRV_Channel::Aux_servo_function_t>(raw_data_2);
    uint8_t data_5003;
    const bool data = static_cast<bool>(ud->find_channel(
            data_2,
            data_5003));

    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_SerialLED_send(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "serialLED");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(16, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->send(
            data_2);

    return 0;
}

static int AP_SerialLED_set_RGB(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "serialLED");
    }

    binding_argcheck(L, 6);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(16, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-1, INT8_MIN)) && (raw_data_3 <= MIN(INT8_MAX, INT8_MAX))), 3, "argument out of range");
    const int8_t data_3 = static_cast<int8_t>(raw_data_3);
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

static int AP_SerialLED_set_num_profiled(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "serialLED");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(16, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(AP_SERIALLED_MAX_LEDS, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->set_num_profiled(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_SerialLED_set_num_neopixel(lua_State *L) {
    AP_SerialLED * ud = AP_SerialLED::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "serialLED");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1, 0)) && (raw_data_2 <= MIN(16, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(AP_SERIALLED_MAX_LEDS, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->set_num_neopixel(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_target_throttle_rate_rpy(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 5);
    const float raw_data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(-100, -INFINITY)) && (raw_data_2 <= MIN(100, INFINITY))), 2, "argument out of range");
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    const float raw_data_4 = luaL_checknumber(L, 4);
    const float data_4 = raw_data_4;
    const float raw_data_5 = luaL_checknumber(L, 5);
    const float data_5 = raw_data_5;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_throttle_rate_rpy(
            data_2,
            data_3,
            data_4,
            data_5));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_nav_script_time_done(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    const uint16_t data_2 = static_cast<uint16_t>(raw_data_2);
    AP::scheduler().get_semaphore().take_blocking();
    ud->nav_script_time_done(
            data_2);

    AP::scheduler().get_semaphore().give();
    return 0;
}

static int AP_Vehicle_nav_script_time(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    uint16_t data_5002;
    uint8_t data_5003;
    float data_5004;
    float data_5005;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->nav_script_time(
            data_5002,
            data_5003,
            data_5004,
            data_5005));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushinteger(L, data_5002);
        lua_pushinteger(L, data_5003);
        lua_pushnumber(L, data_5004);
        lua_pushnumber(L, data_5005);
        return 4;
    }
    return 0;
}

static int AP_Vehicle_get_pan_tilt_norm(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    float data_5002;
    float data_5003;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_pan_tilt_norm(
            data_5002,
            data_5003));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        lua_pushnumber(L, data_5003);
        return 2;
    }
    return 0;
}

static int AP_Vehicle_get_wp_crosstrack_error_m(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    float data_5002;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_wp_crosstrack_error_m(
            data_5002));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_Vehicle_get_wp_bearing_deg(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    float data_5002;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_wp_bearing_deg(
            data_5002));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_Vehicle_get_wp_distance_m(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    float data_5002;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_wp_distance_m(
            data_5002));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_Vehicle_set_steering_and_throttle(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 3);
    const float raw_data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(-1, -INFINITY)) && (raw_data_2 <= MIN(1, INFINITY))), 2, "argument out of range");
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-1, -INFINITY)) && (raw_data_3 <= MIN(1, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_steering_and_throttle(
            data_2,
            data_3));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_circle_rate(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_circle_rate(
            data_2));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_get_circle_radius(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    float data_5002;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_circle_radius(
            data_5002));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_Vehicle_set_target_angle_and_climbrate(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 7);
    const float raw_data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(-180, -INFINITY)) && (raw_data_2 <= MIN(180, INFINITY))), 2, "argument out of range");
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(-90, -INFINITY)) && (raw_data_3 <= MIN(90, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    const float raw_data_4 = luaL_checknumber(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(-360, -INFINITY)) && (raw_data_4 <= MIN(360, INFINITY))), 4, "argument out of range");
    const float data_4 = raw_data_4;
    const float raw_data_5 = luaL_checknumber(L, 5);
    const float data_5 = raw_data_5;
    const bool data_6 = static_cast<bool>(lua_toboolean(L, 6));
    const float raw_data_7 = luaL_checknumber(L, 7);
    const float data_7 = raw_data_7;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_angle_and_climbrate(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6,
            data_7));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_target_velocity_NED(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_velocity_NED(
            data_2));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_target_velaccel_NED(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 8);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    Vector3f & data_3 = *check_Vector3f(L, 3);
    const bool data_4 = static_cast<bool>(lua_toboolean(L, 4));
    const float raw_data_5 = luaL_checknumber(L, 5);
    luaL_argcheck(L, ((raw_data_5 >= MAX(-360, -INFINITY)) && (raw_data_5 <= MIN(+360, INFINITY))), 5, "argument out of range");
    const float data_5 = raw_data_5;
    const bool data_6 = static_cast<bool>(lua_toboolean(L, 6));
    const float raw_data_7 = luaL_checknumber(L, 7);
    const float data_7 = raw_data_7;
    const bool data_8 = static_cast<bool>(lua_toboolean(L, 8));
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_velaccel_NED(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6,
            data_7,
            data_8));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_target_posvelaccel_NED(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 9);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    Vector3f & data_3 = *check_Vector3f(L, 3);
    Vector3f & data_4 = *check_Vector3f(L, 4);
    const bool data_5 = static_cast<bool>(lua_toboolean(L, 5));
    const float raw_data_6 = luaL_checknumber(L, 6);
    luaL_argcheck(L, ((raw_data_6 >= MAX(-360, -INFINITY)) && (raw_data_6 <= MIN(+360, INFINITY))), 6, "argument out of range");
    const float data_6 = raw_data_6;
    const bool data_7 = static_cast<bool>(lua_toboolean(L, 7));
    const float raw_data_8 = luaL_checknumber(L, 8);
    const float data_8 = raw_data_8;
    const bool data_9 = static_cast<bool>(lua_toboolean(L, 9));
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_posvelaccel_NED(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6,
            data_7,
            data_8,
            data_9));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_target_posvel_NED(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 3);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    Vector3f & data_3 = *check_Vector3f(L, 3);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_posvel_NED(
            data_2,
            data_3));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_set_target_pos_NED(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 8);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    const bool data_3 = static_cast<bool>(lua_toboolean(L, 3));
    const float raw_data_4 = luaL_checknumber(L, 4);
    luaL_argcheck(L, ((raw_data_4 >= MAX(-360, -INFINITY)) && (raw_data_4 <= MIN(+360, INFINITY))), 4, "argument out of range");
    const float data_4 = raw_data_4;
    const bool data_5 = static_cast<bool>(lua_toboolean(L, 5));
    const float raw_data_6 = luaL_checknumber(L, 6);
    const float data_6 = raw_data_6;
    const bool data_7 = static_cast<bool>(lua_toboolean(L, 7));
    const bool data_8 = static_cast<bool>(lua_toboolean(L, 8));
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_pos_NED(
            data_2,
            data_3,
            data_4,
            data_5,
            data_6,
            data_7,
            data_8));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_get_target_location(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    Location data_5002 = {};
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_target_location(
            data_5002));

    AP::scheduler().get_semaphore().give();
    if (data) {
        new_Location(L);
        *check_Location(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int AP_Vehicle_set_target_location(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    Location & data_2 = *check_Location(L, 2);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_target_location(
            data_2));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_start_takeoff(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    const float raw_data_2 = luaL_checknumber(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX((-LOCATION_ALT_MAX_M*100+1), -INFINITY)) && (raw_data_2 <= MIN((LOCATION_ALT_MAX_M*100-1), INFINITY))), 2, "argument out of range");
    const float data_2 = raw_data_2;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->start_takeoff(
            data_2));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_get_control_output(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(AP_Vehicle::ControlOutput::Roll)) && (raw_data_2 <= static_cast<int32_t>(((uint32_t)AP_Vehicle::ControlOutput::Last_ControlOutput-1)))), 2, "argument out of range");
    const AP_Vehicle::ControlOutput data_2 = static_cast<AP_Vehicle::ControlOutput>(raw_data_2);
    float data_5003;
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_control_output(
            data_2,
            data_5003));

    AP::scheduler().get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_Vehicle_get_time_flying_ms(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint32_t data = static_cast<uint32_t>(ud->get_time_flying_ms());

    AP::scheduler().get_semaphore().give();
        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_Vehicle_get_likely_flying(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_likely_flying());

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_Vehicle_get_control_mode_reason(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint8_t data = static_cast<uint8_t>(ud->get_control_mode_reason());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Vehicle_get_mode(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 1);
    AP::scheduler().get_semaphore().take_blocking();
    const uint8_t data = static_cast<uint8_t>(ud->get_mode());

    AP::scheduler().get_semaphore().give();
    lua_pushinteger(L, data);
    return 1;
}

static int AP_Vehicle_set_mode(lua_State *L) {
    AP_Vehicle * ud = AP_Vehicle::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "vehicle");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    AP::scheduler().get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_mode(
            data_2,
            ModeReason::SCRIPTING));

    AP::scheduler().get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

#if ENABLE_ONVIF == 1
static int AP_ONVIF_get_pan_tilt_limit_max(lua_State *L) {
    AP_ONVIF * ud = AP_ONVIF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "onvif");
    }

    binding_argcheck(L, 1);
    const Vector2f &data = ud->get_pan_tilt_limit_max();

    new_Vector2f(L);
    *check_Vector2f(L, -1) = data;
    return 1;
}
#endif // ENABLE_ONVIF == 1

#if ENABLE_ONVIF == 1
static int AP_ONVIF_get_pan_tilt_limit_min(lua_State *L) {
    AP_ONVIF * ud = AP_ONVIF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "onvif");
    }

    binding_argcheck(L, 1);
    const Vector2f &data = ud->get_pan_tilt_limit_min();

    new_Vector2f(L);
    *check_Vector2f(L, -1) = data;
    return 1;
}
#endif // ENABLE_ONVIF == 1

#if ENABLE_ONVIF == 1
static int AP_ONVIF_set_absolutemove(lua_State *L) {
    AP_ONVIF * ud = AP_ONVIF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "onvif");
    }

    binding_argcheck(L, 4);
    const float raw_data_2 = luaL_checknumber(L, 2);
    const float data_2 = raw_data_2;
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    const float raw_data_4 = luaL_checknumber(L, 4);
    const float data_4 = raw_data_4;
    const bool data = static_cast<bool>(ud->set_absolutemove(
            data_2,
            data_3,
            data_4));

    lua_pushboolean(L, data);
    return 1;
}
#endif // ENABLE_ONVIF == 1

#if ENABLE_ONVIF == 1
static int AP_ONVIF_start(lua_State *L) {
    AP_ONVIF * ud = AP_ONVIF::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "onvif");
    }

    binding_argcheck(L, 4);
    const char * data_2 = luaL_checkstring(L, 2);
    const char * data_3 = luaL_checkstring(L, 3);
    const char * data_4 = luaL_checkstring(L, 4);
    const bool data = static_cast<bool>(ud->start(
            data_2,
            data_3,
            data_4));

    lua_pushboolean(L, data);
    return 1;
}
#endif // ENABLE_ONVIF == 1

static int GCS_send_named_float(lua_State *L) {
    GCS * ud = GCS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gcs");
    }

    binding_argcheck(L, 3);
    const char * data_2 = luaL_checkstring(L, 2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    const float data_3 = raw_data_3;
    ud->send_named_float(
            data_2,
            data_3);

    return 0;
}

static int GCS_set_message_interval(lua_State *L) {
    GCS * ud = GCS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gcs");
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
        return not_supported_error(L, 1, "gcs");
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
        return not_supported_error(L, 1, "relay");
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
        return not_supported_error(L, 1, "relay");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->enabled(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Relay_off(lua_State *L) {
    AP_Relay * ud = AP_Relay::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "relay");
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
        return not_supported_error(L, 1, "relay");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(AP_RELAY_NUM_RELAYS, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->on(
            data_2);

    return 0;
}

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
static int AP_Terrain_height_above_terrain(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "terrain");
    }

    binding_argcheck(L, 2);
    float data_5002;
    const bool data_3 = static_cast<bool>(lua_toboolean(L, 3));
    const bool data = static_cast<bool>(ud->height_above_terrain(
            data_5002,
            data_3));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
static int AP_Terrain_height_terrain_difference_home(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "terrain");
    }

    binding_argcheck(L, 2);
    float data_5002;
    const bool data_3 = static_cast<bool>(lua_toboolean(L, 3));
    const bool data = static_cast<bool>(ud->height_terrain_difference_home(
            data_5002,
            data_3));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
static int AP_Terrain_height_amsl(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "terrain");
    }

    binding_argcheck(L, 3);
    Location & data_2 = *check_Location(L, 2);
    float data_5003;
    const bool data_4 = static_cast<bool>(lua_toboolean(L, 4));
    const bool data = static_cast<bool>(ud->height_amsl(
            data_2,
            data_5003,
            data_4));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
static int AP_Terrain_status(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "terrain");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->status());

    lua_pushinteger(L, data);
    return 1;
}
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
static int AP_Terrain_enabled(lua_State *L) {
    AP_Terrain * ud = AP_Terrain::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "terrain");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->enabled());

    lua_pushboolean(L, data);
    return 1;
}
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

static int RangeFinder_get_pos_offset_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const Vector3f &data = ud->get_pos_offset_orient(
            data_2);

    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int RangeFinder_has_data_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const bool data = static_cast<bool>(ud->has_data_orient(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int RangeFinder_status_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const uint8_t data = static_cast<uint8_t>(ud->status_orient(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int RangeFinder_ground_clearance_cm_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->ground_clearance_cm_orient(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int RangeFinder_min_distance_cm_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->min_distance_cm_orient(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int RangeFinder_max_distance_cm_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->max_distance_cm_orient(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int RangeFinder_distance_cm_orient(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->distance_cm_orient(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int RangeFinder_has_orientation(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= static_cast<int32_t>(ROTATION_NONE)) && (raw_data_2 <= static_cast<int32_t>(ROTATION_MAX-1))), 2, "argument out of range");
    const Rotation data_2 = static_cast<Rotation>(raw_data_2);
    const bool data = static_cast<bool>(ud->has_orientation(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int RangeFinder_num_sensors(lua_State *L) {
    RangeFinder * ud = RangeFinder::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "rangefinder");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->num_sensors());

    lua_pushinteger(L, data);
    return 1;
}

#if HAL_PROXIMITY_ENABLED == 1
static int AP_Proximity_get_object_angle_and_distance(lua_State *L) {
    AP_Proximity * ud = AP_Proximity::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "proximity");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    float data_5004;
    const bool data = static_cast<bool>(ud->get_object_angle_and_distance(
            data_2,
            data_5003,
            data_5004));

    if (data) {
        lua_pushnumber(L, data_5003);
        lua_pushnumber(L, data_5004);
        return 2;
    }
    return 0;
}
#endif // HAL_PROXIMITY_ENABLED == 1

#if HAL_PROXIMITY_ENABLED == 1
static int AP_Proximity_get_closest_object(lua_State *L) {
    AP_Proximity * ud = AP_Proximity::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "proximity");
    }

    binding_argcheck(L, 1);
    float data_5002;
    float data_5003;
    const bool data = static_cast<bool>(ud->get_closest_object(
            data_5002,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5002);
        lua_pushnumber(L, data_5003);
        return 2;
    }
    return 0;
}
#endif // HAL_PROXIMITY_ENABLED == 1

#if HAL_PROXIMITY_ENABLED == 1
static int AP_Proximity_get_object_count(lua_State *L) {
    AP_Proximity * ud = AP_Proximity::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "proximity");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->get_object_count());

    lua_pushinteger(L, data);
    return 1;
}
#endif // HAL_PROXIMITY_ENABLED == 1

#if HAL_PROXIMITY_ENABLED == 1
static int AP_Proximity_num_sensors(lua_State *L) {
    AP_Proximity * ud = AP_Proximity::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "proximity");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->num_sensors());

    lua_pushinteger(L, data);
    return 1;
}
#endif // HAL_PROXIMITY_ENABLED == 1

#if HAL_PROXIMITY_ENABLED == 1
static int AP_Proximity_get_status(lua_State *L) {
    AP_Proximity * ud = AP_Proximity::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "proximity");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->get_status());

    lua_pushinteger(L, data);
    return 1;
}
#endif // HAL_PROXIMITY_ENABLED == 1

static int AP_Notify_handle_rgb_id(lua_State *L) {
    AP_Notify * ud = AP_Notify::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "notify");
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
    ud->handle_rgb_id(
            data_2,
            data_3,
            data_4,
            data_5);

    return 0;
}

static int AP_Notify_handle_rgb(lua_State *L) {
    AP_Notify * ud = AP_Notify::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "notify");
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
        return not_supported_error(L, 1, "notify");
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
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 1);
    uint8_t data_5002;
    const bool data = static_cast<bool>(ud->first_unconfigured_gps(
            data_5002));

    if (data) {
        lua_pushinteger(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_GPS_get_antenna_offset(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
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
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->have_vertical_velocity(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_GPS_last_message_time_ms(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = static_cast<uint32_t>(ud->last_message_time_ms(
            data_2));

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_last_fix_time_ms(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = static_cast<uint32_t>(ud->last_fix_time_ms(
            data_2));

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_get_vdop(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->get_vdop(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_get_hdop(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->get_hdop(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_time_week_ms(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = static_cast<uint32_t>(ud->time_week_ms(
            data_2));

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_GPS_time_week(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint16_t data = static_cast<uint16_t>(ud->time_week(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_num_sats(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = static_cast<uint8_t>(ud->num_sats(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_ground_course(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = static_cast<float>(ud->ground_course(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int AP_GPS_ground_speed(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = static_cast<float>(ud->ground_speed(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int AP_GPS_velocity(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
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
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->vertical_accuracy(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_GPS_horizontal_accuracy(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->horizontal_accuracy(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_GPS_speed_accuracy(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    float data_5003;
    const bool data = static_cast<bool>(ud->speed_accuracy(
            data_2,
            data_5003));

    if (data) {
        lua_pushnumber(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_GPS_location(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
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
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_sensors(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint8_t data = static_cast<uint8_t>(ud->status(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_primary_sensor(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->primary_sensor());

    lua_pushinteger(L, data);
    return 1;
}

static int AP_GPS_num_sensors(lua_State *L) {
    AP_GPS * ud = AP_GPS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "gps");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->num_sensors());

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_reset_remaining(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float raw_data_3 = luaL_checknumber(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, -INFINITY)) && (raw_data_3 <= MIN(100, INFINITY))), 3, "argument out of range");
    const float data_3 = raw_data_3;
    const bool data = static_cast<bool>(ud->reset_remaining(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_get_cycle_count(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint16_t data_5003;
    const bool data = static_cast<bool>(ud->get_cycle_count(
            data_2,
            data_5003));

    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_BattMonitor_get_temperature(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    float data_5002;
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->get_temperature(
            data_5002,
            data_3));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_BattMonitor_overpower_detected(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->overpower_detected(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_has_failsafed(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->has_failsafed());

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_pack_capacity_mah(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const int32_t data = static_cast<int32_t>(ud->pack_capacity_mah(
            data_2));

    lua_pushinteger(L, data);
    return 1;
}

static int AP_BattMonitor_capacity_remaining_pct(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    uint8_t data_5002;
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->capacity_remaining_pct(
            data_5002,
            data_3));

    if (data) {
        lua_pushinteger(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_BattMonitor_consumed_wh(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    float data_5002;
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->consumed_wh(
            data_5002,
            data_3));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_BattMonitor_consumed_mah(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    float data_5002;
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->consumed_mah(
            data_5002,
            data_3));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_BattMonitor_current_amps(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    float data_5002;
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(ud->num_instances(), UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->current_amps(
            data_5002,
            data_3));

    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_BattMonitor_voltage_resting_estimate(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = static_cast<float>(ud->voltage_resting_estimate(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_voltage(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const float data = static_cast<float>(ud->voltage(
            data_2));

    lua_pushnumber(L, data);
    return 1;
}

static int AP_BattMonitor_healthy(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(ud->num_instances(), UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->healthy(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_BattMonitor_num_instances(lua_State *L) {
    AP_BattMonitor * ud = AP_BattMonitor::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "battery");
    }

    binding_argcheck(L, 1);
    const uint8_t data = static_cast<uint8_t>(ud->num_instances());

    lua_pushinteger(L, data);
    return 1;
}

static int AP_Arming_set_aux_auth_failed(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "arming");
    }

    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const char * data_3 = luaL_checkstring(L, 3);
    ud->set_aux_auth_failed(
            data_2,
            data_3);

    return 0;
}

static int AP_Arming_set_aux_auth_passed(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "arming");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->set_aux_auth_passed(
            data_2);

    return 0;
}

static int AP_Arming_get_aux_auth_id(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "arming");
    }

    binding_argcheck(L, 1);
    uint8_t data_5002;
    const bool data = static_cast<bool>(ud->get_aux_auth_id(
            data_5002));

    if (data) {
        lua_pushinteger(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_Arming_arm(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "arming");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->arm(            AP_Arming::Method::SCRIPTING));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Arming_is_armed(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "arming");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->is_armed());

    lua_pushboolean(L, data);
    return 1;
}

static int AP_Arming_disarm(lua_State *L) {
    AP_Arming * ud = AP_Arming::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "arming");
    }

    binding_argcheck(L, 1);
    const bool data = static_cast<bool>(ud->disarm(            AP_Arming::Method::SCRIPTING));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_initialised(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->initialised());

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_set_origin(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 2);
    Location & data_2 = *check_Location(L, 2);
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_origin(
            data_2));

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_get_origin(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    Location data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_origin(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        new_Location(L);
        *check_Location(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int AP_AHRS_set_home(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 2);
    Location & data_2 = *check_Location(L, 2);
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->set_home(
            data_2));

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_get_vel_innovations_and_variances_for_source(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(3, 0)) && (raw_data_2 <= MIN(6, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    Vector3f data_5003 = {};
    Vector3f data_5004 = {};
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_vel_innovations_and_variances_for_source(
            data_2,
            data_5003,
            data_5004));

    ud->get_semaphore().give();
    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5003;
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5004;
        return 2;
    }
    return 0;
}

static int AP_AHRS_set_posvelyaw_source_set(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 2);
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(2, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->get_semaphore().take_blocking();
    ud->set_posvelyaw_source_set(
            data_2);

    ud->get_semaphore().give();
    return 0;
}

static int AP_AHRS_get_variances(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    float data_5002;
    float data_5003;
    float data_5004;
    Vector3f data_5005 = {};
    float data_5006;
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_variances(
            data_5002,
            data_5003,
            data_5004,
            data_5005,
            data_5006));

    ud->get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        lua_pushnumber(L, data_5003);
        lua_pushnumber(L, data_5004);
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5005;
        lua_pushnumber(L, data_5006);
        return 5;
    }
    return 0;
}

static int AP_AHRS_get_EAS2TAS(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = static_cast<float>(ud->get_EAS2TAS());

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

static int AP_AHRS_body_to_earth(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 2);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->body_to_earth(
            data_2);

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_earth_to_body(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 2);
    Vector3f & data_2 = *check_Vector3f(L, 2);
    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->earth_to_body(
            data_2);

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_vibration(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->get_vibration();

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_airspeed_estimate(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    float data_5002;
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->airspeed_estimate(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_AHRS_healthy(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->healthy());

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_home_is_set(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->home_is_set());

    ud->get_semaphore().give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_AHRS_get_relative_position_NED_origin(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    Vector3f data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_relative_position_NED_origin(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int AP_AHRS_get_relative_position_NED_home(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    Vector3f data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_relative_position_NED_home(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int AP_AHRS_get_velocity_NED(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    Vector3f data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_velocity_NED(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        new_Vector3f(L);
        *check_Vector3f(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int AP_AHRS_groundspeed_vector(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
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
        return not_supported_error(L, 1, "ahrs");
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
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    float data_5002;
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_hagl(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        lua_pushnumber(L, data_5002);
        return 1;
    }
    return 0;
}

static int AP_AHRS_get_accel(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const Vector3f &data = ud->get_accel();

    ud->get_semaphore().give();
    new_Vector3f(L);
    *check_Vector3f(L, -1) = data;
    return 1;
}

static int AP_AHRS_get_gyro(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
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
        return not_supported_error(L, 1, "ahrs");
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
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    Location data_5002 = {};
    ud->get_semaphore().take_blocking();
    const bool data = static_cast<bool>(ud->get_position(
            data_5002));

    ud->get_semaphore().give();
    if (data) {
        new_Location(L);
        *check_Location(L, -1) = data_5002;
        return 1;
    }
    return 0;
}

static int AP_AHRS_get_yaw(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = static_cast<float>(ud->get_yaw());

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

static int AP_AHRS_get_pitch(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = static_cast<float>(ud->get_pitch());

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

static int AP_AHRS_get_roll(lua_State *L) {
    AP_AHRS * ud = AP_AHRS::get_singleton();
    if (ud == nullptr) {
        return not_supported_error(L, 1, "ahrs");
    }

    binding_argcheck(L, 1);
    ud->get_semaphore().take_blocking();
    const float data = static_cast<float>(ud->get_roll());

    ud->get_semaphore().give();
    lua_pushnumber(L, data);
    return 1;
}

const luaL_Reg AP__fwversion___meta[] = {
    {"hash", AP__fwversion___fw_hash_str},
    {"patch", AP__fwversion___patch},
    {"minor", AP__fwversion___minor},
    {"major", AP__fwversion___major},
    {"type", AP__fwversion___vehicle_type},
    {"string", AP__fwversion___fw_short_string},
    {NULL, NULL}
};

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
const luaL_Reg AP__motors___meta[] = {
    {"set_frame_string", AP__motors___set_frame_string},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if defined(HAL_BUILD_AP_PERIPH)
const luaL_Reg AP_Periph_FW_meta[] = {
    {"get_vehicle_state", AP_Periph_FW_get_vehicle_state},
    {"get_yaw_earth", AP_Periph_FW_get_yaw_earth},
    {NULL, NULL}
};
#endif // defined(HAL_BUILD_AP_PERIPH)

const luaL_Reg AP_InertialSensor_meta[] = {
    {"get_temperature", AP_InertialSensor_get_temperature},
    {NULL, NULL}
};

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
const luaL_Reg AP_MotorsMatrix_Scripting_Dynamic_meta[] = {
    {"load_factors", AP_MotorsMatrix_Scripting_Dynamic_load_factors},
    {"add_motor", AP_MotorsMatrix_Scripting_Dynamic_add_motor},
    {"init", AP_MotorsMatrix_Scripting_Dynamic_init},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

const luaL_Reg hal_analogin_meta[] = {
    {"channel", hal_analogin_channel},
    {NULL, NULL}
};

const luaL_Reg hal_gpio_meta[] = {
    {"pinMode", hal_gpio_pinMode},
    {"toggle", hal_gpio_toggle},
    {"write", hal_gpio_write},
    {"read", hal_gpio_read},
    {NULL, NULL}
};

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
const luaL_Reg AP_MotorsMatrix_6DoF_Scripting_meta[] = {
    {"add_motor", AP_MotorsMatrix_6DoF_Scripting_add_motor},
    {"init", AP_MotorsMatrix_6DoF_Scripting_init},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
const luaL_Reg AC_AttitudeControl_Multi_6DoF_meta[] = {
    {"set_offset_roll_pitch", AC_AttitudeControl_Multi_6DoF_set_offset_roll_pitch},
    {"set_forward_enable", AC_AttitudeControl_Multi_6DoF_set_forward_enable},
    {"set_lateral_enable", AC_AttitudeControl_Multi_6DoF_set_lateral_enable},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

const luaL_Reg AP_Frsky_SPort_meta[] = {
    {"prep_number", AP_Frsky_SPort_prep_number},
    {"sport_telemetry_push", AP_Frsky_SPort_sport_telemetry_push},
    {NULL, NULL}
};

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
const luaL_Reg AP_MotorsMatrix_meta[] = {
    {"set_throttle_factor", AP_MotorsMatrix_set_throttle_factor},
    {"add_motor_raw", AP_MotorsMatrix_add_motor_raw},
    {"init", AP_MotorsMatrix_init},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
const luaL_Reg QuadPlane_meta[] = {
    {"in_assisted_flight", QuadPlane_in_assisted_flight},
    {"in_vtol_mode", QuadPlane_in_vtol_mode},
    {NULL, NULL}
};
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED

const luaL_Reg ScriptingLED_meta[] = {
    {"get_rgb", ScriptingLED_get_rgb},
    {NULL, NULL}
};

#if HAL_BUTTON_ENABLED == 1
const luaL_Reg AP_Button_meta[] = {
    {"get_button_state", AP_Button_get_button_state},
    {NULL, NULL}
};
#endif // HAL_BUTTON_ENABLED == 1

const luaL_Reg AP_RPM_meta[] = {
    {"get_rpm", AP_RPM_get_rpm},
    {NULL, NULL}
};

const luaL_Reg AP_Mission_meta[] = {
    {"clear", AP_Mission_clear},
    {"set_item", AP_Mission_set_item},
    {"get_item", AP_Mission_get_item},
    {"num_commands", AP_Mission_num_commands},
    {"get_current_do_cmd_id", AP_Mission_get_current_do_cmd_id},
    {"get_current_nav_id", AP_Mission_get_current_nav_id},
    {"get_prev_nav_cmd_id", AP_Mission_get_prev_nav_cmd_id},
    {"set_current_cmd", AP_Mission_set_current_cmd},
    {"get_current_nav_index", AP_Mission_get_current_nav_index},
    {"state", AP_Mission_state},
    {NULL, NULL}
};

const luaL_Reg AP_Param_meta[] = {
    {"add_param", AP_Param_add_param},
    {"add_table", AP_Param_add_table},
    {"set_and_save", AP_Param_set_and_save_by_name},
    {"set", AP_Param_set_by_name},
    {"get", AP_Param_get},
    {NULL, NULL}
};

#if HAL_WITH_ESC_TELEM == 1
const luaL_Reg AP_ESC_Telem_meta[] = {
    {"get_usage_seconds", AP_ESC_Telem_get_usage_seconds},
    {"get_consumption_mah", AP_ESC_Telem_get_consumption_mah},
    {"get_voltage", AP_ESC_Telem_get_voltage},
    {"get_current", AP_ESC_Telem_get_current},
    {"get_motor_temperature", AP_ESC_Telem_get_motor_temperature},
    {"get_temperature", AP_ESC_Telem_get_temperature},
    {"get_rpm", AP_ESC_Telem_get_rpm},
    {NULL, NULL}
};
#endif // HAL_WITH_ESC_TELEM == 1

#if AP_OPTICALFLOW_ENABLED
const luaL_Reg OpticalFlow_meta[] = {
    {"quality", OpticalFlow_quality},
    {"healthy", OpticalFlow_healthy},
    {"enabled", OpticalFlow_enabled},
    {NULL, NULL}
};
#endif // AP_OPTICALFLOW_ENABLED

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
    {"get_channel", RC_Channels_lua_rc_channel},
    {"has_valid_input", RC_Channels_has_valid_input},
    {"run_aux_function", RC_Channels_run_aux_function},
    {"find_channel_for_option", RC_Channels_find_channel_for_option},
    {"get_pwm", RC_Channels_get_pwm},
    {NULL, NULL}
};

const luaL_Reg SRV_Channels_meta[] = {
    {"set_range", SRV_Channels_set_range},
    {"set_angle", SRV_Channels_set_angle},
    {"set_output_norm", SRV_Channels_set_output_norm},
    {"get_output_scaled", SRV_Channels_get_output_scaled},
    {"get_output_pwm", SRV_Channels_get_output_pwm},
    {"set_output_scaled", SRV_Channels_set_output_scaled},
    {"set_output_pwm_chan_timeout", SRV_Channels_set_output_pwm_chan_timeout},
    {"set_output_pwm_chan", SRV_Channels_set_output_pwm_chan},
    {"set_output_pwm", SRV_Channels_set_output_pwm},
    {"find_channel", SRV_Channels_find_channel},
    {NULL, NULL}
};

const luaL_Reg AP_SerialLED_meta[] = {
    {"send", AP_SerialLED_send},
    {"set_RGB", AP_SerialLED_set_RGB},
    {"set_num_profiled", AP_SerialLED_set_num_profiled},
    {"set_num_neopixel", AP_SerialLED_set_num_neopixel},
    {NULL, NULL}
};

const luaL_Reg AP_Vehicle_meta[] = {
    {"set_target_throttle_rate_rpy", AP_Vehicle_set_target_throttle_rate_rpy},
    {"nav_script_time_done", AP_Vehicle_nav_script_time_done},
    {"nav_script_time", AP_Vehicle_nav_script_time},
    {"get_pan_tilt_norm", AP_Vehicle_get_pan_tilt_norm},
    {"get_wp_crosstrack_error_m", AP_Vehicle_get_wp_crosstrack_error_m},
    {"get_wp_bearing_deg", AP_Vehicle_get_wp_bearing_deg},
    {"get_wp_distance_m", AP_Vehicle_get_wp_distance_m},
    {"set_steering_and_throttle", AP_Vehicle_set_steering_and_throttle},
    {"set_circle_rate", AP_Vehicle_set_circle_rate},
    {"get_circle_radius", AP_Vehicle_get_circle_radius},
    {"set_target_angle_and_climbrate", AP_Vehicle_set_target_angle_and_climbrate},
    {"set_target_velocity_NED", AP_Vehicle_set_target_velocity_NED},
    {"set_target_velaccel_NED", AP_Vehicle_set_target_velaccel_NED},
    {"set_target_posvelaccel_NED", AP_Vehicle_set_target_posvelaccel_NED},
    {"set_target_posvel_NED", AP_Vehicle_set_target_posvel_NED},
    {"set_target_pos_NED", AP_Vehicle_set_target_pos_NED},
    {"get_target_location", AP_Vehicle_get_target_location},
    {"set_target_location", AP_Vehicle_set_target_location},
    {"start_takeoff", AP_Vehicle_start_takeoff},
    {"get_control_output", AP_Vehicle_get_control_output},
    {"get_time_flying_ms", AP_Vehicle_get_time_flying_ms},
    {"get_likely_flying", AP_Vehicle_get_likely_flying},
    {"get_control_mode_reason", AP_Vehicle_get_control_mode_reason},
    {"get_mode", AP_Vehicle_get_mode},
    {"set_mode", AP_Vehicle_set_mode},
    {NULL, NULL}
};

#if ENABLE_ONVIF == 1
const luaL_Reg AP_ONVIF_meta[] = {
    {"get_pan_tilt_limit_max", AP_ONVIF_get_pan_tilt_limit_max},
    {"get_pan_tilt_limit_min", AP_ONVIF_get_pan_tilt_limit_min},
    {"set_absolutemove", AP_ONVIF_set_absolutemove},
    {"start", AP_ONVIF_start},
    {NULL, NULL}
};
#endif // ENABLE_ONVIF == 1

const luaL_Reg GCS_meta[] = {
    {"send_named_float", GCS_send_named_float},
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

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
const luaL_Reg AP_Terrain_meta[] = {
    {"height_above_terrain", AP_Terrain_height_above_terrain},
    {"height_terrain_difference_home", AP_Terrain_height_terrain_difference_home},
    {"height_amsl", AP_Terrain_height_amsl},
    {"status", AP_Terrain_status},
    {"enabled", AP_Terrain_enabled},
    {NULL, NULL}
};
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

const luaL_Reg RangeFinder_meta[] = {
    {"get_pos_offset_orient", RangeFinder_get_pos_offset_orient},
    {"has_data_orient", RangeFinder_has_data_orient},
    {"status_orient", RangeFinder_status_orient},
    {"ground_clearance_cm_orient", RangeFinder_ground_clearance_cm_orient},
    {"min_distance_cm_orient", RangeFinder_min_distance_cm_orient},
    {"max_distance_cm_orient", RangeFinder_max_distance_cm_orient},
    {"distance_cm_orient", RangeFinder_distance_cm_orient},
    {"has_orientation", RangeFinder_has_orientation},
    {"num_sensors", RangeFinder_num_sensors},
    {NULL, NULL}
};

#if HAL_PROXIMITY_ENABLED == 1
const luaL_Reg AP_Proximity_meta[] = {
    {"get_object_angle_and_distance", AP_Proximity_get_object_angle_and_distance},
    {"get_closest_object", AP_Proximity_get_closest_object},
    {"get_object_count", AP_Proximity_get_object_count},
    {"num_sensors", AP_Proximity_num_sensors},
    {"get_status", AP_Proximity_get_status},
    {NULL, NULL}
};
#endif // HAL_PROXIMITY_ENABLED == 1

const luaL_Reg AP_Notify_meta[] = {
    {"handle_rgb_id", AP_Notify_handle_rgb_id},
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
    {"reset_remaining", AP_BattMonitor_reset_remaining},
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
    {"set_aux_auth_failed", AP_Arming_set_aux_auth_failed},
    {"set_aux_auth_passed", AP_Arming_set_aux_auth_passed},
    {"get_aux_auth_id", AP_Arming_get_aux_auth_id},
    {"arm", AP_Arming_arm},
    {"is_armed", AP_Arming_is_armed},
    {"disarm", AP_Arming_disarm},
    {NULL, NULL}
};

const luaL_Reg AP_AHRS_meta[] = {
    {"initialised", AP_AHRS_initialised},
    {"set_origin", AP_AHRS_set_origin},
    {"get_origin", AP_AHRS_get_origin},
    {"set_home", AP_AHRS_set_home},
    {"get_vel_innovations_and_variances_for_source", AP_AHRS_get_vel_innovations_and_variances_for_source},
    {"set_posvelyaw_source_set", AP_AHRS_set_posvelyaw_source_set},
    {"get_variances", AP_AHRS_get_variances},
    {"get_EAS2TAS", AP_AHRS_get_EAS2TAS},
    {"body_to_earth", AP_AHRS_body_to_earth},
    {"earth_to_body", AP_AHRS_earth_to_body},
    {"get_vibration", AP_AHRS_get_vibration},
    {"airspeed_estimate", AP_AHRS_airspeed_estimate},
    {"healthy", AP_AHRS_healthy},
    {"home_is_set", AP_AHRS_home_is_set},
    {"get_relative_position_NED_origin", AP_AHRS_get_relative_position_NED_origin},
    {"get_relative_position_NED_home", AP_AHRS_get_relative_position_NED_home},
    {"get_velocity_NED", AP_AHRS_get_velocity_NED},
    {"groundspeed_vector", AP_AHRS_groundspeed_vector},
    {"wind_estimate", AP_AHRS_wind_estimate},
    {"get_hagl", AP_AHRS_get_hagl},
    {"get_accel", AP_AHRS_get_accel},
    {"get_gyro", AP_AHRS_get_gyro},
    {"get_home", AP_AHRS_get_home},
    {"get_position", AP_AHRS_get_position},
    {"get_yaw", AP_AHRS_get_yaw},
    {"get_pitch", AP_AHRS_get_pitch},
    {"get_roll", AP_AHRS_get_roll},
    {NULL, NULL}
};

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
static int ScriptingCANBuffer_read_frame(lua_State *L) {
    binding_argcheck(L, 1);
    ScriptingCANBuffer * ud = *check_ScriptingCANBuffer(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    AP_HAL::CANFrame data_5002 = {};
    const bool data = static_cast<bool>(ud->read_frame(
            data_5002));

    if (data) {
        new_AP_HAL__CANFrame(L);
        *check_AP_HAL__CANFrame(L, -1) = data_5002;
        return 1;
    }
    return 0;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
static int ScriptingCANBuffer_write_frame(lua_State *L) {
    binding_argcheck(L, 3);
    ScriptingCANBuffer * ud = *check_ScriptingCANBuffer(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    AP_HAL::CANFrame & data_2 = *check_AP_HAL__CANFrame(L, 2);
    const uint32_t raw_data_3 = coerce_to_uint32_t(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0U, 0U)) && (raw_data_3 <= MIN(UINT32_MAX, UINT32_MAX))), 3, "argument out of range");
    const uint32_t data_3 = static_cast<uint32_t>(raw_data_3);
    const bool data = static_cast<bool>(ud->write_frame(
            data_2,
            data_3));

    lua_pushboolean(L, data);
    return 1;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

static int AP_HAL__AnalogSource_voltage_average_ratiometric(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::AnalogSource * ud = *check_AP_HAL__AnalogSource(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const float data = static_cast<float>(ud->voltage_average_ratiometric());

    lua_pushnumber(L, data);
    return 1;
}

static int AP_HAL__AnalogSource_voltage_latest(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::AnalogSource * ud = *check_AP_HAL__AnalogSource(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const float data = static_cast<float>(ud->voltage_latest());

    lua_pushnumber(L, data);
    return 1;
}

static int AP_HAL__AnalogSource_voltage_average(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::AnalogSource * ud = *check_AP_HAL__AnalogSource(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const float data = static_cast<float>(ud->voltage_average());

    lua_pushnumber(L, data);
    return 1;
}

static int AP_HAL__AnalogSource_set_pin(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::AnalogSource * ud = *check_AP_HAL__AnalogSource(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const bool data = static_cast<bool>(ud->set_pin(
            data_2));

    lua_pushboolean(L, data);
    return 1;
}

static int AP_HAL__I2CDevice_set_address(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::I2CDevice * ud = *check_AP_HAL__I2CDevice(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->get_semaphore()->take_blocking();
    ud->set_address(
            data_2);

    ud->get_semaphore()->give();
    return 0;
}

static int AP_HAL__I2CDevice_read_registers(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::I2CDevice * ud = *check_AP_HAL__I2CDevice(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    uint8_t data_5003;
    ud->get_semaphore()->take_blocking();
    const bool data = static_cast<bool>(ud->read_registers(
            data_2,
            &data_5003,
            1));

    ud->get_semaphore()->give();
    if (data) {
        lua_pushinteger(L, data_5003);
        return 1;
    }
    return 0;
}

static int AP_HAL__I2CDevice_write_register(lua_State *L) {
    binding_argcheck(L, 3);
    AP_HAL::I2CDevice * ud = *check_AP_HAL__I2CDevice(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const lua_Integer raw_data_3 = luaL_checkinteger(L, 3);
    luaL_argcheck(L, ((raw_data_3 >= MAX(0, 0)) && (raw_data_3 <= MIN(UINT8_MAX, UINT8_MAX))), 3, "argument out of range");
    const uint8_t data_3 = static_cast<uint8_t>(raw_data_3);
    ud->get_semaphore()->take_blocking();
    const bool data = static_cast<bool>(ud->write_register(
            data_2,
            data_3));

    ud->get_semaphore()->give();
    lua_pushboolean(L, data);
    return 1;
}

static int AP_HAL__I2CDevice_set_retries(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::I2CDevice * ud = *check_AP_HAL__I2CDevice(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(20, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    ud->get_semaphore()->take_blocking();
    ud->set_retries(
            data_2);

    ud->get_semaphore()->give();
    return 0;
}

static int AP_HAL__UARTDriver_set_flow_control(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
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
        return luaL_error(L, "Internal error, null pointer");
    }
    const uint32_t data = static_cast<uint32_t>(ud->available());

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_HAL__UARTDriver_write(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const lua_Integer raw_data_2 = luaL_checkinteger(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(0, 0)) && (raw_data_2 <= MIN(UINT8_MAX, UINT8_MAX))), 2, "argument out of range");
    const uint8_t data_2 = static_cast<uint8_t>(raw_data_2);
    const uint32_t data = static_cast<uint32_t>(ud->write(
            data_2));

        new_uint32_t(L);
        *static_cast<uint32_t *>(luaL_checkudata(L, -1, "uint32_t")) = data;
    return 1;
}

static int AP_HAL__UARTDriver_read(lua_State *L) {
    binding_argcheck(L, 1);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const int16_t data = static_cast<int16_t>(ud->read());

    lua_pushinteger(L, data);
    return 1;
}

static int AP_HAL__UARTDriver_begin(lua_State *L) {
    binding_argcheck(L, 2);
    AP_HAL::UARTDriver * ud = *check_AP_HAL__UARTDriver(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const uint32_t raw_data_2 = coerce_to_uint32_t(L, 2);
    luaL_argcheck(L, ((raw_data_2 >= MAX(1U, 0U)) && (raw_data_2 <= MIN(UINT32_MAX, UINT32_MAX))), 2, "argument out of range");
    const uint32_t data_2 = static_cast<uint32_t>(raw_data_2);
    ud->begin(
            data_2);

    return 0;
}

static int RC_Channel_norm_input_ignore_trim(lua_State *L) {
    binding_argcheck(L, 1);
    RC_Channel * ud = *check_RC_Channel(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const float data = static_cast<float>(ud->norm_input_ignore_trim());

    lua_pushnumber(L, data);
    return 1;
}

static int RC_Channel_get_aux_switch_pos(lua_State *L) {
    binding_argcheck(L, 1);
    RC_Channel * ud = *check_RC_Channel(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const uint8_t data = static_cast<uint8_t>(ud->get_aux_switch_pos());

    lua_pushinteger(L, data);
    return 1;
}

static int RC_Channel_norm_input(lua_State *L) {
    binding_argcheck(L, 1);
    RC_Channel * ud = *check_RC_Channel(L, 1);
    if (ud == NULL) {
        return luaL_error(L, "Internal error, null pointer");
    }
    const float data = static_cast<float>(ud->norm_input());

    lua_pushnumber(L, data);
    return 1;
}

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
const luaL_Reg ScriptingCANBuffer_meta[] = {
    {"read_frame", ScriptingCANBuffer_read_frame},
    {"write_frame", ScriptingCANBuffer_write_frame},
    {NULL, NULL}
};
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

const luaL_Reg AP_HAL__AnalogSource_meta[] = {
    {"voltage_average_ratiometric", AP_HAL__AnalogSource_voltage_average_ratiometric},
    {"voltage_latest", AP_HAL__AnalogSource_voltage_latest},
    {"voltage_average", AP_HAL__AnalogSource_voltage_average},
    {"set_pin", AP_HAL__AnalogSource_set_pin},
    {NULL, NULL}
};

const luaL_Reg AP_HAL__I2CDevice_meta[] = {
    {"set_address", AP_HAL__I2CDevice_set_address},
    {"read_registers", AP_HAL__I2CDevice_read_registers},
    {"write_register", AP_HAL__I2CDevice_write_register},
    {"set_retries", AP_HAL__I2CDevice_set_retries},
    {NULL, NULL}
};

const luaL_Reg AP_HAL__UARTDriver_meta[] = {
    {"set_flow_control", AP_HAL__UARTDriver_set_flow_control},
    {"available", AP_HAL__UARTDriver_available},
    {"write", AP_HAL__UARTDriver_write},
    {"read", AP_HAL__UARTDriver_read},
    {"begin", AP_HAL__UARTDriver_begin},
    {NULL, NULL}
};

const luaL_Reg RC_Channel_meta[] = {
    {"norm_input_ignore_trim", RC_Channel_norm_input_ignore_trim},
    {"get_aux_switch_pos", RC_Channel_get_aux_switch_pos},
    {"norm_input", RC_Channel_norm_input},
    {NULL, NULL}
};

struct userdata_enum {
    const char *name;
    int value;
};

struct userdata_enum AP_Mission_enums[] = {
    {"MISSION_COMPLETE", AP_Mission::MISSION_COMPLETE},
    {"MISSION_RUNNING", AP_Mission::MISSION_RUNNING},
    {"MISSION_STOPPED", AP_Mission::MISSION_STOPPED},
    {NULL, 0}};

#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
struct userdata_enum AP_Terrain_enums[] = {
    {"TerrainStatusOK", AP_Terrain::TerrainStatusOK},
    {"TerrainStatusUnhealthy", AP_Terrain::TerrainStatusUnhealthy},
    {"TerrainStatusDisabled", AP_Terrain::TerrainStatusDisabled},
    {NULL, 0}};
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1

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
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"CANFrame", AP_HAL__CANFrame_meta, NULL},
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"motor_factor_table", AP_MotorsMatrix_Scripting_Dynamic__factor_table_meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"PWMSource", AP_HAL__PWMSource_meta, NULL},
    {"mavlink_mission_item_int_t", mavlink_mission_item_int_t_meta, NULL},
    {"Parameter", Parameter_meta, NULL},
    {"Vector2f", Vector2f_meta, NULL},
    {"Vector3f", Vector3f_meta, NULL},
    {"Location", Location_meta, NULL},
};

const struct userdata_meta singleton_fun[] = {
    {"FWVersion", AP__fwversion___meta, NULL},
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"motors", AP__motors___meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
    {"periph", AP_Periph_FW_meta, NULL},
#endif // defined(HAL_BUILD_AP_PERIPH)
    {"ins", AP_InertialSensor_meta, NULL},
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"Motors_dynamic", AP_MotorsMatrix_Scripting_Dynamic_meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"analog", hal_analogin_meta, NULL},
    {"gpio", hal_gpio_meta, NULL},
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    {"Motors_6DoF", AP_MotorsMatrix_6DoF_Scripting_meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    {"attitude_control", AC_AttitudeControl_Multi_6DoF_meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    {"frsky_sport", AP_Frsky_SPort_meta, NULL},
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"MotorsMatrix", AP_MotorsMatrix_meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
    {"quadplane", QuadPlane_meta, NULL},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
    {"LED", ScriptingLED_meta, NULL},
#if HAL_BUTTON_ENABLED == 1
    {"button", AP_Button_meta, NULL},
#endif // HAL_BUTTON_ENABLED == 1
    {"RPM", AP_RPM_meta, NULL},
    {"mission", AP_Mission_meta, AP_Mission_enums},
    {"param", AP_Param_meta, NULL},
#if HAL_WITH_ESC_TELEM == 1
    {"esc_telem", AP_ESC_Telem_meta, NULL},
#endif // HAL_WITH_ESC_TELEM == 1
#if AP_OPTICALFLOW_ENABLED
    {"optical_flow", OpticalFlow_meta, NULL},
#endif // AP_OPTICALFLOW_ENABLED
    {"baro", AP_Baro_meta, NULL},
    {"serial", AP_SerialManager_meta, NULL},
    {"rc", RC_Channels_meta, NULL},
    {"SRV_Channels", SRV_Channels_meta, NULL},
    {"serialLED", AP_SerialLED_meta, NULL},
    {"vehicle", AP_Vehicle_meta, NULL},
#if ENABLE_ONVIF == 1
    {"onvif", AP_ONVIF_meta, NULL},
#endif // ENABLE_ONVIF == 1
    {"gcs", GCS_meta, NULL},
    {"relay", AP_Relay_meta, NULL},
#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
    {"terrain", AP_Terrain_meta, AP_Terrain_enums},
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
    {"rangefinder", RangeFinder_meta, NULL},
#if HAL_PROXIMITY_ENABLED == 1
    {"proximity", AP_Proximity_meta, NULL},
#endif // HAL_PROXIMITY_ENABLED == 1
    {"notify", AP_Notify_meta, NULL},
    {"gps", AP_GPS_meta, AP_GPS_enums},
    {"battery", AP_BattMonitor_meta, NULL},
    {"arming", AP_Arming_meta, NULL},
    {"ahrs", AP_AHRS_meta, NULL},
};

const struct userdata_meta ap_object_fun[] = {
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"ScriptingCANBuffer", ScriptingCANBuffer_meta, NULL},
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"AP_HAL::AnalogSource", AP_HAL__AnalogSource_meta, NULL},
    {"AP_HAL::I2CDevice", AP_HAL__I2CDevice_meta, NULL},
    {"AP_HAL::UARTDriver", AP_HAL__UARTDriver_meta, NULL},
    {"RC_Channel", RC_Channel_meta, NULL},
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
    "FWVersion",
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    "motors",
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
    "periph",
#endif // defined(HAL_BUILD_AP_PERIPH)
    "ins",
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    "Motors_dynamic",
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    "analog",
    "gpio",
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    "Motors_6DoF",
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    "attitude_control",
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    "frsky_sport",
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    "MotorsMatrix",
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
    "quadplane",
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_QUADPLANE_ENABLED
    "LED",
#if HAL_BUTTON_ENABLED == 1
    "button",
#endif // HAL_BUTTON_ENABLED == 1
    "RPM",
    "mission",
    "param",
#if HAL_WITH_ESC_TELEM == 1
    "esc_telem",
#endif // HAL_WITH_ESC_TELEM == 1
#if AP_OPTICALFLOW_ENABLED
    "optical_flow",
#endif // AP_OPTICALFLOW_ENABLED
    "baro",
    "serial",
    "rc",
    "SRV_Channels",
    "serialLED",
    "vehicle",
#if ENABLE_ONVIF == 1
    "onvif",
#endif // ENABLE_ONVIF == 1
    "gcs",
    "relay",
#if defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
    "terrain",
#endif // defined(AP_TERRAIN_AVAILABLE) && AP_TERRAIN_AVAILABLE == 1
    "rangefinder",
#if HAL_PROXIMITY_ENABLED == 1
    "proximity",
#endif // HAL_PROXIMITY_ENABLED == 1
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
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"CANFrame", new_AP_HAL__CANFrame},
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"motor_factor_table", new_AP_MotorsMatrix_Scripting_Dynamic__factor_table},
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
    {"PWMSource", new_AP_HAL__PWMSource},
    {"mavlink_mission_item_int_t", new_mavlink_mission_item_int_t},
    {"Parameter", new_Parameter},
    {"Vector2f", new_Vector2f},
    {"Vector3f", new_Vector3f},
    {"Location", new_Location},
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"ScriptingCANBuffer", new_ScriptingCANBuffer},
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"AP_HAL::AnalogSource", new_AP_HAL__AnalogSource},
    {"AP_HAL::I2CDevice", new_AP_HAL__I2CDevice},
    {"AP_HAL::UARTDriver", new_AP_HAL__UARTDriver},
    {"RC_Channel", new_RC_Channel},
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

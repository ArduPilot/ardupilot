#pragma once

#include "lua/src/lua.hpp"

int lua_millis(lua_State *L);
int lua_micros(lua_State *L);
int lua_mission_receive(lua_State *L);
int AP_Logger_Write(lua_State *L);
int lua_get_i2c_device(lua_State *L);
int lua_get_CAN_device(lua_State *L);
int lua_get_CAN_device2(lua_State *L);

#if HAL_HIGH_LATENCY2_ENABLED
int lua_mavlink_get_MAVLink(lua_State *L);
int lua_mavlink_create_high_latency_packet(lua_State *L);
int lua_mavlink_receive(lua_State *L);
int lua_mavlink_is_high_latency_enabled(lua_State *L);
int lua_mavlink_set_high_latency_enabled(lua_State *L);
#endif

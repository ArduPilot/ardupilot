#pragma once

#include "lua/src/lua.hpp"
#include "lua_types.h"

int lua_millis(lua_State *L);
int lua_micros(lua_State *L);
int lua_mission_receive(lua_State *L);
int AP_Logger_Write(lua_State *L);
int lua_get_i2c_device(lua_State *L);
int lua_get_CAN_device(lua_State *L);
int lua_get_CAN_device2(lua_State *L);

int lua_userdata_field(lua_State *L, const char *type_name, enum field_type type, uint8_t access_flags, uint16_t field_offset, uint8_t field_size);

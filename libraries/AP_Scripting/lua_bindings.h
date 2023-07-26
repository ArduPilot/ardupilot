#pragma once

#include "lua/src/lua.hpp"

int lua_millis(lua_State *L);
int lua_micros(lua_State *L);
int lua_mission_receive(lua_State *L);
int AP_Logger_Write(lua_State *L);
int lua_get_i2c_device(lua_State *L);
int AP_HAL__I2CDevice_read_registers(lua_State *L);
int lua_get_CAN_device(lua_State *L);
int lua_get_CAN_device2(lua_State *L);
int lua_dirlist(lua_State *L);
int lua_removefile(lua_State *L);
int SRV_Channels_get_safety_state(lua_State *L);
int lua_get_PWMSource(lua_State *L);
int lua_mavlink_init(lua_State *L);
int lua_mavlink_receive_chan(lua_State *L);
int lua_mavlink_register_rx_msgid(lua_State *L);
int lua_mavlink_send_chan(lua_State *L);
int lua_mavlink_block_command(lua_State *L);

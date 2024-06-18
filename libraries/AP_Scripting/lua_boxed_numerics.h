#pragma once

#include "lua/src/lua.hpp"

uint32_t coerce_to_uint32_t(lua_State *L, int arg);
int lua_new_uint32_t(lua_State *L);

int uint32_t___tostring(lua_State *L);
int uint32_t_toint(lua_State *L);
int uint32_t_tofloat(lua_State *L);

int lua_new_uint64_t(lua_State *L);
uint64_t coerce_to_uint64_t(lua_State *L, int arg);

int uint64_t___tostring(lua_State *L);
int uint64_t_toint(lua_State *L);
int uint64_t_tofloat(lua_State *L);
int uint64_t_split(lua_State *L);


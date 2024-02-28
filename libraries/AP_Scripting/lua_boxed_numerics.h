#pragma once

#include "lua/src/lua.hpp"

uint32_t coerce_to_uint32_t(lua_State *L, int arg);
int lua_new_uint32_t(lua_State *L);

int uint32_t___add(lua_State *L);
int uint32_t___sub(lua_State *L);
int uint32_t___mul(lua_State *L);
int uint32_t___div(lua_State *L);
int uint32_t___mod(lua_State *L);
int uint32_t___band(lua_State *L);
int uint32_t___bor(lua_State *L);
int uint32_t___bxor(lua_State *L);
int uint32_t___shl(lua_State *L);
int uint32_t___shr(lua_State *L);
int uint32_t___eq(lua_State *L);
int uint32_t___lt(lua_State *L);
int uint32_t___le(lua_State *L);
int uint32_t___bnot(lua_State *L);
int uint32_t___tostring(lua_State *L);
int uint32_t_toint(lua_State *L);
int uint32_t_tofloat(lua_State *L);

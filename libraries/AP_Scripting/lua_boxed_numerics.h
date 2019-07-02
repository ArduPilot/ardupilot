#pragma once

#include "lua/src/lua.hpp"

int new_uint32_t(lua_State *L);

void load_boxed_numerics(lua_State *L);
void load_boxed_numerics_sandbox(lua_State *L);

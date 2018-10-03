#pragma once

#include "lua/src/lua.hpp"

// load all known lua bindings into the state
void load_lua_bindings(lua_State *state);

void hook(lua_State *L, lua_Debug *ar);

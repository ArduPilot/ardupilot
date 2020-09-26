#pragma once

#include "lua/src/lua.hpp"

// load all known lua bindings into the state
void load_lua_bindings(lua_State *state);

static int AP_Logger_Write(lua_State *L);

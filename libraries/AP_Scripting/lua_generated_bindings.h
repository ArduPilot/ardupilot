#pragma once
// auto generated bindings, don't manually edit
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include "lua/src/lua.hpp"
#include <new>

int new_Vector3f(lua_State *L);
Vector3f * check_Vector3f(lua_State *L, int arg);
int new_Location(lua_State *L);
Location * check_Location(lua_State *L, int arg);
void load_generated_bindings(lua_State *L);
void load_generated_sandbox(lua_State *L);

#pragma once
// auto generated bindings, don't manually edit
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include "lua/src/lua.hpp"
#include <new>

int new_Vector2f(lua_State *L);
Vector2f * check_Vector2f(lua_State *L, int arg);
int new_Vector3f(lua_State *L);
Vector3f * check_Vector3f(lua_State *L, int arg);
int new_Location(lua_State *L);
Location * check_Location(lua_State *L, int arg);
void load_generated_bindings(lua_State *L);
void load_generated_sandbox(lua_State *L);

#pragma once
// auto generated bindings, don't manually edit
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include "lua/src/lua.hpp"
#include <new>

#if !defined(AP_TERRAIN_AVAILABLE) || (AP_TERRAIN_AVAILABLE != 1)
  #error Scripting requires terrain to be available

#endif // !defined(AP_TERRAIN_AVAILABLE) || (AP_TERRAIN_AVAILABLE != 1)


int new_Vector2f(lua_State *L);
Vector2f * check_Vector2f(lua_State *L, int arg);
int new_Vector3f(lua_State *L);
Vector3f * check_Vector3f(lua_State *L, int arg);
int new_Location(lua_State *L);
Location * check_Location(lua_State *L, int arg);
void load_generated_bindings(lua_State *L);
void load_generated_sandbox(lua_State *L);

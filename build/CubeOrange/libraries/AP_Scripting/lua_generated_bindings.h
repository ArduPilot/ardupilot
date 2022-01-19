#pragma once
// auto generated bindings, don't manually edit.  See README.md for details.
#include <AP_Vehicle/AP_Vehicle_Type.h> // needed for APM_BUILD_TYPE #if
#include <AP_Common/AP_FWVersion.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_Motors_Class.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if defined(HAL_BUILD_AP_PERIPH)
#include <../Tools/AP_Periph/AP_Periph.h>
#endif // defined(HAL_BUILD_AP_PERIPH)
#include <AP_HAL/AP_HAL.h>
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_Scripting/AP_Scripting_CANSensor.h>
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_InertialSensor/AP_InertialSensor.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_HAL/I2CDevice.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include <AP_Frsky_Telem/AP_Frsky_SPort.h>
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#include <AP_Motors/AP_MotorsMatrix.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <../ArduPlane/quadplane.h>
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include <AP_Notify/ScriptingLED.h>
#include <AP_Button/AP_Button.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Scripting/AP_Scripting_helpers.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_Vehicle/AP_Vehicle.h>
#if ENABLE_ONVIF == 1
#include <AP_ONVIF/AP_ONVIF.h>
#endif // ENABLE_ONVIF == 1
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_Scripting/lua/src/lua.hpp>
#include <new>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_AP_HAL__CANFrame(lua_State *L);
AP_HAL::CANFrame * check_AP_HAL__CANFrame(lua_State *L, int arg);
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
int new_AP_MotorsMatrix_Scripting_Dynamic__factor_table(lua_State *L);
AP_MotorsMatrix_Scripting_Dynamic::factor_table * check_AP_MotorsMatrix_Scripting_Dynamic__factor_table(lua_State *L, int arg);
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI
int new_AP_HAL__PWMSource(lua_State *L);
AP_HAL::PWMSource * check_AP_HAL__PWMSource(lua_State *L, int arg);
int new_mavlink_mission_item_int_t(lua_State *L);
mavlink_mission_item_int_t * check_mavlink_mission_item_int_t(lua_State *L, int arg);
int new_Parameter(lua_State *L);
Parameter * check_Parameter(lua_State *L, int arg);
int new_Vector2f(lua_State *L);
Vector2f * check_Vector2f(lua_State *L, int arg);
int new_Vector3f(lua_State *L);
Vector3f * check_Vector3f(lua_State *L, int arg);
int new_Location(lua_State *L);
Location * check_Location(lua_State *L, int arg);
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_ScriptingCANBuffer(lua_State *L);
ScriptingCANBuffer ** check_ScriptingCANBuffer(lua_State *L, int arg);
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
int new_AP_HAL__AnalogSource(lua_State *L);
AP_HAL::AnalogSource ** check_AP_HAL__AnalogSource(lua_State *L, int arg);
int new_AP_HAL__I2CDevice(lua_State *L);
AP_HAL::I2CDevice ** check_AP_HAL__I2CDevice(lua_State *L, int arg);
int new_AP_HAL__UARTDriver(lua_State *L);
AP_HAL::UARTDriver ** check_AP_HAL__UARTDriver(lua_State *L, int arg);
int new_RC_Channel(lua_State *L);
RC_Channel ** check_RC_Channel(lua_State *L, int arg);
void load_generated_bindings(lua_State *L);
void load_generated_sandbox(lua_State *L);

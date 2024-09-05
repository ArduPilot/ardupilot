---@meta
-- ArduPilot lua scripting documentation in EmmyLua Annotations
-- This file should be auto generated and then manual edited
-- generate with --scripting-docs, eg  ./waf copter --scripting-docs
-- see: https://github.com/sumneko/lua-language-server/wiki/EmmyLua-Annotations
-- luacheck: ignore 121 (Setting a read-only global variable)
-- luacheck: ignore 122 (Setting a read-only field of a global variable)
-- luacheck: ignore 212 (Unused argument)
-- luacheck: ignore 241 (Local variable is mutated but never accessed)
-- luacheck: ignore 221 (Local variable is accessed but never set.)

-- set and get for field types share function names
---@diagnostic disable: duplicate-set-field
---@diagnostic disable: missing-return

-- integer enum value unknown by docs generator
---@type integer
local enum_integer

-- manual bindings

---@class (exact) uint32_t_ud
---@operator add(uint32_t_ud|integer|number): uint32_t_ud
---@operator sub(uint32_t_ud|integer|number): uint32_t_ud
---@operator mul(uint32_t_ud|integer|number): uint32_t_ud
---@operator div(uint32_t_ud|integer|number): uint32_t_ud
---@operator mod(uint32_t_ud|integer|number): uint32_t_ud
---@operator band(uint32_t_ud|integer|number): uint32_t_ud
---@operator bor(uint32_t_ud|integer|number): uint32_t_ud
---@operator shl(uint32_t_ud|integer|number): uint32_t_ud
---@operator shr(uint32_t_ud|integer|number): uint32_t_ud
local uint32_t_ud = {}

-- create uint32_t_ud with optional value
---@param value? uint32_t_ud|integer|number
---@return uint32_t_ud
function uint32_t(value) end

-- Convert to number
---@return number
function uint32_t_ud:tofloat() end

-- Convert to integer
---@return integer
function uint32_t_ud:toint() end

---@class (exact) uint64_t_ud
---@operator add(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator sub(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator mul(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator div(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator mod(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator band(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator bor(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator shl(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
---@operator shr(uint64_t_ud|uint32_t_ud|integer|number): uint64_t_ud
local uint64_t_ud = {}

-- create uint64_t_ud with optional value
-- Note that lua ints are 32 bits and lua floats will loose resolution at large values
---@param value? uint64_t_ud|uint32_t_ud|integer|number
---@return uint64_t_ud
function uint64_t(value) end

-- create uint64_t_ud from a low and high half
-- value = (high << 32) | low
---@param high uint32_t_ud|integer|number
---@param low uint32_t_ud|integer|number
---@return uint64_t_ud
function uint64_t(high, low) end

-- Convert to number, will loose resolution at large values
---@return number
function uint64_t_ud:tofloat() end

-- Convert to integer, nil if too large to be represented by native int32
---@return integer|nil
function uint64_t_ud:toint() end

-- Split into high and low half's, returning each as a uint32_t_ud
---@return uint32_t_ud -- high (value >> 32)
---@return uint32_t_ud -- low (value & 0xFFFFFFFF)
function uint64_t_ud:split() end

-- system time in milliseconds
---@return uint32_t_ud -- milliseconds
function millis() end

-- system time in microseconds
---@return uint32_t_ud -- microseconds
function micros() end

-- receive mission command from running mission
---@return uint32_t_ud|nil -- command start time milliseconds
---@return integer|nil -- command param 1
---@return number|nil -- command param 2
---@return number|nil -- command param 3
---@return number|nil -- command param 4
function mission_receive() end

-- Print text, if MAVLink is available the value will be sent with debug severity
-- If no MAVLink the value will be sent over can
-- equivalent to gcs:send_text(7, text) or periph:can_printf(text)
---@param text string|number|integer
function print(text) end

-- data flash logging to SD card
logger = {}

-- write value to data flash log with given types and names with units and multipliers, timestamp will be automatically added
---@param name string -- up to 4 characters
---@param labels string -- comma separated value labels, up to 58 characters
---@param format string -- type format string, see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/README.md
---@param units string -- units string
---@param multipliers string -- multipliers string
---@param ... integer|number|uint32_t_ud|string -- data to be logged, type to match format string
function logger:write(name, labels, format, units, multipliers, ...) end

-- write value to data flash log with given types and names, timestamp will be automatically added
---@param name string -- up to 4 characters
---@param labels string -- comma separated value labels, up to 58 characters
---@param format string -- type format string, see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/README.md
---@param ... integer|number|uint32_t_ud|string -- data to be logged, type to match format string
function logger:write(name, labels, format, ...) end

-- log a files content to onboard log
---@param filename string -- file name
function logger:log_file_content(filename) end

-- i2c bus interaction
i2c = {}

-- get a i2c device handler
---@param bus integer -- bus number
---@param address integer -- device address 0 to 128
---@param clock? uint32_t_ud|integer|number -- optional bus clock, default 400000
---@param smbus? boolean -- optional sumbus flag, default false
---@return AP_HAL__I2CDevice_ud
function i2c:get_device(bus, address, clock, smbus) end

-- EFI state structure
---@class (exact) EFI_State_ud
local EFI_State_ud = {}

---@return EFI_State_ud
function EFI_State() end

-- get field
---@return number
function EFI_State_ud:pt_compensation() end

-- set field
---@param value number
function EFI_State_ud:pt_compensation(value) end

-- get field
---@return number
function EFI_State_ud:throttle_out() end

-- set field
---@param value number
function EFI_State_ud:throttle_out(value) end

-- get field
---@return number
function EFI_State_ud:ignition_voltage() end

-- set field
---@param value number
function EFI_State_ud:ignition_voltage(value) end

-- get field
---@return Cylinder_Status_ud
function EFI_State_ud:cylinder_status() end

-- set field
---@param value Cylinder_Status_ud
function EFI_State_ud:cylinder_status(value) end

-- get field
---@return integer
function EFI_State_ud:ecu_index() end

-- set field
---@param value integer
function EFI_State_ud:ecu_index(value) end

-- get field
---@return integer
function EFI_State_ud:throttle_position_percent() end

-- set field
---@param value integer
function EFI_State_ud:throttle_position_percent(value) end

-- get field
---@return number
function EFI_State_ud:estimated_consumed_fuel_volume_cm3() end

-- set field
---@param value number
function EFI_State_ud:estimated_consumed_fuel_volume_cm3(value) end

-- get field
---@return number
function EFI_State_ud:fuel_consumption_rate_cm3pm() end

-- set field
---@param value number
function EFI_State_ud:fuel_consumption_rate_cm3pm(value) end

-- get field
---@return number
function EFI_State_ud:fuel_pressure() end

-- set field
---@param value number
function EFI_State_ud:fuel_pressure(value) end

-- get field
---@return integer
---| '0' # Not supported
---| '1' # Ok
---| '2' # Below nominal
---| '3' # Above nominal
function EFI_State_ud:fuel_pressure_status() end

-- set field
---@param status integer
---| '0' # Not supported
---| '1' # Ok
---| '2' # Below nominal
---| '3' # Above nominal
function EFI_State_ud:fuel_pressure_status(status) end

-- get field
---@return number
function EFI_State_ud:oil_temperature() end

-- set field
---@param value number
function EFI_State_ud:oil_temperature(value) end

-- get field
---@return number
function EFI_State_ud:oil_pressure() end

-- set field
---@param value number
function EFI_State_ud:oil_pressure(value) end

-- get field
---@return number
function EFI_State_ud:coolant_temperature() end

-- set field
---@param value number
function EFI_State_ud:coolant_temperature(value) end

-- get field
---@return number
function EFI_State_ud:intake_manifold_temperature() end

-- set field
---@param value number
function EFI_State_ud:intake_manifold_temperature(value) end

-- get field
---@return number
function EFI_State_ud:intake_manifold_pressure_kpa() end

-- set field
---@param value number
function EFI_State_ud:intake_manifold_pressure_kpa(value) end

-- get field
---@return number
function EFI_State_ud:atmospheric_pressure_kpa() end

-- set field
---@param value number
function EFI_State_ud:atmospheric_pressure_kpa(value) end

-- get field
---@return number
function EFI_State_ud:spark_dwell_time_ms() end

-- set field
---@param value number
function EFI_State_ud:spark_dwell_time_ms(value) end

-- get field
---@return uint32_t_ud
function EFI_State_ud:engine_speed_rpm() end

-- set field
---@param value uint32_t_ud|integer|number
function EFI_State_ud:engine_speed_rpm(value) end

-- get field
---@return integer
function EFI_State_ud:engine_load_percent() end

-- set field
---@param value integer
function EFI_State_ud:engine_load_percent(value) end

-- get field
---@return boolean
function EFI_State_ud:general_error() end

-- set field
---@param value boolean
function EFI_State_ud:general_error(value) end

-- get field
---@return uint32_t_ud
function EFI_State_ud:last_updated_ms() end

-- set field
---@param value uint32_t_ud|integer|number
function EFI_State_ud:last_updated_ms(value) end


-- EFI Cylinder_Status structure
---@class (exact) Cylinder_Status_ud
local Cylinder_Status_ud = {}

---@return Cylinder_Status_ud
function Cylinder_Status() end

-- get field
---@return number
function Cylinder_Status_ud:lambda_coefficient() end

-- set field
---@param value number
function Cylinder_Status_ud:lambda_coefficient(value) end

-- get field
---@return number
function Cylinder_Status_ud:exhaust_gas_temperature() end

-- set field
---@param value number
function Cylinder_Status_ud:exhaust_gas_temperature(value) end

-- get field
---@return number
function Cylinder_Status_ud:exhaust_gas_temperature2() end

-- set field
---@param value number
function Cylinder_Status_ud:exhaust_gas_temperature2(value) end

-- get field
---@return number
function Cylinder_Status_ud:cylinder_head_temperature() end

-- set field
---@param value number
function Cylinder_Status_ud:cylinder_head_temperature(value) end

-- get field
---@return number
function Cylinder_Status_ud:cylinder_head_temperature2() end

-- set field
---@param value number
function Cylinder_Status_ud:cylinder_head_temperature2(value) end

-- get field
---@return number
function Cylinder_Status_ud:injection_time_ms() end

-- set field
---@param value number
function Cylinder_Status_ud:injection_time_ms(value) end

-- get field
---@return number
function Cylinder_Status_ud:ignition_timing_deg() end

-- set field
---@param value number
function Cylinder_Status_ud:ignition_timing_deg(value) end

-- desc
efi = {}

-- desc
---@return EFI_State_ud
function efi:get_state() end

-- get last update time in milliseconds
---@return uint32_t_ud
function efi:get_last_update_ms() end

-- desc
---@param instance integer
---@return AP_EFI_Backend_ud|nil
function efi:get_backend(instance) end

-- CAN bus interaction
CAN = {}

-- get a CAN bus device handler first scripting driver, will return nil if no driver with protocol Scripting is configured
---@param buffer_len uint32_t_ud|integer|number -- buffer length 1 to 25
---@return ScriptingCANBuffer_ud|nil
function CAN:get_device(buffer_len) end

-- get a CAN bus device handler second scripting driver, will return nil if no driver with protocol Scripting2 is configured
---@param buffer_len uint32_t_ud|integer|number -- buffer length 1 to 25
---@return ScriptingCANBuffer_ud|nil
function CAN:get_device2(buffer_len) end

-- Auto generated binding

-- desc
---@class (exact) CANFrame_ud
local CANFrame_ud = {}

---@return CANFrame_ud
function CANFrame() end

-- get field
---@return integer
function CANFrame_ud:dlc() end

-- set field
---@param value integer
function CANFrame_ud:dlc(value) end

-- get array field
---@param index integer
---@return integer
function CANFrame_ud:data(index) end

-- set array field
---@param index integer
---@param value integer
function CANFrame_ud:data(index, value) end

-- get field
---@return uint32_t_ud
function CANFrame_ud:id() end

-- set field
---@param value uint32_t_ud|integer|number
function CANFrame_ud:id(value) end

-- desc
---@return boolean
function CANFrame_ud:isErrorFrame() end

-- desc
---@return boolean
function CANFrame_ud:isRemoteTransmissionRequest() end

-- desc
---@return boolean
function CANFrame_ud:isExtended() end

-- desc
---@return integer
function CANFrame_ud:id_signed() end

-- desc
---@class (exact) motor_factor_table_ud
local motor_factor_table_ud = {}

---@return motor_factor_table_ud
function motor_factor_table() end

-- get array field
---@param index integer
---@return number
function motor_factor_table_ud:throttle(index) end

-- set array field
---@param index integer
---@param value number
function motor_factor_table_ud:throttle(index, value) end

-- get array field
---@param index integer
---@return number
function motor_factor_table_ud:yaw(index) end

-- set array field
---@param index integer
---@param value number
function motor_factor_table_ud:yaw(index, value) end

-- get array field
---@param index integer
---@return number
function motor_factor_table_ud:pitch(index) end

-- set array field
---@param index integer
---@param value number
function motor_factor_table_ud:pitch(index, value) end

-- get array field
---@param index integer
---@return number
function motor_factor_table_ud:roll(index) end

-- set array field
---@param index integer
---@param value number
function motor_factor_table_ud:roll(index, value) end

-- network socket class
---@class (exact) SocketAPM_ud
local SocketAPM_ud = {}

-- Get a new socket
---@param datagram boolean
---@return SocketAPM_ud
function Socket(datagram) end

-- return true if a socket is connected
---@return boolean
function SocketAPM_ud:is_connected() end

-- set blocking state of socket
---@param blocking boolean
---@return boolean
function SocketAPM_ud:set_blocking(blocking) end

-- setup a socket to listen
---@param backlog integer
---@return boolean
function SocketAPM_ud:listen(backlog) end

-- send a lua string. May contain binary data
---@param str string
---@param len uint32_t_ud|integer|number
---@return integer
function SocketAPM_ud:send(str, len) end

-- bind to an address. Use "0.0.0.0" for wildcard bind
---@param IP_address string
---@param port integer
---@return boolean
function SocketAPM_ud:bind(IP_address, port) end

-- connect a socket to an endpoint
---@param IP_address string
---@param port integer
---@return boolean
function SocketAPM_ud:connect(IP_address, port) end

--[[ accept new incoming sockets, returning a new socket.
     Must be used on a stream socket in listen state
--]]
---@return SocketAPM_ud|nil
function SocketAPM_ud:accept() end

-- receive data from a socket
---@param length integer
---@return string|nil
function SocketAPM_ud:recv(length) end

-- check for available input
---@param timeout_ms uint32_t_ud|integer|number
---@return boolean
function SocketAPM_ud:pollin(timeout_ms) end

-- check for availability of space to write to socket
---@param timeout_ms uint32_t_ud|integer|number
---@return boolean
function SocketAPM_ud:pollout(timeout_ms) end

--[[
   close a socket. Note that there is no automatic garbage
   collection of sockets so you must close a socket when you are
   finished with it or you will run out of sockets
--]]
function SocketAPM_ud:close() end

--[[
   setup to send all remaining data from a filehandle to the socket
   this also "closes" the socket and the file from the point of view of lua
   the underlying socket and file are both closed on end of file
--]]
---@param filehandle string
---@return boolean -- success
function SocketAPM_ud:sendfile(filehandle) end

-- enable SO_REUSEADDR on a socket
---@return boolean
function SocketAPM_ud:reuseaddress() end

-- desc
---@class (exact) AP_HAL__PWMSource_ud
local AP_HAL__PWMSource_ud = {}

---@return AP_HAL__PWMSource_ud
function PWMSource() end

-- desc
---@return integer
function AP_HAL__PWMSource_ud:get_pwm_avg_us() end

-- desc
---@return integer
function AP_HAL__PWMSource_ud:get_pwm_us() end

-- desc
---@param pin_number integer
---@return boolean
function AP_HAL__PWMSource_ud:set_pin(pin_number) end


-- desc
---@class (exact) mavlink_mission_item_int_t_ud
local mavlink_mission_item_int_t_ud = {}

---@return mavlink_mission_item_int_t_ud
function mavlink_mission_item_int_t() end

-- get field
---@return integer
function mavlink_mission_item_int_t_ud:current() end

-- set field
---@param value integer
function mavlink_mission_item_int_t_ud:current(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t_ud:frame() end

-- set field
---@param value integer
function mavlink_mission_item_int_t_ud:frame(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t_ud:command() end

-- set field
---@param value integer
function mavlink_mission_item_int_t_ud:command(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t_ud:seq() end

-- set field
---@param value integer
function mavlink_mission_item_int_t_ud:seq(value) end

-- get field
---@return number
function mavlink_mission_item_int_t_ud:z() end

-- set field
---@param value number
function mavlink_mission_item_int_t_ud:z(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t_ud:y() end

-- set field
---@param value integer
function mavlink_mission_item_int_t_ud:y(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t_ud:x() end

-- set field
---@param value integer
function mavlink_mission_item_int_t_ud:x(value) end

-- get field
---@return number
function mavlink_mission_item_int_t_ud:param4() end

-- set field
---@param value number
function mavlink_mission_item_int_t_ud:param4(value) end

-- get field
---@return number
function mavlink_mission_item_int_t_ud:param3() end

-- set field
---@param value number
function mavlink_mission_item_int_t_ud:param3(value) end

-- get field
---@return number
function mavlink_mission_item_int_t_ud:param2() end

-- set field
---@param value number
function mavlink_mission_item_int_t_ud:param2(value) end

-- get field
---@return number
function mavlink_mission_item_int_t_ud:param1() end

-- set field
---@param value number
function mavlink_mission_item_int_t_ud:param1(value) end


-- Parameter access helper.
---@class (exact) Parameter_ud
local Parameter_ud = {}

-- Create a new parameter helper, init must be called with a parameter name.
---@return Parameter_ud
function Parameter() end

-- Set the defualt value of this parameter, if the parameter has not been configured by the user its value will be updated to the new defualt.
---@param value number
---@return boolean
function Parameter_ud:set_default(value) end

-- Return true if the parameter has been configured by the user.
---@return boolean
function Parameter_ud:configured() end

-- Set the parameter to the given value and save. The value will be persistant after a reboot.
---@param value number
---@return boolean
function Parameter_ud:set_and_save(value) end

-- Set the parameter to the given value. The value will not persist a reboot.
---@param value number
---@return boolean
function Parameter_ud:set(value) end

-- Get the current value of a parameter.
-- Returns nil if the init has not been called and a valid parameter found.
---@return number|nil
function Parameter_ud:get() end

-- Init the paramter from a key. This allows the script to load old parameter that have been removed from the main code.
---@param key integer
---@param group_element uint32_t_ud|integer|number
---@param type integer
---| '1' # AP_PARAM_INT8
---| '2' # AP_PARAM_INT16
---| '3' # AP_PARAM_INT32
---| '4' # AP_PARAM_FLOAT
---@return boolean
function Parameter_ud:init_by_info(key, group_element, type) end

-- Init this parameter from a name.
---@param name string
---@return boolean
function Parameter_ud:init(name) end

-- Parameter access helper
---@class (exact) Parameter_ud_const
local Parameter_ud_const = {}

-- Create a new parameter helper with a parameter name.
-- This will error if no parameter with the given name is found.
---@return Parameter_ud_const
---@param name string
function Parameter(name) end

-- Set the defualt value of this parameter, if the parameter has not been configured by the user its value will be updated to the new defualt.
---@param value number
---@return boolean
function Parameter_ud_const:set_default(value) end

-- Retrun true if the parameter has been configured by the user.
---@return boolean
function Parameter_ud_const:configured() end

-- Set the parameter to the given value and save. The value will be persistant after a reboot.
---@param value number
---@return boolean
function Parameter_ud_const:set_and_save(value) end

-- Set the parameter to the given value. The value will not persist a reboot.
---@param value number
---@return boolean
function Parameter_ud_const:set(value) end

-- Get the current value of a parameter.
---@return number
function Parameter_ud_const:get() end

-- Vector2f is a userdata object that holds a 2D vector with x and y components. The components are stored as floating point numbers.
-- To create a new Vector2f you can call Vector2f() to allocate a new one, or call a method that returns one to you.
---@class (exact) Vector2f_ud
---@operator add(Vector2f_ud): Vector2f_ud
---@operator sub(Vector2f_ud): Vector2f_ud
local Vector2f_ud = {}

-- Create Vector2f object
---@return Vector2f_ud
function Vector2f() end

-- Copy this Vector2f returning a new userdata object
---@return Vector2f_ud -- a copy of this Vector2f
function Vector2f_ud:copy() end

-- get y component
---@return number
function Vector2f_ud:y() end

-- set y component
---@param value number
function Vector2f_ud:y(value) end

-- get x component
---@return number
function Vector2f_ud:x() end

-- set x component
---@param value number
function Vector2f_ud:x(value) end

-- rotate vector by angle in radians
---@param angle_rad number -- angle in radians
function Vector2f_ud:rotate(angle_rad) end

-- Check if both components of the vector are zero
---@return boolean -- true if both components are zero
function Vector2f_ud:is_zero() end

-- Check if either components of the vector are infinite
---@return boolean -- true if either components are infinite
function Vector2f_ud:is_inf() end

-- Check if either components of the vector are nan
---@return boolean -- true if either components are nan
function Vector2f_ud:is_nan() end

-- normalize this vector to a unit length
function Vector2f_ud:normalize() end

-- Calculate length of this vector sqrt(x^2 + y^2)
---@return number -- length of this vector
function Vector2f_ud:length() end

-- Calculate the angle of this vector in radians
-- 2PI + atan2(-x, y)
---@return number -- angle in radians
function Vector2f_ud:angle() end

-- Vector3f is a userdata object that holds a 3D vector with x, y and z components.
-- The components are stored as floating point numbers.
-- To create a new Vector3f you can call Vector3f() to allocate a new one, or call a method that returns one to you.
---@class (exact) Vector3f_ud
---@operator add(Vector3f_ud): Vector3f_ud
---@operator sub(Vector3f_ud): Vector3f_ud
local Vector3f_ud = {}

-- Create Vector3f object
---@return Vector3f_ud
function Vector3f() end

-- Copy this Vector3f returning a new userdata object
---@return Vector3f_ud -- a copy of this Vector3f
function Vector3f_ud:copy() end

-- get z component
---@return number
function Vector3f_ud:z() end

-- set z component
---@param value number
function Vector3f_ud:z(value) end

-- get y component
---@return number
function Vector3f_ud:y() end

-- set y component
---@param value number
function Vector3f_ud:y(value) end

-- get x component
---@return number
function Vector3f_ud:x() end

-- set x component
---@param value number
function Vector3f_ud:x(value) end

-- Return a new Vector3 based on this one with scaled length and the same changing direction
---@param scale_factor number
---@return Vector3f_ud -- scaled copy of this vector
function Vector3f_ud:scale(scale_factor) end

-- Cross product of two Vector3fs
---@param vector Vector3f_ud
---@return Vector3f_ud -- result
function Vector3f_ud:cross(vector) end

-- Dot product of two Vector3fs
---@param vector Vector3f_ud
---@return number -- result
function Vector3f_ud:dot(vector) end

-- Check if all components of the vector are zero
---@return boolean -- true if all components are zero
function Vector3f_ud:is_zero() end

-- Check if any components of the vector are infinite
---@return boolean -- true if any components are infinite
function Vector3f_ud:is_inf() end

-- Check if any components of the vector are nan
---@return boolean -- true if any components are nan
function Vector3f_ud:is_nan() end

-- normalize this vector to a unit length
function Vector3f_ud:normalize() end

-- Calculate length of this vector sqrt(x^2 + y^2 + z^2)
---@return number -- length of this vector
function Vector3f_ud:length() end

-- Computes angle between this vector and vector v2
---@param v2 Vector3f_ud
---@return number
function Vector3f_ud:angle(v2) end

-- Rotate vector by angle in radians in xy plane leaving z untouched
---@param param1 number -- XY rotation in radians
function Vector3f_ud:rotate_xy(param1) end

-- return the x and y components of this vector as a Vector2f
---@return Vector2f_ud
function Vector3f_ud:xy() end

-- desc
---@class (exact) Quaternion_ud
---@operator mul(Quaternion_ud): Quaternion_ud
local Quaternion_ud = {}

---@return Quaternion_ud
function Quaternion() end

-- get field
---@return number
function Quaternion_ud:q4() end

-- set field
---@param value number
function Quaternion_ud:q4(value) end

-- get field
---@return number
function Quaternion_ud:q3() end

-- set field
---@param value number
function Quaternion_ud:q3(value) end

-- get field
---@return number
function Quaternion_ud:q2() end

-- set field
---@param value number
function Quaternion_ud:q2(value) end

-- get field
---@return number
function Quaternion_ud:q1() end

-- set field
---@param value number
function Quaternion_ud:q1(value) end

-- Applies rotation to vector argument
---@param vec Vector3f_ud
function Quaternion_ud:earth_to_body(vec) end

-- Returns inverse of quaternion
---@return Quaternion_ud
function Quaternion_ud:inverse() end

-- Integrates angular velocity over small time delta
---@param angular_velocity Vector3f_ud
---@param time_delta number
function Quaternion_ud:from_angular_velocity(angular_velocity, time_delta) end

-- Constructs Quaternion from axis and angle
---@param axis Vector3f_ud
---@param angle number
function Quaternion_ud:from_axis_angle(axis, angle) end

-- Converts Quaternion to axis-angle representation
---@param axis_angle Vector3f_ud
function Quaternion_ud:to_axis_angle(axis_angle) end

-- Construct quaternion from Euler angles
---@param roll number
---@param pitch number
---@param yaw number
function Quaternion_ud:from_euler(roll, pitch, yaw) end

-- Returns yaw component of quaternion
---@return number
function Quaternion_ud:get_euler_yaw() end

-- Returns pitch component of quaternion
---@return number
function Quaternion_ud:get_euler_pitch() end

-- Returns roll component of quaternion
---@return number
function Quaternion_ud:get_euler_roll() end

-- Mutates quaternion have length 1
function Quaternion_ud:normalize() end

-- Returns length or norm of quaternion
---@return number
function Quaternion_ud:length() end

-- Location is a userdata object that holds locations expressed as latitude, longitude, altitude.
-- The altitude can be in several different frames, relative to home, absolute altitude above mean sea level, or relative to terrain.
-- To create a new Location userdata you can call Location() to allocate an empty location object, or call a method that returns one to you.
---@class (exact) Location_ud
local Location_ud = {}

-- Create location object
---@return Location_ud
function Location() end

-- Copy this location returning a new userdata object
---@return Location_ud -- a copy of this location
function Location_ud:copy() end

-- get loiter xtrack
---@return boolean -- Get if the location is used for a loiter location this flags if the aircraft should track from the center point, or from the exit location of the loiter.
function Location_ud:loiter_xtrack() end

-- set loiter xtrack
---@param value boolean -- Set if the location is used for a loiter location this flags if the aircraft should track from the center point, or from the exit location of the loiter.
function Location_ud:loiter_xtrack(value) end

-- get origin alt
---@return boolean -- true if altitude is relative to origin
function Location_ud:origin_alt() end

-- set origin alt
---@param value boolean -- set true if altitude is relative to origin
function Location_ud:origin_alt(value) end

-- get terrain alt
---@return boolean -- true if altitude is relative to terrain
function Location_ud:terrain_alt() end

-- set terrain alt
---@param value boolean -- set true if altitude is relative to home
function Location_ud:terrain_alt(value) end

-- get relative alt
---@return boolean -- true if altitude is relative to home
function Location_ud:relative_alt() end

-- set relative alt
---@param value boolean -- set true if altitude is relative to home
function Location_ud:relative_alt(value) end

-- get altitude in cm
---@return integer -- altitude in cm
function Location_ud:alt() end

-- set altitude in cm
---@param value integer
function Location_ud:alt(value) end

-- get longitude in degrees * 1e7
---@return integer -- longitude in degrees * 1e7
function Location_ud:lng() end

-- set longitude in degrees * 1e7
---@param value integer -- longitude in degrees * 1e7
function Location_ud:lng(value) end

-- get latitude in degrees * 1e7
---@return integer -- latitude in degrees * 1e7
function Location_ud:lat() end

-- set latitude in degrees * 1e7
---@param value integer -- latitude in degrees * 1e7
function Location_ud:lat(value) end

-- get altitude frame of this location
---@return integer -- altitude frame
---| '0' # ABSOLUTE
---| '1' # ABOVE_HOME
---| '2' # ABOVE_ORIGIN
---| '3' # ABOVE_TERRAIN
function Location_ud:get_alt_frame() end

-- Set the altitude frame of this location
---@param desired_frame integer  -- altitude frame
---| '0' # ABSOLUTE
---| '1' # ABOVE_HOME
---| '2' # ABOVE_ORIGIN
---| '3' # ABOVE_TERRAIN
---@return boolean
function Location_ud:change_alt_frame(desired_frame) end

-- Given a Location this calculates the north and east distance between the two locations in meters.
---@param loc Location_ud -- location to compare with
---@return Vector2f_ud -- North east distance vector in meters
function Location_ud:get_distance_NE(loc) end

-- Given a Location this calculates the north, east and down distance between the two locations in meters.
---@param loc Location_ud -- location to compare with
---@return Vector3f_ud -- North east down distance vector in meters
function Location_ud:get_distance_NED(loc) end

-- Given a Location this calculates the relative bearing to the location in radians
---@param loc Location_ud -- location to compare with
---@return number -- bearing in radians
function Location_ud:get_bearing(loc) end

-- Returns the offset from the EKF origin to this location.
-- Returns nil if the EKF origin wasn’t available at the time this was called.
---@return Vector3f_ud|nil -- Vector between origin and location north east up in meters
function Location_ud:get_vector_from_origin_NEU() end

-- Translates this Location by the specified  distance given a bearing.
---@param bearing_deg number -- bearing in degrees
---@param distance number -- distance in meters
function Location_ud:offset_bearing(bearing_deg, distance) end

-- Translates this Location by the specified distance given a bearing and pitch.
---@param bearing_deg number -- bearing in degrees
---@param pitch_deg number -- pitch in degrees
---@param distance number -- distance in meters
function Location_ud:offset_bearing_and_pitch(bearing_deg, pitch_deg, distance) end

-- Translates this Location by the specified north and east distance in meters.
---@param ofs_north number -- north offset in meters
---@param ofs_east number -- east offset in meters
function Location_ud:offset(ofs_north, ofs_east) end

-- Given a Location this calculates the horizontal distance between the two locations in meters.
---@param loc Location_ud -- location to compare with
---@return number -- horizontal distance in meters
function Location_ud:get_distance(loc) end

-- desc
---@class (exact) AP_EFI_Backend_ud
local AP_EFI_Backend_ud = {}

-- desc
---@param state EFI_State_ud
---@return boolean
function AP_EFI_Backend_ud:handle_scripting(state) end

-- desc
---@class (exact) ScriptingCANBuffer_ud
local ScriptingCANBuffer_ud = {}

-- desc
---@return CANFrame_ud|nil
function ScriptingCANBuffer_ud:read_frame() end

-- Add a filter to the CAN buffer, mask is bitwise ANDed with the frame id and compared to value if not match frame is not buffered
-- By default no filters are added and all frames are buffered, write is not affected by filters
-- Maximum number of filters is 8
---@param mask uint32_t_ud|integer|number
---@param value uint32_t_ud|integer|number
---@return boolean -- returns true if the filler was added successfully
function ScriptingCANBuffer_ud:add_filter(mask, value) end

-- desc
---@param frame CANFrame_ud
---@param timeout_us uint32_t_ud|integer|number
---@return boolean
function ScriptingCANBuffer_ud:write_frame(frame, timeout_us) end


-- desc
---@class (exact) AP_HAL__AnalogSource_ud
local AP_HAL__AnalogSource_ud = {}

-- desc
---@return number
function AP_HAL__AnalogSource_ud:voltage_average_ratiometric() end

-- desc
---@return number
function AP_HAL__AnalogSource_ud:voltage_latest() end

-- desc
---@return number
function AP_HAL__AnalogSource_ud:voltage_average() end

-- desc
---@param pin_number integer
---@return boolean
function AP_HAL__AnalogSource_ud:set_pin(pin_number) end


-- desc
---@class (exact) AP_HAL__I2CDevice_ud
local AP_HAL__I2CDevice_ud = {}

-- desc
---@param address integer
function AP_HAL__I2CDevice_ud:set_address(address) end

-- Performs an I2C transfer, sending data_str bytes (see string.pack) and
-- returning a string of any requested read bytes (see string.unpack)
---@param data_str string
---@param read_length integer
---@return string|nil
function AP_HAL__I2CDevice_ud:transfer(data_str, read_length) end

-- If no read length is provided a single register will be read and returned.
-- If read length is provided a table of register values are returned.
---@param register_num integer
---@param read_length? integer
---@return integer|table|nil
function AP_HAL__I2CDevice_ud:read_registers(register_num, read_length) end

-- desc
---@param register_num integer
---@param value integer
---@return boolean
function AP_HAL__I2CDevice_ud:write_register(register_num, value) end

-- desc
---@param retries integer
function AP_HAL__I2CDevice_ud:set_retries(retries) end


-- Serial port access object
---@class (exact) AP_Scripting_SerialAccess_ud
local AP_Scripting_SerialAccess_ud = {}

-- Start serial port with the given baud rate (no effect for device ports)
---@param baud_rate uint32_t_ud|integer|number
function AP_Scripting_SerialAccess_ud:begin(baud_rate) end

-- Writes a single byte
---@param value integer -- byte to write
---@return uint32_t_ud -- 1 if success else 0
function AP_Scripting_SerialAccess_ud:write(value) end

-- Writes a string. The number of bytes actually written, i.e. the length of the
-- written prefix of the string, is returned. It may be 0 up to the length of
-- the string.
---@param data string -- string of bytes to write
---@return integer -- number of bytes actually written, which may be 0
function AP_Scripting_SerialAccess_ud:writestring(data) end

-- Reads a single byte from the serial port
---@return integer -- byte, -1 if error or none available
function AP_Scripting_SerialAccess_ud:read() end

-- Reads up to `count` bytes and returns the bytes read as a string. No bytes
-- may be read, in which case a 0-length string is returned.
---@param count integer -- maximum number of bytes to read
---@return string|nil -- bytes actually read, which may be 0-length, or nil on error
function AP_Scripting_SerialAccess_ud:readstring(count) end

-- Returns number of available bytes to read.
---@return uint32_t_ud
function AP_Scripting_SerialAccess_ud:available() end

-- Set flow control option for serial port (no effect for device ports)
---@param flow_control_setting integer
---| '0' # disabled
---| '1' # enabled
---| '2' # auto
function AP_Scripting_SerialAccess_ud:set_flow_control(flow_control_setting) end


-- desc
---@class (exact) RC_Channel_ud
local RC_Channel_ud = {}

-- desc
---@return number
function RC_Channel_ud:norm_input_ignore_trim() end

-- desc
---@param PWM integer
function RC_Channel_ud:set_override(PWM) end

-- desc
---@return integer
function RC_Channel_ud:get_aux_switch_pos() end

-- desc return input on a channel from -1 to 1, centered on the trim. Ignores the deadzone
---@return number
function RC_Channel_ud:norm_input() end

-- desc return input on a channel from -1 to 1, centered on the trim. Returns zero when within deadzone of the trim
---@return number
function RC_Channel_ud:norm_input_dz() end

-- desc
winch = {}

-- desc
---@return number
function winch:get_rate_max() end

-- desc
---@param param1 number
function winch:set_desired_rate(param1) end

-- desc
---@param param1 number
function winch:release_length(param1) end

-- desc
function winch:relax() end

-- desc
---@return boolean
function winch:healthy() end

-- desc
iomcu = {}

-- Check if the IO is healthy
---@return boolean
function iomcu:healthy() end

-- desc
compass = {}

-- Check if the compass is healthy
---@param instance integer -- the 0-based index of the compass instance to return.
---@return boolean
function compass:healthy(instance) end

-- desc
camera = {}

-- desc
---@param instance integer
---@param distance_m number
function camera:set_trigger_distance(instance, distance_m) end

-- desc
---@param instance integer
---@param start_recording boolean
---@return boolean
function camera:record_video(instance, start_recording) end

-- desc
---@param instance integer
function camera:take_picture(instance) end

-- desc
---@class (exact) AP_Camera__camera_state_t_ud
local AP_Camera__camera_state_t_ud = {}

---@return AP_Camera__camera_state_t_ud
function AP_Camera__camera_state_t() end

-- get field
---@return Vector2f_ud
function AP_Camera__camera_state_t_ud:tracking_p1() end

-- get field
---@return Vector2f_ud
function AP_Camera__camera_state_t_ud:tracking_p2() end

-- get field
---@return integer
function AP_Camera__camera_state_t_ud:tracking_type() end

-- get field
---@return number
function AP_Camera__camera_state_t_ud:focus_value() end

-- get field
---@return integer
function AP_Camera__camera_state_t_ud:focus_type() end

-- get field
---@return number
function AP_Camera__camera_state_t_ud:zoom_value() end

-- get field
---@return integer
function AP_Camera__camera_state_t_ud:zoom_type() end

-- get field
---@return boolean
function AP_Camera__camera_state_t_ud:recording_video() end

-- get field
---@return integer
function AP_Camera__camera_state_t_ud:take_pic_incr() end

-- desc
---@param instance integer
---@return AP_Camera__camera_state_t_ud|nil
function camera:get_state(instance) end

-- Change a camera setting to a given value
---@param instance integer
---@param setting integer
---| '0' # THERMAL_PALETTE
---| '1' # THERMAL_GAIN
---| '2' # THERMAL_RAW_DATA
---@param value number
---@return boolean
function camera:change_setting(instance, setting, value) end

-- desc
mount = {}

-- desc
---@param instance integer
---@param roll_deg number
---@param pitch_deg number
---@param yaw_deg number
function mount:set_attitude_euler(instance, roll_deg, pitch_deg, yaw_deg) end

-- desc
---@param instance integer
---@return Location_ud|nil
function mount:get_location_target(instance) end

-- desc
---@param instance integer
---@return number|nil   -- roll_deg
---@return number|nil   -- pitch_deg
---@return number|nil   -- yaw_deg
---@return boolean|nil  -- yaw_is_earth_frame
function mount:get_angle_target(instance) end

-- desc
---@param instance integer
---@return number|nil   -- roll_degs
---@return number|nil   -- pitch_degs
---@return number|nil   -- yaw_degs
---@return boolean|nil  -- yaw_is_earth_frame
function mount:get_rate_target(instance) end

-- desc
---@param instance integer
---@param target_loc Location_ud
function mount:set_roi_target(instance, target_loc) end

-- desc
---@param instance integer
---@param roll_degs number
---@param pitch_degs number
---@param yaw_degs number
---@param yaw_is_earth_frame boolean
function mount:set_rate_target(instance, roll_degs, pitch_degs, yaw_degs, yaw_is_earth_frame) end

-- desc
---@param instance integer
---@param roll_deg number
---@param pitch_deg number
---@param yaw_deg number
---@param yaw_is_earth_frame boolean
function mount:set_angle_target(instance, roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame) end

-- desc
---@param instance integer
---@param mode integer
function mount:set_mode(instance, mode) end

-- desc
---@param instance integer
---@return integer
function mount:get_mode(instance) end

-- desc
---@param instance integer
---@return number|nil -- roll_deg
---@return number|nil -- pitch_deg
---@return number|nil -- yaw_bf_deg
function mount:get_attitude_euler(instance) end

-- desc
motors = {}

-- Get motors interlock state, the state of motors controlled by AP_Motors, Copter and Quadplane VTOL motors. Not plane forward flight motors.
---@return boolean
---| true  # motors active
---| false # motors inactive
function motors:get_interlock() end

-- get lateral motor output
---@return number
function motors:get_lateral() end

-- set external limit flags for each axis to prevent integrator windup
---@param roll boolean
---@param pitch boolean
---@param yaw boolean
---@param throttle_lower boolean
---@param throttle_upper boolean
function motors:set_external_limits(roll, pitch, yaw, throttle_lower, throttle_upper) end

-- get forward motor output
---@return number
function motors:get_forward() end

-- get throttle motor output
---@return number
function motors:get_throttle() end

-- get throttle motor output
---@return integer
---| '0' # Shut down
---| '1' # Ground idle
---| '2' # Spooling up
---| '3' # Throttle unlimited
---| '4' # Spooling down
function motors:get_spool_state() end

-- desc
---@param param1 string
function motors:set_frame_string(param1) end

-- desc
---@return integer
function motors:get_desired_spool_state() end

-- get yaw FF output
---@return number
function motors:get_yaw_ff() end

-- get yaw P+I+D
---@return number
function motors:get_yaw() end

-- get pitch FF out
---@return number
function motors:get_pitch_ff() end

-- get pitch P+I+D out
---@return number
function motors:get_pitch() end

-- get roll FF out
---@return number
function motors:get_roll_ff() end

-- get roll P+I+D
---@return number
function motors:get_roll() end

-- desc
FWVersion = {}

-- get field
---@return string
function FWVersion:hash() end

-- get field
---@return integer
function FWVersion:patch() end

-- get field
---@return integer
function FWVersion:minor() end

-- get field
---@return integer
function FWVersion:major() end

--get APM_BUILD_? value from AP_Vehicle/AP_Vehicle_Type.h that is checked against APM_BUILD_TYPE()
---@return integer
---| '1' # Rover
---| '2' # ArduCopter
---| '3' # ArduPlane
---| '4' # AntennaTracker
---| '7' # ArduSub
---| '9' # AP_Periph
---| '12' # Blimp
---| '13' # Heli
function FWVersion:type() end

-- get field
---@return string
function FWVersion:string() end


-- desc
periph = {}

-- desc
---@return uint32_t_ud
function periph:get_vehicle_state() end

-- desc
---@return number
function periph:get_yaw_earth() end

-- desc
---@param text string
function periph:can_printf(text) end

-- desc
---@param hold_in_bootloader boolean
function periph:reboot(hold_in_bootloader) end

-- desc
ins = {}

-- desc
---@param instance integer
---@return number
function ins:get_temperature(instance) end

-- Check if the gyrometers are consistent
---@param threshold integer -- the allowed threshold in degrees per second
---@return boolean
function ins:gyros_consistent(threshold) end

-- Check if a specific gyroscope sensor is healthy
---@param instance integer -- the 0-based index of the gyroscope instance to return.
---@return boolean
function ins:get_gyro_health(instance) end

-- Check if the accelerometers are consistent
---@param threshold number -- the threshold allowed before returning false
---@return boolean
function ins:accels_consistent(threshold) end

-- Check if a specific accelerometer sensor is healthy
---@param instance integer -- the 0-based index of the accelerometer instance to return.
---@return boolean
function ins:get_accel_health(instance) end

-- Get if the INS is currently calibrating
---@return boolean
function ins:calibrating() end

-- Get the value of a specific gyroscope
---@param instance integer -- the 0-based index of the gyroscope instance to return.
---@return Vector3f_ud
function ins:get_gyro(instance) end

-- Get the value of a specific accelerometer
---@param instance integer -- the 0-based index of the accelerometer instance to return.
---@return Vector3f_ud
function ins:get_accel(instance) end

-- desc
Motors_dynamic = {}

-- desc
---@param factor_table motor_factor_table_ud
function Motors_dynamic:load_factors(factor_table) end

-- desc
---@param motor_num integer
---@param testing_order integer
function Motors_dynamic:add_motor(motor_num, testing_order) end

-- desc
---@param expected_num_motors integer
---@return boolean
function Motors_dynamic:init(expected_num_motors) end


-- desc
analog = {}

-- return MCU temperature in degrees C
---@return number -- MCU temperature
function analog:mcu_temperature() end

-- desc
---@return AP_HAL__AnalogSource_ud|nil
function analog:channel() end


-- Control of general purpose input/output pins
gpio = {}

-- set GPIO pin mode
---@param pin_number integer
---@param mode integer
---| '0' # input
---| '1' # output
function gpio:pinMode(pin_number, mode) end

-- toggle GPIO output
---@param pin_number integer
function gpio:toggle(pin_number) end

-- write GPIO output
---@param pin_number integer
---@param value integer
---| '0' # low
---| '1' # high
function gpio:write(pin_number, value) end

-- read GPIO input
---@param pin_number integer
---@return boolean -- pin state
function gpio:read(pin_number) end


-- desc
Motors_6DoF = {}

-- desc
---@param motor_num integer
---@param roll_factor number
---@param pitch_factor number
---@param yaw_factor number
---@param throttle_factor number
---@param forward_factor number
---@param right_factor number
---@param reversible boolean
---@param testing_order integer
function Motors_6DoF:add_motor(motor_num, roll_factor, pitch_factor, yaw_factor, throttle_factor, forward_factor, right_factor, reversible, testing_order) end

-- desc
---@param expected_num_motors integer
---@return boolean
function Motors_6DoF:init(expected_num_motors) end


-- desc
attitude_control = {}

-- desc
---@param roll_deg number
---@param pitch_deg number
function attitude_control:set_offset_roll_pitch(roll_deg, pitch_deg) end

-- desc
---@param bool boolean
function attitude_control:set_forward_enable(bool) end

-- desc
---@param bool boolean
function attitude_control:set_lateral_enable(bool) end


-- desc
frsky_sport = {}

-- desc
---@param number integer
---@param digits integer
---@param power integer
---@return integer
function frsky_sport:prep_number(number, digits, power) end

-- desc
---@param sensor integer
---@param frame integer
---@param appid integer
---@param data integer
---@return boolean
function frsky_sport:sport_telemetry_push(sensor, frame, appid, data) end


-- desc
MotorsMatrix = {}

-- desc
---@param motor_num integer
---@param throttle_factor number
---@return boolean
function MotorsMatrix:set_throttle_factor(motor_num, throttle_factor) end

-- desc
---@param motor_num integer
---@param roll_factor number
---@param pitch_factor number
---@param yaw_factor number
---@param testing_order integer
function MotorsMatrix:add_motor_raw(motor_num, roll_factor, pitch_factor, yaw_factor, testing_order) end

-- desc
---@param expected_num_motors integer
---@return boolean
function MotorsMatrix:init(expected_num_motors) end

-- desc get index (starting at 0) of lost motor
---@return integer
function MotorsMatrix:get_lost_motor() end

-- desc return true if we are in thrust boost due to possible lost motor
---@return boolean
function MotorsMatrix:get_thrust_boost() end


-- Sub singleton
sub = {}

-- Return true if joystick button is currently pressed
---@param index integer
---@return boolean
function sub:is_button_pressed(index) end

-- Get count of joystick button presses, then clear count
---@param index integer
---@return integer
function sub:get_and_clear_button_count(index) end

-- Return true if rangefinder is healthy, includes a check for good signal quality
---@return boolean
function sub:rangefinder_alt_ok() end

-- SURFTRAK mode: return the rangefinder target in cm
---@return number
function sub:get_rangefinder_target_cm() end

-- SURFTRAK mode: set the rangefinder target in cm, return true if successful
---@param new_target_cm number
---@return boolean
function sub:set_rangefinder_target_cm(new_target_cm) end


-- desc
quadplane = {}

-- desc
---@return boolean
function quadplane:in_assisted_flight() end

-- desc
---@return boolean
function quadplane:in_vtol_mode() end

-- true in descent phase of VTOL landing
---@return boolean
function quadplane:in_vtol_land_descent() end

-- abort a VTOL landing, climbing back up
---@return boolean
function quadplane:abort_landing() end


-- desc
LED = {}

-- desc
---@return integer
---@return integer
---@return integer
function LED:get_rgb() end


-- button handling
button = {}

-- Returns button state if available. Buttons are 1 indexed.
---@param button_number integer -- button number 1 indexed.
---@return boolean
function button:get_button_state(button_number) end


-- RPM handling
RPM = {}

--  Returns RPM of given instance, or nil if not available
---@param instance integer -- RPM instance
---@return number|nil -- RPM value if available
function RPM:get_rpm(instance) end


-- desc
mission = {}
mission.MISSION_COMPLETE = enum_integer
mission.MISSION_RUNNING = enum_integer
mission.MISSION_STOPPED = enum_integer

-- clear - clears out mission
---@return boolean
function mission:clear() end

-- set any WP items in any order in a mavlink-ish kinda way.
---@param index integer
---@param item mavlink_mission_item_int_t_ud
---@return boolean
function mission:set_item(index, item) end

-- get any WP items in any order in a mavlink-ish kinda way.
---@param index integer
---@return mavlink_mission_item_int_t_ud|nil
function mission:get_item(index) end

-- num_commands - returns total number of commands in the mission
--                 this number includes offset 0, the home location
---@return integer
function mission:num_commands() end

-- get_current_do_cmd_id - returns id of the active "do" command
---@return integer
function mission:get_current_do_cmd_id() end

-- get_current_nav_id - return the id of the current nav command
---@return integer
function mission:get_current_nav_id() end

-- get_prev_nav_cmd_id - returns the previous "navigation" command id
--     if there was no previous nav command it returns AP_MISSION_CMD_ID_NONE (0)
--      we do not return the entire command to save on RAM
---@return integer
function mission:get_prev_nav_cmd_id() end

-- set_current_cmd - jumps to command specified by index
---@param index integer
---@return boolean
function mission:set_current_cmd(index) end

-- get_current_nav_index - returns the current "navigation" command index
-- Note that this will return 0 if there is no command. This is
-- used in MAVLink reporting of the mission command
---@return integer
function mission:get_current_nav_index() end

-- status - returns the status of the mission (i.e. Mission_Started, Mission_Complete, Mission_Stopped
---@return integer
function mission:state() end

-- returns true if the mission cmd has a location
---@param cmd integer
---@return boolean
function mission:cmd_has_location(cmd)end

-- Set the mission index to the first JUMP_TAG with this tag.
-- Returns true on success, else false if no appropriate JUMP_TAG match can be found or if setting the index failed
---@param tag integer
---@return boolean
function mission:jump_to_tag(tag) end

-- desc
---@param tag integer
---@return integer
function mission:get_index_of_jump_tag(tag) end

-- Jump Tags. When a JUMP_TAG is run in the mission, either via DO_JUMP_TAG or
-- by just being the next item, the tag is remembered and the age is set to 1.
-- Only the most recent tag is remembered. It's age is how many NAV items have
-- progressed since the tag was seen. While executing the tag, the
-- age will be 1. The next NAV command after it will tick the age to 2, and so on.
---@return integer|nil
---@return integer|nil
function mission:get_last_jump_tag() end


-- Jump the mission to the start of the closest landing sequence. Returns true if one was found
---@return boolean
function mission:jump_to_landing_sequence() end

-- Jump to the landing abort sequence
---@return boolean
function mission:jump_to_abort_landing_sequence() end

-- Parameter access
param = {}

-- set and save parameter value, this will be saved for subsequent boots
---@param name string -- parameter name
---@param value number -- value to set and save
---@return boolean -- true if parameter was found
function param:set_and_save(name, value) end

-- set parameter value, this will not be retained over a reboot
---@param name string -- parameter name
---@param value number -- value to set
---@return boolean -- true if parameter was found
function param:set(name, value) end

-- Get parameter value
---@param name string -- parameter name
---@return number|nil -- nill if parameter was not found
function param:get(name) end

-- Set default value for a given parameter. If the parameter has not been configured by the user then the set to this default value.
---@param name string -- parameter name
---@param value number -- default value
---@return boolean -- true if parameter was found
function param:set_default(name, value) end

-- desc
---@param table_key integer
---@param prefix string
---@param num_params integer
---@return boolean
function param:add_table(table_key, prefix, num_params) end

-- desc
---@param table_key integer
---@param param_num integer
---@param name string
---@param default_value number
---@return boolean
function param:add_param(table_key, param_num, name, default_value) end

-- desc
---@class (exact) ESCTelemetryData_ud
local ESCTelemetryData_ud = {}

---@return ESCTelemetryData_ud
function ESCTelemetryData() end

-- set motor temperature
---@param value integer
function ESCTelemetryData_ud:motor_temp_cdeg(value) end

-- set consumption
---@param value number
function ESCTelemetryData_ud:consumption_mah(value) end

-- set current
---@param value number
function ESCTelemetryData_ud:current(value) end

-- set voltage
---@param value number
function ESCTelemetryData_ud:voltage(value) end

-- set temperature
---@param value integer
function ESCTelemetryData_ud:temperature_cdeg(value) end

-- desc
esc_telem = {}

-- update telemetry data for an ESC instance
---@param instance integer -- esc instance 0 indexed
---@param telemdata ESCTelemetryData_ud
---@param data_mask integer -- bit mask of what fields are filled in
function esc_telem:update_telem_data(instance, telemdata, data_mask) end

-- Returns an individual ESC’s usage time in seconds, or nil if not available.
---@param instance integer -- esc instance 0 indexed
---@return uint32_t_ud|nil -- usage time in seconds, nill if not available.
function esc_telem:get_usage_seconds(instance) end

-- desc
---@param instance integer -- esc instance 0 indexed
---@return number|nil
function esc_telem:get_consumption_mah(instance) end

-- desc
---@param instance integer -- esc instance 0 indexed
---@return number|nil
function esc_telem:get_voltage(instance) end

-- desc
---@param instance integer -- esc instance 0 indexed
---@return number|nil
function esc_telem:get_current(instance) end

-- desc
---@param instance integer -- esc instance 0 indexed
---@return integer|nil
function esc_telem:get_motor_temperature(instance) end

-- desc
---@param instance integer -- esc instance 0 indexed
---@return integer|nil
function esc_telem:get_temperature(instance) end

-- desc
---@param instance integer -- esc instance 0 indexed
---@return number|nil
function esc_telem:get_rpm(instance) end

-- update RPM for an ESC
---@param esc_index integer -- esc instance 0 indexed
---@param rpm integer -- RPM
---@param error_rate number -- error rate
function esc_telem:update_rpm(esc_index, rpm, error_rate) end

-- set scale factor for RPM on a motor
---@param esc_index integer -- esc instance 0 indexed
---@param scale_factor number -- factor
function esc_telem:set_rpm_scale(esc_index, scale_factor) end

-- get the timestamp of last telemetry data for an ESC
---@param esc_index integer
---@return uint32_t_ud
function esc_telem:get_last_telem_data_ms(esc_index) end

-- desc
optical_flow = {}

-- desc
---@return integer
function optical_flow:quality() end

-- desc
---@return boolean
function optical_flow:healthy() end

-- desc
---@return boolean
function optical_flow:enabled() end


-- desc
baro = {}

-- get external temperature in degrees C
---@return number -- temperature in degrees C
function baro:get_external_temperature() end

-- get temperature in degrees C
---@return number -- temperature in degrees C
function baro:get_temperature() end

-- Returns pressure in Pascal. Divide by 100 for millibars or hectopascals
---@return number -- pressure in Pascal
function baro:get_pressure() end

-- get current altitude in meters relative to altitude at the time
-- of the last calibrate() call, typically at boot
---@return number
function baro:get_altitude() end

-- Check if a baro sensor is healthy
---@param instance integer -- the 0-based index of the BARO instance to return.
---@return boolean
function baro:healthy(instance) end

-- get altitude difference from a base pressure and current pressure
---@param base_pressure number -- first reference pressure in Pa
---@param pressure number -- 2nd pressure in Pa
---@return number -- altitude difference in meters
function baro:get_altitude_difference(base_pressure,pressure) end

-- Serial ports
serial = {}

-- Returns a serial access object that allows a script to interface with a
-- device over a port set to protocol 28 (Scripting) (e.g. SERIALx_PROTOCOL).
-- Instance 0 is the first such port, instance 1 the second, and so on. If the
-- requested instance is not found, returns nil.
---@param instance integer -- 0-based index of the Scripting port to access
---@return AP_Scripting_SerialAccess_ud|nil -- access object for that instance, or nil if not found
function serial:find_serial(instance) end

-- Returns a serial access object that allows a script to simulate a device
-- attached via a specific protocol. The device protocol is configured by
-- SCR_SDEVx_PROTO. Instance 0 is the first such protocol, instance 1 the
-- second, and so on. If the requested instance is not found, or SCR_SDEV_EN is
-- disabled, returns nil.
---@param protocol integer -- protocol to access
---@param instance integer -- 0-based index of the protocol instance to access
---@return AP_Scripting_SerialAccess_ud|nil -- access object for that instance, or nil if not found
function serial:find_simulated_device(protocol, instance) end


-- desc
rc = {}

-- desc
---@param chan_num integer
---@return RC_Channel_ud|nil
function rc:get_channel(chan_num) end

-- desc
---@return boolean
function rc:has_valid_input() end

-- return cached level of aux function
---@param aux_fn integer
---@return integer|nil
function rc:get_aux_cached(aux_fn) end

-- desc
---@param aux_fun integer
---@param ch_flag integer
---| '0' # low
---| '1' # middle
---| '2' # high
---@return boolean
function rc:run_aux_function(aux_fun, ch_flag) end

-- desc
---@param aux_fun integer
---@return RC_Channel_ud|nil
function rc:find_channel_for_option(aux_fun) end

-- Returns the RC input PWM value given a channel number. Note that channel here is indexed from 1. Returns nill if channel is not available.
---@param chan_num integer -- input channel number, 1 indexed
---@return integer|nil -- pwm input or nil if not availables
function rc:get_pwm(chan_num) end


-- desc
SRV_Channels = {}

-- Get emergency stop state if active no motors of any kind will be active
---@return boolean
---| true # E-Stop active
---| false # E-Stop inactive
function SRV_Channels:get_emergency_stop() end

-- Get safety state
---@return boolean
---| true # Disarmed outputs inactive
---| false # Armed outputs live
function SRV_Channels:get_safety_state() end

-- desc
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@param range integer
function SRV_Channels:set_range(function_num, range) end

-- desc
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@param angle integer
function SRV_Channels:set_angle(function_num, angle) end

-- desc
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@param value number
function SRV_Channels:set_output_norm(function_num, value) end

-- Get the scaled value for a given servo function
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@return number -- scaled value
function SRV_Channels:get_output_scaled(function_num) end

-- Returns first servo output PWM value an output assigned output_function (See SERVOx_FUNCTION parameters). Nil if none is assigned.
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@return integer|nil -- output pwm if available
function SRV_Channels:get_output_pwm(function_num) end

-- Set the scaled value of the output function, scale is out of the value set with the set_range or set_angle call
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@param value number -- scaled value
function SRV_Channels:set_output_scaled(function_num, value) end

-- Sets servo channel to specified PWM for a time in ms. This overrides any commands from the autopilot until the timeout expires.
---@param chan integer -- servo channel number (zero indexed)
---@param pwm integer -- pwm value
---@param timeout_ms integer -- duration of the override
function SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, timeout_ms) end

-- Set the pwm for a given servo output channel
---@param chan integer -- servo channel number (zero indexed)
---@param pwm integer -- pwm value
function SRV_Channels:set_output_pwm_chan(chan, pwm) end

-- Set the pwm for a given servo output function
---@param function_num integer -- servo function number (See SERVOx_FUNCTION parameters)
---@param pwm integer -- pwm value
function SRV_Channels:set_output_pwm(function_num, pwm) end

-- Returns first servo output number (zero indexed) of an output assigned output_function (See SERVOx_FUNCTION parameters ). 0 = SERVO1_FUNCTION ect. Nil if none is assigned.
---@param function_num integer -- servo function (See SERVOx_FUNCTION parameters)
---@return integer|nil -- output channel number if available
function SRV_Channels:find_channel(function_num) end


-- This library allows the control of RGB LED strings via an output reserved for scripting and selected by SERVOx_FUNCTION = 94 thru 109 (Script Out 1 thru 16)
serialLED = {}

-- Send the configured RGB values to the LED string
---@param chan integer -- output number to which the leds are attached 1-16
---@return boolean -- true if successful 
function serialLED:send(chan) end

--  Set the data for LED_number on the string attached channel output
---@param chan integer -- output number to which the leds are attached 1-16
---@param led_index integer -- led number 0 index, -1 sets all
---@param red integer -- red value 0 to 255
---@param green integer -- green value 0 to 255
---@param blue integer -- blue value 0 to 255
---@return boolean -- true if successful
function serialLED:set_RGB(chan, led_index, red, green, blue) end

-- Sets the number of LEDs in a profiled string on a servo output.
---@param chan integer -- output number to which the leds are attached 1-16
---@param num_leds integer -- number of leds in the string
---@return boolean -- true if successful
function serialLED:set_num_profiled(chan, num_leds) end

-- Sets the number of LEDs in a neopixel string on a servo output.
---@param chan integer -- output number to which the leds are attached 1-16
---@param num_leds integer -- number of leds in the string
---@return boolean -- true if successful
function serialLED:set_num_neopixel(chan, num_leds) end

-- Sets the number of LEDs in a rgb neopixel string on a servo output.
---@param chan integer -- output number to which the leds are attached 1-16
---@param num_leds integer -- number of leds in the string
---@return boolean -- true if successful
function serialLED:set_num_neopixel_rgb(chan, num_leds) end

-- desc
vehicle = {}

-- override landing descent rate, times out in 1s
---@param rate number
---@return boolean
function vehicle:set_land_descent_rate(rate) end

-- desc
---@param rudder_pct number
---@param run_yaw_rate_control boolean
function vehicle:set_rudder_offset(rudder_pct, run_yaw_rate_control) end

-- desc
---@return boolean
function vehicle:has_ekf_failsafed() end

-- desc
---@return number|nil
---@return number|nil
function vehicle:get_pan_tilt_norm() end

-- desc
---@return number|nil
function vehicle:get_wp_crosstrack_error_m() end

-- desc
---@return number|nil
function vehicle:get_wp_bearing_deg() end

-- desc
---@return number|nil
function vehicle:get_wp_distance_m() end

-- desc
---@param steering number
---@param throttle number
---@return boolean
function vehicle:set_steering_and_throttle(steering, throttle) end

-- desc
---@return number|nil
---@return number|nil
function vehicle:get_steering_and_throttle() end

-- desc
---@param rate_dps number
---@return boolean
function vehicle:set_circle_rate(rate_dps) end

-- desc
---@return number|nil
function vehicle:get_circle_radius() end

-- desc
---@param roll_deg number
---@param pitch_deg number
---@param yaw_deg number
---@param climb_rate_ms number
---@param use_yaw_rate boolean
---@param yaw_rate_degs number
---@return boolean
function vehicle:set_target_angle_and_climbrate(roll_deg, pitch_deg, yaw_deg, climb_rate_ms, use_yaw_rate, yaw_rate_degs) end

-- Sets the target velocity using a Vector3f object in a guided mode.
---@param vel_ned Vector3f_ud -- North, East, Down meters / second
---@return boolean -- true on success
function vehicle:set_target_velocity_NED(vel_ned) end

-- desc
---@param target_vel Vector3f_ud
---@param target_accel Vector3f_ud
---@param use_yaw boolean
---@param yaw_deg number
---@param use_yaw_rate boolean
---@param yaw_rate_degs number
---@param yaw_relative boolean
---@return boolean
function vehicle:set_target_velaccel_NED(target_vel, target_accel, use_yaw, yaw_deg, use_yaw_rate, yaw_rate_degs, yaw_relative) end

-- desc
---@param target_pos Vector3f_ud
---@param target_vel Vector3f_ud
---@param target_accel Vector3f_ud
---@param use_yaw boolean
---@param yaw_deg number
---@param use_yaw_rate boolean
---@param yaw_rate_degs number
---@param yaw_relative boolean
---@return boolean
function vehicle:set_target_posvelaccel_NED(target_pos, target_vel, target_accel, use_yaw, yaw_deg, use_yaw_rate, yaw_rate_degs, yaw_relative) end

-- desc
---@param target_pos Vector3f_ud
---@param target_vel Vector3f_ud
---@return boolean
function vehicle:set_target_posvel_NED(target_pos, target_vel) end

-- desc
---@param target_pos Vector3f_ud
---@param use_yaw boolean
---@param yaw_deg number
---@param use_yaw_rate boolean
---@param yaw_rate_degs number
---@param yaw_relative boolean
---@param terrain_alt boolean
---@return boolean
function vehicle:set_target_pos_NED(target_pos, use_yaw, yaw_deg, use_yaw_rate, yaw_rate_degs, yaw_relative, terrain_alt) end

-- desc
---@param current_target Location_ud -- current target, from get_target_location()
---@param new_target Location_ud -- new target
---@return boolean
function vehicle:update_target_location(current_target, new_target) end

-- Get the current target location if available in current mode
---@return Location_ud|nil -- target location
function vehicle:get_target_location() end

-- Set the target veicle location in a guided mode
---@param target_loc Location_ud -- target location
---@return boolean -- true on success
function vehicle:set_target_location(target_loc) end

-- Trigger a takeoff start if in a auto or guided mode. Not supported by all vehicles
---@param alt number -- takeoff altitude in meters
---@return boolean -- true on success
function vehicle:start_takeoff(alt) end

-- desc
---@param control_output integer
---| '1' # Roll
---| '2' # Pitch
---| '3' # Throttle
---| '4' # Yaw
---| '5' # Lateral
---| '6' # MainSail
---| '7' # WingSail
---| '8' # Walking_Height
---@return number|nil
function vehicle:get_control_output(control_output) end

-- Returns time in milliseconds since the autopilot thinks it started flying, or zero if not currently flying.
---@return uint32_t_ud -- flying time in milliseconds
function vehicle:get_time_flying_ms() end

-- Returns true if the autopilot thinks it is flying. Not guaranteed to be accurate.
---@return boolean -- true if likely flying
function vehicle:get_likely_flying() end

-- desc
---@return integer
function vehicle:get_control_mode_reason() end

--  Returns current vehicle mode by mode_number.
---@return integer -- mode number. Values for each vehcile type can be found here: https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE
function vehicle:get_mode() end

-- Attempts to change vehicle mode to mode_number. Returns true if successful, false if mode change is not successful.
---@param mode_number integer -- mode number values for each vehcile type can be found here: https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE
---@return boolean -- success
function vehicle:set_mode(mode_number) end

-- desc
---@param param1 Vector2f_ud
---@return boolean
function vehicle:set_velocity_match(param1) end

-- desc
---@param param1 integer
---@return boolean
function vehicle:nav_scripting_enable(param1) end

-- desc sets autopilot nav speed (Copter and Rover)
---@param param1 number
---@return boolean
function vehicle:set_desired_speed(param1) end

-- desc
---@param param1 number
---@param param2 number
---@return boolean
function vehicle:set_desired_turn_rate_and_speed(param1, param2) end

-- desc
---@param param1 number -- throttle percent
---@param param2 number -- roll rate deg/s
---@param param3 number -- pitch rate deg/s
---@param param4 number -- yaw rate deg/s
function vehicle:set_target_throttle_rate_rpy(param1, param2, param3, param4) end

-- desc
---@param param1 integer
function vehicle:nav_script_time_done(param1) end

-- desc
---@return integer|nil -- id
---@return integer|nil -- cmd
---@return number|nil -- arg1
---@return number|nil -- arg2
---@return integer|nil -- arg3
---@return integer|nil -- arg4
function vehicle:nav_script_time() end

-- desc
---@param hold_in_bootloader boolean
function vehicle:reboot(hold_in_bootloader) end

-- desc
---@return boolean
function vehicle:is_taking_off() end

-- desc
---@return boolean
function vehicle:is_landing() end

-- Set the previous target location for crosstrack and crosstrack if available in the current mode
-- It's up to the Lua code to ensure the new_start_location makes sense
---@param new_start_location Location_ud
---@return boolean -- true on success
function vehicle:set_crosstrack_start(new_start_location) end

-- desc
onvif = {}

-- desc
---@return Vector2f_ud
function onvif:get_pan_tilt_limit_max() end

-- desc
---@return Vector2f_ud
function onvif:get_pan_tilt_limit_min() end

-- desc
---@param pan number
---@param tilt number
---@param zoom number
---@return boolean
function onvif:set_absolutemove(pan, tilt, zoom) end

-- desc
---@param username string
---@param password string
---@param httphostname string
---@return boolean
function onvif:start(username, password, httphostname) end


-- MAVLink interaction with ground control station
gcs = {}

-- send named float value using NAMED_VALUE_FLOAT message
---@param name string -- up to 10 chars long
---@param value number -- value to send
function gcs:send_named_float(name, value) end

-- set message interval for a given serial port and message id
---@param port_num integer -- serial port number
---@param msg_id uint32_t_ud|integer|number -- MAVLink message id
---@param interval_us integer -- interval in micro seconds
---@return integer
---| '0' # Accepted
---| '4' # Failed
function gcs:set_message_interval(port_num, msg_id, interval_us) end

-- get the vehicle MAV_TYPE
---@return integer
---| '0' # MAV_TYPE_GENERIC=0, /* Generic micro air vehicle | */
---| '1' # MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
---| '2' # MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
---| '3' # MAV_TYPE_COAXIAL=3, /* Coaxial helicopter | */
---| '4' # MAV_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
---| '5' # MAV_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
---| '6' # MAV_TYPE_GCS=6, /* Operator control unit / ground control station | */
---| '7' # MAV_TYPE_AIRSHIP=7, /* Airship, controlled | */
---| '8' # MAV_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
---| '9' # MAV_TYPE_ROCKET=9, /* Rocket | */
---| '10' # MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
---| '11' # MAV_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
---| '12' # MAV_TYPE_SUBMARINE=12, /* Submarine | */
---| '13' # MAV_TYPE_HEXAROTOR=13, /* Hexarotor | */
---| '14' # MAV_TYPE_OCTOROTOR=14, /* Octorotor | */
---| '15' # MAV_TYPE_TRICOPTER=15, /* Tricopter | */
---| '16' # MAV_TYPE_FLAPPING_WING=16, /* Flapping wing | */
---| '17' # MAV_TYPE_KITE=17, /* Kite | */
---| '18' # MAV_TYPE_ONBOARD_CONTROLLER=18, /* Onboard companion controller | */
---| '19' # MAV_TYPE_VTOL_DUOROTOR=19, /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
---| '20' # MAV_TYPE_VTOL_QUADROTOR=20, /* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
---| '21' # MAV_TYPE_VTOL_TILTROTOR=21, /* Tiltrotor VTOL | */
---| '22' # MAV_TYPE_VTOL_RESERVED2=22, /* VTOL reserved 2 | */
---| '23' # MAV_TYPE_VTOL_RESERVED3=23, /* VTOL reserved 3 | */
---| '24' # MAV_TYPE_VTOL_RESERVED4=24, /* VTOL reserved 4 | */
---| '25' # MAV_TYPE_VTOL_RESERVED5=25, /* VTOL reserved 5 | */
---| '26' # MAV_TYPE_GIMBAL=26, /* Gimbal | */
---| '27' # MAV_TYPE_ADSB=27, /* ADSB system | */
---| '28' # MAV_TYPE_PARAFOIL=28, /* Steerable, nonrigid airfoil | */
---| '29' # MAV_TYPE_DODECAROTOR=29, /* Dodecarotor | */
---| '30' # MAV_TYPE_CAMERA=30, /* Camera | */
---| '31' # MAV_TYPE_CHARGING_STATION=31, /* Charging station | */
---| '32' # MAV_TYPE_FLARM=32, /* FLARM collision avoidance system | */
---| '33' # MAV_TYPE_SERVO=33, /* Servo | */
---| '34' # MAV_TYPE_ODID=34, /* Open Drone ID. See https://mavlink.io/en/services/opendroneid.html. | */
---| '35' # MAV_TYPE_DECAROTOR=35, /* Decarotor | */
---| '36' # MAV_TYPE_BATTERY=36, /* Battery | */
---| '37' # MAV_TYPE_PARACHUTE=37, /* Parachute | */
---| '38' # MAV_TYPE_LOG=38, /* Log | */
---| '39' # MAV_TYPE_OSD=39, /* OSD | */
---| '40' # MAV_TYPE_IMU=40, /* IMU | */
---| '41' # MAV_TYPE_GPS=41, /* GPS | */
---| '42' # MAV_TYPE_WINCH=42, /* Winch | */
---| '43' # MAV_TYPE_ENUM_END=43, /*  | */
function gcs:frame_type() end

-- get the throttle value in %
---@return integer
function gcs:get_hud_throttle() end

-- set high latency control state. Analogous to MAV_CMD_CONTROL_HIGH_LATENCY
---@param enabled boolean -- true to enable or false to disable
function gcs:enable_high_latency_connections(enabled) end

-- get the the current state of high latency control
---@return boolean
function gcs:get_high_latency_status() end

-- send text with severity level
---@param severity integer
---| '0' # Emergency: System is unusable. This is a "panic" condition.
---| '1' # Alert: Action should be taken immediately. Indicates error in non-critical systems.
---| '2' # Critical: Action must be taken immediately. Indicates failure in a primary system.
---| '3' # Error: Indicates an error in secondary/redundant systems.
---| '4' # Warning: Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
---| '5' # Notice: An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
---| '6' # Info: Normal operational messages. Useful for logging. No action is required for these messages.
---| '7' # Debug: Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
---@param text string
function gcs:send_text(severity, text) end

-- Return the system time when a gcs with id of SYSID_MYGCS was last seen
---@return uint32_t_ud -- system time in milliseconds
function gcs:last_seen() end

-- call a MAVLink MAV_CMD_xxx command via command_int interface
---@param command integer -- MAV_CMD_xxx
---@param params table -- parameters of p1, p2, p3, p4, x, y and z and frame. Any not specified taken as zero
---@return integer -- MAV_RESULT
function gcs:run_command_int(command, params) end

-- The relay library provides access to controlling relay outputs.
relay = {}

-- Toggles the requested relay from on to off or from off to on.
---@param instance integer -- relay instance
function relay:toggle(instance) end

-- Returns true if the requested relay is enabled.
---@param instance integer -- relay instance
---@return boolean
function relay:enabled(instance) end

-- return state of a relay
---@param instance integer -- relay instance
---@return integer -- relay state
function relay:get(instance) end

-- Turns the requested relay off.
---@param instance integer -- relay instance
function relay:off(instance) end

-- Turns the requested relay on.
---@param instance integer -- relay instance
function relay:on(instance) end


-- The terrain library provides access to checking heights against a terrain database.
terrain = {}
terrain.TerrainStatusOK = enum_integer
terrain.TerrainStatusUnhealthy = enum_integer
terrain.TerrainStatusDisabled = enum_integer

-- Returns the height (in meters) that the vehicle is currently above the terrain, or returns nil if that is not available.
-- If extrapolate is true then allow return of an extrapolated terrain altitude based on the last available data
---@param extrapolate boolean
---@return number|nil -- height above terrain in meters if available
function terrain:height_above_terrain(extrapolate) end

-- find difference between home terrain height and the terrain height at the current location in meters. A positive result means the terrain is higher than home.
-- return false is terrain at the current location or at home location is not available
-- If extrapolate is true then allow return of an extrapolated terrain altitude based on the last available data
---@param extrapolate boolean
---@return number|nil -- height difference in meters if available
function terrain:height_terrain_difference_home(extrapolate) end

--  Returns the terrain height (in meters) above mean sea level at the provided Location userdata, or returns nil if that is not available.
---@param loc Location_ud -- location at which to lookup terrain
---@param corrected boolean -- if true the terrain altitude should be correced based on the diffrence bettween the database and measured altitude at home
---@return number|nil -- amsl altitude of terrain at given locaiton in meters
function terrain:height_amsl(loc, corrected) end

-- Returns the current status of the terrain. Compare this to one of the terrain statuses (terrain.TerrainStatusDisabled, terrain.TerrainStatusUnhealthy, terrain.TerrainStatusOK).
---@return integer -- terrain status
function terrain:status() end

-- Returns true if terrain is enabled.
---@return boolean
function terrain:enabled() end


-- RangeFinder state structure
---@class (exact) RangeFinder_State_ud
local RangeFinder_State_ud = {}

---@return RangeFinder_State_ud
function RangeFinder_State() end

-- get system time (ms) of last successful update from sensor
---@return uint32_t_ud
function RangeFinder_State_ud:last_reading() end

-- set system time (ms) of last successful update from sensor
---@param value uint32_t_ud|integer|number
function RangeFinder_State_ud:last_reading(value) end

-- get sensor status
---@return integer
function RangeFinder_State_ud:status() end

-- set sensor status
---@param value integer
function RangeFinder_State_ud:status(value) end

-- get number of consecutive valid readings (max out at 10)
---@return integer
function RangeFinder_State_ud:range_valid_count() end

-- set number of consecutive valid readings (max out at 10)
---@param value integer
function RangeFinder_State_ud:range_valid_count(value) end

-- get distance in meters
---@return number
function RangeFinder_State_ud:distance() end

-- set distance in meters
---@param value number
function RangeFinder_State_ud:distance(value) end

-- get measurement quality in percent 0-100, -1 -> quality is unknown
---@return integer
function RangeFinder_State_ud:signal_quality() end

-- set measurement quality in percent 0-100, -1 -> quality is unknown
---@param value integer
function RangeFinder_State_ud:signal_quality(value) end

-- get voltage in millivolts, if applicable, otherwise 0
---@return integer
function RangeFinder_State_ud:voltage() end

-- set voltage in millivolts, if applicable, otherwise 0
---@param value integer
function RangeFinder_State_ud:voltage(value) end


-- RangeFinder backend
---@class (exact) AP_RangeFinder_Backend_ud
local AP_RangeFinder_Backend_ud = {}

-- Send range finder measurement to lua rangefinder backend. Returns false if failed
---@param state RangeFinder_State_ud|number
---@return boolean
function AP_RangeFinder_Backend_ud:handle_script_msg(state) end

-- Status of this rangefinder instance
---@return integer
function AP_RangeFinder_Backend_ud:status() end

-- Type of rangefinder of this instance
---@return integer
function AP_RangeFinder_Backend_ud:type() end

-- Orintation of the rangefinder of this instance
---@return integer
function AP_RangeFinder_Backend_ud:orientation() end

-- Current distance of the sensor instance
---@return number
function AP_RangeFinder_Backend_ud:distance() end

-- Current distance measurement signal_quality of the sensor instance
---@return number
function AP_RangeFinder_Backend_ud:signal_quality() end

-- State of most recent range finder measurment
---@return RangeFinder_State_ud
function AP_RangeFinder_Backend_ud:get_state() end


-- desc
rangefinder = {}

-- get backend based on rangefinder instance provided
---@param rangefinder_instance integer
---@return AP_RangeFinder_Backend_ud|nil
function rangefinder:get_backend(rangefinder_instance) end

-- desc
---@param orientation integer
---@return Vector3f_ud
function rangefinder:get_pos_offset_orient(orientation) end

-- desc
---@param orientation integer
---@return boolean
function rangefinder:has_data_orient(orientation) end

-- desc
---@param orientation integer
---@return integer
function rangefinder:status_orient(orientation) end

-- desc
---@param orientation integer
---@return integer
function rangefinder:ground_clearance_cm_orient(orientation) end

-- desc
---@param orientation integer
---@return integer
function rangefinder:min_distance_cm_orient(orientation) end

-- desc
---@param orientation integer
---@return integer
function rangefinder:max_distance_cm_orient(orientation) end

-- desc
---@param orientation integer
---@return integer
function rangefinder:distance_cm_orient(orientation) end

-- Current distance measurement signal quality for range finder at this orientation
---@param orientation integer
---@return integer
function rangefinder:signal_quality_pct_orient(orientation) end

-- desc
---@param orientation integer
---@return boolean
function rangefinder:has_orientation(orientation) end

-- desc
---@return integer
function rangefinder:num_sensors() end

-- Proximity backend methods
---@class (exact) AP_Proximity_Backend_ud
local AP_Proximity_Backend_ud = {}

-- Push virtual proximity boundary into actual boundary
---@return boolean
function AP_Proximity_Backend_ud:update_virtual_boundary() end

-- Set sensor min and max. Only need to do it once
---@param min number
---@param max number
---@return boolean
function AP_Proximity_Backend_ud:set_distance_min_max(min, max) end

-- type of backend
---@return integer
function AP_Proximity_Backend_ud:type() end

-- send 3d object as 3d vector
---@param vector_3d Vector3f_ud
---@param update_boundary boolean
---@return boolean
function AP_Proximity_Backend_ud:handle_script_3d_msg(vector_3d, update_boundary) end

-- send 3d object as angles
---@param dist_m number
---@param yaw_deg number
---@param pitch_deg number
---@param update_boundary boolean
---@return boolean
function AP_Proximity_Backend_ud:handle_script_distance_msg(dist_m, yaw_deg, pitch_deg, update_boundary) end

-- desc
proximity = {}

-- get backend based on proximity instance provided
---@param instance integer
---@return AP_Proximity_Backend_ud|nil
function proximity:get_backend(instance) end

-- desc
---@param object_number integer
---@return number|nil
---@return number|nil
function proximity:get_object_angle_and_distance(object_number) end

-- desc
---@return number|nil
---@return number|nil
function proximity:get_closest_object() end

-- desc
---@return integer
function proximity:get_object_count() end

-- desc
---@return integer
function proximity:num_sensors() end

-- desc
---@return integer
function proximity:get_status() end


-- desc
notify = {}

-- desc
---@param red integer
---@param green integer
---@param blue integer
---@param id integer
function notify:handle_rgb_id(red, green, blue, id) end

-- desc
---@param red integer
---@param green integer
---@param blue integer
---@param rate_hz integer
function notify:handle_rgb(red, green, blue, rate_hz) end

-- Plays a MML tune through the buzzer on the vehicle. The tune is provided as a string.
-- An online tune tester can be found here: https://firmware.ardupilot.org/Tools/ToneTester/
---@param tune string
function notify:play_tune(tune) end

-- Display text on a notify display, text too long to fit will automatically be scrolled.
---@param text string -- upto 50 characters
---@param row integer -- row number to display on, 0 is at the top.
function notify:send_text(text, row) end

-- desc
---@param row integer
function notify:release_text(row) end

-- The GPS library provides access to information about the GPS’s on the vehicle.
gps = {}
gps.GPS_OK_FIX_3D_RTK_FIXED = enum_integer
gps.GPS_OK_FIX_3D_RTK_FLOAT = enum_integer
gps.GPS_OK_FIX_3D_DGPS = enum_integer
gps.GPS_OK_FIX_3D = enum_integer
gps.GPS_OK_FIX_2D = enum_integer
gps.NO_FIX = enum_integer
gps.NO_GPS = enum_integer

-- get unix time
---@param instance integer -- instance number
---@return uint64_t_ud -- unix time microseconds
function gps:time_epoch_usec(instance) end

-- get yaw from GPS in degrees
---@param instance integer -- instance number
---@return number|nil -- yaw in degrees
---@return number|nil -- yaw accuracy in degrees
---@return uint32_t_ud|nil -- time in milliseconds of last yaw reading
function gps:gps_yaw_deg(instance) end

--  Returns nil or the instance number of the first GPS that has not been fully configured. If all GPS’s have been configured this returns nil.
---@return integer|nil
function gps:first_unconfigured_gps() end

-- Returns a Vector3f that contains the offsets of the GPS in meters in the body frame.
---@param instance integer -- instance number
---@return Vector3f_ud -- anteena offset vector forward, right, down in meters
function gps:get_antenna_offset(instance) end

-- Returns true if the GPS instance can report the vertical velocity.
---@param instance integer -- instance number
---@return boolean -- true if vertical velocity is available
function gps:have_vertical_velocity(instance) end

-- desc
---@param instance integer -- instance number
---@return uint32_t_ud
function gps:last_message_time_ms(instance) end

-- Returns the time of the last fix in system milliseconds.
---@param instance integer -- instance number
---@return uint32_t_ud -- system time of last fix in milliseconds
function gps:last_fix_time_ms(instance) end

-- Returns the vertical dilution of precision of the GPS instance.
---@param instance integer -- instance number
---@return integer -- vdop
function gps:get_vdop(instance) end

-- Returns the horizontal dilution of precision of the GPS instance.
---@param instance integer -- instance number
---@return integer -- hdop
function gps:get_hdop(instance) end

-- Returns the number of milliseconds into the current week.
---@param instance integer -- instance number
---@return uint32_t_ud -- milliseconds of current week
function gps:time_week_ms(instance) end

-- Returns the GPS week number.
---@param instance integer -- instance number
---@return integer -- week number
function gps:time_week(instance) end

-- Returns the number of satellites that the GPS is currently tracking.
---@param instance integer -- instance number
---@return integer -- number of satellites
function gps:num_sats(instance) end

-- Returns the ground course of the vehicle in degrees. You must check the status to know if the ground course is still current.
---@param instance integer -- instance number
---@return number -- ground course in degrees
function gps:ground_course(instance) end

-- Returns the ground speed of the vehicle in meters per second. You must check the status to know if the ground speed is still current.
---@param instance integer -- instance number
---@return number -- ground speed m/s
function gps:ground_speed(instance) end

-- Returns a Vector3f that contains the velocity as observed by the GPS.
-- You must check the status to know if the velocity is still current.
---@param instance integer -- instance number
---@return Vector3f_ud -- 3D velocity in m/s, in NED format
function gps:velocity(instance) end

-- desc
---@param instance integer -- instance number
---@return number|nil
function gps:vertical_accuracy(instance) end

-- horizontal RMS accuracy estimate in m
---@param instance integer -- instance number
---@return number|nil -- accuracy in meters
function gps:horizontal_accuracy(instance) end

-- Returns nil, or the speed accuracy of the GPS in meters per second, if the information is available for the GPS instance.
---@param instance integer -- instance number
---@return number|nil -- 3D velocity RMS accuracy estimate in m/s if available
function gps:speed_accuracy(instance) end

-- eturns a Location userdata for the last GPS position. You must check the status to know if the location is still current, if it is NO_GPS, or NO_FIX then it will be returning old data.
---@param instance integer -- instance number
---@return Location_ud --gps location
function gps:location(instance) end

-- Returns the GPS fix status. Compare this to one of the GPS fix types.
-- Posible status are provided as values on the gps object. eg: gps.GPS_OK_FIX_3D
---@param instance integer -- instance number
---@return integer -- status
function gps:status(instance) end

-- Returns which GPS is currently being used as the primary GPS device.
---@return integer -- primary sensor instance
function gps:primary_sensor() end

-- Returns the number of connected GPS devices.
-- If GPS blending is turned on that will show up as the third sensor, and be reported here.
---@return integer -- number of sensors
function gps:num_sensors() end

-- desc
---@class (exact) BattMonitorScript_State_ud
local BattMonitorScript_State_ud = {}

---@return BattMonitorScript_State_ud
function BattMonitorScript_State() end

-- set field
---@param value number
function BattMonitorScript_State_ud:temperature(value) end

-- set field
---@param value number
function BattMonitorScript_State_ud:consumed_wh(value) end

-- set field
---@param value number
function BattMonitorScript_State_ud:consumed_mah(value) end

-- set field
---@param value number
function BattMonitorScript_State_ud:current_amps(value) end

-- set field
---@param value integer
function BattMonitorScript_State_ud:cycle_count(value) end

-- set array field
---@param index integer
---@param value integer
function BattMonitorScript_State_ud:cell_voltages(index, value) end

-- set field
---@param value integer
function BattMonitorScript_State_ud:capacity_remaining_pct(value) end

-- set field
---@param value integer
function BattMonitorScript_State_ud:cell_count(value) end

-- set field
---@param value number
function BattMonitorScript_State_ud:voltage(value) end

-- set field
---@param value boolean
function BattMonitorScript_State_ud:healthy(value) end

-- The battery library provides access to information about the currently connected batteries on the vehicle.
battery = {}

-- desc
---@param idx integer
---@param state BattMonitorScript_State_ud
---@return boolean
function battery:handle_scripting(idx, state) end

-- desc
---@param instance integer -- battery instance
---@param percentage number
---@return boolean
function battery:reset_remaining(instance, percentage) end

-- Returns cycle count of the battery or nil if not available.
---@param instance integer -- battery instance
---@return integer|nil -- cycle count if available
function battery:get_cycle_count(instance) end

-- Returns the temperature of the battery in degrees Celsius if the battery supports temperature monitoring.
---@param instance integer -- battery instance
---@return number|nil -- temperature if available
function battery:get_temperature(instance) end

-- returns true if too much power is being drawn from the battery being monitored.
---@param instance integer -- battery instance
---@return boolean -- true if in overpower condition
function battery:overpower_detected(instance) end

-- Returns true if any of the batteries being monitored have triggered a failsafe.
---@return boolean -- true if any battery has failsafed
function battery:has_failsafed() end

-- Returns the full pack capacity (in milliamp hours) from the battery.
---@param instance integer -- battery instance
---@return integer -- capacity in milliamp hours
function battery:pack_capacity_mah(instance) end

-- Returns the remaining percentage of battery (from 0 to 100), or nil if energy monitoring is not available.
---@param instance integer -- battery instance
---@return integer|nil -- remaining capacity as a percentage of total capacity if available
function battery:capacity_remaining_pct(instance) end

-- Returns the used watt hours from the battery, or nil if energy monitoring is not available.
---@param instance integer -- battery instance
---@return number|nil -- consumed energy in watt hours if available
function battery:consumed_wh(instance) end

-- Returns the capacity (in milliamp hours) used from the battery, or nil if current monitoring is not available.
---@param instance integer -- battery instance
---@return number|nil -- consumed capacity in milliamp hours
function battery:consumed_mah(instance) end

-- Returns the current (in Amps) that is currently being consumed by the battery, or nil if current monitoring is not available.
---@param instance integer -- battery instance
---@return number|nil -- current in amps if available
function battery:current_amps(instance) end

-- Returns the estimated battery voltage if it was not under load.
---@param instance integer -- battery instance
---@return number -- resting voltage
function battery:voltage_resting_estimate(instance) end

-- Returns the estimated internal battery resistance in Ohms
---@param instance integer -- battery instance
---@return number -- estimated internal resistance in Ohms
function battery:get_resistance(instance) end

-- Returns the voltage of the selected battery instance.
---@param instance integer -- battery instance
---@return number -- voltage
function battery:voltage(instance) end

-- Returns true if the requested battery instance is healthy. Healthy is considered to be ArduPilot is currently able to monitor the battery.
---@param instance integer -- battery instance
---@return boolean
function battery:healthy(instance) end

-- Returns the number of battery instances currently available.
---@return integer -- number of instances
function battery:num_instances() end

-- get individual cell voltage
---@param instance integer
---@param cell integer
---@return number|nil
function battery:get_cell_voltage(instance, cell) end


-- The Arming library provides access to arming status and commands.
arming = {}

-- desc
---@param auth_id integer
---@param fail_msg string
function arming:set_aux_auth_failed(auth_id, fail_msg) end

-- desc
---@param auth_id integer
function arming:set_aux_auth_passed(auth_id) end

-- desc
---@return integer|nil
function arming:get_aux_auth_id() end

-- Attempts to arm the vehicle. Returns true if successful.
---@return boolean -- true if armed successfully
function arming:arm() end

-- force arm the vehicle
---@return boolean -- true if armed
function arming:arm_force() end

-- Returns a true if vehicle is currently armed.
---@return boolean -- true if armed
function arming:is_armed() end

-- desc
---@return boolean
function arming:pre_arm_checks() end

-- Disarms the vehicle in all cases. Returns false only if already disarmed.
---@return boolean -- true if disarmed successfully, false if already disarmed.
function arming:disarm() end


-- The ahrs library represents the Attitude Heading Reference System computed by the autopilot. 
-- It provides estimates for the vehicles attitude, and position.
ahrs = {}

-- desc
---@return Quaternion_ud|nil
function ahrs:get_quaternion() end

-- desc
---@return integer
function ahrs:get_posvelyaw_source_set() end

-- desc
---@return boolean
function ahrs:initialised() end

-- desc
---@param loc Location_ud
---@return boolean
function ahrs:set_origin(loc) end

-- desc
---@return Location_ud|nil
function ahrs:get_origin() end

-- desc
---@param loc Location_ud
---@return boolean
function ahrs:set_home(loc) end

-- desc
---@param source integer
---@return Vector3f_ud|nil
---@return Vector3f_ud|nil
function ahrs:get_vel_innovations_and_variances_for_source(source) end

-- desc
---@param source_set_idx integer
---| '0' # PRIMARY
---| '1' # SECONDARY
---| '2' # TERTIARY
function ahrs:set_posvelyaw_source_set(source_set_idx) end

-- desc
---@return number|nil
---@return number|nil
---@return number|nil
---@return Vector3f_ud|nil
---@return number|nil
function ahrs:get_variances() end

-- desc
---@return number
function ahrs:get_EAS2TAS() end

-- desc
---@param vector Vector3f_ud
---@return Vector3f_ud
function ahrs:body_to_earth(vector) end

-- desc
---@param vector Vector3f_ud
---@return Vector3f_ud
function ahrs:earth_to_body(vector) end

-- desc
---@return Vector3f_ud
function ahrs:get_vibration() end

-- Return the estimated airspeed of the vehicle if available
---@return number|nil -- airspeed in meters / second if available
function ahrs:airspeed_estimate() end

-- desc
---@return boolean
function ahrs:healthy() end

-- Returns a true if home position has been set.
---@return boolean -- true if home position has been set
function ahrs:home_is_set() end

-- desc
---@return number
function ahrs:get_relative_position_D_home() end

-- desc
---@return Vector3f_ud|nil
function ahrs:get_relative_position_NED_origin() end

-- desc
---@return Vector3f_ud|nil
function ahrs:get_relative_position_NED_home() end

-- Returns nil, or a Vector3f containing the current NED vehicle velocity in meters/second in north, east, and down components.
---@return Vector3f_ud|nil -- North, east, down velcoity in meters / second if available
function ahrs:get_velocity_NED() end

-- Get current groundspeed vector in meter / second
---@return Vector2f_ud -- ground speed vector, North East, meters / second
function ahrs:groundspeed_vector() end

-- Returns a Vector3f containing the current wind estimate for the vehicle.
---@return Vector3f_ud -- wind estiamte North, East, Down meters / second
function ahrs:wind_estimate() end

-- Determine how aligned heading_deg is with the wind. Return result
-- is 1.0 when perfectly aligned heading into wind, -1 when perfectly
-- aligned with-wind, and zero when perfect cross-wind. There is no
-- distinction between a left or right cross-wind. Wind speed is ignored
---@param heading_deg number
---@return number
function ahrs:wind_alignment(heading_deg) end

-- Forward head-wind component in m/s. Negative means tail-wind
---@return number
function ahrs:head_wind() end

--  Returns nil, or the latest altitude estimate above ground level in meters
---@return number|nil -- height above ground level in meters
function ahrs:get_hagl() end

-- desc
---@return Vector3f_ud
function ahrs:get_accel() end

-- Returns a Vector3f containing the current smoothed and filtered gyro rates (in radians/second)
---@return Vector3f_ud -- roll, pitch, yaw gyro rates in radians / second
function ahrs:get_gyro() end

-- Returns a Location that contains the vehicles current home waypoint.
---@return Location_ud -- home location
function ahrs:get_home() end

-- Returns nil or Location userdata that contains the vehicles current position.
-- Note: This will only return a Location if the system considers the current estimate to be reasonable.
---@return Location_ud|nil -- current location if available
function ahrs:get_location() end

-- same as `get_location` will be removed
---@return Location_ud|nil
function ahrs:get_position() end

-- Returns the current vehicle euler yaw angle in radians.
---@return number -- yaw angle in radians.
function ahrs:get_yaw() end

-- Returns the current vehicle euler pitch angle in radians.
---@return number -- pitch angle in radians.
function ahrs:get_pitch() end

-- Returns the current vehicle euler roll angle in radians.
---@return number -- roll angle in radians
function ahrs:get_roll() end

-- desc
AC_AttitudeControl = {}

-- return slew rates for VTOL controller
---@return number -- roll slew rate
---@return number -- pitch slew rate
---@return number -- yaw slew rate
function AC_AttitudeControl:get_rpy_srate() end

-- desc
AR_AttitudeControl = {}

-- return attitude controller slew rates for rovers
---@return number -- steering slew rate
---@return number -- spees slew rate
function AR_AttitudeControl:get_srate() end

-- desc
AR_PosControl = {}

-- return position controller slew rates for rovers
---@return number -- velocity slew rate
function AR_PosControl:get_srate() end

-- precision landing access
precland = {}

-- get Location of target or nil if target not acquired
---@return Location_ud|nil
function precland:get_target_location() end

-- get NE velocity of target or nil if not available
---@return Vector2f_ud|nil
function precland:get_target_velocity() end

-- get the time of the last valid target
---@return uint32_t_ud
function precland:get_last_valid_target_ms() end

-- return true if target is acquired
---@return boolean
function precland:target_acquired() end

-- return true if precland system is healthy
---@return boolean
function precland:healthy() end

-- desc
follow = {}

-- desc
---@return number|nil
function follow:get_target_heading_deg() end

-- desc
---@return Location_ud|nil
---@return Vector3f_ud|nil
function follow:get_target_location_and_velocity_ofs() end

-- desc
---@return Location_ud|nil
---@return Vector3f_ud|nil
function follow:get_target_location_and_velocity() end

-- desc
---@return uint32_t_ud
function follow:get_last_update_ms() end

-- desc
---@return boolean
function follow:have_target() end

-- desc
scripting = {}

-- desc
function scripting:restart_all() end

-- desc
---@param directoryname string
---@return table|nil -- table of filenames
---@return string|nil -- error string if fails
function dirlist(directoryname) end

--desc
---@param filename string
---@return boolean|nil -- true on success
---@return nil|string -- error string
---@return integer -- error number
function remove(filename) end

-- desc
mavlink = {}

-- initializes mavlink
---@param num_rx_msgid uint32_t_ud|integer|number
---@param msg_queue_length uint32_t_ud|integer|number
function mavlink:init(num_rx_msgid, msg_queue_length) end

-- marks mavlink message for receive, message id can be get using mavlink_msgs.get_msgid("MSG_NAME")
---@param msg_id number
---@return boolean -- false if id has been registered already
function mavlink:register_rx_msgid(msg_id) end

-- receives mavlink message marked for receive using mavlink:register_rx_msgid
---@return string -- bytes
---@return number -- mavlink channel
---@return uint32_t_ud -- receive_timestamp
function mavlink:receive_chan() end

-- sends mavlink message, to use this function the call should be like this:
-- mavlink:send(chan, mavlink_msgs.encode("MSG_NAME", {param1 = value1, param2 = value2, ...}})
---@param chan integer
---@param msgid integer
---@param message string
---@return boolean -- success
function mavlink:send_chan(chan, msgid, message) end

-- Block a given MAV_CMD from being processed by ArduPilot
---@param comand_id integer
---@return boolean
function mavlink:block_command(comand_id) end

-- Geofence library
fence = {}

-- Returns the time at which the current breach started
---@return uint32_t_ud system_time milliseconds
function fence:get_breach_time() end

-- Returns the type bitmask of any breached fences
---@return integer fence_type bitmask
---| 1 # Maximim altitude
---| 2 # Circle
---| 4 # Polygon
---| 8 # Minimum altitude
function fence:get_breaches() end

-- desc
---@class (exact) stat_t_ud
local stat_t_ud = {}

---@return stat_t_ud
function stat_t() end

-- get creation time in seconds
---@return uint32_t_ud
function stat_t_ud:ctime() end

-- get last access time in seconds
---@return uint32_t_ud
function stat_t_ud:atime() end

-- get last modification time in seconds
---@return uint32_t_ud
function stat_t_ud:mtime() end

-- get file mode
---@return integer
function stat_t_ud:mode() end

-- get file size in bytes
---@return uint32_t_ud
function stat_t_ud:size() end

-- return true if this is a directory
---@return boolean
function stat_t_ud:is_directory() end

-- desc
rtc = {}

-- return a time since 1970 in seconds from GMT date elements
---@param year integer -- 20xx
---@param month integer -- 0-11
---@param day integer -- 1-31
---@param hour integer -- 0-23
---@param min integer -- 0-60
---@param sec integer -- 0-60
---@return uint32_t_ud
function rtc:date_fields_to_clock_s(year, month, day, hour, min, sec) end

-- break a time in seconds since 1970 to GMT date elements
---@param param1 uint32_t_ud|integer|number
---@return integer|nil -- year 20xx
---@return integer|nil -- month 0-11
---@return integer|nil -- day 1-31
---@return integer|nil -- hour 0-23
---@return integer|nil -- min 0-60
---@return integer|nil -- sec 0-60
---@return integer|nil -- weekday 0-6, sunday is 0
function rtc:clock_s_to_date_fields(param1) end

-- desc
fs = {}

-- desc
---@param param1 string
---@return stat_t_ud|nil
function fs:stat(param1) end

-- Format the SD card. This is a async operation, use get_format_status to get the status of the format
---@return boolean
function fs:format() end

-- Get the current status of a format. 0=NOT_STARTED, 1=PENDING, 2=IN_PROGRESS, 3=SUCCESS, 4=FAILURE
---@return integer
function fs:get_format_status() end

-- Get crc32 checksum of a file with given name
---@param file_name string
---@return uint32_t_ud|nil
function fs:crc32(file_name) end

-- desc
networking = {}

-- conver uint32_t address to string
---@param ip4addr uint32_t_ud|integer|number
---@return string
function networking:address_to_str(ip4addr) end

-- desc
---@return uint32_t_ud
function networking:get_gateway_active() end

-- desc
---@return uint32_t_ud
function networking:get_netmask_active() end

-- desc
---@return uint32_t_ud
function networking:get_ip_active() end

-- visual odometry object
--@class visual_odom
visual_odom = {}

-- visual odometry health
---@return boolean
function visual_odom:healthy() end

-- visual odometry quality as a percentage from 1 to 100 or 0 if unknown
---@return integer
function visual_odom:quality() end

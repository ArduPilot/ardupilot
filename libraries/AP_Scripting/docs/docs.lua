-- ArduPilot lua scripting documentation in EmmyLua Annotations
-- This file should be auto generated and then manual edited
-- generate with --scripting-docs, eg  ./waf copter --scripting-docs
-- see: https://github.com/sumneko/lua-language-server/wiki/EmmyLua-Annotations

-- set and get for field types share function names
---@diagnostic disable: duplicate-set-field

-- manual bindings

---@class uint32_t_ud
local uint32_t_ud = {}

-- create uint32_t_ud with optional value
---@param value? number|integer
---@return uint32_t_ud
function uint32_t(value) end

-- Convert to number
---@return number
function uint32_t_ud:tofloat() end

-- Convert to integer
---@return integer
function uint32_t_ud:toint() end

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


-- data flash logging to SD card
---@class logger
logger = {}

-- write value to data flash log with given types and names, optional units and multipliers, timestamp will be automatically added
---@param name string -- up to 4 characters
---@param labels string -- comma separated value labels, up to 58 characters
---@param format string -- type format string, see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/README.md
---@param units? string -- optional units string
---@param multipliers? string -- optional multipliers string
---@param data1 integer|number|uint32_t_ud|string -- data to be logged, type to match format string
function logger:write(name, labels, format, units, multipliers, data1, ...) end


-- i2c bus interaction
---@class i2c
i2c = {}

-- get a i2c device handler
---@param bus integer -- bus number
---@param address integer -- device address 0 to 128
---@param clock? uint32_t_ud -- optional bus clock, default 400000
---@param smbus? boolean -- optional sumbus flag, default false
---@return AP_HAL__I2CDevice_ud
function i2c:get_device(bus, address, clock, smbus) end

-- EFI state structure
---@class EFI_State_ud
local EFI_State_ud = {}

---@return EFI_State_ud
function EFI_State() end

-- set field
---@param value Cylinder_Status_ud
function EFI_State_ud:cylinder_status(value) end

-- set field
---@param value integer
function EFI_State_ud:ecu_index(value) end

-- set field
---@param value integer
function EFI_State_ud:throttle_position_percent(value) end

-- set field
---@param value number
function EFI_State_ud:estimated_consumed_fuel_volume_cm3(value) end

-- set field
---@param value number
function EFI_State_ud:fuel_consumption_rate_cm3pm(value) end

-- set field
---@param value number
function EFI_State_ud:fuel_pressure(value) end

-- set field
---@param value number
function EFI_State_ud:oil_temperature(value) end

-- set field
---@param value number
function EFI_State_ud:oil_pressure(value) end

-- set field
---@param value number
function EFI_State_ud:coolant_temperature(value) end

-- set field
---@param value number
function EFI_State_ud:intake_manifold_temperature(value) end

-- set field
---@param value number
function EFI_State_ud:intake_manifold_pressure_kpa(value) end

-- set field
---@param value number
function EFI_State_ud:atmospheric_pressure_kpa(value) end

-- set field
---@param value number
function EFI_State_ud:spark_dwell_time_ms(value) end

-- set field
---@param value uint32_t_ud
function EFI_State_ud:engine_speed_rpm(value) end

-- set field
---@param value integer
function EFI_State_ud:engine_load_percent(value) end

-- set field
---@param value boolean
function EFI_State_ud:general_error(value) end

-- set field
---@param value uint32_t_ud
function EFI_State_ud:last_updated_ms(value) end


-- EFI Cylinder_Status structure
---@class Cylinder_Status_ud
local Cylinder_Status_ud = {}

---@return Cylinder_Status_ud
function Cylinder_Status() end

-- set field
---@param value number
function Cylinder_Status_ud:lambda_coefficient(value) end

-- set field
---@param value number
function Cylinder_Status_ud:exhaust_gas_temperature(value) end

-- set field
---@param value number
function Cylinder_Status_ud:cylinder_head_temperature(value) end

-- set field
---@param value number
function Cylinder_Status_ud:injection_time_ms(value) end

-- set field
---@param value number
function Cylinder_Status_ud:ignition_timing_deg(value) end

-- desc
---@class efi
efi = {}

-- EFI handle scripting update
---@param efi_state EFI_State_ud
function efi:handle_scripting(efi_state) end


-- CAN bus interaction
---@class CAN
CAN = {}

-- get a CAN bus device handler
---@param buffer_len uint32_t_ud -- buffer length 1 to 25
---@return ScriptingCANBuffer_ud
function CAN:get_device(buffer_len) end


-- Auto generated binding

-- desc
---@class CANFrame_ud
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
---@param value uint32_t_ud
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
---@class motor_factor_table_ud
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


-- desc
---@class PWMSource_ud
local PWMSource_ud = {}

---@return PWMSource_ud
function PWMSource() end

-- desc
---@return integer
function PWMSource_ud:get_pwm_avg_us() end

-- desc
---@return integer
function PWMSource_ud:get_pwm_us() end

-- desc
---@param pin_number integer
---@return boolean
function PWMSource_ud:set_pin(pin_number) end


-- desc
---@class mavlink_mission_item_int_t_ud
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


-- desc
---@class Parameter_ud
local Parameter_ud = {}

---@return Parameter_ud
function Parameter() end

-- desc
---@param value number
---@return boolean
function Parameter_ud:set_default(value) end

-- desc
---@return boolean
function Parameter_ud:configured() end

-- desc
---@param value number
---@return boolean
function Parameter_ud:set_and_save(value) end

-- desc
---@param value number
---@return boolean
function Parameter_ud:set(value) end

-- desc
---@return number|nil
function Parameter_ud:get() end

-- desc
---@param key integer
---@param group_element uint32_t_ud
---@param type integer
---| '1' # AP_PARAM_INT8
---| '2' # AP_PARAM_INT16
---| '3' # AP_PARAM_INT32
---| '4' # AP_PARAM_FLOAT
---@return boolean
function Parameter_ud:init_by_info(key, group_element, type) end

-- desc
---@param name string
---@return boolean
function Parameter_ud:init(name) end


-- desc
---@class Vector2f_ud
local Vector2f_ud = {}

---@return Vector2f_ud
function Vector2f() end

-- copy
---@return Vector2f_ud
function Vector2f_ud:copy() end

-- get field
---@return number
function Vector2f_ud:y() end

-- set field
---@param value number
function Vector2f_ud:y(value) end

-- get field
---@return number
function Vector2f_ud:x() end

-- set field
---@param value number
function Vector2f_ud:x(value) end

-- desc
---@param angle_rad number
function Vector2f_ud:rotate(angle_rad) end

-- desc
---@return boolean
function Vector2f_ud:is_zero() end

-- desc
---@return boolean
function Vector2f_ud:is_inf() end

-- desc
---@return boolean
function Vector2f_ud:is_nan() end

-- desc
function Vector2f_ud:normalize() end

-- desc
---@return number
function Vector2f_ud:length() end

-- desc
---@return number
function Vector2f_ud:angle() end

-- desc
---@class Vector3f_ud
local Vector3f_ud = {}

---@return Vector3f_ud
function Vector3f() end

-- copy
---@return Vector3f_ud
function Vector3f_ud:copy() end

-- get field
---@return number
function Vector3f_ud:z() end

-- set field
---@param value number
function Vector3f_ud:z(value) end

-- get field
---@return number
function Vector3f_ud:y() end

-- set field
---@param value number
function Vector3f_ud:y(value) end

-- get field
---@return number
function Vector3f_ud:x() end

-- set field
---@param value number
function Vector3f_ud:x(value) end

-- desc
---@param scale_factor number
---@return Vector3f_ud
function Vector3f_ud:scale(scale_factor) end

-- desc
---@param vector Vector3f_ud
---@return Vector3f_ud
function Vector3f_ud:cross(vector) end

-- desc
---@param vector Vector3f_ud
---@return number
function Vector3f_ud:dot(vector) end

-- desc
---@return boolean
function Vector3f_ud:is_zero() end

-- desc
---@return boolean
function Vector3f_ud:is_inf() end

-- desc
---@return boolean
function Vector3f_ud:is_nan() end

-- desc
function Vector3f_ud:normalize() end

-- desc
---@return number
function Vector3f_ud:length() end

-- desc
---@param param1 number -- XY rotation in radians
function Vector3f_ud:rotate_xy(param1) end

-- desc
---@return Vector2f_ud
function Vector3f_ud:xy() end


-- desc
---@class Location_ud
local Location_ud = {}

---@return Location_ud
function Location() end

-- copy
---@return Location_ud
function Location_ud:copy() end

-- get field
---@return boolean
function Location_ud:loiter_xtrack() end

-- set field
---@param value boolean
function Location_ud:loiter_xtrack(value) end

-- get field
---@return boolean
function Location_ud:origin_alt() end

-- set field
---@param value boolean
function Location_ud:origin_alt(value) end

-- get field
---@return boolean
function Location_ud:terrain_alt() end

-- set field
---@param value boolean
function Location_ud:terrain_alt(value) end

-- get field
---@return boolean
function Location_ud:relative_alt() end

-- set field
---@param value boolean
function Location_ud:relative_alt(value) end

-- get field
---@return integer
function Location_ud:alt() end

-- set field
---@param value integer
function Location_ud:alt(value) end

-- get field
---@return integer
function Location_ud:lng() end

-- set field
---@param value integer
function Location_ud:lng(value) end

-- get field
---@return integer
function Location_ud:lat() end

-- set field
---@param value integer
function Location_ud:lat(value) end

-- desc
---@param desired_frame integer
---| '0' # ABSOLUTE
---| '1' # ABOVE_HOME
---| '2' # ABOVE_ORIGIN
---| '3' # ABOVE_TERRAIN
---@return boolean
function Location_ud:change_alt_frame(desired_frame) end

-- desc
---@param loc Location_ud
---@return Vector2f_ud
function Location_ud:get_distance_NE(loc) end

-- desc
---@param loc Location_ud
---@return Vector3f_ud
function Location_ud:get_distance_NED(loc) end

-- desc
---@param loc Location_ud
---@return number
function Location_ud:get_bearing(loc) end

-- desc
---@return Vector3f_ud|nil
function Location_ud:get_vector_from_origin_NEU() end

-- desc
---@param bearing_deg number
---@param distance number
function Location_ud:offset_bearing(bearing_deg, distance) end

-- desc
---@param ofs_north number
---@param ofs_east number
function Location_ud:offset(ofs_north, ofs_east) end

-- desc
---@param loc Location_ud
---@return number
function Location_ud:get_distance(loc) end


-- desc
---@class ScriptingCANBuffer_ud
local ScriptingCANBuffer_ud = {}

-- desc
---@return CANFrame_ud|nil
function ScriptingCANBuffer_ud:read_frame() end

-- desc
---@param frame CANFrame_ud
---@param timeout_us uint32_t_ud
---@return boolean
function ScriptingCANBuffer_ud:write_frame(frame, timeout_us) end


-- desc
---@class AP_HAL__AnalogSource_ud
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
---@class AP_HAL__I2CDevice_ud
local AP_HAL__I2CDevice_ud = {}

-- desc
---@param address integer
function AP_HAL__I2CDevice_ud:set_address(address) end

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


-- desc
---@class AP_HAL__UARTDriver_ud
local AP_HAL__UARTDriver_ud = {}

-- desc
---@param flow_control_setting integer
---| '0' # disabled
---| '1' # enabled
---| '2' # auto
function AP_HAL__UARTDriver_ud:set_flow_control(flow_control_setting) end

-- desc
---@return uint32_t_ud
function AP_HAL__UARTDriver_ud:available() end

-- desc
---@param value integer
---@return uint32_t_ud
function AP_HAL__UARTDriver_ud:write(value) end

-- desc
---@return integer
function AP_HAL__UARTDriver_ud:read() end

-- desc
---@param baud_rate uint32_t_ud
function AP_HAL__UARTDriver_ud:begin(baud_rate) end


-- desc
---@class RC_Channel_ud
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
---@class mount
mount = {}

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
---@class motors
motors = {}

-- desc
---@param param1 string
function motors:set_frame_string(param1) end


-- desc
---@class FWVersion
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

-- get field
---@return integer
function FWVersion:type() end

-- get field
---@return string
function FWVersion:string() end


-- desc
---@class periph
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
---@class ins
ins = {}

-- desc
---@param instance integer
---@return number
function ins:get_temperature(instance) end


-- desc
---@class Motors_dynamic
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
---@class analog
analog = {}

-- desc
---@return AP_HAL__AnalogSource_ud
function analog:channel() end


-- Control of general purpose input/output pins
---@class gpio
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
---@class Motors_6DoF
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
---@class attitude_control
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
---@class frsky_sport
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
---@class MotorsMatrix
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


-- desc
---@class quadplane
quadplane = {}

-- desc
---@return boolean
function quadplane:in_assisted_flight() end

-- desc
---@return boolean
function quadplane:in_vtol_mode() end


-- desc
---@class LED
LED = {}

-- desc
---@return integer
---@return integer
---@return integer
function LED:get_rgb() end


-- desc
---@class button
button = {}

-- desc
---@param button_number integer
---@return boolean
function button:get_button_state(button_number) end


-- desc
---@class RPM
RPM = {}

-- desc
---@param instance integer
---@return number|nil
function RPM:get_rpm(instance) end


-- desc
---@class mission
---@field MISSION_COMPLETE number
---@field MISSION_RUNNING number
---@field MISSION_STOPPED number
mission = {}

-- desc
---@return boolean
function mission:clear() end

-- desc
---@param index integer
---@param item mavlink_mission_item_int_t_ud
---@return boolean
function mission:set_item(index, item) end

-- desc
---@param index integer
---@return mavlink_mission_item_int_t_ud|nil
function mission:get_item(index) end

-- desc
---@return integer
function mission:num_commands() end

-- desc
---@return integer
function mission:get_current_do_cmd_id() end

-- desc
---@return integer
function mission:get_current_nav_id() end

-- desc
---@return integer
function mission:get_prev_nav_cmd_id() end

-- desc
---@param index integer
---@return boolean
function mission:set_current_cmd(index) end

-- desc
---@return integer
function mission:get_current_nav_index() end

-- desc
---@return integer
function mission:state() end


-- desc
---@class param
param = {}

-- desc
---@param name string
---@param value number
---@return boolean
function param:set_and_save(name, value) end

-- desc
---@param name string
---@param value number
---@return boolean
function param:set(name, value) end

-- desc
---@param name string
---@return number|nil
function param:get(name) end

-- desc
---@param name string
---@param value number
---@return boolean
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
---@class esc_telem
esc_telem = {}

-- desc
---@param instance integer
---@return uint32_t_ud|nil
function esc_telem:get_usage_seconds(instance) end

-- desc
---@param instance integer
---@return number|nil
function esc_telem:get_consumption_mah(instance) end

-- desc
---@param instance integer
---@return number|nil
function esc_telem:get_voltage(instance) end

-- desc
---@param instance integer
---@return number|nil
function esc_telem:get_current(instance) end

-- desc
---@param instance integer
---@return integer|nil
function esc_telem:get_motor_temperature(instance) end

-- desc
---@param instance integer
---@return integer|nil
function esc_telem:get_temperature(instance) end

-- desc
---@param instance integer
---@return number|nil
function esc_telem:get_rpm(instance) end

-- update RPM for an ESC
---@param param1 integer -- ESC number
---@param param2 integer -- RPM
---@param param3 number -- error rate
function esc_telem:update_rpm(esc_index, rpm, error_rate) end

-- set scale factor for RPM on a motor
---@param param1 motor index (0 is first motor)
---@param param2 scale factor
function esc_telem:set_rpm_scale(esc_index, scale_factor) end

-- desc
---@class optical_flow
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
---@class baro
baro = {}

-- desc
---@return number
function baro:get_external_temperature() end

-- desc
---@return number
function baro:get_temperature() end

-- desc
---@return number
function baro:get_pressure() end


-- desc
---@class serial
serial = {}

-- Returns the UART instance that allows connections from scripts (those with SERIALx_PROTOCOL = 28`).
-- For instance = 0, returns first such UART, second for instance = 1, and so on.
-- If such an instance is not found, returns nil.
---@param instance integer -- the 0-based index of the UART instance to return.
---@return AP_HAL__UARTDriver_ud -- the requested UART instance available for scripting, or nil if none.
function serial:find_serial(instance) end


-- desc
---@class rc
rc = {}

-- desc
---@param chan_num integer
---@return RC_Channel_ud
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
---@return RC_Channel_ud
function rc:find_channel_for_option(aux_fun) end

-- desc
---@param chan_num integer
---@return integer|nil
function rc:get_pwm(chan_num) end


-- desc
---@class SRV_Channels
SRV_Channels = {}

-- desc
---@param function_num integer
---@param range integer
function SRV_Channels:set_range(function_num, range) end

-- desc
---@param function_num integer
---@param angle integer
function SRV_Channels:set_angle(function_num, angle) end

-- desc
---@param function_num integer
---@param value number
function SRV_Channels:set_output_norm(function_num, value) end

-- desc
---@param function_num integer
---@return number
function SRV_Channels:get_output_scaled(function_num) end

-- desc
---@param function_num integer
---@return integer|nil
function SRV_Channels:get_output_pwm(function_num) end

-- desc
---@param function_num integer
---@param value number
function SRV_Channels:set_output_scaled(function_num, value) end

-- desc
---@param chan integer
---@param pwm integer
---@param timeout_ms integer
function SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, timeout_ms) end

-- desc
---@param chan integer
---@param pwm integer
function SRV_Channels:set_output_pwm_chan(chan, pwm) end

-- desc
---@param function_num integer
---@param pwm integer
function SRV_Channels:set_output_pwm(function_num, pwm) end

-- desc
---@param function_num integer
---@return integer|nil
function SRV_Channels:find_channel(function_num) end


-- desc
---@class serialLED
serialLED = {}

-- desc
---@param chan integer
function serialLED:send(chan) end

-- desc
---@param chan integer
---@param led_index integer
---@param red integer
---@param green integer
---@param blue integer
function serialLED:set_RGB(chan, led_index, red, green, blue) end

-- desc
---@param chan integer
---@param num_leds integer
---@return boolean
function serialLED:set_num_profiled(chan, num_leds) end

-- desc
---@param chan integer
---@param num_leds integer
---@return boolean
function serialLED:set_num_neopixel(chan, num_leds) end


-- desc
---@class vehicle
vehicle = {}

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

-- desc
---@param vel_ned Vector3f_ud
---@return boolean
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

-- desc
---@return Location_ud|nil
function vehicle:get_target_location() end

-- desc
---@param target_loc Location_ud
---@return boolean
function vehicle:set_target_location(target_loc) end

-- desc
---@param alt number
---@return boolean
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

-- desc
---@return uint32_t_ud
function vehicle:get_time_flying_ms() end

-- desc
---@return boolean
function vehicle:get_likely_flying() end

-- desc
---@return integer
function vehicle:get_control_mode_reason() end

-- desc
---@return integer
function vehicle:get_mode() end

-- desc
---@param mode_number integer
---@return boolean
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
---@class onvif
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
---@class gcs
gcs = {}

-- send named float value using NAMED_VALUE_FLOAT message
---@param name string -- up to 10 chars long
---@param value number -- value to send
function gcs:send_named_float(name, value) end

-- set message interval for a given serial port and message id
---@param port_num integer -- serial port number
---@param msg_id uint32_t_ud -- MAVLink message id
---@param interval_us integer -- interval in micro seconds
---@return integer
---| '0' # Accepted
---| '4' # Failed
function gcs:set_message_interval(port_num, msg_id, interval_us) end

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

-- desc
---@class relay
relay = {}

-- desc
---@param instance integer
function relay:toggle(instance) end

-- desc
---@param instance integer
---@return boolean
function relay:enabled(instance) end

-- return state of a relay
---@param instance integer
---@return uint8_t
function relay:get(instance) end

-- desc
---@param instance integer
function relay:off(instance) end

-- desc
---@param instance integer
function relay:on(instance) end


-- desc
---@class terrain
---@field TerrainStatusOK number
---@field TerrainStatusUnhealthy number
---@field TerrainStatusDisabled number
terrain = {}

-- desc
---@param extrapolate boolean
---@return number|nil
function terrain:height_above_terrain(extrapolate) end

-- desc
---@param extrapolate boolean
---@return number|nil
function terrain:height_terrain_difference_home(extrapolate) end

-- desc
---@param loc Location_ud
---@param corrected boolean
---@return number|nil
function terrain:height_amsl(loc, corrected) end

-- desc
---@return integer
function terrain:status() end

-- desc
---@return boolean
function terrain:enabled() end


-- desc
---@class rangefinder
rangefinder = {}

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

-- desc
---@param orientation integer
---@return boolean
function rangefinder:has_orientation(orientation) end

-- desc
---@return integer
function rangefinder:num_sensors() end


-- desc
---@class proximity
proximity = {}

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
---@class notify
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

-- desc
---@param tune string
function notify:play_tune(tune) end


-- desc
---@class gps
---@field GPS_OK_FIX_3D_RTK_FIXED number
---@field GPS_OK_FIX_3D_RTK_FLOAT number
---@field GPS_OK_FIX_3D_DGPS number
---@field GPS_OK_FIX_3D number
---@field GPS_OK_FIX_2D number
---@field NO_FIX number
---@field NO_GPS number
gps = {}

-- desc
---@return integer|nil
function gps:first_unconfigured_gps() end

-- desc
---@param instance integer
---@return Vector3f_ud
function gps:get_antenna_offset(instance) end

-- desc
---@param instance integer
---@return boolean
function gps:have_vertical_velocity(instance) end

-- desc
---@param instance integer
---@return uint32_t_ud
function gps:last_message_time_ms(instance) end

-- desc
---@param instance integer
---@return uint32_t_ud
function gps:last_fix_time_ms(instance) end

-- desc
---@param instance integer
---@return integer
function gps:get_vdop(instance) end

-- desc
---@param instance integer
---@return integer
function gps:get_hdop(instance) end

-- desc
---@param instance integer
---@return uint32_t_ud
function gps:time_week_ms(instance) end

-- desc
---@param instance integer
---@return integer
function gps:time_week(instance) end

-- desc
---@param instance integer
---@return integer
function gps:num_sats(instance) end

-- desc
---@param instance integer
---@return number
function gps:ground_course(instance) end

-- desc
---@param instance integer
---@return number
function gps:ground_speed(instance) end

-- desc
---@param instance integer
---@return Vector3f_ud
function gps:velocity(instance) end

-- desc
---@param instance integer
---@return number|nil
function gps:vertical_accuracy(instance) end

-- desc
---@param instance integer
---@return number|nil
function gps:horizontal_accuracy(instance) end

-- desc
---@param instance integer
---@return number|nil
function gps:speed_accuracy(instance) end

-- desc
---@param instance integer
---@return Location_ud
function gps:location(instance) end

-- desc
---@param instance integer
---@return integer
function gps:status(instance) end

-- desc
---@return integer
function gps:primary_sensor() end

-- desc
---@return integer
function gps:num_sensors() end


-- desc
---@class battery
battery = {}

-- desc
---@param instance integer
---@param percentage number
---@return boolean
function battery:reset_remaining(instance, percentage) end

-- desc
---@param instance integer
---@return integer|nil
function battery:get_cycle_count(instance) end

-- desc
---@param instance integer
---@return number|nil
function battery:get_temperature(instance) end

-- desc
---@param instance integer
---@return boolean
function battery:overpower_detected(instance) end

-- desc
---@return boolean
function battery:has_failsafed() end

-- desc
---@param instance integer
---@return integer
function battery:pack_capacity_mah(instance) end

-- desc
---@param instance integer
---@return integer|nil
function battery:capacity_remaining_pct(instance) end

-- desc
---@param instance integer
---@return number|nil
function battery:consumed_wh(instance) end

-- desc
---@param instance integer
---@return number|nil
function battery:consumed_mah(instance) end

-- desc
---@param instance integer
---@return number|nil
function battery:current_amps(instance) end

-- desc
---@param instance integer
---@return number
function battery:voltage_resting_estimate(instance) end

-- desc
---@param instance integer
---@return number
function battery:voltage(instance) end

-- desc
---@param instance integer
---@return boolean
function battery:healthy(instance) end

-- desc
---@return integer
function battery:num_instances() end


-- desc
---@class arming
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

-- desc
---@return boolean
function arming:arm() end

-- desc
---@return boolean
function arming:is_armed() end

-- desc
---@return boolean
function arming:pre_arm_checks() end

-- desc
---@return boolean
function arming:disarm() end


-- desc
---@class ahrs
ahrs = {}

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

-- desc
---@return number|nil
function ahrs:airspeed_estimate() end

-- desc
---@return boolean
function ahrs:healthy() end

-- desc
---@return boolean
function ahrs:home_is_set() end

-- desc
---@return Vector3f_ud|nil
function ahrs:get_relative_position_NED_origin() end

-- desc
---@return Vector3f_ud|nil
function ahrs:get_relative_position_NED_home() end

-- desc
---@return Vector3f_ud|nil
function ahrs:get_velocity_NED() end

-- desc
---@return Vector2f_ud
function ahrs:groundspeed_vector() end

-- desc
---@return Vector3f_ud
function ahrs:wind_estimate() end

-- desc
---@return number|nil
function ahrs:get_hagl() end

-- desc
---@return Vector3f_ud
function ahrs:get_accel() end

-- desc
---@return Vector3f_ud
function ahrs:get_gyro() end

-- desc
---@return Location_ud
function ahrs:get_home() end

-- desc
---@return Location_ud|nil
function ahrs:get_location() end

-- desc
---@return number
function ahrs:get_yaw() end

-- desc
---@return number
function ahrs:get_pitch() end

-- desc
---@return number
function ahrs:get_roll() end

-- desc
---@class AC_AttitudeControl
AC_AttitudeControl = {}

-- return slew rates for VTOL controller
---@return number -- roll slew rate
---@return number -- pitch slew rate
---@return number -- yaw slew rate
function AC_AttitudeControl:get_rpy_srate() end

-- desc
---@class follow
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

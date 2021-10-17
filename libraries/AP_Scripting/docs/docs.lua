-- ArduPilot lua scripting documentation in EmmyLua Annotations
-- This file should be auto generated and then manual edited
-- generate with --scripting-docs, eg  ./waf copter --scripting-docs
-- see: https://github.com/sumneko/lua-language-server/wiki/EmmyLua-Annotations

-- desc
---@class CANFrame
---@return CANFrame
function CANFrame() end

-- get field
---@return integer
function CANFrame:dlc() end

-- set field
---@param value integer
function CANFrame:dlc(value) end

-- get field
---@param index integer
---@return integer
function CANFrame:data(index) end

-- set field
---@param index integer
---@param value integer
function CANFrame:data(index, value) end

-- get field
---@return uint32_t
function CANFrame:id() end

-- set field
---@param value uint32_t
function CANFrame:id(value) end

-- desc
---@return boolean
function CANFrame:isErrorFrame() end

-- desc
---@return boolean
function CANFrame:isRemoteTransmissionRequest() end

-- desc
---@return boolean
function CANFrame:isExtended() end


-- desc
---@class motor_factor_table
---@return motor_factor_table
function motor_factor_table() end

-- get field
---@param index integer
---@return number
function motor_factor_table:throttle(index) end

-- get field
---@param index integer
---@return number
function motor_factor_table:yaw(index) end

-- get field
---@param index integer
---@return number
function motor_factor_table:pitch(index) end

-- get field
---@param index integer
---@return number
function motor_factor_table:roll(index) end


-- desc
---@class PWMSource
---@return PWMSource
function PWMSource() end

-- desc
---@return integer
function PWMSource:get_pwm_avg_us() end

-- desc
---@return integer
function PWMSource:get_pwm_us() end

-- desc
---@param param1 integer
---@return boolean
function PWMSource:set_pin(param1) end


-- desc
---@class mavlink_mission_item_int_t
---@return mavlink_mission_item_int_t
function mavlink_mission_item_int_t() end

-- get field
---@return integer
function mavlink_mission_item_int_t:current() end

-- set field
---@param value integer
function mavlink_mission_item_int_t:current(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t:frame() end

-- set field
---@param value integer
function mavlink_mission_item_int_t:frame(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t:command() end

-- set field
---@param value integer
function mavlink_mission_item_int_t:command(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t:seq() end

-- set field
---@param value integer
function mavlink_mission_item_int_t:seq(value) end

-- get field
---@return number
function mavlink_mission_item_int_t:z() end

-- get field
---@return integer
function mavlink_mission_item_int_t:y() end

-- set field
---@param value integer
function mavlink_mission_item_int_t:y(value) end

-- get field
---@return integer
function mavlink_mission_item_int_t:x() end

-- set field
---@param value integer
function mavlink_mission_item_int_t:x(value) end

-- get field
---@return number
function mavlink_mission_item_int_t:param4() end

-- get field
---@return number
function mavlink_mission_item_int_t:param3() end

-- get field
---@return number
function mavlink_mission_item_int_t:param2() end

-- get field
---@return number
function mavlink_mission_item_int_t:param1() end


-- desc
---@class Parameter
---@return Parameter
function Parameter() end

-- desc
---@param param1 number
---@return boolean
function Parameter:set_and_save(param1) end

-- desc
---@param param1 number
---@return boolean
function Parameter:set(param1) end

-- desc
---@return number|nil
function Parameter:get() end

-- desc
---@param param1 string
---@return boolean
function Parameter:init(param1) end


-- desc
---@class Vector2f
---@return Vector2f
function Vector2f() end

-- get field
---@return number
function Vector2f:y() end

-- get field
---@return number
function Vector2f:x() end

-- desc
---@param param1 number
function Vector2f:rotate(param1) end

-- desc
---@return boolean
function Vector2f:is_zero() end

-- desc
---@return boolean
function Vector2f:is_inf() end

-- desc
---@return boolean
function Vector2f:is_nan() end

-- desc
function Vector2f:normalize() end

-- desc
---@return number
function Vector2f:length() end


-- desc
---@class Vector3f
---@return Vector3f
function Vector3f() end

-- get field
---@return number
function Vector3f:z() end

-- get field
---@return number
function Vector3f:y() end

-- get field
---@return number
function Vector3f:x() end

-- desc
---@param param1 number
---@return Vector3f
function Vector3f:scale(param1) end

-- desc
---@param param1 Vector3f
---@return Vector3f
function Vector3f:cross(param1) end

-- desc
---@param param1 Vector3f
---@return number
function Vector3f:dot(param1) end

-- desc
---@return boolean
function Vector3f:is_zero() end

-- desc
---@return boolean
function Vector3f:is_inf() end

-- desc
---@return boolean
function Vector3f:is_nan() end

-- desc
function Vector3f:normalize() end

-- desc
---@return number
function Vector3f:length() end


-- desc
---@class Location
---@return Location
function Location() end

-- get field
---@return boolean
function Location:loiter_xtrack() end

-- set field
---@param value boolean
function Location:loiter_xtrack(value) end

-- get field
---@return boolean
function Location:origin_alt() end

-- set field
---@param value boolean
function Location:origin_alt(value) end

-- get field
---@return boolean
function Location:terrain_alt() end

-- set field
---@param value boolean
function Location:terrain_alt(value) end

-- get field
---@return boolean
function Location:relative_alt() end

-- set field
---@param value boolean
function Location:relative_alt(value) end

-- get field
---@return integer
function Location:alt() end

-- set field
---@param value integer
function Location:alt(value) end

-- get field
---@return integer
function Location:lng() end

-- set field
---@param value integer
function Location:lng(value) end

-- get field
---@return integer
function Location:lat() end

-- set field
---@param value integer
function Location:lat(value) end

-- desc
---@param param1 integer
---@return boolean
function Location:change_alt_frame(param1) end

-- desc
---@param param1 Location
---@return Vector2f
function Location:get_distance_NE(param1) end

-- desc
---@param param1 Location
---@return Vector3f
function Location:get_distance_NED(param1) end

-- desc
---@param param1 Location
---@return number
function Location:get_bearing(param1) end

-- desc
---@return Vector3f|nil
function Location:get_vector_from_origin_NEU() end

-- desc
---@param param1 number
---@param param2 number
function Location:offset_bearing(param1, param2) end

-- desc
---@param param1 number
---@param param2 number
function Location:offset(param1, param2) end

-- desc
---@param param1 Location
---@return number
function Location:get_distance(param1) end


-- desc
---@class ScriptingCANBuffer
ScriptingCANBuffer = {}

-- desc
---@return CANFrame|nil
function ScriptingCANBuffer:read_frame() end

-- desc
---@param param1 CANFrame
---@param param2 uint32_t
---@return boolean
function ScriptingCANBuffer:write_frame(param1, param2) end


-- desc
---@class AP_HAL__AnalogSource
AP_HAL__AnalogSource = {}

-- desc
---@return number
function AP_HAL__AnalogSource:voltage_average_ratiometric() end

-- desc
---@return number
function AP_HAL__AnalogSource:voltage_latest() end

-- desc
---@return number
function AP_HAL__AnalogSource:voltage_average() end

-- desc
---@param param1 integer
---@return boolean
function AP_HAL__AnalogSource:set_pin(param1) end


-- desc
---@class AP_HAL__I2CDevice
AP_HAL__I2CDevice = {}

-- desc
---@param param1 integer
function AP_HAL__I2CDevice:set_address(param1) end

-- desc
---@param param1 integer
---@return integer|nil
function AP_HAL__I2CDevice:read_registers(param1) end

-- desc
---@param param1 integer
---@param param2 integer
---@return boolean
function AP_HAL__I2CDevice:write_register(param1, param2) end

-- desc
---@param param1 integer
function AP_HAL__I2CDevice:set_retries(param1) end


-- desc
---@class AP_HAL__UARTDriver
AP_HAL__UARTDriver = {}

-- desc
---@param param1 integer
function AP_HAL__UARTDriver:set_flow_control(param1) end

-- desc
---@return uint32_t
function AP_HAL__UARTDriver:available() end

-- desc
---@param param1 integer
---@return uint32_t
function AP_HAL__UARTDriver:write(param1) end

-- desc
---@return integer
function AP_HAL__UARTDriver:read() end

-- desc
---@param param1 uint32_t
function AP_HAL__UARTDriver:begin(param1) end


-- desc
---@class RC_Channel
RC_Channel = {}

-- desc
---@return number
function RC_Channel:norm_input_ignore_trim() end

-- desc
---@return integer
function RC_Channel:get_aux_switch_pos() end

-- desc
---@return number
function RC_Channel:norm_input() end


-- desc
---@class periph
periph = {}

-- desc
---@return uint32_t
function periph:get_vehicle_state() end

-- desc
---@return number
function periph:get_yaw_earth() end


-- desc
---@class ins
ins = {}

-- desc
---@param param1 integer
---@return number
function ins:get_temperature(param1) end


-- desc
---@class Motors_dynamic
Motors_dynamic = {}

-- desc
---@param param1 motor_factor_table
function Motors_dynamic:load_factors(param1) end

-- desc
---@param param1 integer
---@param param2 integer
function Motors_dynamic:add_motor(param1, param2) end

-- desc
---@param param1 integer
---@return boolean
function Motors_dynamic:init(param1) end


-- desc
---@class analog
analog = {}

-- desc
---@return AP_HAL__AnalogSource
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
---@param param1 integer
---@param param2 number
---@param param3 number
---@param param4 number
---@param param5 number
---@param param6 number
---@param param7 number
---@param param8 boolean
---@param param9 integer
function Motors_6DoF:add_motor(param1, param2, param3, param4, param5, param6, param7, param8, param9) end

-- desc
---@param param1 integer
---@return boolean
function Motors_6DoF:init(param1) end


-- desc
---@class attitude_control
attitude_control = {}

-- desc
---@param param1 number
---@param param2 number
function attitude_control:set_offset_roll_pitch(param1, param2) end

-- desc
---@param param1 boolean
function attitude_control:set_forward_enable(param1) end

-- desc
---@param param1 boolean
function attitude_control:set_lateral_enable(param1) end


-- desc
---@class frsky_sport
frsky_sport = {}

-- desc
---@param param1 integer
---@param param2 integer
---@param param3 integer
---@return integer
function frsky_sport:prep_number(param1, param2, param3) end

-- desc
---@param param1 integer
---@param param2 integer
---@param param3 integer
---@param param4 integer
---@return boolean
function frsky_sport:sport_telemetry_push(param1, param2, param3, param4) end


-- desc
---@class MotorsMatrix
MotorsMatrix = {}

-- desc
---@param param1 integer
---@param param2 number
---@return boolean
function MotorsMatrix:set_throttle_factor(param1, param2) end

-- desc
---@param param1 integer
---@param param2 number
---@param param3 number
---@param param4 number
---@param param5 integer
function MotorsMatrix:add_motor_raw(param1, param2, param3, param4, param5) end

-- desc
---@param param1 integer
---@return boolean
function MotorsMatrix:init(param1) end


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
---@param param1 integer
---@return boolean
function button:get_button_state(param1) end


-- desc
---@class RPM
RPM = {}

-- desc
---@param param1 integer
---@return number|nil
function RPM:get_rpm(param1) end


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
---@param param1 integer
---@param param2 mavlink_mission_item_int_t
---@return boolean
function mission:set_item(param1, param2) end

-- desc
---@param param1 integer
---@return mavlink_mission_item_int_t|nil
function mission:get_item(param1) end

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
---@param param1 integer
---@return boolean
function mission:set_current_cmd(param1) end

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
---@param param1 string
---@param param2 number
---@return boolean
function param:set_and_save(param1, param2) end

-- desc
---@param param1 string
---@param param2 number
---@return boolean
function param:set(param1, param2) end

-- desc
---@param param1 string
---@return number|nil
function param:get(param1) end


-- desc
---@class esc_telem
esc_telem = {}

-- desc
---@param param1 integer
---@return uint32_t|nil
function esc_telem:get_usage_seconds(param1) end

-- desc
---@param param1 integer
---@return number|nil
function esc_telem:get_consumption_mah(param1) end

-- desc
---@param param1 integer
---@return number|nil
function esc_telem:get_voltage(param1) end

-- desc
---@param param1 integer
---@return number|nil
function esc_telem:get_current(param1) end

-- desc
---@param param1 integer
---@return integer|nil
function esc_telem:get_motor_temperature(param1) end

-- desc
---@param param1 integer
---@return integer|nil
function esc_telem:get_temperature(param1) end

-- desc
---@param param1 integer
---@return number|nil
function esc_telem:get_rpm(param1) end


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

-- desc
---@param param1 integer
---@return AP_HAL__UARTDriver
function serial:find_serial(param1) end


-- desc
---@class rc
rc = {}

-- desc
---@param param1 integer
---@return RC_Channel
function rc:get_channel(param1) end

-- desc
---@return boolean
function rc:has_valid_input() end

-- desc
---@param param1 integer
---@param param2 integer
---@return boolean
function rc:run_aux_function(param1, param2) end

-- desc
---@param param1 integer
---@return RC_Channel
function rc:find_channel_for_option(param1) end

-- desc
---@param param1 integer
---@return integer|nil
function rc:get_pwm(param1) end


-- desc
---@class SRV_Channels
SRV_Channels = {}

-- desc
---@param param1 integer
---@param param2 integer
function SRV_Channels:set_range(param1, param2) end

-- desc
---@param param1 integer
---@param param2 integer
function SRV_Channels:set_angle(param1, param2) end

-- desc
---@param param1 integer
---@param param2 number
function SRV_Channels:set_output_norm(param1, param2) end

-- desc
---@param param1 integer
---@return integer
function SRV_Channels:get_output_scaled(param1) end

-- desc
---@param param1 integer
---@return integer|nil
function SRV_Channels:get_output_pwm(param1) end

-- desc
---@param param1 integer
---@param param2 integer
function SRV_Channels:set_output_scaled(param1, param2) end

-- desc
---@param param1 integer
---@param param2 integer
---@param param3 integer
function SRV_Channels:set_output_pwm_chan_timeout(param1, param2, param3) end

-- desc
---@param param1 integer
---@param param2 integer
function SRV_Channels:set_output_pwm_chan(param1, param2) end

-- desc
---@param param1 integer
---@param param2 integer
function SRV_Channels:set_output_pwm(param1, param2) end

-- desc
---@param param1 integer
---@return integer|nil
function SRV_Channels:find_channel(param1) end


-- desc
---@class serialLED
serialLED = {}

-- desc
---@param param1 integer
function serialLED:send(param1) end

-- desc
---@param param1 integer
---@param param2 integer
---@param param3 integer
---@param param4 integer
---@param param5 integer
function serialLED:set_RGB(param1, param2, param3, param4, param5) end

-- desc
---@param param1 integer
---@param param2 integer
---@return boolean
function serialLED:set_num_profiled(param1, param2) end

-- desc
---@param param1 integer
---@param param2 integer
---@return boolean
function serialLED:set_num_neopixel(param1, param2) end


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
---@param param1 number
---@param param2 number
---@return boolean
function vehicle:set_steering_and_throttle(param1, param2) end

-- desc
---@param param1 number
---@return boolean
function vehicle:set_circle_rate(param1) end

-- desc
---@return number|nil
function vehicle:get_circle_radius() end

-- desc
---@param param1 number
---@param param2 number
---@param param3 number
---@param param4 number
---@param param5 boolean
---@param param6 number
---@return boolean
function vehicle:set_target_angle_and_climbrate(param1, param2, param3, param4, param5, param6) end

-- desc
---@param param1 Vector3f
---@return boolean
function vehicle:set_target_velocity_NED(param1) end

-- desc
---@param param1 Vector3f
---@param param2 Vector3f
---@param param3 boolean
---@param param4 number
---@param param5 boolean
---@param param6 number
---@param param7 boolean
---@return boolean
function vehicle:set_target_velaccel_NED(param1, param2, param3, param4, param5, param6, param7) end

-- desc
---@param param1 Vector3f
---@param param2 Vector3f
---@param param3 Vector3f
---@param param4 boolean
---@param param5 number
---@param param6 boolean
---@param param7 number
---@param param8 boolean
---@return boolean
function vehicle:set_target_posvelaccel_NED(param1, param2, param3, param4, param5, param6, param7, param8) end

-- desc
---@param param1 Vector3f
---@param param2 Vector3f
---@return boolean
function vehicle:set_target_posvel_NED(param1, param2) end

-- desc
---@param param1 Vector3f
---@param param2 boolean
---@param param3 number
---@param param4 boolean
---@param param5 number
---@param param6 boolean
---@param param7 boolean
---@return boolean
function vehicle:set_target_pos_NED(param1, param2, param3, param4, param5, param6, param7) end

-- desc
---@return Location|nil
function vehicle:get_target_location() end

-- desc
---@param param1 Location
---@return boolean
function vehicle:set_target_location(param1) end

-- desc
---@param param1 number
---@return boolean
function vehicle:start_takeoff(param1) end

-- desc
---@param param1 integer
---@return number|nil
function vehicle:get_control_output(param1) end

-- desc
---@return uint32_t
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
---@param param1 integer
---@return boolean
function vehicle:set_mode(param1) end


-- desc
---@class onvif
onvif = {}

-- desc
---@return Vector2f
function onvif:get_pan_tilt_limit_max() end

-- desc
---@return Vector2f
function onvif:get_pan_tilt_limit_min() end

-- desc
---@param param1 number
---@param param2 number
---@param param3 number
---@return boolean
function onvif:set_absolutemove(param1, param2, param3) end

-- desc
---@param param1 string
---@param param2 string
---@param param3 string
---@return boolean
function onvif:start(param1, param2, param3) end


-- desc
---@class gcs
gcs = {}

-- desc
---@param param1 string
---@param param2 number
function gcs:send_named_float(param1, param2) end

-- desc
---@param param1 integer
---@param param2 uint32_t
---@param param3 integer
---@return integer
function gcs:set_message_interval(param1, param2, param3) end

-- desc
---@param param1 integer
---@param param2 string
function gcs:send_text(param1, param2) end


-- desc
---@class relay
relay = {}

-- desc
---@param param1 integer
function relay:toggle(param1) end

-- desc
---@param param1 integer
---@return boolean
function relay:enabled(param1) end

-- desc
---@param param1 integer
function relay:off(param1) end

-- desc
---@param param1 integer
function relay:on(param1) end


-- desc
---@class terrain
---@field TerrainStatusOK number
---@field TerrainStatusUnhealthy number
---@field TerrainStatusDisabled number
terrain = {}

-- desc
---@param param1 boolean
---@return number|nil
function terrain:height_above_terrain(param1) end

-- desc
---@param param1 boolean
---@return number|nil
function terrain:height_terrain_difference_home(param1) end

-- desc
---@param param1 Location
---@param param2 boolean
---@return number|nil
function terrain:height_amsl(param1, param2) end

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
---@param param1 integer
---@return Vector3f
function rangefinder:get_pos_offset_orient(param1) end

-- desc
---@param param1 integer
---@return boolean
function rangefinder:has_data_orient(param1) end

-- desc
---@param param1 integer
---@return integer
function rangefinder:status_orient(param1) end

-- desc
---@param param1 integer
---@return integer
function rangefinder:ground_clearance_cm_orient(param1) end

-- desc
---@param param1 integer
---@return integer
function rangefinder:min_distance_cm_orient(param1) end

-- desc
---@param param1 integer
---@return integer
function rangefinder:max_distance_cm_orient(param1) end

-- desc
---@param param1 integer
---@return integer
function rangefinder:distance_cm_orient(param1) end

-- desc
---@param param1 integer
---@return boolean
function rangefinder:has_orientation(param1) end

-- desc
---@return integer
function rangefinder:num_sensors() end


-- desc
---@class proximity
proximity = {}

-- desc
---@param param1 integer
---@return number|nil
---@return number|nil
function proximity:get_object_angle_and_distance(param1) end

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
---@param param1 integer
---@param param2 integer
---@param param3 integer
---@param param4 integer
function notify:handle_rgb_id(param1, param2, param3, param4) end

-- desc
---@param param1 integer
---@param param2 integer
---@param param3 integer
---@param param4 integer
function notify:handle_rgb(param1, param2, param3, param4) end

-- desc
---@param param1 string
function notify:play_tune(param1) end


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
---@param param1 integer
---@return Vector3f
function gps:get_antenna_offset(param1) end

-- desc
---@param param1 integer
---@return boolean
function gps:have_vertical_velocity(param1) end

-- desc
---@param param1 integer
---@return uint32_t
function gps:last_message_time_ms(param1) end

-- desc
---@param param1 integer
---@return uint32_t
function gps:last_fix_time_ms(param1) end

-- desc
---@param param1 integer
---@return integer
function gps:get_vdop(param1) end

-- desc
---@param param1 integer
---@return integer
function gps:get_hdop(param1) end

-- desc
---@param param1 integer
---@return uint32_t
function gps:time_week_ms(param1) end

-- desc
---@param param1 integer
---@return integer
function gps:time_week(param1) end

-- desc
---@param param1 integer
---@return integer
function gps:num_sats(param1) end

-- desc
---@param param1 integer
---@return number
function gps:ground_course(param1) end

-- desc
---@param param1 integer
---@return number
function gps:ground_speed(param1) end

-- desc
---@param param1 integer
---@return Vector3f
function gps:velocity(param1) end

-- desc
---@param param1 integer
---@return number|nil
function gps:vertical_accuracy(param1) end

-- desc
---@param param1 integer
---@return number|nil
function gps:horizontal_accuracy(param1) end

-- desc
---@param param1 integer
---@return number|nil
function gps:speed_accuracy(param1) end

-- desc
---@param param1 integer
---@return Location
function gps:location(param1) end

-- desc
---@param param1 integer
---@return integer
function gps:status(param1) end

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
---@param param1 integer
---@param param2 number
---@return boolean
function battery:reset_remaining(param1, param2) end

-- desc
---@param param1 integer
---@return integer|nil
function battery:get_cycle_count(param1) end

-- desc
---@param param1 integer
---@return number|nil
function battery:get_temperature(param1) end

-- desc
---@param param1 integer
---@return boolean
function battery:overpower_detected(param1) end

-- desc
---@return boolean
function battery:has_failsafed() end

-- desc
---@param param1 integer
---@return integer
function battery:pack_capacity_mah(param1) end

-- desc
---@param param1 integer
---@return integer|nil
function battery:capacity_remaining_pct(param1) end

-- desc
---@param param1 integer
---@return number|nil
function battery:consumed_wh(param1) end

-- desc
---@param param1 integer
---@return number|nil
function battery:consumed_mah(param1) end

-- desc
---@param param1 integer
---@return number|nil
function battery:current_amps(param1) end

-- desc
---@param param1 integer
---@return number
function battery:voltage_resting_estimate(param1) end

-- desc
---@param param1 integer
---@return number
function battery:voltage(param1) end

-- desc
---@param param1 integer
---@return boolean
function battery:healthy(param1) end

-- desc
---@return integer
function battery:num_instances() end


-- desc
---@class arming
arming = {}

-- desc
---@param param1 integer
---@param param2 string
function arming:set_aux_auth_failed(param1, param2) end

-- desc
---@param param1 integer
function arming:set_aux_auth_passed(param1) end

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
function arming:disarm() end


-- desc
---@class ahrs
ahrs = {}

-- desc
---@return boolean
function ahrs:initialised() end

-- desc
---@param param1 Location
---@return boolean
function ahrs:set_origin(param1) end

-- desc
---@return Location|nil
function ahrs:get_origin() end

-- desc
---@param param1 Location
---@return boolean
function ahrs:set_home(param1) end

-- desc
---@param param1 integer
---@return Vector3f|nil
---@return Vector3f|nil
function ahrs:get_vel_innovations_and_variances_for_source(param1) end

-- desc
---@param param1 integer
function ahrs:set_posvelyaw_source_set(param1) end

-- desc
---@return number|nil
---@return number|nil
---@return number|nil
---@return Vector3f|nil
---@return number|nil
function ahrs:get_variances() end

-- desc
---@return number
function ahrs:get_EAS2TAS() end

-- desc
---@param param1 Vector3f
---@return Vector3f
function ahrs:body_to_earth(param1) end

-- desc
---@param param1 Vector3f
---@return Vector3f
function ahrs:earth_to_body(param1) end

-- desc
---@return Vector3f
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
---@return Vector3f|nil
function ahrs:get_relative_position_NED_origin() end

-- desc
---@return Vector3f|nil
function ahrs:get_relative_position_NED_home() end

-- desc
---@return Vector3f|nil
function ahrs:get_velocity_NED() end

-- desc
---@return Vector2f
function ahrs:groundspeed_vector() end

-- desc
---@return Vector3f
function ahrs:wind_estimate() end

-- desc
---@return number|nil
function ahrs:get_hagl() end

-- desc
---@return Vector3f
function ahrs:get_accel() end

-- desc
---@return Vector3f
function ahrs:get_gyro() end

-- desc
---@return Location
function ahrs:get_home() end

-- desc
---@return Location|nil
function ahrs:get_position() end

-- desc
---@return number
function ahrs:get_yaw() end

-- desc
---@return number
function ahrs:get_pitch() end

-- desc
---@return number
function ahrs:get_roll() end



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

--- Custom 32-bit unsigned integer userdata type.
-- This is used to handle large integer values that may lose precision if stored as standard Lua numbers (which are double-precision floats).
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

-- Create a new uint32_t userdata object with an optional initial value.
---@param value? uint32_t_ud|integer|number -- Optional initial value.
---@return uint32_t_ud
function uint32_t(value) end

-- Convert the uint32_t value to a standard Lua number (float).
---@return number
function uint32_t_ud:tofloat() end

-- Convert the uint32_t value to a standard Lua integer.
---@return integer
function uint32_t_ud:toint() end

--- Custom 64-bit unsigned integer userdata type.
-- Essential for high-resolution timestamps (like from `micros()`) or large IDs where standard Lua numbers would lose precision.
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

-- Create a new uint64_t userdata object with an optional initial value.
-- Note that standard Lua numbers (floats) will lose precision with large integer values.
---@param value? uint64_t_ud|uint32_t_ud|integer|number
---@return uint64_t_ud
function uint64_t(value) end

-- Create a uint64_t userdata object from a low and high 32-bit half.
-- The final value is equivalent to `(high << 32) | low`.
---@param high uint32_t_ud|integer|number -- The most significant 32 bits.
---@param low uint32_t_ud|integer|number -- The least significant 32 bits.
---@return uint64_t_ud
function uint64_t(high, low) end

-- Convert the uint64_t value to a standard Lua number (float). Be aware that this can lose precision for large values.
---@return number
function uint64_t_ud:tofloat() end

-- Convert the uint64_t value to a standard Lua integer. Returns nil if the value is too large to be represented by a native 32-bit integer.
---@return integer|nil
function uint64_t_ud:toint() end

-- Split the 64-bit value into high and low 32-bit halves, returning each as a uint32_t userdata.
---@return uint32_t_ud -- The high 32 bits of the value (value >> 32).
---@return uint32_t_ud -- The low 32 bits of the value (value & 0xFFFFFFFF).
function uint64_t_ud:split() end

-- Returns the system time in milliseconds since the autopilot booted.
---@return uint32_t_ud -- Milliseconds since boot.
function millis() end

-- Returns the system time in microseconds since the autopilot booted.
-- This should be stored in a uint64_t_ud to avoid precision loss.
---@return uint64_t_ud -- Microseconds since boot.
function micros() end

-- Receives data from a currently running NAV_SCRIPT_TIME mission command.
-- This function is the bridge between a mission file and a script, allowing for scripted maneuvers inside an auto mission.
-- Returns nil for all values if no NAV_SCRIPT_TIME command is active.
---@return uint32_t_ud|nil -- The time the NAV_SCRIPT_TIME command started, in milliseconds.
---@return integer|nil -- The value from the mission command's 'command' (param 1) field.
---@return number|nil -- The value from the mission command's 'param 2' field.
---@return number|nil -- The value from the mission command's 'param 3' field.
---@return number|nil -- The value from the mission command's 'param 4' field.
function mission_receive() end

-- Prints text to the GCS and connected peripherals.
-- If a MAVLink GCS is connected, this is equivalent to `gcs:send_text(MAV_SEVERITY.DEBUG, text)`.
-- If the script is running on a peripheral, the text will be sent over the CAN bus.
---@param text string|number|integer -- The content to be printed. Non-string types will be converted to strings.
function print(text) end

-- Dataflash logging object for writing custom data to the vehicle's onboard log files (e.g., .bin files).
-- This is the primary method for recording script-specific data for post-flight analysis.
logger = {}

-- Writes a custom log entry to the dataflash log with full formatting options. A timestamp is automatically added.
-- Example from `revert_param.lua`: `logger:write('QUIK','SRate,Gain,Param', 'ffn', srate, P:get(), axis .. stage)`
---@param name string -- A unique name for the log entry, must be 4 characters or less to avoid conflicts.
---@param labels string -- Comma-separated labels for the data fields, up to 58 characters. E.g., "Lat,Lon,Alt".
---@param format string -- A format string defining the data types of the values. See https://ardupilot.org/dev/docs/code-overview-logger.html for format specifiers.
---@param units string -- A string defining the units for each value (e.g., 'm', 's', 'd' for meters, seconds, degrees). Use '-' for no units.
---@param multipliers string -- A string defining multipliers for each value. Use '-' for no multiplier.
---@param ... integer|number|uint32_t_ud|string|boolean -- The data values to be logged, matching the types and order in the format string.
function logger:write(name, labels, format, units, multipliers, ...) end

-- Writes a custom log entry to the dataflash log. A timestamp is automatically added. This is a simplified version without units or multipliers.
---@param name string -- A unique name for the log entry, must be 4 characters or less.
---@param labels string -- Comma-separated labels for the data fields, up to 58 characters.
---@param format string -- A format string defining the data types of the values.
---@param ... integer|number|uint32_t_ud|string|boolean -- The data values to be logged.
function logger:write(name, labels, format, ...) end

-- Logs the entire content of a specified file to the onboard log. Useful for saving configuration or mission files with the flight data for debugging.
---@param filename string -- The name of the file to log (e.g., "mission.txt").
function logger:log_file_content(filename) end

-- I2C bus interaction object. Provides low-level access for communicating with I2C peripherals.
i2c = {}

-- Gets a device handler for a specific I2C bus and address. This is the first step for any I2C communication.
-- Example from `RM3100_self_test.lua`: `local rm3100 = i2c:get_device(0, 0x20)`
---@param bus integer -- The I2C bus number (e.g., 0 for the first bus, corresponding to I2C1 on the flight controller).
---@param address integer -- The 7-bit I2C device address (0 to 127).
---@param clock? uint32_t_ud|integer|number -- Optional bus clock speed in Hz (e.g., 400000 for 400kHz). Defaults to 400000.
---@param smbus? boolean -- Optional flag to indicate if SMBus protocol should be used. Defaults to false.
---@return AP_HAL__I2CDevice_ud -- Returns an I2C device object used for subsequent read/write operations.
function i2c:get_device(bus, address, clock, smbus) end

-- EFI (Electronic Fuel Injection) state structure. A container for all telemetry data from an EFI system.
-- Scripts typically receive this object from `efi:get_state()` or populate it to send to an EFI backend.
---@class (exact) EFI_State_ud
local EFI_State_ud = {}

---@return EFI_State_ud
function EFI_State() end

-- get field: Pressure/Temperature compensation for the engine.
---@return number
function EFI_State_ud:pt_compensation() end

-- set field
---@param value number
function EFI_State_ud:pt_compensation(value) end

-- get field: The throttle output percentage being sent to the engine.
---@return number
function EFI_State_ud:throttle_out() end

-- set field
---@param value number
function EFI_State_ud:throttle_out(value) end

-- get field: The voltage being supplied to the ignition system.
---@return number
function EFI_State_ud:ignition_voltage() end

-- set field
---@param value number
function EFI_State_ud:ignition_voltage(value) end

-- get field: A sub-object containing status for individual engine cylinders.
---@return Cylinder_Status_ud
function EFI_State_ud:cylinder_status() end

-- set field
---@param value Cylinder_Status_ud
function EFI_State_ud:cylinder_status(value) end

-- get field: The index of the Electronic Control Unit (ECU) providing the data.
---@return integer
function EFI_State_ud:ecu_index() end

-- set field
---@param value integer
function EFI_State_ud:ecu_index(value) end

-- get field: The physical position of the throttle plate as a percentage (0-100).
---@return integer
function EFI_State_ud:throttle_position_percent() end

-- set field
---@param value integer
function EFI_State_ud:throttle_position_percent(value) end

-- get field: The total volume of fuel consumed since boot, in cubic centimeters.
---@return number
function EFI_State_ud:estimated_consumed_fuel_volume_cm3() end

-- set field
---@param value number
function EFI_State_ud:estimated_consumed_fuel_volume_cm3(value) end

-- get field: The current rate of fuel consumption in cubic centimeters per minute.
---@return number
function EFI_State_ud:fuel_consumption_rate_cm3pm() end

-- set field
---@param value number
function EFI_State_ud:fuel_consumption_rate_cm3pm(value) end

-- get field: The current fuel pressure, typically in kPa.
---@return number
function EFI_State_ud:fuel_pressure() end

-- set field
---@param value number
function EFI_State_ud:fuel_pressure(value) end

-- get field: The status of the fuel pressure sensor.
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

-- get field: The engine oil temperature, typically in degrees Celsius.
---@return number
function EFI_State_ud:oil_temperature() end

-- set field
---@param value number
function EFI_State_ud:oil_temperature(value) end

-- get field: The engine oil pressure, typically in kPa.
---@return number
function EFI_State_ud:oil_pressure() end

-- set field
---@param value number
function EFI_State_ud:oil_pressure(value) end

-- get field: The engine coolant temperature, typically in degrees Celsius.
---@return number
function EFI_State_ud:coolant_temperature() end

-- set field
---@param value number
function EFI_State_ud:coolant_temperature(value) end

-- get field: The temperature of the air in the intake manifold, typically in degrees Celsius.
---@return number
function EFI_State_ud:intake_manifold_temperature() end

-- set field
---@param value number
function EFI_State_ud:intake_manifold_temperature(value) end

-- get field: The pressure in the intake manifold, in kilopascals (kPa).
---@return number
function EFI_State_ud:intake_manifold_pressure_kpa() end

-- set field
---@param value number
function EFI_State_ud:intake_manifold_pressure_kpa(value) end

-- get field: The current atmospheric pressure, in kilopascals (kPa).
---@return number
function EFI_State_ud:atmospheric_pressure_kpa() end

-- set field
---@param value number
function EFI_State_ud:atmospheric_pressure_kpa(value) end

-- get field: The duration of the spark dwell time, in milliseconds.
---@return number
function EFI_State_ud:spark_dwell_time_ms() end

-- set field
---@param value number
function EFI_State_ud:spark_dwell_time_ms(value) end

-- get field: The rotational speed of the engine, in revolutions per minute (RPM).
---@return uint32_t_ud
function EFI_State_ud:engine_speed_rpm() end

-- set field
---@param value uint32_t_ud|integer|number
function EFI_State_ud:engine_speed_rpm(value) end

-- get field: The current engine load as a percentage (0-100).
---@return integer
function EFI_State_ud:engine_load_percent() end

-- set field
---@param value integer
function EFI_State_ud:engine_load_percent(value) end

-- get field: A boolean flag indicating a general error state in the EFI system.
---@return boolean
function EFI_State_ud:general_error() end

-- set field
---@param value boolean
function EFI_State_ud:general_error(value) end

-- get field: The system timestamp (in milliseconds) of the last EFI data update.
---@return uint32_t_ud
function EFI_State_ud:last_updated_ms() end

-- set field
---@param value uint32_t_ud|integer|number
function EFI_State_ud:last_updated_ms(value) end


-- EFI Cylinder_Status structure. Contains detailed information for a single engine cylinder.
---@class (exact) Cylinder_Status_ud
local Cylinder_Status_ud = {}

---@return Cylinder_Status_ud
function Cylinder_Status() end

-- get field: The lambda coefficient (air-fuel ratio). A value of 1.0 is stoichiometric.
---@return number
function Cylinder_Status_ud:lambda_coefficient() end

-- set field
---@param value number
function Cylinder_Status_ud:lambda_coefficient(value) end

-- get field: The exhaust gas temperature (EGT), typically in degrees Celsius.
---@return number
function Cylinder_Status_ud:exhaust_gas_temperature() end

-- set field
---@param value number
function Cylinder_Status_ud:exhaust_gas_temperature(value) end

-- get field: The EGT of a second sensor, if available.
---@return number
function Cylinder_Status_ud:exhaust_gas_temperature2() end

-- set field
---@param value number
function Cylinder_Status_ud:exhaust_gas_temperature2(value) end

-- get field: The cylinder head temperature (CHT), typically in degrees Celsius.
---@return number
function Cylinder_Status_ud:cylinder_head_temperature() end

-- set field
---@param value number
function Cylinder_Status_ud:cylinder_head_temperature(value) end

-- get field: The CHT of a second sensor, if available.
---@return number
function Cylinder_Status_ud:cylinder_head_temperature2() end

-- set field
---@param value number
function Cylinder_Status_ud:cylinder_head_temperature2(value) end

-- get field: The duration of the fuel injection pulse, in milliseconds.
---@return number
function Cylinder_Status_ud:injection_time_ms() end

-- set field
---@param value number
function Cylinder_Status_ud:injection_time_ms(value) end

-- get field: The ignition timing advance, in degrees.
---@return number
function Cylinder_Status_ud:ignition_timing_deg() end

-- set field
---@param value number
function Cylinder_Status_ud:ignition_timing_deg(value) end

-- Main EFI interface object. Used to get data from and send data to an EFI system.
efi = {}

-- Returns the most recent, complete state of the EFI system.
-- Example from `efi_speed_check.lua`: `local efi_state = efi:get_state()`
---@return EFI_State_ud -- An `EFI_State_ud` object containing the latest telemetry.
function efi:get_state() end

-- Returns the system timestamp of the last successful EFI data update.
-- Useful for checking if the connection to the EFI is alive.
---@return uint32_t_ud -- Milliseconds since boot.
function efi:get_last_update_ms() end

-- Gets a handle to the EFI backend driver for a specific instance.
-- This is used by custom EFI driver scripts (like EFI_HFE.lua) to push data into the ArduPilot EFI system.
---@param instance integer -- The EFI instance number (0 for the first).
---@return AP_EFI_Backend_ud|nil -- The backend object, or nil if not found.
function efi:get_backend(instance) end

-- CAN bus interaction object. Provides an interface for sending and receiving raw CAN frames.
-- This is a low-level interface and should be used when you need to implement a custom CAN protocol.
-- For the DroneCAN protocol, it is recommended to use the higher-level DroneCAN bindings.
CAN = {}

-- Gets a device handler for the first CAN scripting driver.
-- The corresponding CAN_P1_PROTOCOL parameter must be set to 'Scripting' (28) for this to succeed.
---@param buffer_len uint32_t_ud|integer|number -- The number of CAN frames to buffer (1 to 25). Frames will be dropped if not read before the buffer is full.
---@return ScriptingCANBuffer_ud|nil -- Returns a buffer object for sending/receiving frames, or nil if the driver is not enabled.
function CAN:get_device(buffer_len) end

-- Gets a device handler for the second CAN scripting driver.
-- The corresponding CAN_P2_PROTOCOL parameter must be set to 'Scripting' (28) for this to succeed.
---@param buffer_len uint32_t_ud|integer|number -- The number of CAN frames to buffer (1 to 25).
---@return ScriptingCANBuffer_ud|nil -- Returns a buffer object for sending/receiving frames, or nil if the driver is not enabled.
function CAN:get_device2(buffer_len) end


-- Gets the latest FlexDebug message from a specific DroneCAN node.
-- This is a utility for receiving custom debug information from a DroneCAN peripheral that is transmitting `uavcan.debug.KeyValue` messages.
---@param bus number -- The CAN bus number (0 for the first bus/CAN1, 1 for the second/CAN2).
---@param node number -- The DroneCAN node ID of the peripheral to listen to.
---@param id number -- The specific FlexDebug message ID (the 'key' from the KeyValue pair) to filter for.
---@param last_us uint32_t_ud|integer|number -- The timestamp of the last message you received. The function will only return a new message if its timestamp is different. Pass 0 to get the first message.
---@return uint32_t_ud|nil -- The timestamp (in microseconds) of the new message, or nil if no new message has arrived.
---@return string|nil -- The message payload as a raw string (up to 255 bytes), or nil.
function DroneCAN_get_FlexDebug(bus,node,id,last_us) end

-- Auto generated binding

-- Represents a single, raw CAN frame.
-- This object is used with the low-level CAN scripting driver to send and receive data.
---@class (exact) CANFrame_ud
local CANFrame_ud = {}

---@return CANFrame_ud
function CANFrame() end

-- Get the Data Length Code (DLC) of the frame.
---@return integer -- The number of data bytes in the payload (0-8 for classic CAN).
function CANFrame_ud:dlc() end

-- Set the Data Length Code (DLC) of the frame.
---@param value integer -- The number of data bytes in the payload.
function CANFrame_ud:dlc(value) end

-- Get a byte from the data payload array.
---@param index integer -- The index of the byte to retrieve (1-8 for classic CAN).
---@return integer -- The byte value (0-255).
function CANFrame_ud:data(index) end

-- Set a byte in the data payload array.
---@param index integer -- The index of the byte to set (1-8 for classic CAN).
---@param value integer -- The byte value to set (0-255).
function CANFrame_ud:data(index, value) end

-- Get the CAN frame ID.
---@return uint32_t_ud -- The frame ID (11-bit for standard, 29-bit for extended).
function CANFrame_ud:id() end

-- Set the CAN frame ID.
---@param value uint32_t_ud|integer|number -- The frame ID to set.
function CANFrame_ud:id(value) end

-- Check if the frame is an error frame.
---@return boolean
function CANFrame_ud:isErrorFrame() end

-- Check if the frame is a Remote Transmission Request (RTR).
---@return boolean
function CANFrame__ud:isRemoteTransmissionRequest() end

-- Check if the frame uses an extended (29-bit) ID.
---@return boolean
function CANFrame_ud:isExtended() end

-- Get the CAN frame ID as a signed integer.
---@return integer
function CANFrame_ud:id_signed() end

-- Represents the motor mixing factors table.
-- This is an advanced structure that provides insight into how the flight controller allocates motor power for attitude control.
---@class (exact) motor_factor_table_ud
local motor_factor_table_ud = {}

---@return motor_factor_table_ud
function motor_factor() end

-- Get the throttle factor for a specific motor.
---@param index integer -- The motor number (e.g., 1 for motor 1).
---@return number -- The throttle mixing factor for this motor.
function motor_factor_table_ud:throttle(index) end

-- Set the throttle factor for a specific motor.
---@param index integer
---@param value number
function motor_factor_table_ud:throttle(index, value) end

-- Get the yaw factor for a specific motor.
---@param index integer -- The motor number.
---@return number -- The yaw mixing factor for this motor.
function motor_factor_table_ud:yaw(index) end

-- Set the yaw factor for a specific motor.
---@param index integer
---@param value number
function motor_factor_table_ud:yaw(index, value) end

-- Get the pitch factor for a specific motor.
---@param index integer -- The motor number.
---@return number -- The pitch mixing factor for this motor.
function motor_factor_table_ud:pitch(index) end

-- Set the pitch factor for a specific motor.
---@param index integer
---@param value number
function motor_factor_table_ud:pitch(index, value) end

-- Get the roll factor for a specific motor.
---@param index integer -- The motor number.
---@return number -- The roll mixing factor for this motor.
function motor_factor_table_ud:roll(index) end

-- Set the roll factor for a specific motor.
---@param index integer
---@param value number
function motor_factor_table_ud:roll(index, value) end

-- Represents a network socket for TCP or UDP communication.
-- This class provides the low-level functions needed to create network clients and servers.
-- The `net_webserver.lua` example demonstrates a practical use case for this class.
---@class (exact) SocketAPM_ud
local SocketAPM_ud = {}

-- Creates a new socket object.
---@param datagram integer -- Set to 1 for a UDP (datagram) socket, or 0 for a TCP (stream) socket.
---@return SocketAPM_ud -- The new socket object.
function Socket(datagram) end

-- Converts a string representation of an IPv4 address to its 32-bit unsigned integer format.
---@param str_address string -- The IPv4 address as a string (e.g., "192.168.1.1").
---@return uint32_t_ud -- The IPv4 address as a 32-bit integer.
function string_to_ipv4_addr(str_address) end

-- Converts a 32-bit unsigned integer representation of an IPv4 address back to its string format.
---@param addr uint32_t_ud|integer|number -- The IPv4 address as a 32-bit integer.
---@return string -- The string representation of the address (e.g., "192.168.1.1").
function ipv4_addr_to_string(addr) end

-- Returns true if the socket is currently connected to a remote endpoint.
---@return boolean
function SocketAPM_ud:is_connected() end

-- Sets the blocking or non-blocking state of the socket.
-- In blocking mode (true), send/recv calls will wait until the operation is complete.
-- In non-blocking mode (false), they return immediately.
---@param blocking boolean -- true for blocking, false for non-blocking.
---@return boolean
function SocketAPM_ud:set_blocking(blocking) end

-- Puts a TCP socket into a listening state, ready to accept incoming connections.
-- This must be called before `accept()`.
---@param backlog integer -- The maximum number of pending connections to queue.
---@return boolean -- True on success.
function SocketAPM_ud:listen(backlog) end

-- Sends data over a connected TCP socket.
---@param str string -- The data to send, which can be a binary string.
---@param len uint32_t_ud|integer|number -- The number of bytes to send from the string.
---@return integer -- The number of bytes actually sent, which may be less than `len` if the send buffer is full.
function SocketAPM_ud:send(str, len) end

-- Sends data over a UDP socket to a specific destination.
---@param str string -- The data to send, which can be a binary string.
---@param len uint32_t_ud|integer|number -- The number of bytes to send.
---@param ipaddr uint32_t_ud|integer|number -- The destination IPv4 address as a 32-bit integer.
---@param port integer -- The destination port number.
---@return integer -- The number of bytes sent.
function SocketAPM_ud:sendto(str, len, ipaddr, port) end

-- Binds the socket to a specific local IP address and port.
-- For a server, this is typically used to listen on a specific port for incoming connections or datagrams.
---@param IP_address string -- The local IP address to bind to. Use "0.0.0.0" to bind to all available network interfaces.
---@param port integer -- The local port number to bind to.
---@return boolean -- True on success.
function SocketAPM_ud:bind(IP_address, port) end

-- Connects a TCP socket to a remote server.
---@param IP_address string -- The IP address of the remote server.
---@param port integer -- The port number of the remote server.
---@return boolean -- True on success.
function SocketAPM_ud:connect(IP_address, port) end

-- Accepts a new incoming connection on a listening TCP socket.
-- This function will block until a new connection arrives if the socket is in blocking mode.
---@return SocketAPM_ud|nil -- A new socket object for the accepted connection, or nil if no connection is pending (in non-blocking mode).
function SocketAPM_ud:accept() end

-- Receives data from a socket.
---@param length integer -- The maximum number of bytes to receive.
---@return string|nil -- The received data as a string, or nil on error.
---@return uint32_t_ud|nil -- For UDP sockets, the source IP address of the received packet.
---@return integer|nil -- For UDP sockets, the source port of the received packet.
function SocketAPM_ud:recv(length) end

-- Checks if there is incoming data available to be read from the socket.
---@param timeout_ms uint32_t_ud|integer|number -- The time to wait in milliseconds.
---@return boolean -- True if data is available for reading.
function SocketAPM_ud:pollin(timeout_ms) end

-- Checks if the socket is ready to accept data for writing without blocking.
---@param timeout_ms uint32_t_ud|integer|number -- The time to wait in milliseconds.
---@return boolean -- True if the socket is ready for writing.
function SocketAPM_ud:pollout(timeout_ms) end

-- Closes the socket.
-- Note: Lua does not automatically garbage collect sockets. You must explicitly close a socket when you are finished with it to free up system resources.
function SocketAPM_ud:close() end

-- Efficiently sends the contents of a file over a socket.
-- This offloads the file transfer to the underlying operating system, which is much faster than reading and sending in chunks within the script.
-- The socket and file handle are closed automatically upon completion.
---@param filehandle string -- The handle of the file to send.
---@return boolean -- True on success.
function SocketAPM_ud:sendfile(filehandle) end

-- Enables the SO_REUSEADDR socket option.
-- This allows a server to bind to a port that is in a TIME_WAIT state, which can be useful when restarting a server script quickly.
---@return boolean -- True on success.
function SocketAPM_ud:reuseaddress() end

-- Represents a PWM input source. Used for reading PWM signals from auxiliary input pins.
---@class (exact) AP_HAL__PWMSource_ud
local AP_HAL__PWMSource_ud = {}

-- Creates a new PWMSource object.
---@return AP_HAL__PWMSource_ud
function PWMSource() end

-- Gets the average PWM pulse width since the last call.
---@return integer -- The average pulse width in microseconds (us).
function AP_HAL__PWMSource_ud:get_pwm_avg_us() end

-- Gets the latest PWM pulse width reading.
---@return integer -- The latest pulse width in microseconds (us).
function AP_HAL__PWMSource_ud:get_pwm_us() end

-- Sets the physical pin to be used for this PWM input source.
-- The pin must be configured as a PWM input.
---@param pin_number integer -- The pin number (e.g., 50 for AUX1).
---@return boolean -- True on success.
function AP_HAL__PWMSource_ud:set_pin(pin_number) end

-- Represents a single mission command, mirroring the MAVLink MAV_CMD definitions and the MAVLink Mission Item Protocol.
-- This object is the fundamental building block for creating and modifying autonomous missions.
-- Scripts can get this object using `mission:get_item()` and modify its fields before writing it back with `mission:set_item()`.
---@class (exact) mavlink_mission_item_int_t_ud
local mavlink_mission_item_int_t_ud = {}

-- Creates a new, empty mavlink_mission_item_int_t object. All fields will be initialized to zero.
---@return mavlink_mission_item_int_t_ud
function mavlink_mission_item_int_t() end

-- Get the 'current' field. A value of 1 indicates this is the active command in the mission.
---@return integer
function mavlink_mission_item_int_t_ud:current() end

-- Set the 'current' field.
---@param value integer
function mavlink_mission_item_int_t_ud:current(value) end

-- Get the coordinate frame for this mission item. The frame determines how the x, y, and z values are interpreted.
-- Common values: 3 (MAV_FRAME_GLOBAL_RELATIVE_ALT), 10 (MAV_FRAME_GLOBAL_TERRAIN_ALT).
---@return integer
function mavlink_mission_item_int_t_ud:frame() end

-- Set the coordinate frame.
---@param value integer -- The frame enum value.
function mavlink_mission_item_int_t_ud:frame(value) end

-- Get the command ID (MAV_CMD). This defines the action to be performed.
-- E.g., 16 for MAV_CMD_NAV_WAYPOINT, 21 for MAV_CMD_NAV_LAND, 22 for MAV_CMD_NAV_TAKEOFF.
---@return integer
function mavlink_mission_item_int_t_ud:command() end

-- Set the command ID.
---@param value integer -- The MAV_CMD enum value.
function mavlink_mission_item_int_t_ud:command(value) end

-- Get the sequence number of this command in the mission.
---@return integer
function mavlink_mission_item_int_t_ud:seq() end

-- Set the sequence number.
---@param value integer
function mavlink_mission_item_int_t_ud:seq(value) end

-- Get the 'z' field, which typically represents altitude.
---@return number -- The meaning depends on the command and frame, but it is usually altitude in meters.
function mavlink_mission_item_int_t_ud:z() end

-- Set the 'z' field.
---@param value number -- Altitude in meters.
function mavlink_mission_item_int_t_ud:z(value) end

-- Get the 'y' field, which typically represents longitude for geographic commands.
---@return integer -- Longitude in degrees * 1e7.
function mavlink_mission_item_int_t_ud:y() end

-- Set the 'y' field.
---@param value integer -- Longitude in degrees * 1e7.
function mavlink_mission_item_int_t_ud:y(value) end

-- Get the 'x' field, which typically represents latitude for geographic commands.
---@return integer -- Latitude in degrees * 1e7.
function mavlink_mission_item_int_t_ud:x() end

-- Set the 'x' field.
---@param value integer -- Latitude in degrees * 1e7.
function mavlink_mission_item_int_t_ud:x(value) end

-- Get the 'param4' field. The meaning is specific to the command ID.
---@return number
function mavlink_mission_item_int_t_ud:param4() end

-- Set the 'param4' field.
---@param value number
function mavlink_mission_item_int_t_ud:param4(value) end

-- Get the 'param3' field. The meaning is specific to the command ID (e.g., radius for loiter commands).
---@return number
function mavlink_mission_item_int_t_ud:param3() end

-- Set the 'param3' field.
---@param value number
function mavlink_mission_item_int_t_ud:param3(value) end

-- Get the 'param2' field. The meaning is specific to the command ID (e.g., acceptance radius for waypoints).
---@return number
function mavlink_mission_item_int_t_ud:param2() end

-- Set the 'param2' field.
---@param value number
function mavlink_mission_item_int_t_ud:param2(value) end

-- Get the 'param1' field. The meaning is specific to the command ID (e.g., hold time for waypoints).
---@return number
function mavlink_mission_item_int_t_ud:param1() end

-- Set the 'param1' field.
---@param value number
function mavlink_mission_item_int_t_ud:param1(value) end

-- A helper object for dynamically accessing and modifying an ArduPilot parameter.
-- This object is created empty and then bound to a parameter using the `:init(name)` function.
-- This is useful when the parameter name is not known at the start of the script or may change.
---@class (exact) Parameter_ud
local Parameter_ud = {}

-- Creates a new, unbound parameter helper. You must call :init(name) on this object before it can be used.
---@return Parameter_ud
function Parameter() end

-- Sets the default value of this parameter in the vehicle's parameter metadata.
-- If the parameter has not been configured by the user, its value will be updated to this new default.
---@param value number -- The new default value.
---@return boolean -- True on success.
function Parameter_ud:set_default(value) end

-- Returns true if the parameter has been configured by the user away from its default value.
---@return boolean
function Parameter_ud:configured() end

-- Sets the parameter to the given value and saves it to permanent storage (EEPROM).
-- The value will persist after a reboot. Use this with caution, especially on critical parameters.
---@param value number -- The value to set.
---@return boolean -- True on success.
function Parameter_ud:set_and_save(value) end

-- Sets the parameter to the given value for the current session.
-- The value will NOT persist after a reboot.
---@param value number -- The value to set.
---@return boolean -- True on success.
function Parameter_ud:set(value) end

-- Gets the current value of the parameter.
---@return number|nil -- The parameter's current value, or nil if `:init()` has not been called or the parameter was not found.
function Parameter_ud:get() end

-- Initializes the parameter helper from its internal key information.
-- This is an advanced function that allows a script to load old parameters that may have been removed from the main code.
---@param key integer -- The parameter's internal key.
---@param group_element uint32_t_ud|integer|number
---@param type integer -- The AP_Param type enum.
---| '1' # AP_PARAM_INT8
---| '2' # AP_PARAM_INT16
---| '3' # AP_PARAM_INT32
---| '4' # AP_PARAM_FLOAT
---@return boolean
function Parameter_ud:init_by_info(key, group_element, type) end

-- Initializes (binds) this parameter helper to a parameter specified by name. This must be called before get/set.
---@param name string -- The name of the parameter (e.g., "ANGLE_MAX").
---@return boolean -- True if the parameter was found and successfully initialized.
function Parameter_ud:init(name) end

-- A helper object for accessing and modifying a statically defined ArduPilot parameter.
-- The parameter name is provided at creation and cannot be changed. This is generally safer and more efficient than the dynamic `Parameter_ud`.
-- Example from `Param_Controller.lua`: `local my_param = Parameter("MY_PARAM_NAME")`
---@class (exact) Parameter_ud_const
local Parameter_ud_const = {}

-- Creates a new parameter helper and binds it to the specified parameter name.
-- This will cause a script error if no parameter with the given name is found.
---@param name string -- The name of the parameter (e.g., "ANGLE_MAX").
---@return Parameter_ud_const
function Parameter(name) end

-- Sets the default value of this parameter in the vehicle's parameter metadata.
---@param value number
---@return boolean
function Parameter_ud_const:set_default(value) end

-- Returns true if the parameter has been configured by the user away from its default value.
---@return boolean
function Parameter_ud_const:configured() end

-- Sets the parameter to the given value and saves it to permanent storage (EEPROM). Persists after reboot.
---@param value number
---@return boolean
function Parameter_ud_const:set_and_save(value) end

-- Sets the parameter to the given value for the current session. Does not persist after reboot.
---@param value number
---@return boolean
function Parameter_ud_const:set(value) end

-- Gets the current value of the parameter.
---@return number -- The parameter's current value.
function Parameter_ud_const:get() end

-- Represents a 2D vector with x and y floating point components.
---@class (exact) Vector2f_ud
---@operator add(Vector2f_ud): Vector2f_ud
---@operator sub(Vector2f_ud): Vector2f_ud
local Vector2f_ud = {}

-- Creates a new Vector2f object, initialized to (0,0).
---@return Vector2f_ud
function Vector2f() end

-- Creates a copy of this Vector2f, returning a new object.
---@return Vector2f_ud
function Vector2f_ud:copy() end

-- Gets the y component of the vector.
---@return number
function Vector2f_ud:y() end

-- Sets the y component of the vector.
---@param value number
function Vector2f_ud:y(value) end

-- Gets the x component of the vector.
---@return number
function Vector2f_ud:x() end

-- Sets the x component of the vector.
---@param value number
function Vector2f_ud:x(value) end

-- Rotates the vector in-place by the specified angle.
---@param angle_rad number -- The angle to rotate by, in radians.
function Vector2f_ud:rotate(angle_rad) end

-- Checks if both components of the vector are zero.
---@return boolean
function Vector2f_ud:is_zero() end

-- Checks if either component of the vector is infinite.
---@return boolean
function Vector2f_ud:is_inf() end

-- Checks if either component of the vector is Not a Number (NaN).
---@return boolean
function Vector2f_ud:is_nan() end

-- Normalizes the vector to a unit length (magnitude of 1). The vector is modified in-place.
function Vector2f_ud:normalize() end

-- Calculates the length (magnitude) of this vector: sqrt(x^2 + y^2).
---@return number
function Vector2f_ud:length() end

-- Calculates the angle of this vector from North, in radians.
---@return number -- Angle in radians, in the range of -PI to +PI.
function Vector2f_ud:angle() end

-- Represents a 3D vector with x, y, and z floating point components.
-- Often used to represent positions, velocities, and attitudes in the North-East-Down (NED) frame.
---@class (exact) Vector3f_ud
---@operator add(Vector3f_ud): Vector3f_ud
---@operator sub(Vector3f_ud): Vector3f_ud
local Vector3f_ud = {}

-- Creates a new Vector3f object, initialized to (0,0,0).
---@return Vector3f_ud
function Vector3f() end

-- Creates a copy of this Vector3f, returning a new object.
---@return Vector3f_ud
function Vector3f_ud:copy() end

-- Gets the z component of the vector (typically Down).
---@return number
function Vector3f_ud:z() end

-- Sets the z component of the vector.
---@param value number
function Vector3f_ud:z(value) end

-- Gets the y component of the vector (typically East).
---@return number
function Vector3f_ud:y() end

-- Sets the y component of the vector.
---@param value number
function Vector3f_ud:y(value) end

-- Gets the x component of the vector (typically North).
---@return number
function Vector3f_ud:x() end

-- Sets the x component of the vector.
---@param value number
function Vector3f_ud:x(value) end

-- Returns a new Vector3f with the same direction but scaled length.
---@param scale_factor number
---@return Vector3f_ud
function Vector3f_ud:scale(scale_factor) end

-- Calculates the cross product of this vector and another. The result is a new vector perpendicular to both original vectors.
---@param vector Vector3f_ud -- The other vector.
---@return Vector3f_ud -- The resulting vector.
function Vector3f_ud:cross(vector) end

-- Calculates the dot product of this vector and another.
---@param vector Vector3f_ud -- The other vector.
---@return number -- The resulting scalar value.
function Vector3f_ud:dot(vector) end

-- Checks if all components of the vector are zero.
---@return boolean
function Vector3f_ud:is_zero() end

-- Checks if any component of the vector is infinite.
---@return boolean
function Vector3f_ud:is_inf() end

-- Checks if any component of the vector is Not a Number (NaN).
---@return boolean
function Vector3f_ud:is_nan() end

-- Normalizes the vector to a unit length (magnitude of 1). The vector is modified in-place.
function Vector3f_ud:normalize() end

-- Calculates the length (magnitude) of this vector: sqrt(x^2 + y^2 + z^2).
---@return number
function Vector3f_ud:length() end

-- Computes the angle between this vector and another vector.
---@param v2 Vector3f_ud -- The other vector.
---@return number -- The angle in radians.
function Vector3f_ud:angle(v2) end

-- Rotates the vector in the XY (North-East) plane, leaving the Z (Down) component untouched. The vector is modified in-place.
---@param param1 number -- The rotation angle in radians.
function Vector3f_ud:rotate_xy(param1) end

-- Returns the x and y components of this vector as a new Vector2f object.
---@return Vector2f_ud
function Vector3f_ud:xy() end

-- Represents a 3D rotation in space. Quaternions are used internally for attitude representation as they avoid issues like gimbal lock.
---@class (exact) Quaternion_ud
---@operator mul(Quaternion_ud): Quaternion_ud
local Quaternion_ud = {}

---@return Quaternion_ud
function Quaternion() end

-- Gets the q4 component of the quaternion.
---@return number
function Quaternion_ud:q4() end

-- Sets the q4 component.
---@param value number
function Quaternion_ud:q4(value) end

-- Gets the q3 component of the quaternion.
---@return number
function Quaternion_ud:q3() end

-- Sets the q3 component.
---@param value number
function Quaternion_ud:q3(value) end

-- Gets the q2 component of the quaternion.
---@return number
function Quaternion_ud:q2() end

-- Sets the q2 component.
---@param value number
function Quaternion_ud:q2(value) end

-- Gets the q1 component of the quaternion.
---@return number
function Quaternion_ud:q1() end

-- Sets the q1 component.
---@param value number
function Quaternion_ud:q1(value) end

-- Rotates the provided Vector3f from the earth frame (e.g., NED) to the vehicle's body frame using this quaternion's rotation. The vector is modified in-place.
---@param vec Vector3f_ud
function Quaternion_ud:earth_to_body(vec) end

-- Returns a new quaternion representing the opposite rotation.
---@return Quaternion_ud
function Quaternion_ud:inverse() end

-- Integrates an angular velocity over a small time delta to update this quaternion's rotation.
---@param angular_velocity Vector3f_ud -- The angular velocity in radians/second.
---@param time_delta number -- The time step in seconds.
function Quaternion_ud:from_angular_velocity(angular_velocity, time_delta) end

-- Constructs this quaternion from a rotation axis and an angle.
---@param axis Vector3f_ud -- The axis of rotation.
---@param angle number -- The angle of rotation in radians.
function Quaternion_ud:from_axis_angle(axis, angle) end

-- Converts this quaternion to an axis-angle representation. The provided vector is modified in-place.
---@param axis_angle Vector3f_ud -- A Vector3f that will be populated with the axis-angle data.
function Quaternion_ud:to_axis_angle(axis_angle) end

-- Constructs this quaternion from roll, pitch, and yaw Euler angles.
---@param roll number -- Roll angle in radians.
---@param pitch number -- Pitch angle in radians.
---@param yaw number -- Yaw angle in radians.
function Quaternion_ud:from_euler(roll, pitch, yaw) end

-- Returns the yaw component of this quaternion's rotation in radians.
---@return number
function Quaternion_ud:get_euler_yaw() end

-- Returns the pitch component of this quaternion's rotation in radians.
---@return number
function Quaternion_ud:get_euler_pitch() end

-- Returns the roll component of this quaternion's rotation in radians.
---@return number
function Quaternion_ud:get_euler_roll() end

-- Normalizes the quaternion to have a length of 1. This is important to prevent floating-point drift from accumulating and creating invalid rotations.
function Quaternion_ud:normalize() end

-- Returns the length (norm) of the quaternion. Should be 1.0 for a valid rotation quaternion.
---@return number
function Quaternion_ud:length() end

-- A userdata object that holds a geographic coordinate, including latitude, longitude, and altitude.
-- This object is fundamental for any navigation or position-aware scripting.
-- It's crucial to be aware of the altitude frame (e.g., relative to home, absolute, terrain) for safe operation.
---@class (exact) Location_ud
local Location_ud = {}

-- Creates a new, empty Location object. All fields will be uninitialized.
---@return Location_ud
function Location() end

-- Creates a new Location object that is a deep copy of this one.
-- This is important to avoid modifying the original object when you only intend to work with a temporary copy.
---@return Location_ud -- A new Location object with the same latitude, longitude, and altitude values.
function Location_ud:copy() end

-- Gets whether the location is a loiter point that requires cross-track tracking from the center.
---@return boolean -- True if the aircraft should track from the center point, false if it should track from the exit location.
function Location_ud:loiter_xtrack() end

-- Sets the loiter cross-track behavior for this location.
---@param value boolean -- Set to true to track from the center, false to track from the exit.
function Location_ud:loiter_xtrack(value) end

-- Gets whether the altitude for this location is relative to the EKF origin.
---@return boolean -- True if altitude is relative to the EKF origin.
function Location_ud:origin_alt() end

-- Sets the altitude frame to be relative to the EKF origin.
---@param value boolean -- Set to true to make the altitude relative to the EKF origin.
function Location_ud:origin_alt(value) end

-- Gets whether the altitude for this location is relative to the terrain height.
---@return boolean -- True if altitude is relative to terrain.
function Location_ud:terrain_alt() end

-- Sets the altitude frame to be relative to terrain height.
---@param value boolean -- Set to true to make the altitude relative to terrain.
function Location_ud:terrain_alt(value) end

-- Gets whether the altitude for this location is relative to the home position.
---@return boolean -- True if altitude is relative to home.
function Location_ud:relative_alt() end

-- Sets the altitude frame to be relative to the home position.
---@param value boolean -- Set to true to make the altitude relative to home.
function Location_ud:relative_alt(value) end

-- Gets the altitude component of the location.
---@return integer -- Altitude in **centimeters**. The frame (e.g., AMSL, relative to home) must be checked separately using `get_alt_frame()`.
function Location_ud:alt() end

-- Sets the altitude component of the location.
---@param value integer -- Altitude in **centimeters**.
function Location_ud:alt(value) end

-- Gets the longitude component of the location.
---@return integer -- Longitude in degrees, multiplied by 1e7.
function Location_ud:lng() end

-- Sets the longitude component of the location.
---@param value integer -- Longitude in degrees, multiplied by 1e7.
function Location_ud:lng(value) end

-- Gets the latitude component of the location.
---@return integer -- Latitude in degrees, multiplied by 1e7.
function Location_ud:lat() end

-- Sets the latitude component of the location.
---@param value integer -- Latitude in degrees, multiplied by 1e7.
function Location_ud:lat(value) end

-- Gets the altitude frame of this location. It is critical to check this to understand what the `alt()` value represents.
---@return integer -- The altitude frame enum value.
---| '0' # ABSOLUTE (Above Mean Sea Level)
---| '1' # ABOVE_HOME
---| '2' # ABOVE_ORIGIN (EKF Origin)
---| '3' # ABOVE_TERRAIN
function Location_ud:get_alt_frame() end

-- Changes the altitude frame of this location, automatically converting the altitude value.
---@param desired_frame integer  -- The target altitude frame enum value.
---| '0' # ABSOLUTE
---| '1' # ABOVE_HOME
---| '2' # ABOVE_ORIGIN
---| '3' # ABOVE_TERRAIN
---@return boolean -- True on success.
function Location_ud:change_alt_frame(desired_frame) end

-- Calculates the North and East distance from this location to another.
---@param loc Location_ud -- The other location to compare against.
---@return Vector2f_ud -- A Vector2f object where x=North, y=East, in meters.
function Location_ud:get_distance_NE(loc) end

-- Calculates the North, East, and Down distance from this location to another. A very common function for relative position calculations.
-- Example from `copter_pingpong.lua`: `local vec_from_home = home:get_distance_NED(curr_loc)`
---@param loc Location_ud -- The other location to compare against.
---@return Vector3f_ud -- A Vector3f object where x=North, y=East, z=Down, in meters.
function Location_ud:get_distance_NED(loc) end

-- Calculates the relative bearing from this location to another.
---@param loc Location_ud -- The target location.
---@return number -- The bearing in **radians**.
function Location_ud:get_bearing(loc) end

-- Returns the offset from the EKF origin to this location in the North-East-Up (NEU) frame.
-- Returns nil if the EKF origin is not available.
---@return Vector3f_ud|nil -- A Vector3f where x=North, y=East, z=Up, in **centimeters**.
function Location_ud:get_vector_from_origin_NEU_cm() end

-- Returns the offset from the EKF origin to this location in the North-East-Up (NEU) frame.
-- Returns nil if the EKF origin is not available.
---@return Vector3f_ud|nil -- A Vector3f where x=North, y=East, z=Up, in **meters**.
function Location_ud:get_vector_from_origin_NEU_m() end

--- Deprecated method returning offset from EKF origin. Use `get_vector_from_origin_NEU_cm` or `get_vector_from_origin_NEU_m` instead.
---@return Vector3f_ud|nil -- Vector between origin and location north east up in centimetres.
---@deprecated
function Location_ud:get_vector_from_origin_NEU() end

-- Modifies this location in-place by moving it a specified distance along a given bearing.
---@param bearing_deg number -- The bearing in degrees (0-360).
---@param distance number -- The distance to move in meters.
function Location_ud:offset_bearing(bearing_deg, distance) end

-- Modifies this location in-place by moving it a specified distance along a given bearing and pitch angle.
---@param bearing_deg number -- The bearing in degrees (0-360).
---@param pitch_deg number -- The pitch angle in degrees.
---@param distance number -- The distance to move in meters.
function Location_ud:offset_bearing_and_pitch(bearing_deg, pitch_deg, distance) end

-- Modifies this location in-place by moving it by the specified North and East distances.
---@param ofs_north number -- The North offset in meters.
---@param ofs_east number -- The East offset in meters.
function Location_ud:offset(ofs_north, ofs_east) end

-- Calculates the horizontal (2D) distance between this location and another.
---@param loc Location_ud -- The other location to compare against.
---@return number -- The horizontal distance in meters.
function Location_ud:get_distance(loc) end

-- Represents a scripting backend for an EFI (Electronic Fuel Injection) system.
-- This is an advanced object used by custom EFI driver scripts (like `EFI_HFE.lua`) to feed EFI data into the main ArduPilot system.
---@class (exact) AP_EFI_Backend_ud
local AP_EFI_Backend_ud = {}

-- Pushes a populated EFI_State object into the ArduPilot EFI system.
-- This function is called by a custom EFI driver script after it has received and decoded data from the physical EFI unit.
---@param state EFI_State_ud -- The EFI_State object containing the latest telemetry data to be processed by ArduPilot.
---@return boolean -- True on success.
function AP_EFI_Backend_ud:handle_scripting(state) end

-- Represents a buffered interface to a raw CAN bus, used for sending and receiving CAN frames.
---@class (exact) ScriptingCANBuffer_ud
local ScriptingCANBuffer_ud = {}

-- Reads a single CAN frame from the buffer if one is available.
---@return CANFrame_ud|nil -- A CANFrame object, or nil if the buffer is empty.
function ScriptingCANBuffer_ud:read_frame() end

-- Adds a filter to the CAN buffer to only receive specific CAN frames.
-- The mask is bitwise ANDed with an incoming frame's ID, and the result is compared to the value. If they match, the frame is buffered.
-- By default, no filters are added, and all frames are buffered. Writing is not affected by filters.
---@param mask uint32_t_ud|integer|number -- The bitmask to apply to incoming frame IDs.
---@param value uint32_t_ud|integer|number -- The value to compare against after masking.
---@return boolean -- Returns true if the filter was added successfully (max 8 filters).
function ScriptingCANBuffer_ud:add_filter(mask, value) end

-- Writes a CAN frame to the bus.
---@param frame CANFrame_ud -- The CANFrame object to be sent.
---@param timeout_us uint32_t_ud|integer|number -- The time to wait in microseconds if the send buffer is full.
---@return boolean -- True if the frame was sent successfully.
function ScriptingCANBuffer_ud:write_frame(frame, timeout_us) end


-- Represents a single analog input pin, used for reading voltages.
---@class (exact) AP_HAL__AnalogSource_ud
local AP_HAL__AnalogSource_ud = {}

-- Returns the ratiometric voltage average, scaled against the board's VCC.
---@return number -- The ratiometric voltage.
function AP_HAL__AnalogSource_ud:voltage_average_ratiometric() end

-- Returns the most recent single raw voltage reading from the pin.
---@return number -- The latest voltage reading.
function AP_HAL__AnalogSource_ud:voltage_latest() end

-- Returns a low-pass filtered average of the voltage readings from the pin.
---@return number -- The averaged voltage.
function AP_HAL__AnalogSource_ud:voltage_average() end

-- Sets the physical pin to be used for this analog input source.
---@param pin_number integer -- The pin number (e.g., 15 for a Pixhawk's ADC 3.3V pin).
---@return boolean -- True on success.
function AP_HAL__AnalogSource_ud:set_pin(pin_number) end


-- Represents a specific device on the I2C bus.
-- This object is obtained from `i2c:get_device()` and is used for all subsequent communication with that device.
---@class (exact) AP_HAL__I2CDevice_ud
local AP_HAL__I2CDevice_ud = {}

-- Sets the address for this I2C device handle.
---@param address integer -- The new 7-bit I2C device address.
function AP_HAL__I2CDevice_ud:set_address(address) end

-- Performs a combined write/read I2C transaction.
-- This is a flexible, low-level function for complex interactions that are not simple register reads/writes.
---@param data_str string -- A binary string of bytes to write to the device.
---@param read_length integer -- The number of bytes to read back from the device after the write.
---@return string|nil -- A binary string of the bytes read, or nil on error.
function AP_HAL__I2CDevice_ud:transfer(data_str, read_length) end

-- Reads one or more registers from the I2C device.
---@param register_num integer -- The starting register address to read from.
---@param read_length? integer -- Optional. The number of registers to read. If not provided, a single register is read.
---@return integer|table|nil -- If `read_length` is omitted, returns a single integer value. If `read_length` is provided, returns a table of integer values. Returns nil on error.
function AP_HAL__I2CDevice_ud:read_registers(register_num, read_length) end

-- Writes a single byte to a specific register on the I2C device.
---@param register_num integer -- The register address to write to.
---@param value integer -- The single byte value (0-255) to write.
---@return boolean -- True on success.
function AP_HAL__I2CDevice_ud:write_register(register_num, value) end

-- Sets the number of times to retry a transaction on a bus error.
---@param retries integer -- The number of retries.
function AP_HAL__I2CDevice_ud:set_retries(retries) end


-- Provides an interface for reading from and writing to a serial port (UART).
-- This can be used to communicate with external sensors or devices like GPS, Lidar, or companion computers.
---@class (exact) AP_Scripting_SerialAccess_ud
local AP_Scripting_SerialAccess_ud = {}

-- Initializes the serial port with a given baud rate. This must be called before any other operations.
---@param baud_rate uint32_t_ud|integer|number -- The desired baud rate (e.g., 57600, 115200).
function AP_Scripting_SerialAccess_ud:begin(baud_rate) end

-- Configures the parity for the serial port.
---@param parity integer -- 0 for None, 1 for Odd, 2 for Even.
function AP_Scripting_SerialAccess_ud:configure_parity(parity) end

-- Configures the number of stop bits for the serial port.
---@param stop_bits integer -- 1 or 2.
function AP_Scripting_SerialAccess_ud:set_stop_bits(stop_bits) end

-- Writes a single byte to the serial port.
---@param value integer -- The byte to write (0-255).
---@return uint32_t_ud -- 1 if the byte was successfully written to the buffer, 0 otherwise.
function AP_Scripting_SerialAccess_ud:write(value) end

-- Writes a string of bytes to the serial port.
---@param data string -- The string of bytes to write.
---@return integer -- The number of bytes actually written, which may be less than the string length if the buffer is full.
function AP_Scripting_SerialAccess_ud:writestring(data) end

-- Reads a single byte from the serial port's receive buffer.
---@return integer -- The byte read (0-255), or -1 if no byte is available.
function AP_Scripting_SerialAccess_ud:read() end

-- Reads multiple bytes from the serial port's receive buffer.
---@param count integer -- The maximum number of bytes to read.
---@return string|nil -- A string containing the bytes actually read (which may be 0-length), or nil on error.
function AP_Scripting_SerialAccess_ud:readstring(count) end

-- Returns the number of bytes available to be read from the serial port's receive buffer.
---@return uint32_t_ud
function AP_Scripting_SerialAccess_ud:available() end

-- Configures hardware flow control (RTS/CTS) for the serial port.
---@param flow_control_setting integer
---| '0' # Disabled
---| '1' # Enabled
---| '2' # Auto
function AP_Scripting_SerialAccess_ud:set_flow_control(flow_control_setting) end


-- Represents a single RC input channel.
---@class (exact) RC_Channel_ud
local RC_Channel_ud = {}

-- Returns the normalized input (-1 to 1) of the channel, ignoring the trim value.
---@return number
function RC_Channel_ud:norm_input_ignore_trim() end

-- Overrides the channel's output with a specific PWM value. This is used for scripting-based control.
---@param PWM integer -- The PWM value to send (typically 1000-2000).
function RC_Channel_ud:set_override(PWM) end

-- For channels configured as switches, this returns the switch position.
---@return integer -- 0 for low, 1 for middle, 2 for high position.
function RC_Channel_ud:get_aux_switch_pos() end

-- Returns the normalized input (-1 to 1) of the channel, centered on the trim value and ignoring the deadzone.
---@return number
function RC_Channel_ud:norm_input() end

-- Returns the normalized input (-1 to 1) of the channel, centered on the trim value. Returns zero when the stick is within the deadzone.
---@return number
function RC_Channel_ud:norm_input_dz() end

-- Interface for controlling a winch peripheral.
winch = {}

-- Gets the maximum configured rate for the winch.
---@return number -- The maximum rate.
function winch:get_rate_max() end

-- Sets the desired rate of the winch.
---@param param1 number -- The desired rate.
function winch:set_desired_rate(param1) end

-- Commands the winch to release a specific length of line.
---@param param1 number -- The length to release in meters.
function winch:release_length(param1) end

-- Commands the winch to relax, releasing tension.
function winch:relax() end

-- Returns true if the winch is healthy and responding.
---@return boolean
function winch:healthy() end

-- Interface for interacting with the I/O MCU (if present).
iomcu = {}

-- Checks if the I/O MCU is healthy and communicating with the main flight controller.
---@return boolean
function iomcu:healthy() end

-- Interface for accessing compass (magnetometer) data.
compass = {}

-- Checks if a specific compass instance is healthy.
---@param instance integer -- The 0-based index of the compass instance to check (e.g., 0 for the first compass).
---@return boolean -- True if the compass is healthy.
function compass:healthy(instance) end

-- The main interface for interacting with a camera connected to the autopilot.
-- This object allows for triggering the shutter, starting/stopping video recording, and querying the camera's state.
-- It operates on the MAVLink camera protocol, so the connected camera must support it.
camera = {}

-- Sets the distance the vehicle must travel before the camera is triggered again. Used for distance-based survey missions.
---@param instance integer -- The 0-indexed camera instance.
---@param distance_m number -- The distance in meters.
function camera:set_trigger_distance(instance, distance_m) end

-- Starts or stops video recording.
---@param instance integer -- The 0-indexed camera instance.
---@param start_recording boolean -- True to start recording, false to stop.
---@return boolean -- True on success.
function camera:record_video(instance, start_recording) end

-- Commands the camera to take a single picture.
---@param instance integer -- The 0-indexed camera instance.
function camera:take_picture(instance) end

-- A data structure holding a snapshot of the camera's current state.
---@class (exact) AP_Camera__camera_state_t_ud
local AP_Camera__camera_state_t_ud = {}

-- get field: The top-left point of the tracking rectangle.
---@return Vector2f_ud
function AP_Camera__camera_state_t_ud:tracking_p1() end

-- get field: The bottom-right point of the tracking rectangle.
---@return Vector2f_ud
function AP_Camera__camera_state_t_ud:tracking_p2() end

-- get field: The type of tracking mode (e.g., pointing, tracking rectangle).
---@return integer
function AP_Camera__camera_state_t_ud:tracking_type() end

-- get field: The current focus value.
---@return number
function AP_Camera__camera_state_t_ud:focus_value() end

-- get field: The type of focus (e.g., auto, manual, percentage).
---@return integer
function AP_Camera__camera_state_t_ud:focus_type() end

-- get field: The current zoom value.
---@return number
function AP_Camera__camera_state_t_ud:zoom_value() end

-- get field: The type of zoom (e.g., optical, digital, percentage).
---@return integer
function AP_Camera__camera_state_t_ud:zoom_type() end

-- get field: True if the camera is currently recording video.
---@return boolean
function AP_Camera__camera_state_t_ud:recording_video() end

-- get field: A counter that increments each time a picture is taken.
---@return integer
function AP_Camera__camera_state_t_ud:take_pic_incr() end

-- Gets a snapshot of the camera's current state.
---@param instance integer -- The 0-indexed camera instance.
---@return AP_Camera__camera_state_t_ud|nil -- A camera state object, or nil if the camera is not available.
function camera:get_state(instance) end

-- Changes a specific camera setting. This is typically used for thermal cameras or other advanced payloads.
---@param instance integer -- The 0-indexed camera instance.
---@param setting integer -- The setting to change, based on the `AP_Camera::Camera_Setting` enum.
---| '0' # THERMAL_PALETTE
---| '1' # THERMAL_GAIN
---| '2' # THERMAL_RAW_DATA
---@param value number -- The new value for the setting.
---@return boolean -- True on success.
function camera:change_setting(instance, setting, value) end

-- A data structure representing the MAVLink CAMERA_INFORMATION message.
-- Used by custom camera driver scripts to define a camera's capabilities to the GCS.
---@class (exact) mavlink_camera_information_t_ud
local mavlink_camera_information_t_ud = {}

---@return mavlink_camera_information_t_ud
function mavlink_camera_information_t() end

-- get field
---@return uint32_t_ud
function mavlink_camera_information_t_ud:time_boot_ms() end

-- set field
---@param value uint32_t_ud|integer|number
function mavlink_camera_information_t_ud:time_boot_ms(value) end

-- get field
---@return uint32_t_ud
function mavlink_camera_information_t_ud:firmware_version() end

-- set field
---@param value uint32_t_ud|integer|number
function mavlink_camera_information_t_ud:firmware_version(value) end

-- get field
---@return number
function mavlink_camera_information_t_ud:focal_length() end

-- set field
---@param value number
function mavlink_camera_information_t_ud:focal_length(value) end

-- get field
---@return number
function mavlink_camera_information_t_ud:sensor_size_h() end

-- set field
---@param value number
function mavlink_camera_information_t_ud:sensor_size_h(value) end

-- get field
---@return number
function mavlink_camera_information_t_ud:sensor_size_v() end

-- set field
---@param value number
function mavlink_camera_information_t_ud:sensor_size_v(value) end

-- get field
---@return uint32_t_ud
function mavlink_camera_information_t_ud:flags() end

-- set field
---@param value uint32_t_ud|integer|number
function mavlink_camera_information_t_ud:flags(value) end

-- get field
---@return integer
function mavlink_camera_information_t_ud:resolution_h() end

-- set field
---@param value integer
function mavlink_camera_information_t_ud:resolution_h(value) end

-- get field
---@return integer
function mavlink_camera_information_t_ud:resolution_v() end

-- set field
---@param value integer
function mavlink_camera_information_t_ud:resolution_v(value) end

-- get field
---@return integer
function mavlink_camera_information_t_ud:cam_definition_version() end

-- set field
---@param value integer
function mavlink_camera_information_t_ud:cam_definition_version(value) end

-- get array field: Vendor name as a C-style string (array of ASCII bytes).
---@param index integer
---@return integer
function mavlink_camera_information_t_ud:vendor_name(index) end

-- set array field
---@param index integer
---@param value integer
function mavlink_camera_information_t_ud:vendor_name(index, value) end

-- get array field: Model name as a C-style string (array of ASCII bytes).
---@param index integer
---@return integer
function mavlink_camera_information_t_ud:model_name(index) end

-- set array field
---@param index integer
---@param value integer
function mavlink_camera_information_t_ud:model_name(index, value) end

-- get field
---@return integer
function mavlink_camera_information_t_ud:lens_id() end

-- set field
---@param value integer
function mavlink_camera_information_t_ud:lens_id(value) end

-- get array field: URI of the camera definition file.
---@param index integer
---@return integer
function mavlink_camera_information_t_ud:cam_definition_uri(index) end

-- set array field
---@param index integer
---@param value integer
function mavlink_camera_information_t_ud:cam_definition_uri(index, value) end

-- get field
---@return integer
function mavlink_camera_information_t_ud:gimbal_device_id() end

-- set field
---@param value integer
function mavlink_camera_information_t_ud:gimbal_device_id(value) end

-- Populates and sends the CAMERA_INFORMATION message for a custom camera driver.
---@param instance integer -- The 0-indexed camera instance.
---@param cam_info mavlink_camera_information_t_ud -- The populated camera information object.
function camera:set_camera_information(instance, cam_info) end

-- A data structure representing the MAVLink VIDEO_STREAM_INFORMATION message.
-- Used by custom camera driver scripts to define a video stream's properties.
---@class (exact) mavlink_video_stream_information_t_ud
local mavlink_video_stream_information_t_ud = {}

---@return mavlink_video_stream_information_t_ud
function mavlink_video_stream_information_t() end

-- get field
---@return number
function mavlink_video_stream_information_t_ud:framerate() end

-- set field
---@param value number
function mavlink_video_stream_information_t_ud:framerate(value) end

-- get field
---@return uint32_t_ud
function mavlink_video_stream_information_t_ud:bitrate() end

-- set field
---@param value uint32_t_ud|integer|number
function mavlink_video_stream_information_t_ud:bitrate(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:flags() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:flags(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:resolution_h() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:resolution_h(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:resolution_v() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:resolution_v(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:rotation() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:rotation(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:hfov() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:hfov(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:stream_id() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:stream_id(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:count() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:count(value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:type() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:type(value) end

-- get array field
---@param index integer
---@return integer
function mavlink_video_stream_information_t_ud:name(index) end

-- set array field
---@param index integer
---@param value integer
function mavlink_video_stream_information_t_ud:name(index, value) end

-- get array field
---@param index integer
---@return integer
function mavlink_video_stream_information_t_ud:uri(index) end

-- set array field
---@param index integer
---@param value integer
function mavlink_video_stream_information_t_ud:uri(index, value) end

-- get field
---@return integer
function mavlink_video_stream_information_t_ud:encoding() end

-- set field
---@param value integer
function mavlink_video_stream_information_t_ud:encoding(value) end

-- Populates and sends the VIDEO_STREAM_INFORMATION message for a custom camera driver.
---@param instance integer -- The 0-indexed camera instance.
---@param stream_info mavlink_video_stream_information_t_ud -- The populated stream information object.
function camera:set_stream_information(instance, stream_info) end

-- Main interface for controlling a camera gimbal/mount.
mount = {}

-- Sets the attitude of the mount using Euler angles. This is a deprecated function.
-- Prefer `set_angle_target` for more explicit control over the yaw frame.
---@param instance integer -- The 0-indexed mount instance.
---@param roll_deg number -- Roll angle in degrees.
---@param pitch_deg number -- Pitch angle in degrees.
---@param yaw_deg number -- Yaw angle in degrees, relative to the vehicle's body frame (nose is 0).
function mount:set_attitude_euler(instance, roll_deg, pitch_deg, yaw_deg) end

-- Gets the geographic location the mount is currently pointing at (if in ROI mode).
---@param instance integer -- The 0-indexed mount instance.
---@return Location_ud|nil -- A Location object, or nil if not in ROI mode.
function mount:get_location_target(instance) end

-- Gets the current angle target of the mount.
---@param instance integer -- The 0-indexed mount instance.
---@return number|nil roll_deg -- Target roll angle in degrees.
---@return number|nil pitch_deg -- Target pitch angle in degrees.
---@return number|nil yaw_deg -- Target yaw angle in degrees.
---@return boolean|nil yaw_is_earth_frame -- True if yaw is a compass heading, false if it's relative to the vehicle.
function mount:get_angle_target(instance) end

-- Gets the current angular rate target of the mount.
---@param instance integer -- The 0-indexed mount instance.
---@return number|nil roll_degs -- Target roll rate in degrees/second.
---@return number|nil pitch_degs -- Target pitch rate in degrees/second.
---@return number|nil yaw_degs -- Target yaw rate in degrees/second.
---@return boolean|nil yaw_is_earth_frame -- True if yaw rate is in the earth frame, false if in the body frame.
function mount:get_rate_target(instance) end

-- Commands the mount to point at and track a specific geographic location (Region of Interest).
---@param instance integer -- The 0-indexed mount instance.
---@param target_loc Location_ud -- A Location object representing the ROI.
function mount:set_roi_target(instance, target_loc) end

-- Commands the mount to move at a specific angular rate.
---@param instance integer -- The 0-indexed mount instance.
---@param roll_degs number -- Target roll rate in degrees/second.
---@param pitch_degs number -- Target pitch rate in degrees/second.
---@param yaw_degs number -- Target yaw rate in degrees/second.
---@param yaw_is_earth_frame boolean -- If true, yaw_degs is a rate of change of a compass heading. If false, it's relative to the vehicle's frame.
function mount:set_rate_target(instance, roll_degs, pitch_degs, yaw_degs, yaw_is_earth_frame) end

-- Commands the mount to move to a specific angle.
---@param instance integer -- The 0-indexed mount instance.
---@param roll_deg number -- Target roll angle in degrees.
---@param pitch_deg number -- Target pitch angle in degrees.
---@param yaw_deg number -- Target yaw angle in degrees.
---@param yaw_is_earth_frame boolean -- If true, yaw_deg is a compass heading (earth frame). If false, it is relative to the vehicle's nose (body frame).
function mount:set_angle_target(instance, roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame) end

-- Sets the operational mode of the mount.
---@param instance integer -- The 0-indexed mount instance.
---@param mode integer -- The MAV_MOUNT_MODE enum value (e.g., 0:Retract, 1:Neutral, 2:MAVLink Targeting, 3:RC Targeting, 4:GPS Point).
function mount:set_mode(instance, mode) end

-- Gets the current operational mode of the mount.
---@param instance integer -- The 0-indexed mount instance.
---@return integer -- The current MAV_MOUNT_MODE enum value.
function mount:get_mode(instance) end

-- Gets the current attitude of the mount as Euler angles.
---@param instance integer -- The 0-indexed mount instance.
---@return number|nil roll_deg -- Current roll angle in degrees.
---@return number|nil pitch_deg -- Current pitch angle in degrees.
---@return number|nil yaw_bf_deg -- Current yaw angle in degrees, relative to the vehicle's body frame.
function mount:get_attitude_euler(instance) end

-- The main interface for controlling the vehicle's motors.
-- This provides access to the final mixed outputs for throttle, attitude control, and more.
-- It is vehicle-specific and applies to Copter and QuadPlane VTOL motors, not fixed-wing forward motors.
motors = {}

-- Gets the motor interlock state. The interlock is a safety feature that prevents motors from spinning.
-- It is different from the arming state; motors can be armed but not interlocked (e.g., on the ground before throttle is raised).
---@return boolean
---| true  # Motors are active and can spin.
---| false # Motors are inactive and cannot spin.
function motors:get_interlock() end

-- Gets the final commanded lateral (right/left) motor output.
---@return number -- The normalized output, typically -1 to 1.
function motors:get_lateral() end

-- Sets external limits on the attitude controllers.
-- This is an advanced function used to prevent integrator windup when an external system (like a script) is controlling the vehicle's attitude.
---@param roll boolean -- True to limit the roll controller.
---@param pitch boolean -- True to limit the pitch controller.
---@param yaw boolean -- True to limit the yaw controller.
---@param throttle_lower boolean -- True to limit the throttle from going lower.
---@param throttle_upper boolean -- True to limit the throttle from going higher.
function motors:set_external_limits(roll, pitch, yaw, throttle_lower, throttle_upper) end

-- Gets the final commanded forward motor output.
---@return number -- The normalized output, typically -1 to 1.
function motors:get_forward() end

-- Gets the final commanded collective throttle output.
---@return number -- The normalized output, typically 0 to 1.
function motors:get_throttle() end

-- Gets the current spool state of the motors. This is particularly relevant for helicopters and vehicles with complex startup procedures.
---@return integer -- The spool state enum value.
---| '0' # Shut down
---| '1' # Ground idle
---| '2' # Spooling up
---| '3' # Throttle unlimited
---| '4' # Spooling down
function motors:get_spool_state() end

-- Sets the frame string displayed in some GCS interfaces. Useful for custom motor mixers.
---@param param1 string -- The string to display (e.g., "Dynamic example").
function motors:set_frame_string(param1) end

-- Gets the desired spool state requested by the flight controller.
---@return integer -- The desired spool state enum value.
function motors:get_desired_spool_state() end

-- Gets the feedforward component of the yaw controller output.
---@return number
function motors:get_yaw_ff() end

-- Gets the combined P, I, and D components of the yaw controller output.
---@return number
function motors:get_yaw() end

-- Gets the feedforward component of the pitch controller output.
---@return number
function motors:get_pitch_ff() end

-- Gets the combined P, I, and D components of the pitch controller output.
---@return number
function motors:get_pitch() end

-- Gets the feedforward component of the roll controller output.
---@return number
function motors:get_roll_ff() end

-- Gets the combined P, I, and D components of the roll controller output.
---@return number
function motors:get_roll() end

-- Provides information about the currently running ArduPilot firmware version.
FWVersion = {}

-- Gets the git hash of the firmware build.
---@return string
function FWVersion:hash() end

-- Gets the patch version number of the firmware (e.g., the '2' in 4.3.2).
---@return integer
function FWVersion:patch() end

-- Gets the minor version number of the firmware (e.g., the '3' in 4.3.2).
---@return integer
function FWVersion:minor() end

-- Gets the major version number of the firmware (e.g., the '4' in 4.3.2).
---@return integer
function FWVersion:major() end

-- Gets the vehicle type for which the firmware was built. This is critical for scripts that are vehicle-specific.
---@return integer -- The vehicle type enum value.
---| '1' # Rover
---| '2' # ArduCopter
---| '3' # ArduPlane
---| '4' # AntennaTracker
---| '7' # ArduSub
---| '9' # AP_Periph
---| '12' # Blimp
---| '13' # Heli
function FWVersion:type() end

-- Gets the full firmware version string (e.g., "ArduCopter V4.3.2").
---@return string
function FWVersion:string() end


-- Interface for AP_Periph CAN peripheral devices. Allows a peripheral to get information about the main vehicle.
periph = {}

-- Gets the vehicle's state bitmask from the peripheral's point of view.
---@return uint32_t_ud
function periph:get_vehicle_state() end

-- Gets the vehicle's yaw in the earth frame (compass heading).
---@return number
function periph:get_yaw_earth() end

-- Prints a debug message over the CAN bus.
---@param text string
function periph:can_printf(text) end

-- Reboots the peripheral device.
---@param hold_in_bootloader boolean -- If true, the device will reboot into the bootloader for firmware updates.
function periph:reboot(hold_in_bootloader) end

-- Interface for the Inertial Navigation System (INS), providing access to raw sensor data.
ins = {}

-- Gets the temperature of a specific IMU instance.
---@param instance integer -- The 0-indexed IMU instance.
---@return number -- Temperature in degrees Celsius.
function ins:get_temperature(instance) end

-- Checks if the gyroscopes are consistent with each other.
---@param threshold integer -- The allowed threshold in degrees per second.
---@return boolean -- True if the gyros are consistent.
function ins:gyros_consistent(threshold) end

-- Checks if a specific gyroscope sensor is healthy.
---@param instance integer -- The 0-indexed gyroscope instance.
---@return boolean
function ins:get_gyro_health(instance) end

-- Checks if the accelerometers are consistent with each other.
---@param threshold number -- The threshold allowed before returning false.
---@return boolean -- True if the accelerometers are consistent.
function ins:accels_consistent(threshold) end

-- Checks if a specific accelerometer sensor is healthy.
---@param instance integer -- The 0-indexed accelerometer instance.
---@return boolean
function ins:get_accel_health(instance) end

-- Returns true if the INS is currently undergoing calibration.
---@return boolean
function ins:calibrating() end

-- Gets the raw rotational rates from a specific gyroscope.
---@param instance integer -- The 0-indexed gyroscope instance.
---@return Vector3f_ud -- A Vector3f of angular velocities in radians/second.
function ins:get_gyro(instance) end

-- Gets the raw acceleration values from a specific accelerometer.
---@param instance integer -- The 0-indexed accelerometer instance.
---@return Vector3f_ud -- A Vector3f of accelerations in meters/second^2.
function ins:get_accel(instance) end

-- Interface for the dynamic, scriptable motor mixer. Allows for real-time changes to the motor mixing logic.
Motors_dynamic = {}

-- Loads a new set of motor factors into the dynamic mixer.
---@param factor_table motor_factor_table_ud -- A table of motor factors to apply.
function Motors_dynamic:load_factors(factor_table) end

-- Adds a motor to the dynamic mixer configuration.
---@param motor_num integer -- The motor number (0-indexed).
---@param testing_order integer -- The order in which this motor should be tested in the motor test sequence.
function Motors_dynamic:add_motor(motor_num, testing_order) end

-- Initializes the dynamic motor mixer with the specified number of motors. Must be called after adding all motors.
---@param expected_num_motors integer -- The total number of motors in the configuration.
---@return boolean -- True on success.
function Motors_dynamic:init(expected_num_motors) end


-- Interface for the flight controller's analog input pins.
analog = {}

-- Returns the temperature of the main flight controller MCU.
---@return number -- MCU temperature in degrees Celsius.
function analog:mcu_temperature() end

-- Returns the current voltage being supplied to the MCU.
---@return number -- MCU voltage.
function analog:mcu_voltage() end

-- Gets a handle to an analog input channel.
---@return AP_HAL__AnalogSource_ud|nil -- An analog source object, or nil if none are available.
function analog:channel() end


-- Interface for controlling General Purpose Input/Output (GPIO) pins.
gpio = {}

-- Sets the mode of a specific GPIO pin.
---@param pin_number integer -- The GPIO pin number.
---@param mode integer
---| '0' # Input
---| '1' # Output
function gpio:pinMode(pin_number, mode) end

-- Toggles the state of a GPIO pin configured as an output.
---@param pin_number integer
function gpio:toggle(pin_number) end

-- Writes a value to a GPIO pin configured as an output.
---@param pin_number integer
---@param value integer
---| '0' # Low
---| '1' # High
function gpio:write(pin_number, value) end

-- Reads the state of a GPIO pin configured as an input.
---@param pin_number integer
---@return boolean -- The pin's state (true for high, false for low).
function gpio:read(pin_number) end

-- Sets the full pin mode, including options like pull-up/pull-down resistors. (Advanced)
---@param pin_number integer
---@param mode uint32_t_ud|integer|number -- The full mode value (platform-specific).
function gpio:set_mode(pin_number, mode) end

-- Gets the full pin mode. (Advanced)
---@param pin_number integer
---@return uint32_t_ud|nil -- The full pin mode value.
function gpio:get_mode(pin_number) end

-- Deprecated alias for set_mode.
---@param pin_number integer
---@param mode uint32_t_ud|integer|number
function gpio:setPinFullMode(pin_number, mode) end

-- Deprecated alias for get_mode.
---@param pin_number integer
---@return uint32_t_ud|nil
function gpio:getPinFullMode(pin_number) end


-- Interface for the 6-Degrees-of-Freedom (6DoF) motor mixer.
-- Used for advanced vehicle types like omnidirectional copters.
Motors_6DoF = {}

-- Adds a motor to the 6DoF mixer, defining its contribution to each axis of control.
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

-- Initializes the 6DoF motor mixer.
---@param expected_num_motors integer -- The total number of motors.
---@return boolean -- True on success.
function Motors_6DoF:init(expected_num_motors) end


-- Interface for direct interaction with the attitude controller.
attitude_control = {}

-- Sets a persistent roll and pitch offset to the attitude controller.
-- Useful for compensating for a known center of gravity offset.
---@param roll_deg number -- Roll offset in degrees.
---@param pitch_deg number -- Pitch offset in degrees.
function attitude_control:set_offset_roll_pitch(roll_deg, pitch_deg) end

-- Enables or disables the forward/backward (pitch) component of the 6DoF controller.
---@param bool boolean
function attitude_control:set_forward_enable(bool) end

-- Enables or disables the lateral (roll) component of the 6DoF controller.
---@param bool boolean
function attitude_control:set_lateral_enable(bool) end


-- Interface for sending FrSky S.Port telemetry data.
frsky_sport = {}

-- Prepares a number for transmission over S.Port by packing it into the required format.
---@param number integer
---@param digits integer
---@param power integer
---@return integer
function frsky_sport:prep_number(number, digits, power) end

-- Pushes a telemetry packet onto the S.Port bus.
---@param sensor integer -- The sensor ID to send from.
---@param frame integer -- The data frame type.
---@param appid integer -- The application ID (data ID).
---@param data integer -- The 32-bit data value to send.
---@return boolean -- True on success.
function frsky_sport:sport_telemetry_push(sensor, frame, appid, data) end


-- Interface for the static, scriptable motor mixer.
-- This allows a script to define a custom, fixed motor layout at startup.
MotorsMatrix = {}

-- Sets the throttle factor for a specific motor in the matrix.
---@param motor_num integer -- The motor number (0-indexed).
---@param throttle_factor number -- The new throttle factor.
---@return boolean -- True on success.
function MotorsMatrix:set_throttle_factor(motor_num, throttle_factor) end

-- Adds a motor to the static mixer, defining its roll, pitch, and yaw factors.
---@param motor_num integer -- The motor number (0-indexed).
---@param roll_factor number
---@param pitch_factor number
---@param yaw_factor number
---@param testing_order integer -- The order for the motor test sequence.
function MotorsMatrix:add_motor_raw(motor_num, roll_factor, pitch_factor, yaw_factor, testing_order) end

-- Initializes the static motor mixer with the specified number of motors. Must be called after adding all motors.
---@param expected_num_motors integer -- The total number of motors.
---@return boolean -- True on success.
function MotorsMatrix:init(expected_num_motors) end

-- Gets the index (0-based) of a motor that has been identified as failed by the flight controller.
---@return integer -- -1 if no motor has failed.
function MotorsMatrix:get_lost_motor() end

-- Returns true if the flight controller has boosted thrust to compensate for a potential motor failure.
---@return boolean
function MotorsMatrix:get_thrust_boost() end

-- Interface for ArduSub specific functionality.
sub = {}

-- Returns true if a specific joystick button is currently pressed.
---@param index integer -- The 1-indexed button number.
---@return boolean
function sub:is_button_pressed(index) end

-- Gets the number of times a joystick button has been pressed since the last call, then clears the count.
---@param index integer -- The 1-indexed button number.
---@return integer -- The number of presses.
function sub:get_and_clear_button_count(index) end

-- Returns true if the downward-facing rangefinder is healthy and providing a good quality signal.
-- This is a key safety check for altitude-holding modes like Depth Hold.
---@return boolean
function sub:rangefinder_alt_ok() end

-- In SURFTRAK mode, this returns the current target altitude above the seafloor.
---@return number -- Target altitude in centimeters.
function sub:get_rangefinder_target_cm() end

-- In SURFTRAK mode, this sets a new target altitude above the seafloor.
---@param new_target_cm number -- The new target altitude in centimeters.
---@return boolean -- True on success.
function sub:set_rangefinder_target_cm(new_target_cm) end


-- Interface for QuadPlane specific states and actions.
quadplane = {}

-- Returns true if the vehicle is in an assisted flight mode where the quad motors are active to provide stability or lift.
---@return boolean
function quadplane:in_assisted_flight() end

-- Returns true if the vehicle is currently in a VTOL mode (e.g., QLOITER, QHOVER, QLAND).
---@return boolean
function quadplane:in_vtol_mode() end

-- Returns true if the vehicle is in the final descent phase of a VTOL landing.
---@return boolean
function quadplane:in_vtol_land_descent() end

-- Commands the vehicle to abort a VTOL landing and climb back up.
---@return boolean -- True if the abort was successfully initiated.
function quadplane:abort_landing() end


-- Interface to control the primary RGB notify LED.
LED = {}

-- Gets the current color of the notify LED.
---@return integer -- Red component (0-255).
---@return integer -- Green component (0-255).
---@return integer -- Blue component (0-255).
function LED:get_rgb() end


-- Interface for reading the state of physical buttons connected to the flight controller's GPIO pins.
button = {}

-- Returns the state of a specific button.
---@param button_number integer -- The 1-indexed button number.
---@return boolean -- True if the button is currently pressed.
function button:get_button_state(button_number) end


-- Interface for reading data from RPM sensors.
RPM = {}

-- Returns the RPM of a specific sensor instance.
---@param instance integer -- The 1-indexed RPM sensor instance.
---@return number|nil -- The current RPM value, or nil if the sensor is not available or healthy.
function RPM:get_rpm(instance) end


-- The main interface for interacting with the autonomous mission.
-- Allows for reading, modifying, and controlling the mission flow.
mission = {}
mission.MISSION_COMPLETE = enum_integer
mission.MISSION_RUNNING = enum_integer
mission.MISSION_STOPPED = enum_integer

-- Clears all commands from the current mission.
---@return boolean -- True on success.
function mission:clear() end

-- Sets or updates a single mission command at a specific index.
---@param index integer -- The index to write the item to.
---@param item mavlink_mission_item_int_t_ud -- The mission item object to set.
---@return boolean -- True on success.
function mission:set_item(index, item) end

-- Gets a single mission command from a specific index.
---@param index integer -- The index of the item to retrieve.
---@return mavlink_mission_item_int_t_ud|nil -- The mission item object, or nil if the index is invalid.
function mission:get_item(index) end

-- Returns the total number of commands in the mission, including the home location at index 0.
---@return integer
function mission:num_commands() end

-- Returns the command ID of the active "do" command (a non-navigation command).
---@return integer
function mission:get_current_do_cmd_id() end

-- Returns the command ID of the current navigation command.
---@return integer
function mission:get_current_nav_id() end

-- Returns the command ID of the previously executed navigation command.
---@return integer -- Returns 0 (AP_MISSION_CMD_ID_NONE) if there was no previous navigation command.
function mission:get_prev_nav_cmd_id() end

-- Jumps the mission execution to the command at the specified index.
---@param index integer -- The index of the command to jump to.
---@return boolean -- True on success.
function mission:set_current_cmd(index) end

-- Returns the index of the current navigation command.
---@return integer -- Returns 0 if no command is active.
function mission:get_current_nav_index() end

-- Returns the current status of the mission execution.
---@return integer -- The mission state enum value (MISSION_RUNNING, MISSION_COMPLETE, etc.).
function mission:state() end

-- Returns true if the specified command ID involves a geographic location.
---@param cmd integer -- The MAV_CMD ID to check.
---@return boolean
function mission:cmd_has_location(cmd)end

-- Finds the first JUMP_TAG command with the specified tag and jumps the mission execution to it.
---@param tag integer -- The tag value to search for.
---@return boolean -- True on success, false if no matching tag is found.
function mission:jump_to_tag(tag) end

-- Returns the index of the first JUMP_TAG command with the specified tag.
---@param tag integer -- The tag value to search for.
---@return integer -- The index of the command, or 0 if not found.
function mission:get_index_of_jump_tag(tag) end

-- Gets the most recently executed JUMP_TAG and its age.
-- The age is the number of navigation commands that have been completed since the tag was seen.
---@return integer|nil -- The tag value.
---@return integer|nil -- The age of the tag.
function mission:get_last_jump_tag() end


-- Finds the closest landing sequence in the mission and jumps execution to it.
---@return boolean -- True if a landing sequence was found and the jump was successful.
function mission:jump_to_landing_sequence() end

-- Jumps mission execution to the landing abort sequence.
---@return boolean
function mission:jump_to_abort_landing_sequence() end

-- A simple, direct interface for getting and setting parameters by name.
param = {}

-- Sets a parameter's value and saves it to permanent storage (EEPROM). The value will persist after reboot.
---@param name string -- The parameter name (e.g., "ANGLE_MAX").
---@param value number -- The value to set and save.
---@return boolean -- True if the parameter was found.
function param:set_and_save(name, value) end

-- Sets a parameter's value for the current session only. The value will NOT persist after reboot.
---@param name string -- The parameter name.
---@param value number -- The value to set.
---@return boolean -- True if the parameter was found.
function param:set(name, value) end

-- Gets a parameter's current value.
---@param name string -- The parameter name.
---@return number|nil -- The parameter's value, or nil if the name was not found.
function param:get(name) end

-- Sets a parameter's default value. If the user has not changed the parameter, it will be set to this value.
---@param name string -- The parameter name.
---@param value number -- The default value to set.
---@return boolean -- True if the parameter was found.
function param:set_default(name, value) end

-- Creates a new table of script-specific parameters that can be accessed from a GCS.
---@param table_key integer -- A unique key for the table.
---@param prefix string -- The prefix for the parameter names in this table.
---@param num_params integer -- The number of parameters in the table.
---@return boolean -- True on success.
function param:add_table(table_key, prefix, num_params) end

-- Adds a single parameter to a previously created script parameter table.
---@param table_key integer -- The key of the table to add to.
---@param param_num integer -- The 1-indexed number of this parameter within the table.
---@param name string -- The name of the parameter.
---@param default_value number -- The default value.
---@return boolean -- True on success.
function param:add_param(table_key, param_num, name, default_value) end

-- A data structure for holding telemetry data from a single ESC.
---@class (exact) ESCTelemetryData_ud
local ESCTelemetryData_ud = {}

---@return ESCTelemetryData_ud
function ESCTelemetryData() end

-- set motor temperature
---@param value integer -- Temperature in degrees centi-grade * 10
function ESCTelemetryData_ud:motor_temp_cdeg(value) end

-- set consumption
---@param value number -- Consumed energy in mAh.
function ESCTelemetryData_ud:consumption_mah(value) end

-- set current
---@param value number -- Current in Amps.
function ESCTelemetryData_ud:current(value) end

-- set voltage
---@param value number -- Voltage in Volts.
function ESCTelemetryData_ud:voltage(value) end

-- set temperature
---@param value integer -- Temperature in degrees centi-grade * 10
function ESCTelemetryData_ud:temperature_cdeg(value) end

-- Interface for reading and providing ESC (Electronic Speed Controller) telemetry data.
esc_telem = {}

-- Updates the telemetry data for a specific ESC instance. Used by custom ESC driver scripts.
---@param instance integer -- The 0-indexed ESC instance.
---@param telemdata ESCTelemetryData_ud -- The populated telemetry data object.
---@param data_mask integer -- A bitmask indicating which fields in the `telemdata` object are valid.
function esc_telem:update_telem_data(instance, telemdata, data_mask) end

-- Returns an individual ESC’s usage time in seconds.
---@param instance integer -- The 0-indexed ESC instance.
---@return uint32_t_ud|nil -- Usage time in seconds, or nil if not available.
function esc_telem:get_usage_seconds(instance) end

-- Returns the consumed energy for a specific ESC.
---@param instance integer -- The 0-indexed ESC instance.
---@return number|nil -- Consumption in mAh, or nil if not available.
function esc_telem:get_consumption_mah(instance) end

-- Returns the voltage for a specific ESC.
---@param instance integer -- The 0-indexed ESC instance.
---@return number|nil -- Voltage in Volts, or nil if not available.
function esc_telem:get_voltage(instance) end

-- Returns the current for a specific ESC.
---@param instance integer -- The 0-indexed ESC instance.
---@return number|nil -- Current in Amps, or nil if not available.
function esc_telem:get_current(instance) end

-- Returns the motor temperature for a specific ESC.
---@param instance integer -- The 0-indexed ESC instance.
---@return integer|nil -- Temperature in degrees Celsius, or nil if not available.
function esc_telem:get_motor_temperature(instance) end

-- Returns the ESC temperature.
---@param instance integer -- The 0-indexed ESC instance.
---@return integer|nil -- Temperature in degrees Celsius, or nil if not available.
function esc_telem:get_temperature(instance) end

-- Returns the RPM for a specific ESC.
---@param instance integer -- The 0-indexed ESC instance.
---@return number|nil -- RPM, or nil if not available.
function esc_telem:get_rpm(instance) end

-- Updates the RPM for an ESC. Used by custom ESC driver scripts.
---@param esc_index integer -- The 0-indexed ESC instance.
---@param rpm integer -- The new RPM value.
---@param error_rate number -- The error rate.
function esc_telem:update_rpm(esc_index, rpm, error_rate) end

-- Sets the RPM scaling factor for a motor.
---@param esc_index integer -- The 0-indexed ESC instance.
---@param scale_factor number -- The scaling factor.
function esc_telem:set_rpm_scale(esc_index, scale_factor) end

-- Gets the timestamp of the last telemetry data received from an ESC.
---@param esc_index integer
---@return uint32_t_ud
function esc_telem:get_last_telem_data_ms(esc_index) end

-- Interface for the optical flow sensor.
optical_flow = {}

-- Returns the quality metric of the optical flow sensor. Higher numbers are better.
---@return integer -- Quality metric (0-255).
function optical_flow:quality() end

-- Returns true if the optical flow sensor is healthy and providing data.
---@return boolean
function optical_flow:healthy() end

-- Returns true if the optical flow sensor is enabled in parameters.
---@return boolean
function optical_flow:enabled() end

--- Interface for the barometer sensor(s).
baro = {}

-- Gets the temperature from an external barometer sensor, if available.
---@return number -- Temperature in degrees Celsius.
function baro:get_external_temperature() end

-- Gets the temperature from the primary internal barometer sensor.
---@return number -- Temperature in degrees Celsius.
function baro:get_temperature() end

-- Returns the current atmospheric pressure.
---@return number -- Pressure in Pascals. Divide by 100 for millibars or hPa.
function baro:get_pressure() end

-- Gets the current altitude relative to the altitude at the time of the last calibration (typically at boot).
---@return number -- Altitude in meters.
function baro:get_altitude() end

-- Checks if a specific barometer sensor is healthy.
---@param instance integer -- The 0-based index of the barometer instance to check.
---@return boolean -- True if the sensor is healthy.
function baro:healthy(instance) end

-- Calculates the altitude difference between two pressure readings.
---@param base_pressure number -- The first reference pressure in Pascals.
---@param pressure number -- The second pressure in Pascals.
---@return number -- The resulting altitude difference in meters.
function baro:get_altitude_difference(base_pressure,pressure) end

-- Interface for hardware serial ports (UARTs).
serial = {}

-- Returns a serial access object for a port configured for scripting.
-- The corresponding SERIALx_PROTOCOL parameter must be set to 28 (Scripting).
---@param instance integer -- The 0-based index of the scripting port to access.
---@return AP_Scripting_SerialAccess_ud|nil -- An access object for that instance, or nil if not found.
function serial:find_serial(instance) end

-- Returns a serial access object to simulate a device attached via a specific protocol.
-- This allows a script to act as a device (e.g., a GPS) for testing or custom integrations. SCR_SDEV_EN must be enabled.
---@param protocol integer -- The protocol to simulate (e.g., 5 for GPS).
---@param instance integer -- The 0-based index of the protocol instance.
---@return AP_Scripting_SerialAccess_ud|nil -- An access object for that instance, or nil if not found.
function serial:find_simulated_device(protocol, instance) end


-- Interface for the Radio Control (RC) receiver inputs.
rc = {}

-- Gets a specific RC channel object.
---@param chan_num integer -- The 1-indexed channel number.
---@return RC_Channel_ud|nil -- The channel object, or nil if the channel is invalid.
function rc:get_channel(chan_num) end

-- Returns true if the RC receiver has valid input and is not in failsafe.
---@return boolean
function rc:has_valid_input() end

-- Gets the cached state of an auxiliary function switch.
---@param aux_fn integer -- The auxiliary function number.
---@return integer|nil -- The cached switch position (0: low, 1: middle, 2: high).
function rc:get_aux_cached(aux_fn) end

-- Manually triggers an auxiliary function.
---@param aux_fun integer -- The auxiliary function to trigger.
---@param ch_flag integer -- The switch position to simulate (0: low, 1: middle, 2: high).
---@return boolean -- True on success.
function rc:run_aux_function(aux_fun, ch_flag) end

-- Finds the RC channel object assigned to a specific RC_xOPTION.
---@param aux_fun integer -- The RC_xOPTION number to find.
---@return RC_Channel_ud|nil -- The corresponding RC channel object, or nil if none is assigned.
function rc:find_channel_for_option(aux_fun) end

-- Returns the raw PWM input value for a specific channel.
---@param chan_num integer -- The 1-indexed input channel number.
---@return integer|nil -- The raw PWM value in microseconds, or nil if the channel is not available.
function rc:get_pwm(chan_num) end


-- Interface for controlling servo and relay outputs. This is distinct from the RC input object.
SRV_Channels = {}

-- Gets the emergency stop state. If true, all motors are inactive.
---@return boolean
---| true # E-Stop is active.
---| false # E-Stop is not active.
function SRV_Channels:get_emergency_stop() end

-- Gets the safety state (from the safety switch).
---@return boolean
---| true # Disarmed, outputs are inactive.
---| false # Armed, outputs are live.
function SRV_Channels:get_safety_state() end

-- Sets the output range for a servo function.
---@param function_num integer -- The servo function number (from the SERVOx_FUNCTION parameter list).
---@param range integer -- The output range in degrees.
function SRV_Channels:set_range(function_num, range) end

-- Sets the output angle for a servo function.
---@param function_num integer -- The servo function number.
---@param angle integer -- The output angle in degrees.
function SRV_Channels:set_angle(function_num, angle) end

-- Sets a normalized output value (-1 to 1) for a channel assigned the specified servo function.
---@param function_num integer -- The servo function number.
---@param value number -- The normalized output value from -1.0 to 1.0.
function SRV_Channels:set_output_norm(function_num, value) end

-- Gets the scaled output value (typically -1 to 1) for a given servo function.
---@param function_num integer -- The servo function number.
---@return number -- The current scaled output value.
function SRV_Channels:get_output_scaled(function_num) end

-- Returns the raw PWM value of the first output assigned the specified servo function.
---@param function_num integer -- The servo function number.
---@return integer|nil -- The output PWM in microseconds, or nil if no output is assigned that function.
function SRV_Channels:get_output_pwm(function_num) end

-- Sets the scaled output value for a given servo function. The scale is determined by the range set with `set_range` or `set_angle`.
---@param function_num integer -- The servo function number.
---@param value number -- The scaled output value.
function SRV_Channels:set_output_scaled(function_num, value) end

-- Overrides a specific servo output channel with a PWM value for a limited time.
---@param chan integer -- The 0-indexed servo output channel number (e.g., 0 for SERVO1).
---@param pwm integer -- The PWM value to set.
---@param timeout_ms integer -- The duration of the override in milliseconds.
function SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, timeout_ms) end

-- Sets the raw PWM value for a specific servo output channel.
---@param chan integer -- The 0-indexed servo output channel number.
---@param pwm integer -- The PWM value to set.
function SRV_Channels:set_output_pwm_chan(chan, pwm) end

-- Gets the raw PWM value for a specific servo output channel.
---@param chan integer -- The 0-indexed servo output channel number.
---@return integer|nil -- The current output PWM, or nil if not available.
function SRV_Channels:get_output_pwm_chan(chan) end

-- Sets the raw PWM value for a given servo output function.
---@param function_num integer -- The servo function number.
---@param pwm integer -- The PWM value to set.
function SRV_Channels:set_output_pwm(function_num, pwm) end

-- Finds the first servo output channel (0-indexed) assigned to a specific function.
---@param function_num integer -- The servo function number.
---@return integer|nil -- The output channel number (e.g., 0 for SERVO1), or nil if none is assigned.
function SRV_Channels:find_channel(function_num) end


-- This library allows the control of RGB LED strings (like NeoPixel) via a servo output reserved for scripting.
-- The corresponding SERVOx_FUNCTION must be set to a Scripting output (94-109).
serialLED = {}

-- Sends the currently configured RGB values to the physical LED string. This must be called to see any changes made with `set_RGB`.
---@param chan integer -- The scripting output number (1-16) the LEDs are attached to.
---@return boolean -- True if successful.
function serialLED:send(chan) end

-- Sets the color for a single LED or all LEDs in the string.
---@param chan integer -- The scripting output number (1-16).
---@param led_index integer -- The 0-indexed LED number. Use -1 to set all LEDs to the same color.
---@param red integer -- The red component (0-255).
---@param green integer -- The green component (0-255).
---@param blue integer -- The blue component (0-255).
---@return boolean -- True if successful.
function serialLED:set_RGB(chan, led_index, red, green, blue) end

-- Configures the number of LEDs in a 'Profiled' type string.
---@param chan integer -- The scripting output number (1-16).
---@param num_leds integer -- The number of LEDs in the string.
---@return boolean -- True if successful.
function serialLED:set_num_profiled(chan, num_leds) end

-- Configures the number of LEDs in a 'NeoPixel' type string.
---@param chan integer -- The scripting output number (1-16).
---@param num_leds integer -- The number of LEDs in the string.
---@return boolean -- True if successful.
function serialLED:set_num_neopixel(chan, num_leds) end

-- Configures the number of LEDs in an 'RGB NeoPixel' type string.
---@param chan integer -- The scripting output number (1-16).
---@param num_leds integer -- The number of LEDs in the string.
---@return boolean -- True if successful.
function serialLED:set_num_neopixel_rgb(chan, num_leds) end

-- The main vehicle object, providing high-level control and status information.
vehicle = {}

-- Overrides the landing descent rate for 1 second.
---@param rate number -- The descent rate in cm/s.
---@return boolean
function vehicle:set_land_descent_rate(rate) end

-- Sets a rudder offset. (Rover specific)
---@param rudder_pct number -- Rudder offset percentage.
---@param run_yaw_rate_control boolean -- True to run the yaw rate controller.
function vehicle:set_rudder_offset(rudder_pct, run_yaw_rate_control) end

-- Returns true if the EKF has entered a failsafe state.
---@return boolean
function vehicle:has_ekf_failsafed() end

-- Gets the normalized pan and tilt values.
---@return number|nil
---@return number|nil
function vehicle:get_pan_tilt_norm() end

-- Gets the cross-track error from the current waypoint track.
---@return number|nil -- The error in meters.
function vehicle:get_wp_crosstrack_error_m() end

-- Gets the bearing to the current navigation target waypoint.
---@return number|nil -- The bearing in degrees.
function vehicle:get_wp_bearing_deg() end

-- Gets the distance to the current navigation target waypoint.
---@return number|nil -- The distance in meters.
function vehicle:get_wp_distance_m() end

-- Sets the steering and throttle outputs. (Rover specific)
---@param steering number -- Normalized steering (-1 to 1).
---@param throttle number -- Normalized throttle (-1 to 1).
---@return boolean
function vehicle:set_steering_and_throttle(steering, throttle) end

-- Gets the current steering and throttle outputs. (Rover specific)
---@return number|nil -- Steering.
---@return number|nil -- Throttle.
function vehicle:get_steering_and_throttle() end

-- Sets the turn rate for circle mode.
---@param rate_dps number -- The rate of turn in degrees per second.
---@return boolean
function vehicle:set_circle_rate(rate_dps) end

-- Gets the radius of the current circle mode flight path.
---@return number|nil -- The radius in meters.
function vehicle:get_circle_radius() end

-- Sets a target attitude and climb rate.
---@param roll_deg number
---@param pitch_deg number
---@param yaw_deg number
---@param climb_rate_ms number -- Climb rate in m/s.
---@param use_yaw_rate boolean -- True to use yaw_rate_degs instead of yaw_deg.
---@param yaw_rate_degs number -- Target yaw rate in degrees/second.
---@return boolean
function vehicle:set_target_angle_and_climbrate(roll_deg, pitch_deg, yaw_deg, climb_rate_ms, use_yaw_rate, yaw_rate_degs) end

-- Sets target roll, pitch, and yaw rates along with a collective throttle value in guided mode.
---@param roll_rate_dps number -- Target roll rate in degrees per second.
---@param pitch_rate_dps number -- Target pitch rate in degrees per second.
---@param yaw_rate_dps number -- Target yaw rate in degrees per second.
---@param throttle number -- Throttle demand from 0.0 to 1.0.
---@return boolean -- True if successful.
function vehicle:set_target_rate_and_throttle(roll_rate_dps, pitch_rate_dps, yaw_rate_dps, throttle) end

-- Sets the target velocity in the North-East-Down frame for guided modes.
---@param vel_ned Vector3f_ud -- A Vector3f where x=North, y=East, z=Down, in meters/second.
---@return boolean -- True on success.
function vehicle:set_target_velocity_NED(vel_ned) end

-- Sets a target velocity and acceleration in the NED frame.
---@param target_vel Vector3f_ud
---@param target_accel Vector3f_ud
---@param use_yaw boolean
---@param yaw_deg number
---@param use_yaw_rate boolean
---@param yaw_rate_degs number
---@param yaw_relative boolean
---@return boolean
function vehicle:set_target_velaccel_NED(target_vel, target_accel, use_yaw, yaw_deg, use_yaw_rate, yaw_rate_degs, yaw_relative) end

-- Sets a target position, velocity, and acceleration in the NED frame.
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

-- Sets a target position and velocity in the NED frame.
---@param target_pos Vector3f_ud
---@param target_vel Vector3f_ud
---@return boolean
function vehicle:set_target_posvel_NED(target_pos, target_vel) end

-- Sets a target position in the NED frame.
---@param target_pos Vector3f_ud
---@param use_yaw boolean
---@param yaw_deg number
---@param use_yaw_rate boolean
---@param yaw_rate_degs number
---@param yaw_relative boolean
---@param terrain_alt boolean -- If true, the z-component of target_pos is an altitude above terrain.
---@return boolean
function vehicle:set_target_pos_NED(target_pos, use_yaw, yaw_deg, use_yaw_rate, yaw_rate_degs, yaw_relative, terrain_alt) end

-- Updates the target location.
---@param current_target Location_ud -- The current target location, from `get_target_location()`.
---@param new_target Location_ud -- The new target location.
---@return boolean
function vehicle:update_target_location(current_target, new_target) end

-- Gets the current target location if the vehicle is in a mode that uses one.
---@return Location_ud|nil -- The target Location object.
function vehicle:get_target_location() end

-- Sets the target vehicle location for a guided mode.
---@param target_loc Location_ud -- The target Location object.
---@return boolean -- True on success.
function vehicle:set_target_location(target_loc) end

-- Initiates a takeoff in an auto or guided mode. Not supported by all vehicle types.
---@param alt number -- Takeoff altitude in meters.
---@return boolean -- True on success.
function vehicle:get_control_output(control_output) end

-- Returns the time in milliseconds since the autopilot determined it started flying.
---@return uint32_t_ud
function vehicle:get_time_flying_ms() end

-- Returns true if the autopilot believes the vehicle is flying. This is not always guaranteed to be accurate.
---@return boolean
function vehicle:get_likely_flying() end

-- Gets the reason for the current control mode.
---@return integer
function vehicle:get_control_mode_reason() end

-- Returns the current vehicle flight mode number. See https://mavlink.io/en/messages/ardupilotmega.html for vehicle-specific mode enums.
---@return integer
function vehicle:get_mode() end

-- Attempts to change the vehicle's flight mode. Returns true on success.
---@param mode_number integer -- The target mode number.
---@return boolean
function vehicle:set_mode(mode_number) end

-- Sets a target velocity to match.
---@param param1 Vector2f_ud
---@return boolean
function vehicle:set_velocity_match(param1) end

-- Enables or disables navigation control by a script.
---@param param1 integer -- 1 to enable, 0 to disable.
---@return boolean
function vehicle:nav_scripting_enable(param1) end

-- Sets the desired speed for Copter and Rover in navigation modes.
---@param param1 number -- Speed in m/s.
---@return boolean
function vehicle:set_desired_speed(param1) end

-- Sets the desired turn rate and speed. (Rover specific)
---@param param1 number -- Turn rate.
---@param param2 number -- Speed.
---@return boolean
function vehicle:set_desired_turn_rate_and_speed(param1, param2) end

-- Sets the target throttle and attitude rates.
---@param param1 number -- Throttle percentage.
---@param param2 number -- Roll rate in deg/s.
---@param param3 number -- Pitch rate in deg/s.
---@param param4 number -- Yaw rate in deg/s.
function vehicle:set_target_throttle_rate_rpy(param1, param2, param3, param4) end

-- Informs the mission planner that a NAV_SCRIPT_TIME command has completed.
---@param param1 integer -- The unique ID of the completed command.
function vehicle:nav_script_time_done(param1) end

-- Gets the parameters for a currently running NAV_SCRIPT_TIME command.
---@return integer|nil id -- The unique ID of the command.
---@return integer|nil cmd -- The 'command' (param1) field.
---@return number|nil arg1 -- The 'param2' field.
---@return number|nil arg2 -- The 'param3' field.
---@return integer|nil arg3 -- The 'param4' field (x_lat).
---@return integer|nil arg4 -- The 'param4' field (y_lon).
function vehicle:nav_script_time() end

-- Reboots the flight controller.
---@param hold_in_bootloader boolean -- If true, the device will reboot into the bootloader.
function vehicle:reboot(hold_in_bootloader) end

-- Returns true if the vehicle is in the takeoff portion of a mission.
---@return boolean
function vehicle:is_taking_off() end

-- Returns true if the vehicle is in the landing portion of a mission.
---@return boolean
function vehicle:is_landing() end

-- Sets the start point of the current navigation leg for crosstrack calculations.
---@param new_start_location Location_ud -- The new start location.
---@return boolean -- True on success.
function vehicle:set_crosstrack_start(new_start_location) end

-- Registers a custom flight mode. This behaves like a guided mode but will report a custom name and number to the GCS.
---@param number integer -- The custom mode number to use (should be > 100).
---@param full_name string -- The full name of the mode.
---@param short_name string -- A short name for the mode (max 4 characters).
---@return AP_Vehicle__custom_mode_state_ud|nil -- Returns a state object to customize behavior, or nil if registration fails.
function vehicle:register_custom_mode(number, full_name, short_name) end

-- A state object for a registered custom mode, allowing customization of its behavior.
---@class (exact) AP_Vehicle__custom_mode_state_ud
local AP_Vehicle__custom_mode_state_ud = {}

-- Gets whether the vehicle is allowed to enter this custom mode.
---@return boolean
function AP_Vehicle__custom_mode_state_ud:allow_entry() end

-- Sets whether the vehicle is allowed to enter this custom mode.
---@param value boolean
function AP_Vehicle__custom_mode_state_ud:allow_entry(value) end

-- Interface for controlling ONVIF-compliant IP cameras.
-- Allows for network-based control of camera Pan, Tilt, and Zoom (PTZ).
-- See the `ONVIF_Camera_Control.lua` script for a practical example.
onvif = {}

-- Gets the maximum pan and tilt limits of the camera.
---@return Vector2f_ud -- A Vector2f where x is the max pan and y is the max tilt.
function onvif:get_pan_tilt_limit_max() end

-- Gets the minimum pan and tilt limits of the camera.
---@return Vector2f_ud -- A Vector2f where x is the min pan and y is the min tilt.
function onvif:get_pan_tilt_limit_min() end

-- Commands the camera to move to an absolute pan, tilt, and zoom position.
---@param pan number -- The target pan position.
---@param tilt number -- The target tilt position.
---@param zoom number -- The target zoom position.
---@return boolean -- True on success.
function onvif:set_absolutemove(pan, tilt, zoom) end

-- Starts and initializes the connection to the ONVIF camera.
---@param username string -- The username for camera authentication.
---@param password string -- The password for camera authentication.
---@param httphostname string -- The IP address or hostname of the camera.
---@return boolean -- True on success.
function onvif:start(username, password, httphostname) end


-- MAVLink interaction with the Ground Control Station (GCS).
gcs = {}

-- Sends a named float value using the NAMED_VALUE_FLOAT MAVLink message.
-- This is useful for sending custom debug values or telemetry to a GCS that can display them.
---@param name string -- The name of the value, up to 10 characters long.
---@param value number -- The floating-point value to send.
function gcs:send_named_float(name, value) end

-- Sets the message interval for a specific MAVLink message on a given serial port.
-- This is a powerful tool for managing telemetry bandwidth. Use with caution.
---@param port_num integer -- The serial port number (0 for Serial0, 1 for Serial1, etc.).
---@param msg_id uint32_t_ud|integer|number -- The MAVLink message ID to configure (e.g., 33 for GLOBAL_POSITION_INT).
---@param interval_us integer -- The desired interval in microseconds. Set to -1 to restore the default rate.
---@return integer
---| '0' # Accepted
---| '4' # Failed
function gcs:set_message_interval(port_num, msg_id, interval_us) end

-- Gets the MAV_TYPE of the vehicle.
---@return integer -- The MAV_TYPE enum value (e.g., 2 for QUADROTOR, 10 for GROUND_ROVER).
function gcs:frame_type() end

-- Gets the throttle value as a percentage (0-100) as displayed on the GCS HUD.
---@return integer
function gcs:get_hud_throttle() end

-- Enables or disables high latency mode for MAVLink communications.
---@param enabled boolean -- True to enable, false to disable.
function gcs:enable_high_latency_connections(enabled) end

-- Gets the current status of high latency mode.
---@return boolean -- True if high latency mode is enabled.
function gcs:get_high_latency_status() end

-- Sends a text message to the GCS. This is the primary method for providing script feedback to the user.
---@param severity integer -- The severity level of the message (see MAV_SEVERITY enum).
---@param text string -- The text string to send (max 50 characters).
function gcs:send_text(severity, text) end

-- Returns the system time when the primary GCS was last seen.
---@return uint32_t_ud -- System time in milliseconds.
function gcs:last_seen() end

-- Returns true if the GCS is currently allowed to set parameters.
---@return boolean
function gcs:get_allow_param_set() end

-- Sets whether the GCS is allowed to set parameters.
---@param new_allow_value boolean
function gcs:set_allow_param_set(new_allow_value) end

-- Executes a MAVLink command using the COMMAND_INT interface, as if it were sent from a GCS.
---@param command integer -- The MAV_CMD_xxx command ID.
---@param params table -- A table of parameters: {p1, p2, p3, p4, x, y, z, frame}. Any omitted parameters are treated as zero.
---@return integer -- The MAV_RESULT enum value (e.g., 0 for accepted).
function gcs:run_command_int(command, params) end

-- The relay library provides a simple interface for controlling relay outputs.
relay = {}

-- Toggles the state of a specific relay (on to off, or off to on).
---@param instance integer -- The 0-indexed relay instance.
function relay:toggle(instance) end

-- Returns true if the specified relay is enabled (i.e., its RELAY_PIN parameter is not -1).
---@param instance integer -- The 0-indexed relay instance.
---@return boolean
function relay:enabled(instance) end

-- Gets the current state of a relay.
---@param instance integer -- The 0-indexed relay instance.
---@return integer -- 1 if on, 0 if off.
function relay:get(instance) end

-- Turns the specified relay off.
---@param instance integer -- The 0-indexed relay instance.
function relay:off(instance) end

-- Turns the specified relay on.
---@param instance integer -- The 0-indexed relay instance.
function relay:on(instance) end


-- The terrain library provides access to checking heights against the onboard terrain database.
-- Requires a terrain database to be present on the SD card.
terrain = {}
terrain.TerrainStatusOK = enum_integer
terrain.TerrainStatusUnhealthy = enum_integer
terrain.TerrainStatusDisabled = enum_integer

-- Returns the vehicle's current height above the terrain (AGL).
---@param extrapolate boolean -- If true, allows the return of an extrapolated altitude if the vehicle is outside the known terrain data area.
---@return number|nil -- The height above terrain in meters, or nil if not available.
function terrain:height_above_terrain(extrapolate) end

-- Returns the difference in terrain height between the vehicle's current location and its home position.
---@param extrapolate boolean
---@return number|nil -- Height difference in meters (positive means current location is higher), or nil if unavailable.
function terrain:height_terrain_difference_home(extrapolate) end

-- Looks up the terrain height (above mean sea level) at a specific geographic location.
---@param loc Location_ud -- The location at which to look up the terrain altitude.
---@param corrected boolean -- If true, the terrain altitude will be corrected based on the difference between the database and measured altitude at home.
---@return number|nil -- The AMSL altitude of the terrain in meters, or nil if unavailable.
function terrain:height_amsl(loc, corrected) end

-- Returns the current status of the terrain database.
---@return integer -- The terrain status enum value (e.g., terrain.TerrainStatusOK).
function terrain:status() end

-- Returns true if terrain following is enabled in the parameters.
---@return boolean
function terrain:enabled() end


-- A data structure for holding a single reading from a rangefinder.
---@class (exact) RangeFinder_State_ud
local RangeFinder_State_ud = {}

---@return RangeFinder_State_ud
function RangeFinder_State() end

-- get system time (ms) of the last successful update from this sensor
---@return uint32_t_ud
function RangeFinder_State_ud:last_reading() end
-- set system time (ms)
---@param value uint32_t_ud|integer|number
function RangeFinder_State_ud:last_reading(value) end

-- get sensor status
---@return integer
function RangeFinder_State_ud:status() end

-- set sensor status
---@param value integer
function RangeFinder_State_ud:status(value) end

-- get number of consecutive valid readings (maxes out at 10)
---@return integer
function RangeFinder_State_ud:range_valid_count() end

-- set number of consecutive valid readings
---@param value integer
function RangeFinder_State_ud:range_valid_count(value) end

-- get distance in meters
---@return number
function RangeFinder_State_ud:distance() end

-- set distance in meters
---@param value number
function RangeFinder_State_ud:distance(value) end

-- get measurement quality in percent (0-100). -1 means quality is unknown.
---@return integer
function RangeFinder_State_ud:signal_quality() end

-- set measurement quality
---@param value integer
function RangeFinder_State_ud:signal_quality(value) end

-- get voltage in millivolts, if applicable.
---@return integer
function RangeFinder_State_ud:voltage() end

-- set voltage in millivolts
---@param value integer
function RangeFinder_State_ud:voltage(value) end


-- Represents a scripting backend for a rangefinder. Used to create custom rangefinder drivers.
---@class (exact) AP_RangeFinder_Backend_ud
local AP_RangeFinder_Backend_ud = {}

-- Sends a rangefinder measurement from a script into the ArduPilot system.
---@param state RangeFinder_State_ud -- The populated state object containing the measurement.
---@return boolean -- True on success.
function AP_RangeFinder_Backend_ud:handle_script_msg(state) end

-- Status of this rangefinder instance
---@return integer
function AP_RangeFinder_Backend_ud:status() end

-- Type of rangefinder of this instance
---@return integer
function AP_RangeFinder_Backend_ud:type() end

-- Orientation of the rangefinder of this instance
---@return integer
function AP_RangeFinder_Backend_ud:orientation() end

-- Current distance of the sensor instance
---@return number
function AP_RangeFinder_Backend_ud:distance() end

-- Current distance measurement signal quality of the sensor instance
---@return number
function AP_RangeFinder_Backend_ud:signal_quality() end

-- State of the most recent rangefinder measurement
---@return RangeFinder_State_ud
function AP_RangeFinder_Backend_ud:get_state() end


-- The main interface for accessing rangefinder data.
rangefinder = {}

-- Gets a backend handle for a specific rangefinder instance. Used for custom rangefinder drivers.
---@param rangefinder_instance integer -- The 0-indexed rangefinder instance.
---@return AP_RangeFinder_Backend_ud|nil -- The backend object, or nil if not found.
function rangefinder:get_backend(rangefinder_instance) end

-- Gets the configured position offset for a sensor with a specific orientation.
---@param orientation integer -- The sensor orientation enum value.
---@return Vector3f_ud -- The position offset vector.
function rangefinder:get_pos_offset_orient(orientation) end

-- Returns true if there is data from a sensor with the specified orientation.
---@param orientation integer
---@return boolean
function rangefinder:has_data_orient(orientation) end

-- Returns the status of the sensor with the specified orientation.
---@param orientation integer
---@return integer
function rangefinder:status_orient(orientation) end

---@param orientation integer
---@return integer
---@deprecated Use ground_clearance_orient (in meters).
function rangefinder:ground_clearance_cm_orient(orientation) end

---@param orientation integer
---@return integer
---@deprecated Use min_distance_orient (in meters).
function rangefinder:min_distance_cm_orient(orientation) end

---@param orientation integer
---@return integer
---@deprecated Use max_distance_orient (in meters).
function rangefinder:max_distance_cm_orient(orientation) end

---@param orientation integer
---@return integer
---@deprecated Use distance_orient (in meters).
function rangefinder:distance_cm_orient(orientation) end

-- Gets the configured ground clearance for a rangefinder with a specific orientation.
---@param orientation integer
---@return number -- Ground clearance in meters.
function rangefinder:ground_clearance_orient(orientation) end

-- Gets the configured minimum measurable distance for a rangefinder with a specific orientation.
---@param orientation integer
---@return number -- Minimum distance in meters.
function rangefinder:min_distance_orient(orientation) end

-- Gets the configured maximum measurable distance for a rangefinder with a specific orientation.
---@param orientation integer
---@return number -- Maximum distance in meters.
function rangefinder:max_distance_orient(orientation) end

-- Gets the current measured distance from a rangefinder with a specific orientation.
---@param orientation integer
---@return number -- Current distance in meters.
function rangefinder:distance_orient(orientation) end

-- Gets the current signal quality for a rangefinder with a specific orientation.
---@param orientation integer
---@return integer -- Signal quality percentage (0-100).
function rangefinder:signal_quality_pct_orient(orientation) end

-- Returns true if a rangefinder with the specified orientation is configured.
---@param orientation integer
---@return boolean
function rangefinder:has_orientation(orientation) end

-- Returns the total number of configured rangefinder sensors.
---@return integer
function rangefinder:num_sensors() end

-- Represents a scripting backend for a proximity sensor. Used to create custom avoidance sensors.
---@class (exact) AP_Proximity_Backend_ud
local AP_Proximity_Backend_ud = {}

-- Pushes the virtual proximity boundary into the actual boundary used by the avoidance system.
---@return boolean
function AP_Proximity_Backend_ud:update_virtual_boundary() end

-- Sets the minimum and maximum detection distance for the virtual sensor.
---@param min number -- Minimum distance in meters.
---@param max number -- Maximum distance in meters.
---@return boolean
function AP_Proximity_Backend_ud:set_distance_min_max(min, max) end

-- Gets the type of the backend.
---@return integer
function AP_Proximity_Backend_ud:type() end

-- Sends a 3D object detection from a script into the proximity system using a vector.
---@param vector_3d Vector3f_ud -- A vector representing the detected object.
---@param update_boundary boolean -- If true, updates the avoidance boundary immediately.
---@return boolean
function AP_Proximity_Backend_ud:handle_script_3d_msg(vector_3d, update_boundary) end

-- Sends a 3D object detection from a script into the proximity system using angles and distance.
---@param dist_m number -- The distance to the object in meters.
---@param yaw_deg number -- The yaw angle to the object in degrees.
---@param pitch_deg number -- The pitch angle to the object in degrees.
---@param update_boundary boolean -- If true, updates the avoidance boundary immediately.
---@return boolean
function AP_Proximity_Backend_ud:handle_script_distance_msg(dist_m, yaw_deg, pitch_deg, update_boundary) end

-- The main interface for accessing proximity sensor data for object avoidance.
proximity = {}

-- Gets a backend handle for a specific proximity sensor instance. Used for custom drivers.
---@param instance integer
---@return AP_Proximity_Backend_ud|nil
function proximity:get_backend(instance) end

-- Gets the angle and distance to a specific detected object.
---@param object_number integer
---@return number|nil -- Angle
---@return number|nil -- Distance
function proximity:get_object_angle_and_distance(object_number) end

-- Gets the angle and distance to the closest detected object.
---@return number|nil -- Angle
---@return number|nil -- Distance
function proximity:get_closest_object() end

-- Gets the total number of objects currently being tracked by the proximity sensor.
---@return integer
function proximity:get_object_count() end

-- Returns the total number of configured proximity sensors.
---@return integer
function proximity:num_sensors() end

-- Gets the current status of the proximity sensor system.
---@return integer
function proximity:get_status() end

-- The main interface for user notification, controlling onboard LEDs and the buzzer.
notify = {}

-- Sets the color of a specific notify LED instance.
---@param red integer -- The red component (0-255).
---@param green integer -- The green component (0-255).
---@param blue integer -- The blue component (0-255).
---@param id integer -- The 0-indexed LED instance.
function notify:handle_rgb_id(red, green, blue, id) end

-- Sets the color and blink rate of the primary notify LED.
---@param red integer -- The red component (0-255).
---@param green integer -- The green component (0-255).
---@param blue integer -- The blue component (0-255).
---@param rate_hz integer -- The blink rate in Hz.
function notify:handle_rgb(red, green, blue, rate_hz) end

-- Plays a tune through the vehicle's buzzer using the MML (Music Macro Language) format.
-- An online tune tester can be found here: https://firmware.ardupilot.org/Tools/ToneTester/
---@param tune string -- The MML tune string.
function notify:play_tune(tune) end

-- Displays text on a notify display (e.g., an OLED screen). Text that is too long to fit will be automatically scrolled.
---@param text string -- The text to display (up to 50 characters).
---@param row integer -- The row number to display on, where 0 is the top row.
function notify:send_text(text, row) end

-- Releases a row on the notify display, allowing it to be used by other system messages.
---@param row integer -- The row number to release.
function notify:release_text(row) end

-- The GPS library provides access to information about the GPS receivers on the vehicle.
gps = {}
gps.GPS_OK_FIX_3D_RTK_FIXED = 6
gps.GPS_OK_FIX_3D_RTK_FLOAT = 5
gps.GPS_OK_FIX_3D_DGPS = 4
gps.GPS_OK_FIX_3D = 3
gps.GPS_OK_FIX_2D = 2
gps.NO_FIX = 1
gps.NO_GPS = 0

-- Gets the UTC time from the GPS as a Unix epoch timestamp.
---@param instance integer -- The 0-indexed GPS instance.
---@return uint64_t_ud -- The time in microseconds since 1/1/1970.
function gps:time_epoch_usec(instance) end

-- Gets the yaw from a GPS with dual antennas (if available).
---@param instance integer -- The 0-indexed GPS instance.
---@return number|nil -- The yaw angle in degrees.
---@return number|nil -- The yaw accuracy estimate in degrees.
---@return uint32_t_ud|nil -- The system time of the last yaw reading in milliseconds.
function gps:gps_yaw_deg(instance) end

-- Returns the instance number of the first GPS that has not been fully configured. Returns nil if all are configured.
---@return integer|nil
function gps:first_unconfigured_gps() end

-- Returns a Vector3f containing the configured antenna offsets from the vehicle's center of gravity.
---@param instance integer -- The 0-indexed GPS instance.
---@return Vector3f_ud -- The antenna offset vector (forward, right, down) in meters.
function gps:get_antenna_offset(instance) end

-- Returns true if the GPS instance can report vertical velocity.
---@param instance integer -- The 0-indexed GPS instance.
---@return boolean -- True if vertical velocity is available.
function gps:have_vertical_velocity(instance) end

-- Returns the system time of the last message received from the GPS.
---@param instance integer -- The 0-indexed GPS instance.
---@return uint32_t_ud -- The time in milliseconds.
function gps:last_message_time_ms(instance) end

-- Returns the system time of the last valid position fix.
---@param instance integer -- The 0-indexed GPS instance.
---@return uint32_t_ud -- The time in milliseconds.
function gps:last_fix_time_ms(instance) end

-- Returns the vertical dilution of precision (VDOP) of the GPS instance. Lower is better.
---@param instance integer -- The 0-indexed GPS instance.
---@return integer -- The VDOP value.
function gps:get_vdop(instance) end

-- Returns the horizontal dilution of precision (HDOP) of the GPS instance. Lower is better.
---@param instance integer -- The 0-indexed GPS instance.
---@return integer -- The HDOP value.
function gps:get_hdop(instance) end

-- Returns the number of milliseconds into the current GPS week.
---@param instance integer -- The 0-indexed GPS instance.
---@return uint32_t_ud -- The time in milliseconds.
function gps:time_week_ms(instance) end

-- Returns the GPS week number.
---@param instance integer -- The 0-indexed GPS instance.
---@return integer -- The week number.
function gps:time_week(instance) end

-- Returns the number of satellites the GPS is currently tracking.
---@param instance integer -- The 0-indexed GPS instance.
---@return integer -- The number of satellites.
function gps:num_sats(instance) end

-- Returns the ground course (direction of travel) of the vehicle.
---@param instance integer -- The 0-indexed GPS instance.
---@return number -- The ground course in degrees.
function gps:ground_course(instance) end

-- Returns the ground speed of the vehicle.
---@param instance integer -- The 0-indexed GPS instance.
---@return number -- The ground speed in meters per second.
function gps:ground_speed(instance) end

-- Returns a Vector3f containing the velocity as observed by the GPS in the NED frame.
---@param instance integer -- The 0-indexed GPS instance.
---@return Vector3f_ud -- A 3D velocity vector in m/s.
function gps:velocity(instance) end

-- Returns the vertical accuracy estimate of the GPS, if available.
---@param instance integer -- The 0-indexed GPS instance.
---@return number|nil -- The vertical accuracy in meters.
function gps:vertical_accuracy(instance) end

-- Returns the horizontal RMS accuracy estimate of the GPS.
---@param instance integer -- The 0-indexed GPS instance.
---@return number|nil -- The horizontal accuracy in meters.
function gps:horizontal_accuracy(instance) end

-- Returns the speed accuracy estimate of the GPS, if available.
---@param instance integer -- The 0-indexed GPS instance.
---@return number|nil -- The 3D velocity RMS accuracy estimate in m/s.
function gps:speed_accuracy(instance) end

-- Returns a Location object for the last known GPS position.
-- It is critical to check `gps:status()` to ensure the returned location is current and valid.
---@param instance integer -- The 0-indexed GPS instance.
---@return Location_ud -- The GPS location.
function gps:location(instance) end

-- Returns the current GPS fix status. This should always be checked before using GPS location data.
---@param instance integer -- The 0-indexed GPS instance.
---@return integer -- The status enum value (e.g., `gps.GPS_OK_FIX_3D_RTK_FIXED`).
function gps:status(instance) end

-- Returns which GPS instance is currently being used as the primary source for navigation.
---@return integer -- The primary sensor instance index.
function gps:primary_sensor() end

-- Returns the total number of connected GPS devices. This may include a blended GPS instance if enabled.
---@return integer -- The number of sensors.
function gps:num_sensors() end

-- Inject a packet of raw binary to a GPS (e.g., RTCM3)
---@param data string -- binary data to inject
function gps:inject_data(data) end

-- A data structure for holding the state of a battery. Used by custom battery monitor drivers.
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
-- set state of health, 255 if not available (this is the default)
---@param value integer
function BattMonitorScript_State_ud:state_of_health_pct(value) end

-- The temperature library provides access to information about connected temperature sensors.
temperature_sensor = {}

-- Returns the temperature from a specific sensor instance.
---@param instance integer -- The 1-indexed temperature sensor instance.
---@return number|nil -- Temperature in degrees Celsius, or nil if not available.
function temperature_sensor:get_temperature(instance) end

-- The battery library provides access to information about the connected batteries.
battery = {}

-- Pushes a populated battery state into the ArduPilot system. Used by custom battery monitor driver scripts.
---@param idx integer -- The 0-indexed battery instance.
---@param state BattMonitorScript_State_ud -- The populated state object.
---@return boolean
function battery:handle_scripting(idx, state) end

-- Resets the remaining capacity of a battery to a specific percentage.
---@param instance integer -- The 1-indexed battery instance.
---@param percentage number -- The percentage (0-100) to set as the remaining capacity.
---@return boolean
function battery:reset_remaining(instance, percentage) end

-- Returns the cycle count of the battery, if available.
---@param instance integer -- The 1-indexed battery instance.
---@return integer|nil
function battery:get_cycle_count(instance) end

-- Returns the temperature of the battery, if available.
---@param instance integer -- The 1-indexed battery instance.
---@return number|nil -- Temperature in degrees Celsius.
function battery:get_temperature(instance) end

-- Returns true if the battery is currently in an overpower condition.
---@param instance integer -- The 1-indexed battery instance.
---@return boolean
function battery:overpower_detected(instance) end

-- Returns true if any battery has triggered a failsafe condition.
---@return boolean
function battery:has_failsafed() end

-- Returns the full pack capacity of the battery.
---@param instance integer -- The 1-indexed battery instance.
---@return integer -- Capacity in mAh.
function battery:pack_capacity_mah(instance) end

-- Returns the remaining battery capacity as a percentage.
---@param instance integer -- The 1-indexed battery instance.
---@return integer|nil -- Remaining capacity (0-100), or nil if not available.
function battery:capacity_remaining_pct(instance) end

-- Returns the consumed energy from the battery in Watt-hours.
---@param instance integer -- The 1-indexed battery instance.
---@return number|nil
function battery:consumed_wh(instance) end

-- Returns the consumed capacity from the battery in milliamp-hours.
---@param instance integer -- The 1-indexed battery instance.
---@return number|nil
function battery:consumed_mah(instance) end

-- Returns the current being drawn from the battery.
---@param instance integer -- The 1-indexed battery instance.
---@return number|nil -- Current in Amps.
function battery:current_amps(instance) end

-- Returns the estimated resting voltage of the battery (voltage without load).
---@param instance integer -- The 1-indexed battery instance.
---@return number
function battery:voltage_resting_estimate(instance) end

-- Returns the estimated internal resistance of the battery.
---@param instance integer -- The 1-indexed battery instance.
---@return number -- Resistance in Ohms.
function battery:get_resistance(instance) end

-- Returns the current voltage of the battery.
---@param instance integer -- The 1-indexed battery instance.
---@return number -- Voltage.
function battery:voltage(instance) end

-- Returns true if the battery monitor is healthy and providing data.
---@param instance integer -- The 1-indexed battery instance.
---@return boolean
function battery:healthy(instance) end

-- Returns the total number of configured battery instances.
---@return integer
function battery:num_instances() end

-- Gets the voltage of an individual cell in the battery pack.
---@param instance integer -- The 1-indexed battery instance.
---@param cell integer -- The 1-indexed cell number.
---@return number|nil -- The cell voltage, or nil if not available.
function battery:get_cell_voltage(instance, cell) end


-- The Arming library provides access to arming status and commands.
arming = {}

-- Sets an auxiliary authorization to a failed state, preventing arming.
---@param auth_id integer -- The auxiliary authorization ID.
---@param fail_msg string -- The failure message to be displayed.
function arming:set_aux_auth_failed(auth_id, fail_msg) end

-- Sets an auxiliary authorization to a passed state.
---@param auth_id integer -- The auxiliary authorization ID.
function arming:set_aux_auth_passed(auth_id) end

-- Gets the ID of the current auxiliary authorization check being performed.
---@return integer|nil
function arming:get_aux_auth_id() end

-- Attempts to arm the vehicle, running all pre-arm checks.
---@return boolean -- True if armed successfully.
function arming:arm() end

-- Forcefully arms the vehicle, skipping some pre-arm checks. Use with extreme caution.
---@return boolean -- True if armed.
function arming:arm_force() end

-- Returns true if the vehicle is currently armed.
---@return boolean
function arming:is_armed() end

-- Runs the pre-arm checks and returns true if they all pass.
---@return boolean
function arming:pre_arm_checks() end

-- Disarms the vehicle.
---@return boolean -- True if disarmed successfully, false if already disarmed.
function arming:disarm() end

-- The AHRS (Attitude and Heading Reference System) library is the core of the autopilot's state estimation.
-- It provides the best available estimate for the vehicle's attitude, position, and velocity.
ahrs = {}

-- Supplies an external position estimate to the EKF. This is used for external navigation sources like visual odometry.
---@param location Location_ud -- The estimated location object. The altitude component is ignored.
---@param accuracy number -- The 1-sigma accuracy of the position estimate in meters.
---@param timestamp_ms uint32_t_ud|integer|number -- The timestamp of the reading in milliseconds since boot.
---@return boolean -- True if the estimate was successfully passed to the EKF.
function ahrs:handle_external_position_estimate(location, accuracy, timestamp_ms) end

-- Gets the vehicle's attitude as a quaternion.
---@return Quaternion_ud|nil -- The quaternion representing the vehicle's attitude, or nil if not available.
function ahrs:get_quaternion() end

-- Gets the currently active EKF source set (primary, secondary, etc.).
---@return integer
function ahrs:get_posvelyaw_source_set() end

-- Returns true if the AHRS has completed its initialization.
---@return boolean
function ahrs:initialised() end

-- Sets the EKF origin to a specific location.
---@param loc Location_ud -- The Location object to set as the origin.
---@return boolean
function ahrs:set_origin(loc) end

-- Gets the current EKF origin location.
---@return Location_ud|nil
function ahrs:get_origin() end

-- Sets the home position to a specific location.
---@param loc Location_ud -- The Location object to set as home.
---@return boolean
function ahrs:set_home(loc) end

-- Gets the velocity innovations and variances for a specific EKF source. (Advanced)
---@param source integer
---@return Vector3f_ud|nil
---@return Vector3f_ud|nil
function ahrs:get_vel_innovations_and_variances_for_source(source) end

-- Sets the active EKF source set.
---@param source_set_idx integer
---| '0' # PRIMARY
---| '1' # SECONDARY
---| '2' # TERTIARY
function ahrs:set_posvelyaw_source_set(source_set_idx) end

-- Gets the EKF's internal state variances. (Advanced)
---@return number|nil
---@return number|nil
---@return number|nil
---@return Vector3f_ud|nil
---@return number|nil
function ahrs:get_variances() end

-- Gets the ratio of Equivalent Airspeed (EAS) to True Airspeed (TAS).
---@return number
function ahrs:get_EAS2TAS() end

-- Rotates a vector from the vehicle's body frame to the earth frame (NED).
---@param vector Vector3f_ud -- The vector in the body frame to be rotated. The vector is modified in-place.
---@return Vector3f_ud -- The rotated vector in the earth frame.
function ahrs:body_to_earth(vector) end

-- Rotates a vector from the earth frame (NED) to the vehicle's body frame.
---@param vector Vector3f_ud -- The vector in the earth frame to be rotated. The vector is modified in-place.
---@return Vector3f_ud -- The rotated vector in the body frame.
function ahrs:earth_to_body(vector) end

-- Gets a metric for the vehicle's current vibration levels.
---@return Vector3f_ud
function ahrs:get_vibration() end

-- Returns the estimated true airspeed of the vehicle, if available.
---@return number|nil -- Airspeed in meters/second.
function ahrs:airspeed_estimate() end

-- Returns true if the AHRS system is healthy and providing reliable data. This should be checked before trusting any AHRS data.
---@return boolean
function ahrs:healthy() end

-- Returns true if the home position has been set.
---@return boolean
function ahrs:home_is_set() end

-- Gets the vehicle's altitude relative to the home position.
---@return number -- Altitude in meters (down is positive).
function ahrs:get_relative_position_D_home() end

-- Gets the vehicle's position relative to the EKF origin in the NED frame.
---@return Vector3f_ud|nil -- A Vector3f of North, East, Down positions in meters.
function ahrs:get_relative_position_NED_origin() end

-- Gets the vehicle's position relative to its home position in the NED frame.
---@return Vector3f_ud|nil -- A Vector3f of North, East, Down positions in meters.
function ahrs:get_relative_position_NED_home() end

-- Returns a Vector3f containing the current vehicle velocity in the NED frame.
---@return Vector3f_ud|nil -- North, East, Down velocity in meters/second, or nil if not available.
function ahrs:get_velocity_NED() end

-- Gets the current ground speed as a 2D vector.
---@return Vector2f_ud -- A Vector2f where x=North speed, y=East speed, in meters/second.
function ahrs:groundspeed_vector() end

-- Returns a Vector3f containing the current wind estimate.
---@return Vector3f_ud -- A Vector3f of wind velocity (North, East, Down) in meters/second.
function ahrs:wind_estimate() end

-- Determines how aligned a given heading is with the wind.
---@param heading_deg number -- The heading to check, in degrees.
---@return number -- Returns 1.0 for a perfect headwind, -1.0 for a perfect tailwind, and 0 for a perfect crosswind.
function ahrs:wind_alignment(heading_deg) end

-- Returns the forward headwind component.
---@return number -- The headwind component in m/s (negative means tailwind).
function ahrs:head_wind() end

-- Returns the latest altitude estimate above ground level (AGL). Requires a valid terrain or rangefinder source.
---@return number|nil -- Height above ground level in meters.
function ahrs:get_hagl() end

-- Gets the current accelerometer readings.
---@return Vector3f_ud -- A Vector3f of accelerations in m/s^2.
function ahrs:get_accel() end

-- Returns a Vector3f containing the current smoothed and filtered gyro rates.
---@return Vector3f_ud -- Roll, pitch, yaw gyro rates in radians/second.
function ahrs:get_gyro() end

-- Returns a Location object representing the vehicle's home position.
---@return Location_ud
function ahrs:get_home() end

-- Returns a Location object of the vehicle's current estimated position.
-- Note: This will only return a Location if the system's position estimate is considered valid.
---@return Location_ud|nil
function ahrs:get_location() end

-- Deprecated alias for `get_location`.
---@return Location_ud|nil
---@deprecated Use get_location()
function ahrs:get_position() end

-- Deprecated alias for `get_yaw_rad`.
---@return number
---@deprecated Use get_yaw_rad()
function ahrs:get_yaw() end

-- Deprecated alias for `get_pitch_rad`.
---@return number
---@deprecated Use get_pitch_rad()
function ahrs:get_pitch() end

-- Deprecated alias for `get_roll_rad`.
---@return number
---@deprecated Use get_roll_rad()
function ahrs:get_roll() end

-- Returns the current vehicle yaw angle.
---@return number -- Yaw angle in radians (0 to 2*PI).
function ahrs:get_yaw_rad() end

-- Returns the current vehicle pitch angle.
---@return number -- Pitch angle in radians.
function ahrs:get_pitch_rad() end

-- Returns the current vehicle roll angle.
---@return number -- Roll angle in radians.
function ahrs:get_roll_rad() end

-- Copter-specific attitude controller interface.
AC_AttitudeControl = {}

-- Returns the attitude slew rates for the VTOL controller.
---@return number -- Roll slew rate.
---@return number -- Pitch slew rate.
---@return number -- Yaw slew rate.
function AC_AttitudeControl:get_rpy_srate() end

-- Returns the angle between the target thrust vector and the current thrust vector.
---@return number -- The attitude error in degrees.
function AC_AttitudeControl:get_att_error_angle_deg() end

-- Rover-specific attitude controller interface.
AR_AttitudeControl = {}

-- Returns the slew rates for the rover's steering and speed controllers.
---@return number -- Steering slew rate.
---@return number -- Speed slew rate.
function AR_AttitudeControl:get_srate() end

-- Copter-specific position controller interface.
poscontrol = {}

-- Adds a temporary offset to the position controller's target.
---@param pos_offset_NED Vector3f_ud -- Position offset in the NED frame (meters).
---@param vel_offset_NED Vector3f_ud -- Velocity offset in the NED frame (m/s).
---@param accel_offset_NED Vector3f_ud -- Acceleration offset in the NED frame (m/s/s).
---@return boolean
function poscontrol:set_posvelaccel_offset(pos_offset_NED, vel_offset_NED, accel_offset_NED) end

-- Gets the current position, velocity, and acceleration offsets from the position controller.
---@return Vector3f_ud|nil -- Position offset.
---@return Vector3f_ud|nil -- Velocity offset.
---@return Vector3f_ud|nil -- Acceleration offset.
function poscontrol:get_posvelaccel_offset() end

-- Gets the position controller's current velocity target in the NED frame.
---@return Vector3f_ud|nil -- Velocity target in m/s.
function poscontrol:get_vel_target() end

-- Gets the position controller's current acceleration target in the NED frame.
---@return Vector3f_ud|nil -- Acceleration target in m/s/s.
function poscontrol:get_accel_target() end

-- Interface for the precision landing system.
precland = {}

-- Gets the location of the precision landing target, if acquired.
---@return Location_ud|nil
function precland:get_target_location() end

-- Gets the velocity of the precision landing target in the North-East frame, if available.
---@return Vector2f_ud|nil -- Velocity in m/s.
function precland:get_target_velocity() end

-- Gets the system time of the last valid target update.
---@return uint32_t_ud -- Time in milliseconds.
function precland:get_last_valid_target_ms() end

-- Returns true if a precision landing target has been acquired.
---@return boolean
function precland:target_acquired() end

-- Returns true if the precision landing system is healthy.
---@return boolean
function precland:healthy() end

-- Interface for the follow-me mode.
follow = {}

-- Gets the heading of the follow-me target.
---@return number|nil -- Heading in degrees.
function follow:get_target_heading_deg() end

-- Gets the target's location and velocity offset.
---@return Location_ud|nil
---@return Vector3f_ud|nil
function follow:get_target_location_and_velocity_ofs() end

-- Gets the target's location and velocity.
---@return Location_ud|nil
---@return Vector3f_ud|nil
function follow:get_target_location_and_velocity() end

-- Gets the system time of the last update from the follow-me target.
---@return uint32_t_ud -- Time in milliseconds.
function follow:get_last_update_ms() end

-- Returns true if the vehicle currently has a follow-me target.
---@return boolean
function follow:have_target() end

-- General scripting utilities.
scripting = {}

-- Restarts all running Lua scripts.
function scripting:restart_all() end

-- Lists the contents of a directory on the SD card.
---@param directoryname string -- The path of the directory to list (e.g., "/APM/scripts/").
---@return table|nil -- A table of filenames.
---@return string|nil -- An error string if the operation fails.
function dirlist(directoryname) end

-- Removes (deletes) a file from the SD card.
---@param filename string -- The path of the file to remove.
---@return boolean|nil -- True on success.
---@return nil|string -- An error string on failure.
---@return integer -- An error number on failure.
function remove(filename) end

-- MAVLink message interface for sending and receiving binary MAVLink messages.
mavlink = {}

-- Initializes the scripting MAVLink buffer. This must be called before sending or receiving MAVLink messages.
---@param msg_queue_length uint32_t_ud|integer|number -- The maximum number of incoming messages to queue.
---@param num_rx_msgid uint32_t_ud|integer|number -- The number of unique message IDs you intend to receive.
function mavlink:init(msg_queue_length, num_rx_msgid) end

-- Registers a specific MAVLink message ID to be received by the script.
---@param msg_id number -- The message ID to register (e.g., from `mavlink_msgs.get_msgid("HEARTBEAT")`).
---@return boolean -- False if the ID has already been registered.
function mavlink:register_rx_msgid(msg_id) end

-- Receives a MAVLink message that has been registered for reception.
---@return string -- The raw message payload as a binary string.
---@return number -- The MAVLink channel it was received on.
---@return uint32_t_ud -- The timestamp of reception.
function mavlink:receive_chan() end

-- Sends a pre-encoded MAVLink message.
-- Example: `mavlink:send_chan(chan, mavlink_msgs.encode("HEARTBEAT", {params...}))`
---@param chan integer -- The MAVLink channel to send on.
---@param msgid integer -- The message ID.
---@param message string -- The encoded message payload as a binary string.
---@return boolean -- True on success.
function mavlink:send_chan(chan, msgid, message) end

-- Blocks a specific MAV_CMD from being processed by the autopilot's main command handler.
---@param comand_id integer -- The MAV_CMD ID to block.
---@return boolean
function mavlink:block_command(comand_id) end

-- Interface for the geofence system.
fence = {}

-- Gets the system time at which the current geofence breach started.
---@return uint32_t_ud -- Time in milliseconds.
function fence:get_breach_time() end

-- Gets the system time at which the geofence margin was first breached.
---@return uint32_t_ud -- Time in milliseconds.
function fence:get_margin_breach_time() end

-- Gets a bitmask of all currently breached fences.
---@return integer -- The breach bitmask.
---| 1 # Maximum altitude
---| 2 # Circle
---| 4 # Polygon
---| 8 # Minimum altitude
function fence:get_breaches() end

-- Gets a bitmask of all currently breached fence margins.
---@return integer -- The margin breach bitmask.
function fence:get_margin_breaches() end

-- Gets the shortest distance to a specific type of geofence.
---@param fence_type integer -- The fence type bitmask.
---@return number -- The distance in meters.
function fence:get_breach_distance(fence_type) end

-- A data structure for holding file system statistics, similar to a Unix `stat` struct.
---@class (exact) stat_t_ud
local stat_t_ud = {}

---@return stat_t_ud
function stat_t() end

-- get creation time
---@return uint32_t_ud -- Time in seconds since the Unix epoch.
function stat_t_ud:ctime() end
-- get last access time
---@return uint32_t_ud -- Time in seconds since the Unix epoch.
function stat_t_ud:atime() end
-- get last modification time
---@return uint32_t_ud -- Time in seconds since the Unix epoch.
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

-- Interface for the Real-Time Clock (RTC).
rtc = {}

-- Converts GMT date and time fields to a Unix epoch timestamp.
---@param year integer -- e.g., 2023
---@param month integer -- 1-12
---@param day integer -- 1-31
---@param hour integer -- 0-23
---@param min integer -- 0-59
---@param sec integer -- 0-59
---@return uint32_t_ud -- Time in seconds since 1970.
function rtc:date_fields_to_clock_s(year, month, day, hour, min, sec) end

-- Breaks a Unix epoch timestamp into its GMT date and time components.
---@param param1 uint32_t_ud|integer|number -- Time in seconds since 1970.
---@return integer|nil year
---@return integer|nil month
---@return integer|nil day
---@return integer|nil hour
---@return integer|nil min
---@return integer|nil sec
---@return integer|nil weekday (0-6, Sunday is 0)
function rtc:clock_s_to_date_fields(param1) end

-- File system interface.
fs = {}

-- Gets statistics for a specific file or directory.
---@param param1 string -- The path to the file or directory.
---@return stat_t_ud|nil -- A stat object, or nil on error.
function fs:stat(param1) end

-- Formats the SD card. This is an asynchronous operation. Use `get_format_status()` to check its progress.
---@return boolean
function fs:format() end

-- Gets the current status of an SD card format operation.
---@return integer -- 0=NOT_STARTED, 1=PENDING, 2=IN_PROGRESS, 3=SUCCESS, 4=FAILURE
function fs:get_format_status() end

-- Calculates the CRC32 checksum of a file.
---@param file_name string
---@return uint32_t_ud|nil
function fs:crc32(file_name) end

-- Networking utilities.
networking = {}

-- Converts a 32-bit integer IPv4 address to its string representation.
---@param ip4addr uint32_t_ud|integer|number
---@return string
function networking:address_to_str(ip4addr) end

-- Gets the active network gateway address.
---@return uint32_t_ud
function networking:get_gateway_active() end

-- Gets the active network subnet mask.
---@return uint32_t_ud
function networking:get_netmask_active() end

-- Gets the active network IP address.
---@return uint32_t_ud
function networking:get_ip_active() end

-- Visual odometry interface.
visual_odom = {}

-- Returns true if the visual odometry system is healthy.
---@return boolean
function visual_odom:healthy() end

-- Returns the quality of the visual odometry system.
---@return integer -- Quality as a percentage (1-100), or 0 if unknown.
function visual_odom:quality() end

-- Interface for smart servo telemetry.
servo_telem = {}

-- Gets the telemetry data for a specific servo.
---@param servo_index integer -- The 0-indexed servo number.
---@return AP_Servo_Telem_Data_ud|nil -- The telemetry data object, or nil if not available.
function servo_telem:get_telem(servo_index) end

-- A data structure for holding telemetry data from a single smart servo.
---@class AP_Servo_Telem_Data_ud
local AP_Servo_Telem_Data_ud = {}

-- Get timestamp of last telemetry update
---@return uint32_t_ud -- milliseconds since boot
function AP_Servo_Telem_Data_ud:last_update_ms() end
-- Get type-specific status flags
---@return integer|nil -- flags or nil if not available
function AP_Servo_Telem_Data_ud:status_flags() end
-- Get PCB temperature
---@return integer|nil -- temperature in centidegrees or nil if not available
function AP_Servo_Telem_Data_ud:pcb_temperature_cdeg() end
-- Get motor temperature
---@return integer|nil -- temperature in centidegrees or nil if not available
function AP_Servo_Telem_Data_ud:motor_temperature_cdeg() end
-- Get duty cycle
---@return integer|nil -- duty cycle 0% to 100% or nil if not available
function AP_Servo_Telem_Data_ud:duty_cycle() end
-- get current
---@return number|nil -- current in amps or nil if not available
function AP_Servo_Telem_Data_ud:current() end
-- get voltage
---@return number|nil -- voltage in volts or nil if not available
function AP_Servo_Telem_Data_ud:voltage() end
-- get speed
---@return number|nil -- speed in degrees per second or nil if not available
function AP_Servo_Telem_Data_ud:speed() end
-- get force
---@return number|nil -- force in newton meters or nil if not available
function AP_Servo_Telem_Data_ud:force() end
-- get measured position
---@return number|nil -- measured position in degrees or nil if not available
function AP_Servo_Telem_Data_ud:measured_position() end
-- get commanded position
---@return number|nil -- commanded position in degrees or nil if not available
function AP_Servo_Telem_Data_ud:command_position() end

-- Simulator-specific bindings for SITL.
sim = {}

-- Sets the pose of a simulated vehicle. Requires `AHRS_EKF_TYPE=10` (SITL).
---@param instance integer -- 0 for the first vehicle.
---@param loc Location_ud -- The new location.
---@param orient Quaternion_ud -- The new orientation.
---@param velocity_bf Vector3f_ud -- The new body frame velocity.
---@param gyro_rads Vector3f_ud -- The new gyro body rate in rad/s.
---@return boolean
function sim:set_pose(instance, loc, orient, velocity_bf, gyro_rads) end

-- A data structure representing a parameter in a CRSF menu.
---@class (exact) CRSFParameter_ud
local CRSFParameter_ud = {}

-- Creates a new CRSF menu parameter object.
---@return CRSFParameter_ud
function CRSFParameter() end

-- Gets the ID of the parameter.
---@return integer
function CRSFParameter_ud:id() end
-- Gets the contents of the parameter as a packed binary string.
---@return string
function CRSFParameter_ud:data() end

-- A data structure representing a menu in the CRSF system.
---@class (exact) CRSFMenu_ud
local CRSFMenu_ud = {}

-- Creates a new CRSF menu object with a pre-sized capacity.
---@param size integer -- The number of parameters this menu will hold.
---@return CRSFMenu_ud
function CRSFMenu(size) end

-- Gets the ID of the menu.
---@return integer
function CRSFMenu_ud:id() end
-- Gets the name of the menu.
---@return string
function CRSFMenu_ud:name() end
-- Gets the number of parameters in the menu.
---@return integer
function CRSFMenu_ud:num_params() end
-- Adds a new parameter to the menu.
---@param data string -- The binary encoded parameter data.
---@return CRSFParameter_ud|nil -- The newly created parameter object.
function CRSFMenu_ud:add_parameter(data) end
-- Adds a new sub-menu to this menu.
---@param name string -- The name for the new sub-menu.
---@return CRSFMenu_ud|nil -- The newly created menu object.
function CRSFMenu_ud:add_menu(name) end

-- Interface for creating and managing CRSF (Crossfire) menus.
-- This allows scripts to create custom menus that can be accessed and modified from a compatible remote control.
crsf = {}

-- Adds a new top-level CRSF menu.
---@param name string -- The name of the menu to add.
---@return CRSFMenu_ud|nil -- The new CRSF menu object.
function crsf:add_menu(name) end

-- Gets a pending CRSF menu event (e.g., a user changing a parameter value from their radio).
---@param events integer -- A bitmask of events to listen for.
---| '1' # PARAMETER READ
---| '2' # PARAMETER WRITE
---@return integer -- The parameter ID of the event.
---@return string -- The binary encoded response payload.
---@return integer -- The bitmask of triggered events.
function crsf:get_menu_event(events) end

-- Sends a response to a CRSF parameter write request.
---@param data string -- The binary encoded response payload.
---@return boolean -- True if the response was sent successfully.
function crsf:send_write_response(data) end

-- The DroneCAN_Handle object is the primary tool for interacting with the DroneCAN bus.
-- It allows for creating subscriptions to specific message types and for sending messages.
---@class DroneCAN_Handle_ud
local DroneCAN_Handle_ud = {}

-- Creates a DroneCAN_Handle, which is required for all other DroneCAN message operations.
-- This object encapsulates the necessary information to send or receive a specific DroneCAN message.
-- Example from DroneCAN_test.lua: `local dc_handle = DroneCAN_Handle(0, SIGNATURE, ID, true)`
---@param driver_index number -- DroneCAN driver index (e.g., 0 for CAN1, 1 for CAN2). This must correspond to a driver enabled with CAN_Dx_PROTOCOL = 10 or 12.
---@param signature uint64_t_ud -- The unique 64-bit signature of the message type.
---@param data_type number -- The data type ID of the message.
---@param canfd? boolean -- Optional. If true, messages will be sent as CAN FD frames, which allows for larger payloads. Defaults to false.
---@return DroneCAN_Handle_ud -- The created DroneCAN handle object.
function DroneCAN_Handle(driver_index, signature, data_type, canfd) end

-- Subscribes the handle to receive messages that match its signature and data type.
-- This must be called before `check_message` will return any broadcast or request messages for this handle.
-- Example: `nodestatus_handle:subscribe()`
---@return boolean -- Returns true on success.
function DroneCAN_Handle_ud:subscribe() end

-- Checks if a new message has been received for a subscribed handle.
-- This function is non-blocking and should be polled periodically in your script's main loop to process incoming data.
-- Example: `local payload, nodeid = airspeed_handle:check_message()`
---@return string|nil payload -- The raw payload of the message as a binary string, or nil if no new message is available.
---@return number|nil nodeid -- The node ID of the device that sent the message, or nil.
---@return uint64_t_ud|nil timestamp -- The timestamp of the message in microseconds since 1/1/1970, or nil.
---@return boolean|nil canfd -- True if the message was received as a CANFD frame, or nil.
function DroneCAN_Handle_ud:check_message() end

-- Sends a DroneCAN request (a unicast message) to a specific target node.
-- This is used for point-to-point communication. The target node is expected to send a response.
-- Example: `param_handle:request(TARGET_NODE:get(), payload)`
---@param target_node number -- The node ID of the device to send the request to.
---@param payload string -- The payload for the message, packed as a binary string.
---@return boolean -- Returns true if the request was sent successfully.
function DroneCAN_Handle_ud:request(target_node, payload) end

-- Sends a DroneCAN broadcast message to all nodes on the bus.
-- This is used for one-to-many communication where no response is expected.
-- Example: `dc_handle:broadcast(payload)`
---@param payload string -- The payload for the message, packed as a binary string.
---@return boolean -- Returns true if the broadcast was sent successfully.
function DroneCAN_Handle_ud:broadcast(payload) end


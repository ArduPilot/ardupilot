--[[
   driver for UltraMotion servos
   https://www.ultramotion.com/
   based on an earlier driver by Fred Darnell
--]]

local PARAM_TABLE_KEY = 89
local PARAM_TABLE_PREFIX = "UM_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: UM_SERVO_MASK
  // @DisplayName: Mask of UltraMotion servos
  // @Description: Mask of UltraMotion servos
  // @Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12
  // @User: Standard
--]]
UM_SERVO_MASK = bind_add_param("SERVO_MASK", 1, 0)

--[[
  // @Param: UM_RATE_HZ
  // @DisplayName: Update rate for UltraMotion servos
  // @Description: Update rate for UltraMotion servos
  // @Units: Hz
  // @Range: 1 400
  // @User: Standard
--]]
UM_RATE_HZ = bind_add_param("RATE_HZ", 2, 70)

--[[
  // @Param: UM_TORQUE_MAX
  // @DisplayName: Maximum torque to command
  // @Description: Maximum torque to command
  // @Units: %
  // @Range: 1 100
  // @User: Standard
--]]
UM_TORQUE_MAX = bind_add_param("TORQUE_MAX", 3, 30.5) -- default matches datasheet

--[[
  // @Param: UM_OPTIONS
  // @DisplayName: Optional settings
  // @Description: Optional settings
  // @Bitmask: 0:LogAllFrames,1:ParseTelemetry
  // @User: Standard
--]]
UM_OPTIONS = bind_add_param("OPTIONS", 4, 0)

--[[
  // @Param: UM_TELEM_TXID
  // @DisplayName: Telemetry ID
  // @Description: Telemetry ID
  // @User: Standard
--]]
UM_TELEM_TXID = bind_add_param("TELEM_TXID", 5, 0)

local OPTION_LOGALLFRAMES = 0x01
local OPTION_PARSETELEM = 0x02

if UM_SERVO_MASK:get() == 0 then
   gcs:send_text(MAV_SEVERITY.INFO, "UltraMotion UM_SERVO_MASK is empty")
   return
end

-- Load CAN driver, using the scripting protocol
-- use a buffer size of 25
local driver = CAN:get_device(25)
if not driver then
   gcs:send_text(MAV_SEVERITY.INFO, "UltraMotion: init failed")
   return
end

local frame_count = 0

-- marker for extended frame format
local CAN_FLAG_EFF = uint32_t(1)<<31

--[[
   frame logging - can be replayed with Tools/scripts/CAN/CAN_playback.py
--]]
local function log_can_frame(frame)
   logger:write("CANF",'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7','IBIBBBBBBBB',
                frame:id(),
                frame:dlc(),
                frame_count,
                frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                frame:data(4), frame:data(5), frame:data(6), frame:data(7))
   frame_count = frame_count + 1
end

--[[
   create a new actuator object
--]]
function Actuator(unitID)
    local o = {}
    o.unitID = unitID or 0
    -- pre-fill the msg ID to avoid expensive uint32_t operations at runtime
    o.msg = CANFrame()
    o.msg:id(CAN_FLAG_EFF | uint32_t(unitID))
    o.msg:dlc(4)
    return o
end

--[[
   put a 16 bit little endian value
--]]
local function put_uint16(msg, ofs, value)
   msg:data(ofs, value & 0xFF)
   msg:data(ofs+1, value >> 8)
end

--[[
   use the UM_SERVO_MASK to create a table of actuators
   need to restart scripting to change the UM_SERVO_MASK
--]]
local actuators = {}
for i = 1, 32 do
   local mask = 1 << (i-1)
   if UM_SERVO_MASK:get() & mask ~= 0 then
      actuators[i] = Actuator(i)
   end
end

--[[
   send outputs to all servos
--]]
local function send_outputs()
   local noutputs = #actuators
   local torque_max = math.floor(UM_TORQUE_MAX:get() * 32767.0 * 0.01)
   for i = 1, noutputs do
      local pwm = SRV_Channels:get_output_pwm_chan(actuators[i].unitID)
      local msg = actuators[i].msg

      put_uint16(msg, 0, pwm)
      put_uint16(msg, 2, torque_max)

      driver:write_frame(msg, 10000)
   end
end

--[[
   parse one telemetry frame. The telemetry data format depends on
   the txData parameter set in the servos

   This code assumes txData is pABCDEFS
--]]
local function parse_telemetry(frame)
   local bytes = ""
   local dlc = frame:dlc()
   for i = 1, dlc do
      bytes = bytes .. string.char(frame:data(i-1))
   end
   local unitid, status, current, pos = string.unpack("<BiHB", bytes)
   logger:write("UMSV",'Id,Status,Curr,Pos','Biff', unitid, status, current, pos)
end

--[[
   read any incoming CAN frames
--]]
local function read_frames()
   for _ = 1,30 do
      local frame = driver:read_frame()
      if not frame then
         return
      end
      if UM_OPTIONS:get() & OPTION_LOGALLFRAMES ~= 0 then
         log_can_frame(frame)
      end
      if UM_OPTIONS:get() & OPTION_PARSETELEM ~= 0 then
         if frame:id_signed() == UM_TELEM_TXID:get() then
            parse_telemetry(frame)
         end
      end
   end
end

function update()
   send_outputs()
   read_frames()
   return update, 1000/UM_RATE_HZ:get()
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded UltraMotion with %u actuators", #actuators))

return update, 100

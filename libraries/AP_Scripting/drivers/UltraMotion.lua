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
  // @Param: UM_CANDRV
  // @DisplayName: Set CAN driver
  // @Description: Set CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local UM_CANDRV = bind_add_param('CANDRV', 2, 1)    -- CAN driver to use

--[[
  // @Param: UM_RATE_HZ
  // @DisplayName: Update rate for UltraMotion servos
  // @Description: Update rate for UltraMotion servos
  // @Units: Hz
  // @Range: 1 400
  // @User: Standard
--]]
UM_RATE_HZ = bind_add_param("RATE_HZ", 3, 70)

--[[
  // @Param: UM_OPTIONS
  // @DisplayName: Optional settings
  // @Description: Optional settings
  // @Bitmask: 0:LogAllFrames,1:ParseTelemetry,2:SendPosAsNamedValueFloat
  // @User: Standard
--]]
UM_OPTIONS = bind_add_param("OPTIONS", 5, 0)

local OPTION_LOGALLFRAMES = 0x01
local OPTION_PARSETELEM = 0x02
local OPTION_NVF_TELEM_POS = 0x04

if UM_SERVO_MASK:get() == 0 then
   gcs:send_text(MAV_SEVERITY.INFO, "UltraMotion UM_SERVO_MASK is empty")
   return
end

-- Load CAN driver, using the scripting protocol
-- use a buffer size of 25
local CAN_BUF_LEN = 25
if UM_CANDRV:get() == 1 then
   driver = CAN:get_device(CAN_BUF_LEN)
elseif UM_CANDRV:get() == 2 then
   driver = CAN:get_device2(CAN_BUF_LEN)
end
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
    o.msg:dlc(2)
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
   for _, actuator in pairs(actuators) do
      local pwm = SRV_Channels:get_output_pwm_chan(actuator.unitID-1)
      local msg = actuator.msg

      put_uint16(msg, 0, pwm)

      driver:write_frame(msg, 10000)
   end
end

--[[
   parse one telemetry frame. The telemetry data format depends on
   the txData parameter set in the servos

   This code assumes txData is KLMGHEFY. See the datasheet for the
   meaning of these data codes
--]]
local function parse_telemetry(frame)
   local bytes = ""
   local dlc = frame:dlc()
   for i = 1, dlc do
      bytes = bytes .. string.char(frame:data(i-1))
   end
   local swordlow, sword24, pos, curr, temp = string.unpack("<HBHHB", bytes)
   local statusword = swordlow | (sword24 << 16)
   local txid = frame:id_signed()
   local pos_scaled = pos / 65535.0
   local current_scaled = curr / 32767.0
   local temp_scaled = temp - 50.0
   if txid < 256 then
      logger:write('UMSV','Id,Status,Curr,Pos,Temp','Bifff','#----','-----', txid, statusword, current_scaled, pos_scaled, temp_scaled)
      if UM_OPTIONS:get() & OPTION_NVF_TELEM_POS ~= 0 then
         gcs:send_named_float(string.format('UMPOS_%u',txid), pos_scaled)
      end
   end
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
         if frame:dlc() == 8 then
            -- assume any 8 byte frame is a telemetry frame
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

-- Build an array of IDs to print out
local ids = {}
for _, actuator in pairs(actuators) do
   table.insert(ids, actuator.unitID)
end
gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded UltraMotion with %u actuators: %s", #ids, table.concat(ids, ",")))

return update, 100

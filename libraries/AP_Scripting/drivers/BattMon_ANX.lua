--[[ 
 device driver for ANX CAN battery monitor
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 45
local PARAM_TABLE_PREFIX = "BATT_ANX_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Type conversion functions, little endian
function get_uint16(frame, ofs)
    return frame:data(ofs) + (frame:data(ofs + 1) << 8)
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: BATT_ANX_ENABLE
  // @DisplayName: Enable ANX battery support
  // @Description: Enable ANX battery support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local BATT_ANX_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: BATT_ANX_CANDRV
  // @DisplayName: Set ANX CAN driver
  // @Description: Set ANX CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local BATT_ANX_CANDRV     = bind_add_param('CANDRV',     2, 1)

--[[
  // @Param: BATT_ANX_INDEX
  // @DisplayName: ANX CAN battery index
  // @Description: ANX CAN battery index
  // @Range: 1 10
  // @User: Standard
--]]
local BATT_ANX_INDEX     = bind_add_param('INDEX',     3, 1)

--[[
  // @Param: BATT_ANX_OPTIONS
  // @DisplayName: ANX CAN battery options
  // @Description: ANX CAN battery options
  // @Bitmask: 0:LogAllFrames
  // @User: Advanced
--]]
local BATT_ANX_OPTIONS   = bind_add_param('OPTIONS',    4, 0)

local OPTION_LOGALLFRAMES = 0x01

if BATT_ANX_ENABLE:get() == 0 then
   gcs:send_text(0, string.format("BATT_ANX: disabled"))
   return
end

-- Register for the CAN drivers
local driver

local CAN_BUF_LEN = 25
if BATT_ANX_CANDRV:get() == 1 then
   driver = CAN.get_device(CAN_BUF_LEN)
elseif BATT_ANX_CANDRV:get() == 2 then
   driver = CAN.get_device2(CAN_BUF_LEN)
end

if not driver then
    gcs:send_text(0, string.format("BATT_ANX: Failed to load driver"))
    return
end

local assembly = {}
assembly.num_frames = 0
assembly.frames = {}

--[[
   xmodem CRC implementation thanks to https://github.com/cloudwu/skynet
   under MIT license
--]]
local XMODEMCRC16Lookup = {
   0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
   0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
   0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
   0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
   0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
   0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
   0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
   0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
   0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
   0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
   0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
   0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
   0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
   0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
   0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
   0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
   0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
   0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
   0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
   0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
   0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
   0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
   0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
   0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
   0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
   0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
   0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
   0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
   0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
   0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
   0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
   0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
}

local function crc_ANX(bytes)
   -- ANX CRC uses xmodem with a seed of 0xa635
   local crc = 0xa635
	for i=1,#bytes do
		local b = string.byte(bytes,i,i)
		crc = ((crc<<8) & 0xffff) ~ XMODEMCRC16Lookup[(((crc>>8)~b) & 0xff) + 1]
	end
    return crc
end

local frame_count = 0

local function log_can_frame(frame)
   logger.write("CANF",'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7','IBIBBBBBBBB',
                frame:id(),
                frame:dlc(),
                frame_count,
                frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                frame:data(4), frame:data(5), frame:data(6), frame:data(7))
   frame_count = frame_count + 1
end
 

local function parse_volt_frame(payload)
   if #payload < 12 then
      -- invalid length
      return
   end
   local total_volt, current, rem_cap, temperature, _ = string.unpack("<HhHhH", string.sub(payload, 1, 10))
   total_volt = total_volt * 0.01
   -- it is a discharge, so use minus current
   current = -current * 0.01
   rem_cap = rem_cap * 0.1
   temperature = temperature * 0.1
   local num_cells = (#payload - 10) / 2
   local cells = {}
   for i = 1, num_cells do
      cells[i] = string.unpack("<H", string.sub(payload, 11+(i-1)*2, 11+i*2))
      cells[i] = cells[i]
   end
   -- API allows for up to 32 cells, but battery backend is more limited
   if num_cells > 32 then
      num_cells = 32
   end

   local state = BattMonitorScript_State()
   state:healthy(true)
   state:voltage(total_volt)
   state:cell_count(num_cells)
   state:capacity_remaining_pct(math.floor(rem_cap))
   for i = 1, num_cells do
      state:cell_voltages(i-1, cells[i])
   end
   state:current_amps(current)
   state:temperature(temperature)

   battery:handle_scripting(BATT_ANX_INDEX:get()-1, state)
end


--[[
   process a set of frames for a whole packet
--]]
local function process_frames(msg_type_id)
   local bytes = ""
   for i = 1, assembly.num_frames do
      local dlc = assembly.frames[i]:dlc()
      for ofs = 1, dlc do
         bytes = bytes .. string.char(assembly.frames[i]:data(ofs-1))
      end
   end
   local crc = string.unpack("<H", string.sub(bytes, 1, 2))
   local payload = string.sub(bytes, 3, #bytes)
   if crc ~= crc_ANX(payload) then
      return
   end
   -- message types with cell voltages are from 721 to 727
   if msg_type_id >= 721 and msg_type_id <= 727 then
      parse_volt_frame(payload)
   end
end

--[[
   read from CAN bus, updating battery backend
--]]
local function read_can()
   while true do
      local frame = driver:read_frame()
      if not frame then
         return
      end
      if BATT_ANX_OPTIONS:get() & OPTION_LOGALLFRAMES ~= 0 then
         log_can_frame(frame)
      end
      if not frame:isExtended() then
         -- only want extended frames
         break
      end
      local id = frame:id_signed()
      -- local sender_id = id&0x7
      local last_pkt_id = (id>>3) & 1
      local pkt_count = (id>>4) & 0x3F
      -- local pkt_id = (id>>10) & 0x7f
      -- local trans_type = (id>>17) & 0x03
      local msg_type_id = (id>>19) & 0x3FF

      if pkt_count ~= assembly.num_frames then
         -- reset, non-contiguous packets
         assembly.num_frames = 0
      end

      assembly.num_frames = assembly.num_frames + 1
      assembly.frames[assembly.num_frames] = frame
      if last_pkt_id == 1 then
         process_frames(msg_type_id)
         -- reset for next frame
         assembly.num_frames = 0
      end
   end
end

function update()
   read_can()
   return update,10
end

gcs:send_text(MAV_SEVERITY.INFO, "BATT_ANX: Started")

return update()

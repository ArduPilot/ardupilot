--[[
   playback CANF frames from a log captured with CAN_Pn_OPTIONS=1

   Note! The log should have been stripped of everything but CANF messages, like this:

     mavlogdump.py -q -o CANF.bin inlog.BIN --type CANF

   For detailed instructions see CAN_playback.md

   This script replays ArrayCommand for playing back CAN servo commands. Edit below for
   playing back other frame types
--]]

local LOGNAME = "CANF.bin"

local can_drivers = { CAN:get_device(25), CAN:get_device2(25) }

if not can_drivers[1] and not can_drivers[2] then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local tstart = millis()

local logf = io.open(LOGNAME, "r")
assert(logf ~= nil, "failed to open CANF.bin")

local next_frame = nil
local first_frame_ms = nil
local frame_ms = nil

local CANF_len = 25

local ArrayCommand = 0x0803F20A

local driver = nil

local msg = CANFrame()
msg:id(uint32_t(ArrayCommand) | uint32_t(0x80000000))

local frame_buf = nil
local frame_buf_ofs = 0

--[[
   get a frame from a frame buffer
--]]
local function get_frame()
   local buf_count = 100
   if frame_buf == nil then
      -- read 100 at a time
      frame_buf = logf:read(CANF_len*buf_count)
      frame_buf_ofs = 0
   end
   local frame = string.sub(frame_buf, frame_buf_ofs+1, frame_buf_ofs+CANF_len)
   if #frame ~= CANF_len then
      frame_buf = logf:read(CANF_len*buf_count)
      frame_buf_ofs = 0
      if frame_buf == nil then
         return nil
      end
      frame = string.sub(frame_buf, frame_buf_ofs+1, frame_buf_ofs+CANF_len)
      if #frame ~= CANF_len then
         return nil
      end
   end
   frame_buf_ofs = frame_buf_ofs + CANF_len
   return frame
end

--[[
   check for a new frame
--]]
local function check_frame()
   local frame = get_frame()
   if not frame or #frame ~= CANF_len then
      gcs:send_text(MAV_SEVERITY.INFO, "CAN_playback: rewind")
      logf:seek("set", 0)
      first_frame_ms = nil
      tstart = millis()
      return
   end
   local h0, h1, _, t1, t2, bus, id, DLC = string.unpack('<BBBIIBIB', string.sub(frame, 1, 18))
   local tus = uint64_t(t2, t1)
   local tms = tus / 1000
   assert(h0 == 0xA3 and h1 == 0x95, "CAN_playback: bad frame")
   local mask = 0x0FFFFFFF
   if (id & mask) == ArrayCommand then
      if first_frame_ms == nil then
         first_frame_ms = tms
      end
      local data = string.sub(frame, 18)
      driver = can_drivers[bus+1]
      if driver ~= nil then
         for i=1, 8 do
            if i <= DLC then
               msg:data(i-1, data:byte(i))
            else
               msg:data(i-1, 0)
            end
         end
         msg:dlc(DLC)
         frame_ms = tms
         next_frame = frame
      end
   end
end

--[[
   read frames from the log and write to the bus when they match
--]]
function read_log()
   for _ = 1, 100 do
      if not next_frame then
         check_frame()
      end
      if next_frame then
         local now = millis()
         if (frame_ms - first_frame_ms):toint() > (now - tstart):toint() then
            -- not due yet
            return
         end
         if driver and driver:write_frame(msg, 10000) then
            next_frame = nil
         end
      end
   end
end

local function update()
   read_log()
   return update, 1
end

gcs:send_text(MAV_SEVERITY.INFO, "CAN_playback: starting")

return update(),1000

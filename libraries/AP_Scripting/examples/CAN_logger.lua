--[[
   This script captures raw packets to a log file for later playback using Tools/scripts/CAN/CAN_playback.py onto a CAN bus.
   Set CAN_D1_PROTOCOL to 10 for scripting.
   Also need LOG_DISARMED set to 1 if running this while disarmed.
--]]

---@diagnostic disable: param-type-mismatch

local can_driver = CAN:get_device(25)

if not can_driver then
   gcs:send_text(0,"No scripting CAN interface found")
   return
end

local last_print_ms = millis()
local frame_count = 0
local last_frame_count = 0

function update()

   local more_frames = true
   for _ = 1, 25 do
      local frame = can_driver:read_frame()
      if not frame then
         more_frames = false
         break
      end
      local id = frame:id()
      logger.write("CANF",'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7','IBIBBBBBBBB',
                   id,
                   frame:dlc(),
                   frame_count,
                   frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                   frame:data(4), frame:data(5), frame:data(6), frame:data(7))
      frame_count = frame_count + 1
   end

   local now = millis()
   if now - last_print_ms >= 1000 then
      local dt = (now - last_print_ms):tofloat()*0.001
      gcs:send_text(0, string.format("CAN: %.2f fps", (frame_count-last_frame_count)/dt))
      last_print_ms = now
      last_frame_count = frame_count
   end

   if more_frames then
      -- sleep for min possible time to try not to lose frames
      return update, 0
   end
   return update, 2
end

return update()

-- control SilentArrow drop

local button_number = 1
local button_state = button:get_button_state(button_number)

local MODE_MANUAL = 0
local MODE_AUTO = 10

local release_start_t = 0.0
local last_tick_t = 0.0

-- NOTES:
-- -- how to know if RC mode change should be allowed?

function release_trigger()
   gcs:send_text(0, string.format("release trigger"))
   arming:arm_force()
   vehicle:set_mode(MODE_AUTO)
   mission:set_current_cmd(1)
   notify:handle_rgb(0,255,0,0)
end

function set_standby()
   local mode = vehicle:get_mode()
   if mode ~= MODE_MANUAL then
      gcs:send_text(0, string.format("forcing standby MANUAL"))
      vehicle:set_mode(MODE_MANUAL)
      mission:set_current_cmd(1)
   end
   local gps_status = gps:status(0)
   if gps_status == 0 then
      -- red for no GPS
      notify:handle_rgb(255,0,0,2)
   elseif gps_status >= 3 then
      -- green blinking 3D lock, manual
      notify:handle_rgb(0,255,0,2)
   else
      -- yellow for no 3D lock
      notify:handle_rgb(255,255,0,2)
   end
   local t = 0.001 * millis():tofloat()
   if t - last_tick_t > 10 then
      last_tick_t = t
      gcs:send_text(0, string.format("Drop: MANUAL idle "))
   end
end

function update()
   local t = 0.001 * millis():tofloat()
   local state = button:get_button_state(button_number)
   if state ~= button_state then
      gcs:send_text(0, string.format("release: " .. tostring(state)))
      button_state = state
      if button_state then
         release_start_t = t
      end
   end

   if button_state and release_start_t > 0 and (t - release_start_t) > param:get('SCR_USER1') then
      release_trigger()
      release_start_t = 0.0
   end

   if not button_state then
      set_standby()
   end

   --reset_AHRS()

   -- run at 10Hz
   return update, 100
end

return update() -- run immediately before starting to reschedule

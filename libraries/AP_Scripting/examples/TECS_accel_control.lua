-- toggle TECS_OPTIONS bit 2 (value 4) for height acceleration control
-- set RCx_OPTION = 305 to control this script

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local AUX_FUNCTION_NUM = 305
local TECS_OPTIONS = Parameter('TECS_OPTIONS')
local HGT_ACCEL_BIT = 4  -- bit 2

local last_sw_pos = -1

function update()
   local sw_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
   if not sw_pos then
      return update, 100
   end
   if sw_pos == last_sw_pos then
      return update, 100
   end
   last_sw_pos = sw_pos

   local current = TECS_OPTIONS:get()
   if sw_pos == 2 then
      -- switch high: enable height accel control
      if (current & HGT_ACCEL_BIT) == 0 then
         TECS_OPTIONS:set(current | HGT_ACCEL_BIT)
         gcs:send_text(MAV_SEVERITY.INFO, "TECS height accel control enabled")
      end
   else
      -- switch low or middle: disable height accel control
      if (current & HGT_ACCEL_BIT) ~= 0 then
         TECS_OPTIONS:set(current & ~HGT_ACCEL_BIT)
         gcs:send_text(MAV_SEVERITY.INFO, "TECS height accel control disabled")
      end
   end
   return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded TECS height accel control switch")

return update()

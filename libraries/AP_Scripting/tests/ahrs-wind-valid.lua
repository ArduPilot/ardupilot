-- report whether the AHRS wind estimate is valid, for autotest
-- consumption; validity is not otherwise visible in telemetry (the
-- WIND message is sent regardless)

local function update()
   local wind = ahrs:get_wind()
   local valid = 0
   if wind then
      valid = 1
   end
   gcs:send_named_float('WINDVALID', valid)
   return update, 200
end

return update()

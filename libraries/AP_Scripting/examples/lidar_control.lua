-- enable use of Lidar on quadplanes only for landing, by changing RNGFN_LANDING

local RNGFND_LANDING = Parameter("RNGFND_LANDING")

MODE_QLAND = 20
MODE_QRTL = 21
MODE_AUTO = 10

local NAV_LAND = 21
local NAV_VTOL_LAND = 85

function in_landing()
   local mode = vehicle:get_mode()
   if mode == MODE_QRTL or mode == MODE_QLAND then
      return true
   end
   if mode == MODE_AUTO then
      local id = mission:get_current_nav_id()
      if id == NAV_VTOL_LAND or id == NAV_LAND then
         return true
      end
   end
   return false
end

-- convert a boolean to an int
function bool_to_int(v)
  return v and 1 or 0
end

function update()
   local v = bool_to_int(in_landing())
   if v ~= RNGFND_LANDING:get() then
      if v == 1 then
         gcs:send_text(0,"Enabling Lidar")
      else
         gcs:send_text(0,"Disabling Lidar")
      end
      RNGFND_LANDING:set(v)
   end

  -- run at 1Hz
  return update, 1000
end

return update()

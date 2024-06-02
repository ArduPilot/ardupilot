--[[
   demonstrate setting the airspeed for a plane in fixed wing mode
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local AUX_FN = 300
local last_sw = -1

--[[
   If the user changes the position of the scripting switch choose either
   LOW: AIRSPEED_MIN
   MID: AIRSPEED_CRUISE
   HIGH: AIRSPEED_MAX
--]]
local function set_desired_speed()
    local sw_current = rc:get_aux_cached(AUX_FN)
    if not sw_current then
       -- ignore untill the user flips the switch
       return
    end
    if sw_current ~= last_sw then
        last_sw = sw_current
        if sw_current == 0 then
           new_airspeed = Parameter("AIRSPEED_MIN"):get()
           gcs:send_text(MAV_SEVERITY.INFO, string.format("Speed set to AIRSPEED_MIN %f", new_airspeed))
        elseif sw_current == 1 then
            new_airspeed = Parameter("AIRSPEED_CRUISE"):get()
            gcs:send_text(MAV_SEVERITY.INFO, string.format("Speed set to AIRSPEED_CRUISE %f", new_airspeed))
        else
            new_airspeed = Parameter("AIRSPEED_MAX"):get()
            gcs:send_text(MAV_SEVERITY.INFO, string.format("Speed set to AIRSPEED_MAX %f", new_airspeed))
        end
        if not vehicle:set_desired_airspeed(new_airspeed) then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("Failed to set airspeed"))
        end
     end
end

local function update()
   if arming:is_armed() then
      set_desired_speed()
   end
   return update, 1000
end

return update, 1000

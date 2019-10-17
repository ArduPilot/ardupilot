-- Adds a smart failsafe that accounts for how far the plane is from home
-- the average battery consumption, and the wind to decide when to failsafe
--
-- CAUTION: This script only works for Plane


-- store the batt info as { instance, filtered, capacity }
local batt_info = { {0, 0, 0}, -- main battery
                               -- add more batteries to monitor here
                  }

-- FIXME: check that the script is actually being run on ArduPilane 

local margin = 30 -- margin in seconds that should be added for the time to get home
local filter = 0.9 -- filter gain
local air_speed = 25.0 -- this should really be fetched from a parameter

-- calculates the amount of time it will take for the vehicle to return home
-- returns 0 if there is no position, wind or home available
-- returns a negative number if it will take excessively long time, or is impossible
-- otherwise returns the time in seconds to get back
local function time_to_home()
  local home = ahrs:get_home()
  local position = ahrs:get_position()
  local wind = ahrs:wind_estimate()
  if home and position and wind then
    local bearing = position:get_bearing(home)
    local wind_bearing = math.atan(-wind:y(), -wind:x())
    local awa = math.abs(wind_bearing - bearing) -- acute wind angle
    local ws = wind:length() -- wind speed (m/s)
    local wca = math.asin(ws * math.sin(awa) / air_speed) -- wind correction angle
    -- adjust the wind speed if it's pushing us the way we want to go
    if awa > math.rad(90) then
      ws = ws * -1
    end
    -- compute the effective ground speed for getting home
    local effective_speed = air_speed - (math.cos(wca) * ws)
    -- clamp to prevent very large numbers
    if effective_speed < 1.0 then
      return -1 -- FIXME: this should really be infinity
    end
    return (position:get_distance(home) / effective_speed)
  end

  return 0 -- nothing useful available
end

local return_start

-- this is an alternate update function that is simply used to track how long it will take to get home
-- it's really only used for debugging how the prediction rules are working
function track_return_time()
  local home = ahrs:get_home()
  local position = ahrs:get_position()
  if home and position then
    if position:get_distance(home) < 200 then
      gcs:send_text(0, "Failsafe: RTL took " .. tostring((millis() - return_start) / 1000) .. " seconds")
      return
    end
  end
  return track_return_time, 100
end

-- the main update function that is used to decide when we should do a failsafe
function update()
  for i = 1, #batt_info do
    local instance, filtered_amps, rated_capacity_mah = table.unpack(batt_info[i])
    local amps = battery:current_amps(instance)
    local consumed_mah = battery:consumed_mah(instance)
    if amps and consumed_mah then
      -- update all the current consumption rates
      filtered_amps = (filtered_amps * filter) + (amps * (1.0 - filter))
      batt_info[i][2] = filtered_amps

      local remaining_capacity = (rated_capacity_mah - consumed_mah) * 3.6 -- amp seconds
      local remaining_time = remaining_capacity / filtered_amps
      local return_time = time_to_home()
      if (return_time < 0) or (remaining_time < (return_time + margin)) then
        -- FIXME: We need more insight into what the vehicles already doing. IE don't trigger RTL if we are already landing
        gcs:send_text(0, string.format("Failsafe: Estimated %.0f seconds to home", return_time))
        vehicle:set_mode(11) -- plane RTL FIXME: we need a set of enums defined for the vehicles
        -- swap to tracking the time rather then retrigger
        return_start = millis()
        return track_return_time, 100
      end
    end
  end

  return update, 100
end

-- validate that all the expected monitors have current monitoring capability, and fetch initial values
for i = 1, #batt_info do
  -- check that the instance exsists
  local instance = batt_info[i][1]
  if instance > battery:num_instances() then
    error("Battery " .. instance .. " does not exsist")
  end
  -- checkthat we can actually read current from the instance
  if not battery:current_amps(instance) then
    error("Battery " .. instance .. " does not support current monitoring")
  end
  -- store the pack capacity for later use, it's assumed to never change mid flight
  local rated_cap = battery:pack_capacity_mah(instance)
  if rated_cap then
    batt_info[i][3] = rated_cap
  else
    error("Battery " .. instance .. " does not support current monitoring")
  end
end

return update, 100

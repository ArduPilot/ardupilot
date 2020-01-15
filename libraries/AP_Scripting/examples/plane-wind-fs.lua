-- Adds a smart failsafe that accounts for how far the plane is from home
-- the average battery consumption, and the wind to decide when to failsafe
--
-- CAUTION: This script only works for Plane

-- store the batt info as { instance, filtered, capacity, margin_mah }
-- instance: the battery monitor instance (zero indexed)
-- filtered: internal variable for current draw
-- capacity: internal variable populated from battery monitor capacity param
-- margin_mah: the mah to be remaining when you reach home and use the time margin, note that this is on top of CRT_MAH
local batt_info = { {0, 0, 0, 0}, -- main battery
                                  -- add more batteries to monitor here
                  }

-- FIXME: check that the script is actually being run on ArduPlane 

local margin = 30 -- margin in seconds of flight time that should remain once we have reached 
local time_SF = 1 -- the return time safety factor, 1.1 gives 10% extra time for return flight
local filter = 0.9 -- filter gain
local min_flying_time = 30 -- seconds, must have been flying for this long before script will start sampling current
local print_time = 15 -- seconds between update prints (set to zero to disable)

-- if true current draw is normalized with dynamic pressure
-- this gives better prediction of current draws at other airspeeds
-- airspeed sensor recommended
local airspeed_normalize = false

-- hard code wind to match SITL (until param get need to set manually on line 72)
-- should get exact return time estimates < +- 5 seconds
local SITL_wind = true

-- set these variables to match your parameters

local air_speed = 22 -- TRIM_ARSPD_CM / 100
local min_air_speed = 10 -- ARSPD_FBW_MIN
local min_ground_speed = 0 -- MIN_GNDSPD_CM / 100
local max_bank_angle = 65 -- LIM_ROLL_CD / 100

-- https://en.wikipedia.org/wiki/Standard_rate_turn#Radius_of_turn_formula
-- the radius is equal to the circumference per 1 radian
local turn_rad = (air_speed^2) / (9.81 * math.tan(math.rad(max_bank_angle)))

-- Read the radius we expect to to circle at when we get home
local home_reached_rad = 200


-- internal global variables
local return_start
local return_distance
local return_amps
local trigger_instance = batt_info[1][1]
local last_print = 0

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

    -- wind is in NED, convert for readability
    local wind_north = wind:x()
    local wind_east = wind:y()

    -- hard code wind for testing
    if SITL_wind then
      wind_north = 0
      wind_east = 0
    end
    --gcs:send_text(0, string.format("Wind: north %0.2f, east  %0.2f",wind_north,wind_east))

    -- rotate the wind vector inline with the home bearing
    local tail_wind =  math.sin(bearing) * wind_east + math.cos(bearing) * wind_north
    local cross_wind = math.cos(bearing) * wind_east + math.sin(bearing) * wind_north   -- left to right

    -- we can't get home
    if math.abs(cross_wind) > air_speed then
      return -1, air_speed -- FIXME: this should really be infinity
    end

    -- calculate the crab angle required to cancel out the cross wind
    local crab_angle = math.asin(-cross_wind / air_speed)

    -- the resulting speed in the desired direction
    local home_airspeed = air_speed * math.cos(crab_angle)

    local effective_speed = home_airspeed + tail_wind

    -- Estimate the extra distance required to turn
    local yaw = ahrs:get_yaw()
    -- this is the estimated angle we have to turn to be at the home bearing and crab angle
    local turn_angle_rad = bearing - crab_angle - yaw 
    -- wrap to +- PI
    if turn_angle_rad < math.rad(-180) then
      turn_angle_rad = turn_angle_rad + math.rad(360)
    elseif turn_angle_rad > math.rad(180) then
      turn_angle_rad = turn_angle_rad - math.rad(360)
    end
    --gcs:send_text(0, "turn " .. tostring(math.deg(turn_angle_rad)) .. " deg")

    -- Take into account min ground speed and resulting increased RTL airspeed
    local return_air_speed = air_speed
    if min_ground_speed > 0 and effective_speed < min_ground_speed then
      -- we travel home at the min ground speed 
      effective_speed = min_ground_speed

      -- work out the resulting airspeed
      return_air_speed = math.sqrt((effective_speed-tail_wind)^2 + cross_wind^2)
    end

    -- distance to travel over ground speed + turn circumference over airspeed
    return (position:get_distance(home) / effective_speed) + (math.abs(turn_angle_rad*turn_rad) / air_speed), return_air_speed
  end

  return 0, air_speed -- nothing useful available
end

-- idle function, to stop no scripts to run warning
function idle()
  return idle, 10000
end

-- time margin update function
function margin_update()
  if not vehicle:get_likely_flying() then
    -- no longer flying, idle function
    return idle, 10000
  end

  -- display the remaining battery capacity on the triggered monitor
  local capacity = battery:pack_capacity_mah(trigger_instance) 
  local consumed = battery:consumed_mah(trigger_instance)
  if capacity and consumed then
    gcs:send_text(0, string.format("Failsafe: %is margin elapsed %.2fmAh remain",margin, capacity - consumed))
  end

  -- idle function
  return idle, 10000
end

-- this is an alternate update function that is simply used to track how long it will take to get home
-- it's really only used for debugging how the prediction rules are working
function track_return_time()
  if not vehicle:get_likely_flying() then
      -- no longer flying, idle function
      return idle, 10000
  end

  local home = ahrs:get_home()
  local position = ahrs:get_position()
  if home and position then
    local now = millis()

    local home_dist = position:get_distance(home)
    if home_dist < home_reached_rad then
      -- calculate the extra time to fly the reached rad distance
      local time_home = time_to_home()
      local total_time = time_home + ((now-return_start)/1000)

      -- display the remaining battery capacity on the triggered monitor
      local capacity = battery:pack_capacity_mah(trigger_instance) 
      local consumed = battery:consumed_mah(trigger_instance)
      if capacity and consumed then
        -- estimate the extra capacity used for the reached rad distance
        return_capacity = return_amps * time_home  * (1000 / 60^2) -- convert from amp second's to mAh
        
        gcs:send_text(0, "Failsafe: RTL took " .. tostring(total_time) .. string.format("s, %.2fmAh remain", capacity - consumed - return_capacity) )

        if margin > 0 then
          return margin_update, margin*1000
        end
      else
        gcs:send_text(0, "Failsafe: RTL took " .. tostring(total_time) .. " s")
      end

      -- idle function
      return idle, 10000
    end

    -- print updates tracking progress
    if last_print + (print_time * 1000) < now and print_time > 0 then
      last_print = now
      local return_time = time_to_home()
      if (return_time < 0) then
        gcs:send_text(6, "Failsafe: ground speed low can not get home")
      elseif (return_time > 0) then
        local total_time = return_time + ((now-return_start)/1000)
        -- cannot get string.format() to work with total time, wrong variable type? ie not %f or %i?
        gcs:send_text(0, "Failsafe: Estimated " .. tostring(total_time) .. string.format("s, %.0fs remain", return_time) )
      end
    end

  end
  return track_return_time, 100
end

-- the main update function that is used to decide when we should do a failsafe
function update()
  local flying_time = vehicle:get_time_flying_ms()
  if flying_time < (min_flying_time * 1000) then
    -- we have not been flying for long enough, skip
    return update, 100
  end
  local air_speed_in = ahrs:airspeed_estimate()
  if not air_speed_in then
    error("Could not read airspeed")
  end
  if air_speed_in  < min_air_speed * 0.75 then
    -- we are not flying fast enough, skip
    return update, 100
  end

  local now = millis()
  local min_remaining_time = 86400 -- 24 hours

  -- find the return time and airspeed
  local return_time, return_airspeed = time_to_home()

  -- default to no normalization
  local q = 1
  local return_q = 1

  -- normalize current with dynamic pressure
  if airspeed_normalize then

    -- we could probably just use air speed^2 
    local press = baro:get_pressure()
    local temp = baro:get_external_temperature() + 273.2 -- convert deg centigrade to kelvin
    local density =  press / (temp * 287.058) -- calculate the air density, ideal gas law, constant is (R) specific gas constant for air
    q = 0.5 * density * air_speed_in^2

    return_q = 0.5 * density * return_airspeed^2 -- we could estimate the change in density also, but will be negligible
  end

  for i = 1, #batt_info do
    local instance, norm_filtered_amps, rated_capacity_mah = table.unpack(batt_info[i])
    local amps = battery:current_amps(instance)
    local consumed_mah = battery:consumed_mah(instance)

    if amps and consumed_mah then

      local norm_amps = amps / q

      -- update all the current consumption rates
      norm_filtered_amps = (norm_filtered_amps * filter) + (norm_amps * (1.0 - filter))
      batt_info[i][2] = norm_filtered_amps

      -- calculate the estimated return amps, estimate the return current if we were to fly at a different airspeed
      return_amps = norm_filtered_amps * return_q

      local remaining_capacity = (rated_capacity_mah - consumed_mah) * 3.6 -- amp seconds (60^2 / 1000)
      local remaining_time = remaining_capacity / return_amps
      
      local buffer_time = remaining_time - ((return_time * time_SF) + margin)
      if  (return_time < 0) or buffer_time < 0 then
        if return_time < 0 then
          gcs:send_text(0, "Failsafe: ground speed low can not get home")
        elseif #batt_info == 1 then
          gcs:send_text(0, string.format("Failsafe: Estimated %.0fs to home", return_time))
        else
          trigger_instance = instance
          gcs:send_text(0, string.format("Failsafe: Estimated %.0fs to home, instance %i", return_time, instance))
        end

        last_print = now
        -- FIXME: We need more insight into what the vehicles already doing. IE don't trigger RTL if we are already landing
        vehicle:set_mode(11) -- plane RTL FIXME: we need a set of enums defined for the vehicles
        -- swap to tracking the time rather then re trigger
        return_start = now

        -- Print the return distance
        --[[local home = ahrs:get_home()
        local position = ahrs:get_position()
        if home and position then
          return_distance = position:get_distance(home)
        end
        gcs:send_text(0, string.format("Failsafe: %.0f m to home", return_distance))]]

        -- print the current draw we estimate
        --gcs:send_text(0, string.format("Failsafe: %.2fa", return_amps))

        return track_return_time, 100
      end
      min_remaining_time = math.min(min_remaining_time, buffer_time)
    end
  end

  -- print updates tracking progress
  if last_print + (print_time * 1000) < now and print_time > 0 then
    last_print = now
    gcs:send_text(6, string.format("%.0f seconds of flight remaining before RTL", min_remaining_time))
  end

  return update, 100
end

-- validate that all the expected monitors have current monitoring capability, and fetch initial values
for i = 1, #batt_info do
  -- check that the instance exists
  local instance = batt_info[i][1]
  if instance > battery:num_instances() then
    error("Battery " .. instance .. " does not exist")
  end
  -- check that we can actually read current from the instance
  if not battery:current_amps(instance) then
    error("Battery " .. instance .. " does not support current monitoring")
  end
  -- store the pack capacity for later use, it's assumed to never change mid flight
  -- subtract the capacity we want remaining when we get home
  local rated_cap = battery:pack_capacity_mah(instance)
  if rated_cap then
    batt_info[i][3] = rated_cap - batt_info[i][4]
  else
    error("Battery " .. instance .. " does not support current monitoring")
  end
end

return update, 100

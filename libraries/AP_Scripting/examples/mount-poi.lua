-- mount-poi.lua: finds the point-of-interest that the gimbal mount is pointing at using the vehicle's current Location, mount attitude and terrain database
--
-- How To Use
--   1. set RCx_OPTION to 300 to enable triggering the POI calculation from an auxiliary switch
--   2. optionally set POI_DIST_MAX to the maximum distance (in meters) that the POI point could be from the vehicle
--   3. fly the vehicle and point the camera gimbal at a point on the ground
--   4. raise the RC auxiliary switch and check the GCS's messages tab for the latitude, longitude and alt (above sea-level)
--
-- How It Works
--   1. retrieve the POI_DIST_MAX and TERRAIN_SPACING param values
--   2. get the vehicle Location (lat, lon, height above sea-level), initialise test-loc and prev-test-loc
--   3. get the vehicle's current alt-above-terrain
--   4. get gimbal attitude (only pitch and yaw are used)
--   5. "test_loc" is initialised to the vehicle's location
--   6. "prev_test_loc" is a backup of test_loc
--   7. test_loc is moved along the line defined by the gimbal's pitch and yaw by TERRAIN_SPACING (meters)
--   8. retrieve the terrain's altitude (above sea-level) at the test_loc
--   9. repeat step 6, 7 and 8 until the test_loc's altitude falls below the terrain altitude
--  10. interpolate between test_loc and prev_test_loc to find the lat, lon, alt (above sea-level) where alt-above-terrain is zero
--  11. display the POI to the user

-- global definitions
local ALT_FRAME_ABSOLUTE = 0
local UPDATE_INTERVAL_MS = 100

-- add new param POI_DIST_MAX
local PARAM_TABLE_KEY = 78
assert(param:add_table(PARAM_TABLE_KEY, "POI_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DIST_MAX", 10000), "could not add POI_DIST_MAX param")

local poi_dist_max = Parameter()
assert(poi_dist_max:init("POI_DIST_MAX"), "could not find POI_DIST_MAX param")

-- bind to other parameters this script depends upon
TERRAIN_SPACING = Parameter("TERRAIN_SPACING")

-- local variables and definitions
local user_update_interval_ms = 10000   -- send user updates every 10 sec
local last_user_update_ms = 0           -- system time that update was last sent to user
local last_rc_switch_pos = 0            -- last known rc switch position.  Used to detect change in RC switch position

-- helper functions
function wrap_360(angle_deg)
  local res = math.fmod(angle_deg, 360.0)
  if res < 0 then
    res = res + 360.0
  end
  return res
end

function wrap_180(angle_deg)
  local res = wrap_360(angle_deg)
  if res > 180 then
    res = res - 360
  end
  return res
end

function swap_float(f1, f2)
  return f2, f1
end

function interpolate(low_output, high_output, var_value, var_low, var_high)
  -- support either polarity
  if (var_low > var_high) then
    var_low, var_high = swap_float(var_low, var_high)
    low_output, high_output = swap_float(low_output, high_output)
  end
  if (var_value <= var_low) then
    return low_output
  end
  if (var_value > var_high) then
    return high_output
  end
  local p = (var_value - var_low) / (var_high - var_low)
  return (low_output + p * (high_output - low_output))
end


-- the main update function that performs a simplified version of RTL
function update()

  -- get current system time
  local now_ms = millis()

  -- find RC channel used to trigger POI
  rc_switch_ch = rc:find_channel_for_option(300) --scripting ch 1
  if (rc_switch_ch == nil) then
      gcs:send_text(3, 'MountPOI: RCx_OPTION = 300 not set')    -- MAV_SEVERITY_ERROR
      return update, 10000  -- check again in 10 seconds
  end

  -- check if user has raised RC switch
  local rc_switch_pos = rc_switch_ch:get_aux_switch_pos()
  if rc_switch_pos == last_rc_switch_pos then
    return update, UPDATE_INTERVAL_MS
  end

  -- switch has changed position
  last_rc_switch_pos = rc_switch_pos
  if rc_switch_pos ~= 2 then
    return update, UPDATE_INTERVAL_MS
  end

  -- POI has been requested

  -- retrieve vehicle location
  local vehicle_loc = ahrs:get_location()
  if vehicle_loc == nil then
    gcs:send_text(3, "POI: vehicle pos unavailable") -- MAV_SEVERITY_ERROR
    return update, UPDATE_INTERVAL_MS
  end
  
  -- change vehicle location to ASML
  vehicle_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)

  -- retrieve gimbal attitude
  local roll_deg, pitch_deg, yaw_bf_deg = mount:get_attitude_euler(0)
  if pitch_deg == nil or yaw_bf_deg == nil then
    gcs:send_text(3, "POI: gimbal attitude unavailable") -- MAV_SEVERITY_ERROR
    return update, UPDATE_INTERVAL_MS
  end

  -- project forward from vehicle looking for terrain
  -- start testing at vehicle's location
  local test_loc = vehicle_loc:copy()
  local prev_test_loc = test_loc:copy()

  -- get terrain altitude (asml) at test_loc
  local terrain_amsl_m = terrain:height_amsl(test_loc, true)  -- terrain alt (above amsl) at test_loc
  local prev_terrain_amsl_m = terrain_amsl_m    -- terrain alt (above amsl) at prev_test_loc

  -- fail if terrain alt cannot be retrieved
  if terrain_amsl_m == nil then
    gcs:send_text(3, "POI: failed to get terrain alt") -- MAV_SEVERITY_ERROR
    return update, UPDATE_INTERVAL_MS
  end

  -- get gimbal mount's pitch and yaw
  local mount_pitch_deg = pitch_deg
  local mount_yaw_ef_deg = wrap_180(yaw_bf_deg + math.deg(ahrs:get_yaw()))
  local dist_increment_m = TERRAIN_SPACING:get()

  -- initialise total distance test_loc has moved
  local total_dist = 0
  local dist_max = poi_dist_max:get()

  -- iteratively move test_loc forward until its alt-above-sea-level is below terrain-alt-above-sea-level
  while (total_dist < dist_max) and ((test_loc:alt() * 0.01) > terrain_amsl_m) do
    total_dist = total_dist + dist_increment_m

    -- take backup of previous test location and terrain asml
    prev_test_loc = test_loc:copy()
    prev_terrain_amsl_m = terrain_amsl_m

    -- move test location forward
    test_loc:offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_increment_m)

    -- get terrain's alt-above-sea-level (at test_loc)
    terrain_amsl_m = terrain:height_amsl(test_loc, true)

    -- fail if terrain alt cannot be retrieved
    if terrain_amsl_m == nil then
      gcs:send_text(3, "POI: failed to get terrain alt") -- MAV_SEVERITY_ERROR
      return update, UPDATE_INTERVAL_MS
    end
  end

  -- check for errors
  if (total_dist >= dist_max) then
    gcs:send_text(3, "POI: unable to find terrain within " .. tostring(dist_max) .. " m")
  elseif not terrain_amsl_m then
    gcs:send_text(3, "POI: failed to retrieve terrain alt")
  else
    -- test location has dropped below terrain
    -- interpolate along line between prev_test_loc and test_loc
    local dist_interp_m = interpolate(0, dist_increment_m, 0, prev_test_loc:alt() * 0.01 - prev_terrain_amsl_m, test_loc:alt() * 0.01 - terrain_amsl_m)
    local poi_loc = prev_test_loc:copy()
    poi_loc:offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_interp_m)
    local poi_terr_asml_m = terrain:height_amsl(poi_loc, true) 
    gcs:send_text(6, string.format("POI %.7f, %.7f, %.2f (asml)", poi_loc:lat()/10000000.0, poi_loc:lng()/10000000.0, poi_loc:alt() * 0.01))
  end

  return update, UPDATE_INTERVAL_MS
end

return update()

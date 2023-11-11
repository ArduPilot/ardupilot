-- mount-poi.lua: finds the point-of-interest that the gimbal mount is pointing at using the vehicle's current Location, mount attitude and terrain database
--
-- How To Use
--   1. Set RCx_OPTION to 300 or 301 to enable triggering the POI calculation from an auxiliary switch.  If 301 is used the gimbal will also lock onto the location
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
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local ALT_FRAME_ABSOLUTE = 0
local UPDATE_INTERVAL_MS = 100

-- add new param POI_DIST_MAX
local PARAM_TABLE_KEY = 78
assert(param:add_table(PARAM_TABLE_KEY, "POI_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DIST_MAX", 10000), "could not add POI_DIST_MAX param")

--[[
  // @Param: POI_DIST_MAX
  // @DisplayName: Mount POI distance max
  // @Description: POI's max distance (in meters) from the vehicle
  // @Range: 0 10000
  // @User: Standard
--]]
local POI_DIST_MAX = Parameter("POI_DIST_MAX")

-- bind to other parameters this script depends upon
TERRAIN_SPACING = Parameter("TERRAIN_SPACING")

-- local variables and definitions
local last_poi_switch_pos = 0           -- last known rc poi switch position.  Used to detect change in RC switch position
local last_roi_switch_pos = 0           -- last known rc roi switch position.  Used to detect change in RC switch position
local success_count = 0                 -- count of the number of POI calculations (sent to GCS in CAMERA_FEEDBACK message)

-- mavlink message definition
-- initialise mavlink rx with number of messages, and buffer depth
mavlink.init(1, 10)
local messages = {}
messages[180] = { -- CAMERA_FEEDBACK
             { "time_usec", "<I8" },
             { "lat", "<i4" },
             { "lng", "<i4" },
             { "alt_msl", "<f" },
             { "alt_rel", "<f" },
             { "roll", "<f" },
             { "pitch", "<f" },
             { "yaw", "<f" },
             { "foc_len", "<f" },
             { "img_idx", "<I2" },
             { "target_system", "<B" },
             { "cam_idx", "<B" },
             { "flags", "<B" },
             { "completed_captures", "<I2" },
             }

function encode(msgid, message, messages_array)
  local message_map = messages_array[msgid]
  if not message_map then
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgid)
  end

  local packString = "<"
  local packedTable = {}                  
  local packedIndex = 1
  for i,v in ipairs(message_map) do
    if v[3] then
      packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
      for j = 1, v[3] do
        packedTable[packedIndex] = message[message_map[i][1]][j]
        packedIndex = packedIndex + 1
      end
    else
      packString = (packString .. string.sub(v[2], 2))
      packedTable[packedIndex] = message[message_map[i][1]]
      packedIndex = packedIndex + 1
    end
  end

  return string.pack(packString, table.unpack(packedTable))
end

-- send CAMERA_FEEDBACK message to GCS
function send_camera_feedback(lat_degE7, lon_degE7, alt_msl_m, alt_rel_m, roll_deg, pitch_deg, yaw_deg, foc_len_mm, feedback_flags, captures_count)
  -- prepare camera feedback msg
  local camera_feedback_msg = {
      time_usec = micros():toint(),
      target_system = 0,
      cam_idx = 0,
      img_idx = 1,
      lat = lat_degE7,
      lng = lon_degE7,
      alt_msl = alt_msl_m,
      alt_rel = alt_rel_m,
      roll = roll_deg,
      pitch = pitch_deg,
      yaw = yaw_deg,
      foc_len = foc_len_mm,
      flags = feedback_flags,
      completed_captures = captures_count
  }

  -- send camera feedback msg
  local encoded_msg = encode(180, camera_feedback_msg, messages)
  mavlink.send_chan(0, 180, encoded_msg)
  mavlink.send_chan(1, 180, encoded_msg)
end

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

gcs:send_text(MAV_SEVERITY.INFO, "Mount-poi script started")

-- the main update function called at 10hz
function update()

  -- check if user has raised POI switch
  local poi_switch_pos = rc:get_aux_cached(300) -- scripting ch 1 (drop icon on map where mount is pointing)
  local poi_switch_pulled_high = (poi_switch_pos ~= nil) and (poi_switch_pos ~= last_poi_switch_pos) and (poi_switch_pos == 2)
  last_poi_switch_pos = poi_switch_pos

  -- check if user has raised ROI switch
  local roi_switch_pos = rc:get_aux_cached(301) -- scripting ch 2 (drop icon and lock mount on location)
  local roi_switch_pulled_high = (roi_switch_pos ~= nil) and (roi_switch_pos ~= last_roi_switch_pos) and (roi_switch_pos == 2)
  last_roi_switch_pos = roi_switch_pos
  
  -- return if neither switch was pulled high
  if not poi_switch_pulled_high and not roi_switch_pulled_high then
    return update, UPDATE_INTERVAL_MS
  end

  -- POI or ROI has been requested

  -- retrieve vehicle location
  local vehicle_loc = ahrs:get_location()
  if vehicle_loc == nil then
    gcs:send_text(MAV_SEVERITY.ERROR, "POI: vehicle pos unavailable")
    return update, UPDATE_INTERVAL_MS
  end
  
  -- change vehicle location to ASML
  vehicle_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)

  -- retrieve gimbal attitude
  local _, pitch_deg, yaw_bf_deg = mount:get_attitude_euler(0)
  if pitch_deg == nil or yaw_bf_deg == nil then
    gcs:send_text(MAV_SEVERITY.ERROR, "POI: gimbal attitude unavailable")
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
    gcs:send_text(MAV_SEVERITY.ERROR, "POI: failed to get terrain alt")
    return update, UPDATE_INTERVAL_MS
  end

  -- get gimbal mount's pitch and yaw
  local mount_pitch_deg = pitch_deg
  local mount_yaw_ef_deg = wrap_180(yaw_bf_deg + math.deg(ahrs:get_yaw()))
  local dist_increment_m = TERRAIN_SPACING:get()

  -- initialise total distance test_loc has moved
  local total_dist = 0
  local dist_max = POI_DIST_MAX:get()

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
      gcs:send_text(MAV_SEVERITY.ERROR, "POI: failed to get terrain alt")
      return update, UPDATE_INTERVAL_MS
    end
  end

  -- check for errors
  if (total_dist >= dist_max) then
    gcs:send_text(MAV_SEVERITY.ERROR, "POI: unable to find terrain within " .. tostring(dist_max) .. " m")
  elseif not terrain_amsl_m then
    gcs:send_text(MAV_SEVERITY.ERROR, "POI: failed to retrieve terrain alt")
  else
    -- test location has dropped below terrain
    -- interpolate along line between prev_test_loc and test_loc
    local dist_interp_m = interpolate(0, dist_increment_m, 0, prev_test_loc:alt() * 0.01 - prev_terrain_amsl_m, test_loc:alt() * 0.01 - terrain_amsl_m)
    local poi_loc = prev_test_loc:copy()
    poi_loc:offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_interp_m)
    gcs:send_text(MAV_SEVERITY.INFO, string.format("POI %.7f, %.7f, %.2f (asml)", poi_loc:lat()/10000000.0, poi_loc:lng()/10000000.0, poi_loc:alt() * 0.01))

    -- if ROI requested then also lock gimbal to location
    if roi_switch_pulled_high then
      mount:set_roi_target(0, poi_loc)
    end

    -- send feedback to GCS so it can display icon on map
    success_count = success_count + 1
    send_camera_feedback(poi_loc:lat(), poi_loc:lng(), poi_loc:alt(), poi_loc:alt(), 0, 0, 0, 0, 0, success_count)
  end

  return update, UPDATE_INTERVAL_MS
end

return update()

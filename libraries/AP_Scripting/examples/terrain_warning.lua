-- height above terrain warning script

-- min altitude above terrain, script will warn if lower than this
local terrain_min_alt = 20

-- warning is only enabled further than this distance from home
local home_dist_enable = 25

-- must have climbed at least this distance above terrain since arming to enable warning
local height_enable = 10

-- warning repeat time in ms
local warn_ms = 10000


local height_threshold_passed = false
local last_warn = 0
function update()

  if not arming:is_armed() then
    -- not armed, nothing to do, reset height threshold
    height_threshold_passed = false
    return update, 1000
  end

  -- get the height above terrain, allowing extrapolation
  -- this used the terrain database only, not considering rangefinders
  local terrain_height = terrain:height_above_terrain(true)
  if not terrain_height then
    -- could not get a valid terrain alt
    return update, 1000
  end

  if (not height_threshold_passed) and (terrain_height < height_enable) then
    -- not climbed far enough to enable, nothing to do
    return update, 1000
  end
  height_threshold_passed = true

  local home_dist = ahrs:get_relative_position_NED_home()
  if home_dist then
    -- only check home dist if we have a valid home
    -- do not consider altitude above home in radius calc
    home_dist:z(0)
    if home_dist:length() < home_dist_enable then
      -- to close to home
      return update, 1000
    end
  end

  if terrain_height < terrain_min_alt then
    -- send message at severity level 2 (MAV_SEVERITY_CRITICAL), this will appear on the MP HUD and be read out if speech is enabled
    gcs:send_text(2, string.format("Terrain Warning: %0.1f meters",terrain_height))
    return update, warn_ms
  end

  return update, 1000
end

return update, 10000

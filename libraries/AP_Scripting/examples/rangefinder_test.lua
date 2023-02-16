-- This script checks RangeFinder
-- luacheck: only 0

local rotation_downward = 25
local rotation_forward = 0

function update()
  local sensor_count = rangefinder:num_sensors()
  gcs:send_text(0, string.format("%d rangefinder sensors found.", sensor_count))

  for i = 0, rangefinder:num_sensors() do
    if rangefinder:has_data_orient(rotation_downward) then
      info(rotation_downward)
    elseif rangefinder:has_data_orient(rotation_forward) then
      info(rotation_forward)
    end
  end

  return update, 1000 -- check again in 1Hz
end

function info(rotation)
  local ground_clearance = rangefinder:ground_clearance_cm_orient(rotation)
  local distance_min = rangefinder:min_distance_cm_orient(rotation)
  local distance_max = rangefinder:max_distance_cm_orient(rotation)
  local offset = rangefinder:get_pos_offset_orient(rotation)
  local distance_cm = rangefinder:distance_cm_orient(rotation)

  gcs:send_text(0, string.format("Rotation %d %.0f cm range %d - %d offset %.0f %.0f %.0f ground clearance %.0f", rotation, distance_cm, distance_min, distance_max, offset:x(), offset:y(), offset:z(), ground_clearance))
end

return update(), 1000 -- first message may be displayed 1 seconds after start-up

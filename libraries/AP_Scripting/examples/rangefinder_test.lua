-- This script checks RangeFinder

local rotation_downward = 25
local rotation_forward = 0

function update()
  local sensor_count = rangefinder:num_sensors()
  gcs:send_text(0, string.format("%d rangefinder sensors found.", sensor_count))

  for _ = 0, rangefinder:num_sensors() do
    if rangefinder:has_data_orient(rotation_downward) then
      info(rotation_downward)
    elseif rangefinder:has_data_orient(rotation_forward) then
      info(rotation_forward)
    end
  end

  return update, 1000 -- check again in 1Hz
end

function info(rotation)
  local ground_clearance = rangefinder:ground_clearance_orient(rotation)
  local distance_min = rangefinder:min_distance_orient(rotation)
  local distance_max = rangefinder:max_distance_orient(rotation)
  local offset = rangefinder:get_pos_offset_orient(rotation)
  local distance = rangefinder:distance_orient(rotation)

  gcs:send_text(
    0,
    string.format(
      "rot=%d distance=%.2f min=%.2f max=%.2f offset=(%.0f,%.0f,%.0f) ground-clearance=%.2f",
      rotation,
      distance,
      distance_min,
      distance_max,
      offset:x(),
      offset:y(),
      offset:z(),
      ground_clearance
    )
  )
end

return update(), 1000 -- first message may be displayed 1 seconds after start-up

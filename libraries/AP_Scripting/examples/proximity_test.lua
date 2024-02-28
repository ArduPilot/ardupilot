-- This script checks Proximity

function update()
  sensor_count = proximity:num_sensors()
  gcs:send_text(0, string.format("%d proximity sensors found.", sensor_count))

  if sensor_count > 0 then
    object_count = proximity:get_object_count()
    gcs:send_text(0, string.format("%d objects found.", object_count))

    closest_angle, closest_distance = proximity:get_closest_object()
    if closest_angle and closest_distance then
      gcs:send_text(0, "Closest object at angle "..closest_angle.." distance "..closest_distance)
    end

    for i = 0, object_count do
      angle, distance = proximity:get_object_angle_and_distance(i)
      if angle and distance then
        gcs:send_text(0, "Object "..i.." at angle "..angle.." distance "..distance)
      end
    end
  end

  return update, 2000 -- check again in 0.5Hz
end

return update(), 2000 -- first message may be displayed 2 seconds after start-up

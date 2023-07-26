-- Maintain a constant ground speed while flying in Circle mode even if the pilot adjusts the radius
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and in circle mode and then
-- updates the target rate around the circle (in degrees / sec) to maintain the desired ground speed

local circle_ground_speed = 6 -- the ground speed in circle mode (m/s)

function update() -- this is the loop which periodically runs

  -- must be armed, flying and in circle mode
  if (not arming:is_armed()) or (not vehicle:get_likely_flying()) or (vehicle:get_mode() ~= 7) then
    return update, 1000 -- reschedules the loop, 1hz
  end

  -- get circle radius
  local radius = vehicle:get_circle_radius()
  if not radius then
    gcs:send_text(0, "speed-circle.lua: failed to get circle radius")
    return update, 1000
  end

  -- set the rate to give the desired ground speed at the current radius
  local new_rate = 360 * (circle_ground_speed / (radius*math.pi*2))
  new_rate = math.max(new_rate, -90)
  new_rate = math.min(new_rate, 90)
  if not vehicle:set_circle_rate(new_rate) then
    gcs:send_text(0, "speed-circle.lua: failed to set rate")
  else
    gcs:send_text(0, string.format("speed-circle.lua: radius:%f new_rate=%f", radius, new_rate))
  end

  return update, 100 -- reschedules the loop, 10hz
end

return update()

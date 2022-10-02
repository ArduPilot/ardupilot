-- Lua Can Driver for RDS02C Benewake Sensor


-- User settable parameters
local sensor_max_range = 25  --max range of sensors in meters
local sensor_min_range = 0    --minimum range of sensors in meters
local update_rate_ms    = 25    -- update rate (in ms) of the driver
local update_rate_error_ms  = 5000 -- update rate incase there is a fatal error (in ms)
local debug_enable = false    -- true to enable debug messages

-- Global variables (DO NOT CHANGE)
local param_num_lua_backend = 14         -- parameter number for lua rangefinder
local lua_backend                        -- store lua prx backend here
local lua_driver_found = false           --true if user has configured lua prx backend
local can_driver = CAN:get_device(5)


function setup_prx_sensor()

  if not can_driver then
    gcs:send_text(0,"No scripting CAN interfaces found")
    return setup_prx_sensor, update_rate_error_ms
  end

  local sensor_count = proximity:num_sensors() -- number of sensors connected
  for j = 0, sensor_count -1 do
    local device = proximity:get_backend(j)
    if ((not lua_driver_found) and  device and (device:type() == param_num_lua_backend)) then
      -- this is a lua driver
      lua_driver_found = true
      lua_backend = device
      lua_backend:set_distance_min_max(sensor_min_range, sensor_max_range)
    end
  end
  if not lua_driver_found then
    -- We can't use this script if user hasn't setup a lua backend
    gcs:send_text(0, string.format("Configure Lua Proximity Sensor"))
    return setup_prx_sensor, update_rate_error_ms
  end
end

function show_frame(frame)
  gcs:send_text(0,string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i", frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
end

function parse_can_frame(frame)
  local object_x_cm = (frame:data(0)*256 + frame:data(1)) - 32768
  local object_y_cm = (frame:data(2)*256) + frame:data(3)
  return object_x_cm*0.01, object_y_cm*0.01
end



function update()
  if not lua_driver_found then
    setup_prx_sensor()
  end

  frame = can_driver:read_frame()
  if not frame then
    -- no frame to parse
    return update, update_rate_ms
  end

  if debug_enable then
    show_frame(frame)
  end

  object_x_m, object_y_m = parse_can_frame(frame)

  if (object_x_m <= 0 or object_y_m <= 0) then
    -- invalid data
    return update, update_rate_ms
  end

  local sent_successfully = lua_backend:handle_script_distance_msg(object_x_m, object_y_m, 0, true)
  if not sent_successfully then
    -- This should never happen as we already checked for a valid configured lua backend above
    gcs:send_text(0, string.format("Proximity Lua Script Error"))
    return update, update_rate_error_ms
  end

  return update, update_rate_ms -- check again in 25ms
end

return update(), 5000 -- first data may be  checked 5 seconds after start-up


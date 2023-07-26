-- Lua Can Driver for Benewake CAN Rangefinder

-- User settable parameters
local update_rate_ms    = 10    -- update rate (in ms) of the driver
local debug_enable = false    -- true to enable debug messages

-- Global variables (DO NOT CHANGE)
local param_num_lua_rfnd_backend = 36         -- parameter number for lua rangefinder
local lua_rfnd_backend                        -- store lua backend here
local lua_rfnd_driver_found = false           -- true if user has configured lua backend


local can_driver = CAN:get_device(5)

gcs:send_text(0,"Scripting started")


-- -------------------------------- RFND DRIVER --------------------------------

function setup_rfnd_sensor()
  if not can_driver then
    gcs:send_text(0,"No scripting CAN interfaces found")
    return
  end

  local sensor_count = rangefinder:num_sensors() -- number of sensors connected
  for j = 0, sensor_count -1 do
    local device = rangefinder:get_backend(j)
    if ((not lua_rfnd_driver_found) and  device and (device:type() == param_num_lua_rfnd_backend)) then
      -- this is a lua driver
      lua_rfnd_driver_found = true
      lua_rfnd_backend = device
    end
  end
  if not lua_rfnd_driver_found then
    -- We can't use this script if user hasn't setup a lua backend
    gcs:send_text(0, string.format("Configure Lua RFND Sensor"))
    return
  end
end

function show_rfnd_frame(frame_rfnd)
  gcs:send_text(0,string.format("RFND msg from " .. tostring(frame_rfnd:id())))
end

function parse_rfnd_can_frame(frame_rfnd)
  local height_cm = (frame_rfnd:data(0)*256 + frame_rfnd:data(1))
  return height_cm*0.01
end

function handle_rfnd_frame(frame_rfnd)
  if debug_enable then
    show_rfnd_frame(frame_rfnd)
  end

  rfnd_dist = parse_rfnd_can_frame(frame_rfnd)

  if (rfnd_dist > 0) then
    local sent_successfully = lua_rfnd_backend:handle_script_msg(rfnd_dist)
    if not sent_successfully then
      -- This should never happen as we already checked for a valid configured lua backend above
      gcs:send_text(0, string.format("RFND Lua Script Error"))
      return
    end
  end
end

-- -------------------------------- MAIN --------------------------------

function update()

  if not lua_rfnd_driver_found then
    setup_rfnd_sensor()
  end

  if (not lua_rfnd_driver_found) then
    -- We can't use this script if user hasn't setup a lua backend
    return
  end

  frame = can_driver:read_frame()
  if not frame then
    -- no frame to parse
    return
  end

  handle_rfnd_frame(frame)

end


-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
      gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
  end
  return protected_wrapper, update_rate_ms
end

-- start running update loop
return protected_wrapper()

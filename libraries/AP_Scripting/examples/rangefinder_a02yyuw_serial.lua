-- Lua driver for A02YYUW serial Rangefinder

-- Set a rangefinder to type 36 (LUA)
-- Set serial to type 28  LUA (e.g. SERIAL5_PROTOCOL 28)


-- User settable parameters
local serial_port = serial:find_serial(0)

-- Global variables (DO NOT CHANGE)
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Type {}.
local RNGFND_TYPE_LUA = 36.0

local A02YYUW_MAX_RANGE_M = 4.5
local A02YYUW_HEADER = 0xFF

local lua_rfnd_backend                        -- store lua backend here
local lua_rfnd_driver_found = false           -- true if user has configured lua backend
local buff = {}
local buffer_index = 1

gcs:send_text(0,"Scripting started")


-- -------------------------------- RFND DRIVER --------------------------------

function setup_rfnd_sensor()
  if not serial_port then
    gcs:send_text(0,"No scripting Serial interfaces found")
    return
  end

  local sensor_count = rangefinder:num_sensors() -- number of sensors connected
  for j = 0, sensor_count -1 do
    local device = rangefinder:get_backend(j)
    if ((not lua_rfnd_driver_found) and  device and (device:type() == RNGFND_TYPE_LUA)) then
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

  serial_port:begin(9600)
  serial_port:set_flow_control(0)
end

-- returns the distance in metres once a full valid frame is decoded, nil otherwise
function decode_rfnd(byte)
  if buffer_index == 1 and byte ~= A02YYUW_HEADER then
    return nil
  end

  buff[buffer_index] = byte
  buffer_index = buffer_index + 1

  if buffer_index <= 4 then
    return nil
  end

  buffer_index = 1
  local crc = (buff[1] + buff[2] + buff[3]) & 0x00FF
  if crc ~= buff[4] then
    -- bad CRC, discard
    return nil
  end
  return ((buff[2] << 8) | buff[3]) * 0.001
end

function handle_rfnd()
  -- format is: [ 0xFF | DATA_H | DATA_L | SUM ]

  -- read all available frames and average the valid readings
  local sum = 0
  local count = 0
  local count_out_of_range = 0
  local n_bytes = serial_port:available()

  while n_bytes > 0 do
    local dist_m = decode_rfnd(serial_port:read())
    if dist_m then
      if dist_m > A02YYUW_MAX_RANGE_M then
        count_out_of_range = count_out_of_range + 1
      else
        sum = sum + dist_m
        count = count + 1
      end
    end
    n_bytes = n_bytes - 1
  end

  local reading_m
  if count > 0 then
    reading_m = sum / count
  elseif count_out_of_range > 0 then
    -- readings beyond the sensor's rated range are not trusted,
    -- report out-of-range-high
    reading_m = math.max(A02YYUW_MAX_RANGE_M, rangefinder:max_distance_orient(lua_rfnd_backend:orientation())) + 1
  else
    return
  end

  -- the backend sets the status and valid count from RNGFNDx_MIN/MAX
  if not lua_rfnd_backend:handle_script_msg(reading_m) then
    -- This should never happen as we already checked for a valid configured lua backend above
    gcs:send_text(0, string.format("RFND Lua Script Error"))
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

  handle_rfnd()

  return update, 100
end

return update, 100

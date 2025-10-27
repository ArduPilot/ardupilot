-- Lua Can Driver for SinoSonar serial Rangefinder

-- Set a rangefinder to type 36 (LUA)
-- Set serial to type 28  LUA (e.g. SERIAL5_PROTOCOL 28)


-- User settable parameters
local serial_port = serial:find_serial(0)

-- Global variables (DO NOT CHANGE)
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Type {}.
local RNGFND_TYPE_LUA = 36.0
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Status {}.
local RNGFND_STATUS_NO_DATA = 1
local RNGFND_STATUS_GOOD = 4
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h
local SIGNAL_QUALITY_MIN = 0
local SIGNAL_QUALITY_MAX = 100

local RNGFND_STATUS_VALID = 10
local RNGFND_STATUS_INVALID = 0

local SINOSONAR_SERIAL_MAX_RANGE_M = 4.5;
local SINOSONAR_HEADER = 0xFF;

local lua_rfnd_backend                        -- store lua backend here
local lua_rfnd_driver_found = false           -- true if user has configured lua backend
local rf_state = RangeFinder_State()
local buff = {}
for i=1, 4 do
  buff[i] = 0
end
local buffer_index = 0
local valid_count = 0
local reading_m = 0.0

gcs:send_text(0,"Scripting started")


-- -------------------------------- RFND DRIVER --------------------------------

function setup_rfnd_sensor()
  serial_port:begin(9600)
  serial_port:set_flow_control(0)

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
end

function decode_rfnd(byte)
  if buffer_index == 1 and byte ~= SINOSONAR_HEADER then
    return false
  end

  buff[buffer_index] = byte;
  buffer_index = buffer_index +1

  if buffer_index == 5 then
    buffer_index = 1;
    local crc = (buff[1] + buff[2] + buff[3]) & 0x00FF;
    if crc ~= buff[4] then
      -- bad CRC, discard
      return false
    end
    reading_m = ((buff[2] << 8) + buff[3]) * 0.001;
    return reading_m <= SINOSONAR_SERIAL_MAX_RANGE_M;

  end
end

function handle_rfnd()
  -- format is: [ 0xFF | DATA_H | DATA_L | SUM ]

  -- read any available lines from the lidar
  local n_bytes = serial_port:available()

  while n_bytes > 0 do
    local byte = serial_port:read()
    if decode_rfnd(byte) then

      -- The full state udata must be initialized.
      rf_state:last_reading(millis():toint())
      rf_state:voltage(0)

      -- Return this measurement to the range finder backend
      rf_state:status(RNGFND_STATUS_GOOD)
      if valid_count < RNGFND_STATUS_VALID then
        valid_count = valid_count + 1
      end
      rf_state:range_valid_count(valid_count)
      rf_state:distance(reading_m)
      rf_state:signal_quality(SIGNAL_QUALITY_MAX)
      break
    else
      valid_count = RNGFND_STATUS_INVALID
    end
    n_bytes = n_bytes - 1
  end


  if (reading_m > 0) then
    local sent_successfully = lua_rfnd_backend:handle_script_msg(rf_state)
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

  handle_rfnd()

  return update, 100
end

return update, 100
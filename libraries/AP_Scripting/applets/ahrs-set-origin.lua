-- Sets the AHRS/EKF origin to a specified Location
--
-- Parameter descriptions
-- AHRS_ORIG_LAT : AHRS Origin Latitude (in degrees)
-- AHRS_ORIG_LON : AHRS Origin Longitude (in degrees)
-- AHRS_ORIGIN_ALT : AHRS Origin Altitude (in meters above sea level)
--
-- How to use
-- 1. Install this script on the flight controller
-- 2. Set AHRS_ORIG_LAT, AHRS_ORIG_LON, AHRS_ORIG_ALT to the desired location
-- 3. A message should appear on the messages screen when the AHRS/EKF origin has been set and the vehicle will most often then appear on the map

local PARAM_TABLE_KEY = 87
PARAM_TABLE_PREFIX = "AHRS_ORIG_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SEND_TEXT_PREFIX = "ahrs-set-origin: "

-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
  return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- add param table
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), SEND_TEXT_PREFIX .. 'could not add param table')

--[[
  // @Param: AHRS_ORIG_LAT
  // @DisplayName: AHRS/EKF Origin Latitude
  // @Description: AHRS/EKF origin will be set to this latitude if not already set
  // @Range: -180 180
  // @User: Standard
--]]
local AHRS_ORIG_LAT = bind_add_param('LAT', 1, 0)

--[[
  // @Param: AHRS_ORIG_LON
  // @DisplayName: AHRS/EKF Origin Longitude
  // @Description: AHRS/EKF origin will be set to this longitude if not already set
  // @Range: -180 180
  // @User: Standard
--]]
local AHRS_ORIG_LON = bind_add_param('LON', 2, 0)

--[[
  // @Param: AHRS_ORIG_ALT
  // @DisplayName: AHRS/EKF Origin Altitude
  // @Description: AHRS/EKF origin will be set to this altitude (in meters above sea level) if not already set
  // @Range: 0 10000
  // @User: Standard
--]]
local AHRS_ORIG_ALT = bind_add_param('ALT', 3, 0)

-- print welcome message
gcs:send_text(MAV_SEVERITY.INFO, SEND_TEXT_PREFIX .. "started")

function update()

    -- wait for AHRS to be initialised
    if not ahrs:initialised() then
        return update, 5000
    end

    -- exit if AHRS/EKF origin has already been set
    if ahrs:get_origin() then
        gcs:send_text(MAV_SEVERITY.WARNING, SEND_TEXT_PREFIX .. "EKF origin already set")
        return
    end

    -- return if parameters have not been set
    if AHRS_ORIG_LAT:get() == 0 and AHRS_ORIG_LON:get() == 0 and AHRS_ORIG_ALT == 0 then
        -- try again in 5 seconds
        return update, 5000
    end

    -- create new origin
    local location = Location()
    location:lat(math.floor(AHRS_ORIG_LAT:get() * 10000000.0))
    location:lng(math.floor(AHRS_ORIG_LON:get() * 10000000.0))
    location:alt(math.floor(AHRS_ORIG_ALT:get() * 100.0))

    -- attempt to send origin
    if ahrs:set_origin(location) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format(SEND_TEXT_PREFIX .. "origin set Lat:%.7f Lon:%.7f Alt:%.1f", AHRS_ORIG_LAT:get(), AHRS_ORIG_LON:get(), AHRS_ORIG_ALT:get()))
    else
        gcs:send_text(MAV_SEVERITY.WARNING, SEND_TEXT_PREFIX .. "failed to set origin")
    end

    -- return and do not try again
    return
end

return update()
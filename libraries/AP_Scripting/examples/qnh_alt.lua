--[[
    example that sends QNH altitude to GCS as a NAMED_VALUE_FLOAT QNG_ALT_FT in feet

    Note: operator must set QNH_PRESSURE in hPa before each flight!
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 93
local PARAM_TABLE_PREFIX = "QNH_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: QNH_PRESSURE
  // @DisplayName: QNH pressure in hPa
  // @Description: QNH pressure in hPa
  // @Range: 900 1200
  // @Units: hPa
  // @User: Standard
--]]
local QNH_PRESSURE = bind_add_param('PRESSURE', 1, 0)

--[[
    send QNH based alt to GCS
--]]
local function send_qnh_alt()
    local pressure_Pa = baro:get_pressure()
    local qnh_Pa = QNH_PRESSURE:get()*100
    local alt_m = baro:get_altitude_difference(qnh_Pa, pressure_Pa)
    local alt_feet = alt_m * 3.280839895013123
    gcs:send_named_float("QNH_ALT_FT", alt_feet)
end

function update()
    if QNH_PRESSURE:get() > 0 then
        send_qnh_alt()
    end
    return update,1000
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded qnh_alt")

return update,1000

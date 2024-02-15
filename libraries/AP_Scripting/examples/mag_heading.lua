--[[ 
   send magnetic heading in degrees as NAMED_VALUE_FLOAT[MAG_HEAD] and NAMED_VALUE_FLOAT[MAG_GCRS]
--]]

local RATE_HZ = 2

-- bind a parameter to a variable given
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

COMPASS_DEC = bind_param("COMPASS_DEC")

function wrap_360(angle)
    local res = angle % 360
    if res < 0 then
      res = res + 360
    end
    return res
end

function update()
   local yaw_deg = wrap_360(math.deg(ahrs:get_yaw() - COMPASS_DEC:get()))
   local gspd = ahrs:groundspeed_vector()
   local gcrs_deg = wrap_360(math.deg(math.atan(gspd:y(), gspd:x()) - COMPASS_DEC:get()))
   gcs:send_named_float("MAG_HEAD", yaw_deg)
   gcs:send_named_float("MAG_GCRS", gcrs_deg)
end

gcs:send_text(0, "MagHeading loaded")

local MAV_SEVERITY_ERROR = 3

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
    return protected_wrapper, math.floor(1000 / RATE_HZ)
end

-- start running update loop
return protected_wrapper()
